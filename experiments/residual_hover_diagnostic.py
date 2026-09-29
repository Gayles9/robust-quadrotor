"""ADR 0032 authenticated offline supported-hover diagnosis; no flight execution."""

import argparse
import hashlib
import io
import json
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import source_sha256
from experiments.early_flight_diagnostic import (
    authenticated,
    check_plant,
    down_axis,
    force_budget,
    held_indices,
)
from experiments.hover_response import Array, cascade_model, pd_response
from experiments.robustness_evidence import ensure, equal_json, save_json
from experiments.supported_start import configuration_for, plain_configuration
from experiments.supported_start_validation import score
from experiments.supported_validation_protocol import Job, jobs, modes, prepare_job
from quadrotor_math.attitude_control import compute_attitude_control
from quadrotor_math.dynamics import quadratic_drag_force_body
from quadrotor_math.position_control import PositionReference, compute_position_control
from quadrotor_math.rotations import rotation_matrix_body_to_world

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0032-residual-supported-hover-diagnosis.md"
INPUT_SHA = "b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63"
CHANNELS = (
    "initial",
    "reference",
    "navigation_position",
    "navigation_velocity",
    "limits",
    "attitude_estimation",
    "attitude_tracking",
    "mass",
    "motor",
    "drag",
    "within_step",
)


def authenticate(directory: Path) -> dict[str, Any]:
    """Authenticate the fixed report and every referenced payload; rescore all 34 flights."""
    report: dict[str, Any] = json.loads(authenticated(directory / "report.json", INPUT_SHA))
    equal_json(
        json.loads((directory / "protocol.json").read_bytes()), report["protocol"], "input protocol"
    )
    ensure([r["name"] for r in report["cases"]] == [j.name for j in jobs()], "input job order")
    count = 0
    for job, row in zip(jobs(), report["cases"], strict=True):
        target = directory / job.name
        equal_json(json.loads((target / "case.json").read_bytes()), row, "input case")
        for record in (row["support"], row["release"]):
            authenticated(target / record["file"], record["sha256"])
        ensure(set(row["modes"]) == set(modes(job)), "input modes")
        for item in row["modes"].values():
            for record in [item["configuration"], item["diagnostic"], *item["history"]]:
                authenticated(target / record["file"], record["sha256"])
            record = item["history"][0]
            with np.load(
                io.BytesIO(authenticated(target / record["file"], record["sha256"])),
                allow_pickle=False,
            ) as data:
                metrics = score(dict(data), job.case)
            equal_json(
                metrics,
                {k: v for k, v in item["metrics"].items() if k != "abort_reason"},
                "full saved score",
            )
            count += 1
    ensure(count == 34, "input flight count")
    return report


def command_residual(a: dict[str, Any], args: dict[str, Any]) -> float:
    """Recompute both command loops with the actual saved estimate and reference."""
    time = a["mission_time_s"]
    outer = np.searchsorted(time, a["mission_position_control_time_s"])
    inner = np.searchsorted(time, a["mission_control_time_s"])
    held = held_indices(a["mission_position_control_time_s"], a["mission_control_time_s"])
    residual = 0.0
    commands = []
    for j, k in enumerate(outer):
        # Hover has fixed zero yaw throughout, inherited from the frozen plan.
        ref = PositionReference(
            a["mission_reference_position_W"][k],
            a["mission_reference_velocity_W"][k],
            a["mission_reference_acceleration_W"][k],
            0.0,
        )
        out = compute_position_control(
            a["estimate_position_W"][k],
            a["estimate_velocity_W"][k],
            ref,
            args["position_controller"],
        )
        commands.append(out)
        for field in ("requested_acceleration_W", "feasible_acceleration_W"):
            residual = max(
                residual, float(np.max(np.abs(getattr(out, field) - a["mission_" + field][j])))
            )
        ensure(
            np.array_equal(
                a["mission_outer_limit_flags"][j],
                [out.acceleration_limited, out.tilt_limited, out.thrust_limited],
            ),
            "outer flags",
        )
    for j, k in enumerate(inner):
        target = commands[held[j]]
        attitude = compute_attitude_control(
            a["estimate_q_WB"][k],
            a["angular_velocity_estimate_B"][k],
            target.q_reference_WB,
            target.collective_thrust,
            args["attitude_controller"],
        )
        for first, second in (
            (attitude.allocation.commanded_rotor_omega, a["mission_commanded_rotor_omega"][j]),
            (target.q_reference_WB, a["mission_q_reference_WB"][j]),
            (target.collective_thrust, a["mission_collective_thrust"][j]),
        ):
            residual = max(residual, float(np.max(np.abs(first - second))))
    ensure(residual <= 1e-12, "saved command reconstruction")
    return residual


def response_budget(a: dict[str, Any], args: dict[str, Any]) -> dict[str, Array]:
    """Closed-loop sample-held PD response to an ordered acceleration identity."""
    t, p, v = (a["mission_" + key] for key in ("time_s", "position_W", "velocity_W"))
    gain = args["position_controller"]
    outer = held_indices(a["mission_position_control_time_s"], t)
    outer_epoch = np.searchsorted(t, a["mission_position_control_time_s"])[outer]
    b = force_budget(a, args)
    drag = np.array(
        [
            (r := rotation_matrix_body_to_world(q))
            @ quadratic_drag_force_body(
                speed,
                r,
                args["truth_world"].wind_velocity_W,
                args["truth_body"].quadratic_drag_coefficient_B,
            )
            / args["truth_body"].mass
            for q, speed in zip(a["mission_q_WB"], v, strict=True)
        ]
    )
    acceleration = np.zeros((len(t) - 1, len(CHANNELS), 3))

    def put(name: str, value: Array) -> None:
        acceleration[:, CHANNELS.index(name)] = value[:-1]

    put(
        "reference",
        (
            gain.position_gain_W * a["mission_reference_position_W"]
            + gain.velocity_gain_W * a["mission_reference_velocity_W"]
            + a["mission_reference_acceleration_W"]
        )[outer_epoch],
    )
    put("navigation_position", -gain.position_gain_W * (a["estimate_position_W"] - p)[outer_epoch])
    put("navigation_velocity", -gain.velocity_gain_W * (a["estimate_velocity_W"] - v)[outer_epoch])
    put(
        "limits",
        (a["mission_feasible_acceleration_W"] - a["mission_requested_acceleration_W"])[outer],
    )
    for name in ("attitude_estimation", "attitude_tracking", "mass", "motor"):
        put(name, b[name])
    put("drag", drag)
    actual = b["actual"] + drag
    reconstructed = (
        -gain.position_gain_W * p[outer_epoch[:-1]]
        - gain.velocity_gain_W * v[outer_epoch[:-1]]
        + acceleration.sum(axis=1)
    )
    closure = float(np.max(np.abs(reconstructed - actual[:-1])))
    ensure(closure <= 1e-11, "acceleration identity closure")
    h = np.diff(t)[:, None]
    dp, dv = np.zeros_like(acceleration), np.zeros_like(acceleration)
    # Exact integration remainder; independently reconstructed plant already checked.
    dp[:, -1] = np.diff(p, axis=0) - h * v[:-1] - h**2 / 2 * actual[:-1]
    dv[:, -1] = np.diff(v, axis=0) - h * actual[:-1]
    initial_p, initial_v = np.zeros((len(CHANNELS), 3)), np.zeros((len(CHANNELS), 3))
    initial_p[0], initial_v[0] = p[0], v[0]
    pp, vv = pd_response(
        t,
        outer_epoch[:-1],
        gain.position_gain_W,
        gain.velocity_gain_W,
        acceleration,
        dp,
        dv,
        initial_p,
        initial_v,
    )
    residual = max(
        float(np.max(np.abs(pp.sum(axis=1) - p))), float(np.max(np.abs(vv.sum(axis=1) - v)))
    )
    ensure(residual <= 1e-10, "full response closure")
    early = (t[:-1] < 0.5)[:, None, None]
    early_forcing = acceleration.copy()
    early_forcing[:, CHANNELS.index("reference")] = 0
    ep, ev = pd_response(
        t,
        outer_epoch[:-1],
        gain.position_gain_W,
        gain.velocity_gain_W,
        early_forcing * early,
        dp * early,
        dv * early,
        initial_p,
        initial_v,
    )
    errors = pp.copy()
    errors[:, CHANNELS.index("reference")] -= a["mission_reference_position_W"]
    return {
        "response_position_W": pp,
        "response_velocity_W": vv,
        "response_error_W": errors,
        "early_response_position_W": ep,
        "early_response_velocity_W": ev,
        "forcing_W": acceleration,
        "position_increment_W": dp,
        "velocity_increment_W": dv,
        "actual_acceleration_W": actual,
        "outer_epoch": outer_epoch,
        "acceleration_closure": np.array(closure),
        "response_closure": np.array(residual),
        "axis_estimation_deg": b["attitude_estimation_deg"],
        "axis_tracking_deg": b["attitude_tracking_deg"],
    }


def local_cascade(
    a: dict[str, Any], args: dict[str, Any]
) -> tuple[dict[str, Array], dict[str, Any]]:
    """Use the horizontal isotropic near-level model with all observed error drives."""
    outer, inner = args["position_controller"], args["attitude_controller"]
    ensure(
        outer.position_gain_W[0] == outer.position_gain_W[1]
        and outer.velocity_gain_W[0] == outer.velocity_gain_W[1]
        and inner.attitude_gain_B[0] == inner.attitude_gain_B[1]
        and inner.rate_gain_B[0] == inner.rate_gain_B[1],
        "isotropic horizontal gains",
    )
    ensure(np.array_equal(inner.nominal_inertia_B, args["truth_body"].inertia_B), "matched inertia")
    ensure(np.all(a["mission_reference_position_W"][:, :2] == 0), "zero horizontal reference")
    gravity = args["truth_world"].gravity_acceleration
    inclination, rate = [], []
    for quats, omega in (
        (a["mission_q_WB"], a["mission_omega_B"]),
        (a["estimate_q_WB"], a["angular_velocity_estimate_B"]),
    ):
        inclination.append(-gravity * down_axis(quats)[:, :2])
        rate.append(
            np.array(
                [
                    -gravity * (rotation_matrix_body_to_world(q) @ np.cross(w, [0.0, 0.0, 1.0]))[:2]
                    for q, w in zip(quats, omega, strict=True)
                ]
            )
        )
    initial = np.array(
        [
            a["mission_position_W"][0, :2],
            a["mission_velocity_W"][0, :2],
            inclination[0][0],
            rate[0][0],
            np.zeros(2),
        ]
    )
    states, eigenvalues = cascade_model(
        a["mission_time_s"],
        initial,
        (a["estimate_position_W"] - a["mission_position_W"])[:, :2],
        (a["estimate_velocity_W"] - a["mission_velocity_W"])[:, :2],
        inclination[1] - inclination[0],
        rate[1] - rate[0],
        kp=float(outer.position_gain_W[0]),
        kv=float(outer.velocity_gain_W[0]),
        ka=float(inner.attitude_gain_B[0]),
        kr=float(inner.rate_gain_B[0]),
        tau=args["truth_rotors"].motor_time_constant_s,
        outer_stride=args["numerics"].position_stride,
        inner_stride=args["numerics"].attitude_stride,
    )
    difference = states[:, 0] - a["mission_position_W"][:, :2]
    poles = np.log(eigenvalues.astype(complex)) / (
        args["numerics"].time_step_s * args["numerics"].position_stride
    )
    return {"cascade_states": states, "cascade_error_W": difference}, {
        "horizontal_rms_discrepancy_m": float(np.sqrt(np.mean(np.sum(difference**2, axis=1)))),
        "horizontal_peak_discrepancy_m": float(np.linalg.norm(difference, axis=1).max()),
        "discrete_poles": [[float(x.real), float(x.imag)] for x in eigenvalues],
        "equivalent_poles_per_s": [[float(x.real), float(x.imag)] for x in poles],
        "spectral_radius": float(np.max(np.abs(eigenvalues))),
    }


def describe(a: dict[str, Any], b: dict[str, Array]) -> dict[str, Any]:
    t = a["mission_time_s"]
    physical = a["mission_position_W"] - a["mission_reference_position_W"]
    navigation = a["estimate_position_W"] - a["mission_position_W"]
    candidates = np.flatnonzero((t >= 5) & (t <= 11))
    k = candidates[np.argmax(np.linalg.norm(physical[candidates], axis=1))]
    peak = {
        "time_s": float(t[k]),
        "true_error_m": physical[k].tolist(),
        "estimated_error_m": (physical[k] + navigation[k]).tolist(),
        "navigation_error_m": navigation[k].tolist(),
        "true_velocity_m_s": a["mission_velocity_W"][k].tolist(),
        "navigation_velocity_error_m_s": (
            a["estimate_velocity_W"][k] - a["mission_velocity_W"][k]
        ).tolist(),
        "response_m": dict(zip(CHANNELS, b["response_error_W"][k].tolist(), strict=True)),
        "early_response_m": dict(
            zip(CHANNELS, b["early_response_position_W"][k].tolist(), strict=True)
        ),
    }
    windows = {}
    for name, mask in {
        "full": t >= 0,
        "release_0_0p5": t <= 0.5,
        "takeoff_0p5_5": (t >= 0.5) & (t <= 5),
        "hover_5_11": (t >= 5) & (t <= 11),
    }.items():
        signals = {
            "navigation_position_m": navigation,
            "navigation_velocity_m_s": a["estimate_velocity_W"] - a["mission_velocity_W"],
            "accelerometer_bias_error_m_s2": a["estimate_accelerometer_bias_B"]
            - a["true_accelerometer_bias_B"],
            "gyroscope_bias_error_rad_s": a["estimate_gyroscope_bias_B"]
            - a["true_gyroscope_bias_B"],
        }
        windows[name] = {
            key: {
                "rms": float(np.sqrt(np.mean(np.sum(x[mask] ** 2, axis=1)))),
                "peak": float(np.max(np.linalg.norm(x[mask], axis=1))),
            }
            for key, x in signals.items()
        }
        for key in ("axis_estimation_deg", "axis_tracking_deg"):
            windows[name][key] = {
                "rms": float(np.sqrt(np.mean(b[key][mask] ** 2))),
                "peak": float(b[key][mask].max()),
            }
    return {
        "peak": peak,
        "windows": windows,
        "outer_limited_rows": int(np.any(a["mission_outer_limit_flags"], axis=1).sum()),
        "inner_limited_rows": int(np.any(a["mission_inner_limit_flags"], axis=1).sum()),
    }


def run(campaign: Path, output: Path) -> dict[str, Any]:
    report = authenticate(campaign)
    output.mkdir(exist_ok=False, parents=True)
    rows = []
    for row in report["cases"]:
        if row["case"] != "nominal_hover":
            continue
        prepared = prepare_job(Job(row["case"], row["seed"]))
        for mode in ("unaligned", "aligned"):
            target = campaign / row["name"]
            item = row["modes"][mode]
            args = configuration_for(prepared, mode)
            record = item["configuration"]
            equal_json(
                json.loads(authenticated(target / record["file"], record["sha256"])),
                {key: plain_configuration(value) for key, value in args.items()},
                "diagnostic configuration",
            )
            record = item["history"][0]
            with np.load(
                io.BytesIO(authenticated(target / record["file"], record["sha256"])),
                allow_pickle=False,
            ) as data:
                a = dict(data)
            plant = check_plant(a, args)
            command = command_residual(a, args)
            budget = response_budget(a, args)
            model, fit = local_cascade(a, args)
            name = row["name"] + "-" + mode
            path = output / (name + ".npz")
            signals: dict[str, Any] = {
                **budget,
                **model,
                **{k: v for k, v in a.items() if not k.startswith("baseline_")},
            }
            with path.open("xb") as stream:
                np.savez_compressed(stream, **signals)
            rows.append(
                {
                    "name": name,
                    "source": item["history"],
                    "metrics": score(a, row["case"]),
                    "plant": plant,
                    "command_residual": command,
                    "acceleration_closure": float(budget["acceleration_closure"]),
                    "response_closure": float(budget["response_closure"]),
                    "diagnosis": describe(a, budget),
                    "local_model": fit,
                    "signals": {
                        "file": path.name,
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    },
                }
            )
            print(name, "reconstruction PASS", flush=True)
    result = {
        "input_report_sha256": INPUT_SHA,
        "audited_commit": "5e532ab1f8b991643aff7628ea59d69d2915cacc",
        "source_sha256": source_sha256(ROOT),
        "protocol_sha256": hashlib.sha256(ADR.read_bytes()).hexdigest(),
        "channels": CHANNELS,
        "cases": rows,
        "new_scientific_flights": 0,
        "integration_go": False,
        "diagnostic_complete": len(rows) == 6,
    }
    save_json(output, "report.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.campaign, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
