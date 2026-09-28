"""Authenticated, offline mass/hover diagnosis; ADR 0025, never qualification."""

import argparse
import hashlib
import io
import json
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import source_sha256
from experiments.estimated_feedback_evidence import load_history, unpack_history
from experiments.robustness_evidence import audit_result, ensure, load_json, save_json
from experiments.robustness_protocol import configuration
from experiments.vertical_compensation_validation import POLICY
from quadrotor_math.attitude_simulation import _plant_step
from quadrotor_math.missions import MissionPhase

ROOT = Path(__file__).resolve().parents[1]
REPORT_DIGESTS = {
    "baseline": "ec85a0d6b55acc278acb18b1c7af7b02cb56c6e985dc48eaf66fbc7fd520bb09",
    "candidate": "46a506cc25881cb86a22fd7c4a3aeb85ff1f6e28244d51b492305922dae8b4bc",
}
Array = NDArray[np.float64]


def authenticated(path: Path, digest: str) -> bytes:
    data = path.read_bytes()
    ensure(hashlib.sha256(data).hexdigest() == digest, f"byte digest mismatch: {path.name}")
    return data


def independent_metrics(a: dict[str, Any], case: str) -> dict[str, float]:
    error = a["mission_position_W"] - a["mission_reference_position_W"]
    norm = np.linalg.norm(error, axis=1)
    result = {
        "tracking_rmse_m": float(np.sqrt(np.mean(norm**2))),
        "final_position_error_m": float(norm[-1]),
        "final_true_speed_m_s": float(np.linalg.norm(a["mission_velocity_W"][-1])),
        "terminal_time_s": float(a["mission_time_s"][-1]),
    }
    if case == "nominal_hover":
        mask = (a["mission_time_s"] >= 5) & (a["mission_time_s"] <= 11)
        result["hover_peak_m"] = float(np.max(norm[mask]))
    return result


def authenticate_campaign(directory: Path, label: str) -> dict[str, Any]:
    """Authenticate every referenced byte and independently rescore all executions."""
    report: dict[str, Any] = json.loads(
        authenticated(directory / "report.json", REPORT_DIGESTS[label])
    )
    for row in report["cases"]:
        for item in row["modes"].values():
            for record in [*item["history"], item["diagnostic"]]:
                authenticated(directory / row["name"] / record["file"], record["sha256"])
        for mode, item in row["modes"].items():
            raw = authenticated(
                directory / row["name"] / item["history"][0]["file"], item["history"][0]["sha256"]
            )
            with np.load(io.BytesIO(raw), allow_pickle=False) as data:
                metrics = independent_metrics(dict(data), row["name"])
            for name, value in metrics.items():
                ensure(
                    abs(value - row["metrics"]["modes"][mode][name]) <= 1e-12,
                    "full-flight metric mismatch",
                )
    return report


def down_axis(q_WB: Array) -> Array:
    """Third column of a unit scalar-first body-to-world quaternion, vectorized."""
    q = np.asarray(q_WB, dtype=float)
    ensure(q.ndim == 2 and q.shape[1] == 4 and np.all(np.isfinite(q)), "quaternion shape")
    ensure(np.allclose(np.sum(q * q, axis=1), 1, atol=1e-12, rtol=0), "unit quaternion")
    w, x, y, z = q.T
    return np.column_stack((2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)))


def held_indices(clock: Array, time: Array) -> NDArray[np.int64]:
    """Right-continuous holds; terminal diagnostics retain the last actual command."""
    ensure(len(clock) > 0 and clock[0] == time[0] == 0, "hold origin")
    ensure(np.all(np.diff(clock) > 0) and np.all(np.diff(time) > 0), "increasing clocks")
    return np.asarray(np.searchsorted(clock, time, side="right") - 1, dtype=np.int64)


def scalar_motor_step(
    z: float,
    v: float,
    speed: float,
    command: float,
    *,
    mass: float,
    kf: float,
    gravity: float,
    tau: float,
    h: float,
) -> tuple[float, float, float]:
    """Exact 1-D translation under four equal exponentially responding rotors.

    This integrates squared rotor speed analytically, independently of RK4.
    Positive z and gravity point down; thrust opposes them.
    """
    ensure(
        all(np.isfinite(x) for x in (z, v, speed, command, mass, kf, gravity, tau, h)),
        "finite scalar inputs",
    )
    ensure(min(mass, kf, gravity, tau, h) > 0 and min(speed, command) >= 0, "scalar domain")
    d, decay = speed - command, np.exp(-h / tau)
    iv = command**2 * h + 2 * command * d * tau * (1 - decay) + d * d * tau / 2 * (1 - decay**2)
    ip = (
        command**2 * h * h / 2
        + 2 * command * d * (h * tau - tau * tau * (1 - decay))
        + d * d * (h * tau / 2 - tau * tau / 4 * (1 - decay**2))
    )
    return (
        z + v * h + gravity * h * h / 2 - 4 * kf / mass * ip,
        v + gravity * h - 4 * kf / mass * iv,
        command + d * decay,
    )


def vertical_model(
    a: dict[str, Any], diagnostic: dict[str, Any], compensated: bool
) -> dict[str, Array]:
    """Unfitted scalar model: perfect vertical state, level thrust, saved reference.

    Actual initial z/v and motor energy, original mass/gains/clocks and saved
    health flags are retained. There is no attitude, estimator noise or drag.
    """
    args = configuration("mass_tracking", "campaign")
    p = args["position_controller"]
    t = a["mission_time_s"]
    h = args["numerics"].time_step_s
    stride = args["numerics"].position_stride
    z, v = float(a["mission_position_W"][0, 2]), float(a["mission_velocity_W"][0, 2])
    speed = float(np.sqrt(np.mean(a["mission_actual_rotor_omega"][0] ** 2)))
    integral, command = 0.0, speed
    states = np.zeros((len(t), 4))
    for k in range(len(t)):
        states[k] = z, v, speed, integral
        if k == len(t) - 1:
            break
        if k % stride == 0:
            e = float(a["mission_reference_position_W"][k, 2] - z)
            velocity = float(a["mission_reference_velocity_W"][k, 2] - v)
            acceleration = (
                a["mission_reference_acceleration_W"][k, 2]
                + p.position_gain_W[2] * e
                + p.velocity_gain_W[2] * velocity
                + integral
            )
            ensure(abs(acceleration) < p.maximum_acceleration_W[2], "scalar model limit reached")
            thrust = p.nominal_mass * (p.nominal_gravity_acceleration - acceleration)
            command = float(np.sqrt(thrust / (4 * args["truth_rotors"].thrust_coefficient)))
            if compensated and diagnostic["vertical"][k // stride]["observations_healthy"]:
                integral += POLICY.integral_gain * POLICY.update_period_s * e
                ensure(abs(integral) < POLICY.maximum_acceleration, "scalar integral limit reached")
        z, v, speed = scalar_motor_step(
            z,
            v,
            speed,
            command,
            mass=args["truth_body"].mass,
            kf=args["truth_rotors"].thrust_coefficient,
            gravity=args["truth_world"].gravity_acceleration,
            tau=args["truth_rotors"].motor_time_constant_s,
            h=h,
        )
    return {
        "position_m": states[:, 0],
        "velocity_m_s": states[:, 1],
        "rotor_speed_rad_s": states[:, 2],
        "integral_m_s2": states[:, 3],
    }


def force_budget(a: dict[str, Any], args: dict[str, Any]) -> dict[str, Array]:
    """Exact ordered acceleration identity, not additive intervention effects."""
    t = a["mission_time_s"]
    inner = held_indices(a["mission_control_time_s"], t)
    outer = held_indices(a["mission_position_control_time_s"], t)
    true_down, estimate_down = down_axis(a["mission_q_WB"]), down_axis(a["estimate_q_WB"])
    desired_down = down_axis(a["mission_q_reference_WB"])[inner]
    thrust = args["truth_rotors"].thrust_coefficient * np.sum(
        a["mission_actual_rotor_omega"] ** 2, axis=1
    )
    commanded = a["mission_collective_thrust"][inner]
    mass, nominal = args["truth_body"].mass, args["position_controller"].nominal_mass
    gravity = np.array([0.0, 0.0, args["truth_world"].gravity_acceleration])
    desired = a["mission_feasible_acceleration_W"][outer]
    terms = {
        "requested": desired,
        "mass": (1 - nominal / mass) * (gravity - desired),
        "attitude_estimation": -commanded[:, None] / mass * (true_down - estimate_down),
        "attitude_tracking": -commanded[:, None] / mass * (estimate_down - desired_down),
        "motor": -(thrust - commanded)[:, None] / mass * true_down,
    }
    actual = gravity - thrust[:, None] / mass * true_down
    ensure(np.max(np.abs(sum(terms.values()) - actual)) < 2e-13, "force-budget closure")
    return {
        **terms,
        "actual": actual,
        "actual_thrust_N": thrust,
        "commanded_thrust_N": commanded,
        "attitude_estimation_deg": np.rad2deg(
            np.arccos(np.clip(np.sum(true_down * estimate_down, axis=1), -1, 1))
        ),
        "attitude_tracking_deg": np.rad2deg(
            np.arccos(np.clip(np.sum(estimate_down * desired_down, axis=1), -1, 1))
        ),
    }


def check_plant(a: dict[str, Any], args: dict[str, Any]) -> dict[str, float]:
    """Reconstruct each stage-resolved nonlinear plant interval from saved commands."""
    t = a["mission_time_s"]
    inner = held_indices(a["mission_control_time_s"], t)
    fields = ("position_W", "velocity_W", "q_WB", "omega_B")
    residual, motor_residual = 0.0, 0.0
    h = args["numerics"].time_step_s
    for k in range(len(t) - 1):
        state = tuple(a["mission_" + f][k] for f in fields)
        next_state, speeds = _plant_step(
            state,
            a["mission_actual_rotor_omega"][k],
            a["mission_commanded_rotor_omega"][inner[k]],
            np.zeros(3),
            args["truth_body"],
            args["truth_rotors"],
            args["truth_world"],
            h,
        )
        residual = max(
            residual,
            max(
                float(np.max(np.abs(s - a["mission_" + f][k + 1])))
                for s, f in zip(next_state, fields, strict=True)
            ),
        )
        motor_residual = max(
            motor_residual, float(np.max(np.abs(speeds - a["mission_actual_rotor_omega"][k + 1])))
        )
    ensure(residual == motor_residual == 0, "nonlinear plant/motor reconstruction")
    return {"plant_max_abs_residual": residual, "motor_max_abs_residual": motor_residual}


def rms_axis(x: Array) -> list[float]:
    return np.sqrt(np.mean(x * x, axis=0)).tolist()  # type: ignore[no-any-return]


def describe(
    a: dict[str, Any], case: str, diagnostic: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    t = a["mission_time_s"]
    args = configuration(case, "campaign")
    error = a["mission_position_W"] - a["mission_reference_position_W"]
    estimate = a["estimate_position_W"] - a["mission_position_W"]
    norms = np.linalg.norm(error, axis=1)
    budget = force_budget(a, args)
    windows = {
        "full": np.ones(len(t), dtype=bool),
        "0_1": t <= 1,
        "1_5": (t >= 1) & (t <= 5),
        "5_11": (t >= 5) & (t <= 11),
        "last_5": t >= t[-1] - 5,
    }
    windows.update(
        {
            "phase_" + MissionPhase(int(p)).name: a["mission_phase"] == p
            for p in np.unique(a["mission_phase"])
        }
    )
    summaries = {}
    for name, mask in windows.items():
        summaries[name] = {
            "samples": int(mask.sum()),
            "tracking_rmse_axis_m": rms_axis(error[mask]),
            "estimation_rmse_axis_m": rms_axis(estimate[mask]),
            "tracking_rmse_m": float(np.sqrt(np.mean(norms[mask] ** 2))),
            "acceleration_rms_axis_m_s2": {
                key: rms_axis(budget[key][mask])
                for key in (
                    "requested",
                    "mass",
                    "attitude_estimation",
                    "attitude_tracking",
                    "motor",
                )
            },
        }
    peak_mask = windows["5_11"] if case == "nominal_hover" else windows["full"]
    peak = int(np.flatnonzero(peak_mask)[np.argmax(norms[peak_mask])])
    timeline = sorted(
        {0, peak, *[int(round(x / 0.0025)) for x in (0.2, 1, 2.28, 5, 11, 15) if x <= t[-1]]}
    )
    outer = np.searchsorted(t, a["mission_position_control_time_s"])
    pc = args["position_controller"]
    pd = (
        a["mission_reference_acceleration_W"][outer]
        + pc.position_gain_W
        * (a["mission_reference_position_W"][outer] - a["estimate_position_W"][outer])
        + pc.velocity_gain_W
        * (a["mission_reference_velocity_W"][outer] - a["estimate_velocity_W"][outer])
    )
    integral = np.array([s["applied_acceleration"] for s in diagnostic.get("vertical", [])])
    if len(integral):
        pd[:, 2] += integral
    ensure(
        np.max(np.abs(pd - a["mission_requested_acceleration_W"])) < 2e-14,
        "PD/integral sign reconstruction",
    )
    rows = [
        {
            "time_s": float(t[k]),
            "tracking_m": error[k].tolist(),
            "estimation_m": estimate[k].tolist(),
            "attitude_estimation_deg": float(budget["attitude_estimation_deg"][k]),
            "attitude_tracking_deg": float(budget["attitude_tracking_deg"][k]),
            "thrust_N": float(budget["actual_thrust_N"][k]),
            "integral_m_s2": float(integral[min(k // 8, len(integral) - 1)])
            if len(integral)
            else 0,
        }
        for k in timeline
    ]
    result = {
        "metrics": independent_metrics(a, case),
        "windows": summaries,
        "peak_time_s": float(t[peak]),
        "peak_error_m": error[peak].tolist(),
        "peak_norm_m": float(norms[peak]),
        "timeline": rows,
        "limits": {
            "inner": int(a["mission_inner_limit_flags"].sum()),
            "outer": int(a["mission_outer_limit_flags"].sum()),
            "rotor_bound_contacts": int(
                np.sum(
                    (a["mission_actual_rotor_omega"] <= args["truth_rotors"].minimum_rotor_omega)
                    | (a["mission_actual_rotor_omega"] >= args["truth_rotors"].maximum_rotor_omega)
                )
            ),
        },
        "terminal_phase": MissionPhase(int(a["mission_phase"][-1])).name,
        "health_transitions": {
            kind: [
                (s["time_s"], s[kind]["state"])
                for i, s in enumerate(diagnostic["health"])
                if i == 0 or s[kind]["state"] != diagnostic["health"][i - 1][kind]["state"]
            ]
            for kind in ("local_position", "barometric_altitude")
        },
        **check_plant(a, args),
    }
    signals = {"time_s": t, "tracking_error_W": error, "estimation_error_W": estimate, **budget}
    if case == "mass_tracking":
        model = vertical_model(a, diagnostic, bool(len(integral)))
        difference = model["position_m"] - a["mission_position_W"][:, 2]
        model_error = model["position_m"] - a["mission_reference_position_W"][:, 2]
        index = int(np.argmax(np.abs(model_error)))
        result["scalar_model"] = {
            "trajectory_discrepancy_rmse_m": float(np.sqrt(np.mean(difference**2))),
            "adequate_within_1cm": bool(np.sqrt(np.mean(difference**2)) <= 0.01),
            "peak_vertical_error_m": float(model_error[index]),
            "peak_time_s": float(t[index]),
            "terminal_integral_m_s2": float(model["integral_m_s2"][-1]),
            "tracking_rmse_z_m": float(np.sqrt(np.mean(model_error**2))),
        }
        signals.update({"model_" + k: v for k, v in model.items()})
    return result, signals


def run(baseline: Path, candidate: Path, output: Path) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=False)
    source = source_sha256(ROOT)
    reports = {
        key: authenticate_campaign(path, key)
        for key, path in (("baseline", baseline), ("candidate", candidate))
    }
    summary: dict[str, Any] = {
        "diagnostic_source_sha256": source,
        "input_reports": REPORT_DIGESTS,
        "flight_source_sha256": {k: r["source_sha256"] for k, r in reports.items()},
        "qualification": False,
        "new_flights": 0,
        "metrics_recomputed": 48,
        "cases": {},
    }
    for label, directory in (("baseline", baseline), ("candidate", candidate)):
        for case in ("mass_tracking", "nominal_hover"):
            row = next(r for r in reports[label]["cases"] if r["name"] == case)
            record = row["modes"]["on"]
            a = load_history(directory / case, 1, record["history"])
            result, _ = unpack_history(a)
            diagnostic = load_json(directory / case, "diagnostic-on.json", record["diagnostic"])
            audit_result(
                case,
                "campaign",
                "on",
                result,
                diagnostic,
                compensation_policy=POLICY if label == "candidate" else None,
            )
            item, signals = describe(a, case, diagnostic)
            name = label + "_" + case
            path = output / (name + ".npz")
            with path.open("xb") as stream:
                np.savez_compressed(stream, **signals)
            item["signals"] = {
                "file": path.name,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            summary["cases"][name] = item
            print(
                json.dumps(
                    {
                        "case": name,
                        "metrics": item["metrics"],
                        "scalar_model": item.get("scalar_model"),
                    }
                ),
                flush=True,
            )
    ensure(source_sha256(ROOT) == source, "diagnostic source changed during analysis")
    save_json(output, "report.json", summary)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.baseline, args.candidate, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
