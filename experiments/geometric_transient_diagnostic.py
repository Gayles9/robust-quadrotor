"""Authenticate and reconstruct the bounded startup/hover command path (ADR 0020)."""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.geometric_correction_validation import verify_payloads
from experiments.geometric_reimplementation_validation import configuration
from experiments.geometric_sampled_analysis import GeometricHoverMap
from experiments.geometric_transient_study import candidate_configuration, shaped_reference
from quadrotor_math.attitude_simulation import _wrench
from quadrotor_math.geometric_control import compute_geometric_control
from quadrotor_math.geometric_filter import FeedbackDerivativeFilter
from quadrotor_math.geometric_reference import (
    build_geometric_reference,
    projected_collective_thrust,
    reference_jerk_snap,
)
from quadrotor_math.missions import mission_reference
from quadrotor_math.rotations import rotation_matrix_body_to_world as rotation


def analyze(directory: Path, row: dict[str, Any], output: Path, shaped: bool) -> dict[str, Any]:
    """Reconstruct from posterior/gyro histories; truth is used only in diagnostics."""
    config = (candidate_configuration if shaped else configuration)(row["job"])
    data_file = next(r["file"] for r in row["files"] if r["file"].endswith("-data.npz"))
    with np.load(directory / data_file, allow_pickle=False) as data:
        a = {k: data[k] for k in data.files}
    with np.load(directory / "mission.npz", allow_pickle=False) as data:
        m = {k: data[k] for k in data.files}
    times, control_times = m["time_s"], m["control_time_s"]
    outer, inner, gains = (
        config[k] for k in ("position_controller", "attitude_controller", "geometric_controller")
    )
    corrections = np.zeros((len(times), 6))
    metadata = json.loads(a["metadata_json"].tobytes())
    for event in metadata["events"]:
        k = event["observation"]["delivery_index"]
        if event["update"] is not None and k is not None and 0 <= k < len(times):
            corrections[k] += np.asarray(event["update"]["error_state_correction"])[:6]
    memory = FeedbackDerivativeFilter(0.02, gains.filter_pole_rad_s)
    builder = shaped_reference if shaped else build_geometric_reference
    target = None
    records: dict[str, list[Any]] = {
        key: []
        for key in (
            "lift_W",
            "lift_rate_W",
            "lift_acceleration_W",
            "desired_down_W",
            "terms_Nm",
            "reconstructed_moment_B",
            "reconstructed_allocated_B",
            "reconstructed_thrust_N",
        )
    }
    rate = acceleration = np.zeros(3)
    for time in control_times:
        k = int(np.searchsorted(times, time))
        if times[k] != time:
            raise ValueError("control timestamp is not on the saved plant clock")
        if k % config["numerics"].position_stride == 0:
            ref, _ = mission_reference(config["plan"], float(time))
            jerk, snap = reference_jerk_snap(config["plan"], float(time))
            target, memory = builder(
                a["estimate_position_W"][k],
                a["estimate_velocity_W"][k],
                ref,
                jerk,
                snap,
                outer,
                memory,
                k // config["numerics"].position_stride,
            )
            w, s = memory.pole_rad_s, memory.sections
            rate = -outer.nominal_mass * jerk + w * (s[1] - s[2])
            acceleration = -outer.nominal_mass * snap + w * w * (s[0] - 2 * s[1] + s[2])
        assert target is not None
        q, omega = a["estimate_q_WB"][k], a["angular_velocity_estimate_B"][k]
        R, Rd = rotation(q), rotation(target.rotation.q_reference_WB)
        A, J = R.T @ Rd, inner.nominal_inertia_B
        E = Rd.T @ R - A
        er = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]]) / 4
        desired = A @ target.rotation.omega_reference_D
        terms = np.array(
            [
                -gains.attitude_stiffness * er,
                -gains.rate_damping * omega,
                gains.rate_damping * desired,
                np.cross(omega, J @ omega),
                -J @ np.cross(omega, desired),
                J @ A @ target.rotation.alpha_reference_D,
            ]
        )
        thrust = projected_collective_thrust(target, q, outer)
        command = compute_geometric_control(q, omega, target.rotation, thrust, inner, gains)
        for key, value in zip(
            records,
            (
                target.lift_W,
                rate,
                acceleration,
                Rd[:, 2],
                terms,
                terms.sum(axis=0),
                command.allocation.allocated_moment_B,
                thrust,
            ),
            strict=True,
        ):
            records[key].append(value)
    arrays = {key: np.asarray(value) for key, value in records.items()}
    residuals = {
        "requested_moment_Nm": float(
            np.max(np.abs(arrays["reconstructed_moment_B"] - m["moment_requested_B"]))
        ),
        "allocated_moment_Nm": float(
            np.max(np.abs(arrays["reconstructed_allocated_B"] - m["allocated_moment_B"]))
        ),
        "collective_thrust_N": float(
            np.max(np.abs(arrays["reconstructed_thrust_N"] - m["collective_thrust"]))
        ),
        "actual_moment_Nm": float(
            np.max(
                np.abs(
                    np.array(
                        [_wrench(v, config["truth_rotors"])[1] for v in m["actual_rotor_omega"]]
                    )
                    - m["actual_moment_B"]
                )
            )
        ),
    }
    if max(residuals.values()) > 1e-12:
        raise ValueError(f"command reconstruction failed: {residuals}")
    truth_error = m["position_W"] - m["reference_position_W"]
    estimate_error = a["estimate_position_W"] - m["position_W"]
    tilt_error = np.array(
        [
            rotation(e)[:, 2] - rotation(t)[:, 2]
            for e, t in zip(a["estimate_q_WB"], m["q_WB"], strict=True)
        ]
    )
    norm = np.linalg.norm(truth_error, axis=1)
    hold = (
        (times >= 5) & (times <= 65) if row["job"]["mode"] == "hover" else np.ones(len(times), bool)
    )
    peak = int(np.flatnonzero(hold)[np.argmax(norm[hold])])
    first_fail = np.flatnonzero(hold & (norm > 0.08))
    samples = []
    for requested in (0.0, 0.2, 1.0, 2.0, 3.0, 4.0, 5.0, float(times[peak])):
        k = int(np.argmin(np.abs(times - requested)))
        samples.append(
            dict(
                time_s=float(times[k]),
                true_position_error_m=float(norm[k]),
                position_estimation_error_m=float(np.linalg.norm(estimate_error[k])),
                horizontal_down_axis_error_rad_approx=float(np.linalg.norm(tilt_error[k, :2])),
                true_horizontal_speed_m_s=float(np.linalg.norm(m["velocity_W"][k, :2])),
            )
        )
    arrays.update(
        time_s=times,
        control_time_s=control_times,
        truth_position_error_W=truth_error,
        position_estimation_error_W=estimate_error,
        down_axis_estimation_error_W=tilt_error,
        accepted_position_velocity_corrections=corrections,
        moment_requested_B=m["moment_requested_B"],
        allocated_moment_B=m["allocated_moment_B"],
        actual_moment_B=m["actual_moment_B"],
        actual_rotor_omega=m["actual_rotor_omega"],
        commanded_rotor_omega=m["commanded_rotor_omega"],
    )
    file = output / (directory.name + "-aligned.npz")
    np.savez_compressed(file, allow_pickle=False, **arrays)
    return dict(
        job=row["job"],
        reconstruction_residuals=residuals,
        maximum_error_m=float(norm[peak]),
        peak_time_s=float(times[peak]),
        first_hold_failure_time_s=float(times[first_fail[0]]) if len(first_fail) else None,
        samples=samples,
        term_rms_Nm=np.sqrt(np.mean(np.sum(arrays["terms_Nm"] ** 2, axis=2), axis=0)).tolist(),
        file=file.name,
        sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--shaped", action="store_true")
    args = parser.parse_args()
    report = json.loads((args.input / "report.json").read_bytes())
    verification = verify_payloads(report["rows"], args.input)
    if not verification["passed"]:
        raise ValueError(f"unauthenticated input: {verification}")
    args.output.mkdir(parents=True, exist_ok=False)
    rows = [
        analyze(args.input / r["directory"], r, args.output, args.shaped)
        for r in report["rows"]
        if r["job"]["controller"] == "geometric"
    ]
    models = []
    for shaped, pole in ((False, 30.0), (True, 10.0)):
        for inertia in (0.025, 0.02):
            transition = GeometricHoverMap(inertia, pole, shaped).transition()
            discrete = np.linalg.eigvals(transition).astype(complex)
            # A reset coordinate has a zero eigenvalue, not a finite log pole.
            equivalent = np.log(discrete[np.abs(discrete) > 1e-12]) / 0.02
            models.append(
                dict(
                    shaped=shaped,
                    pole_rad_s=pole,
                    inertia=inertia,
                    spectral_radius=float(np.max(np.abs(discrete))),
                    equivalent_poles=[[float(p.real), float(p.imag)] for p in equivalent],
                )
            )
    result = dict(
        rows=rows,
        models=models,
        input_report_sha256=hashlib.sha256((args.input / "report.json").read_bytes()).hexdigest(),
        source_sha256=source_sha256(Path(__file__).resolve().parents[1]),
        payload_verification=verification,
    )
    (args.output / "diagnostic.json").write_bytes(canonical_json(result) + b"\n")
    print(json.dumps(rows, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
