"""Frozen true-state inner-loop evidence (ADR 0011); no estimator or position loop.

python -m experiments.attitude_control_validation --partition fixed --output /tmp/new.json
JSON stores the complete trial ledger and histories; generated evidence stays outside Git.
"""

import argparse
import hashlib
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, fields, replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from quadrotor_math.attitude_control import AttitudeControllerParameters, attitude_error_body
from quadrotor_math.attitude_simulation import (
    AttitudeControlSchedule,
    AttitudeSimulationResult,
    simulate_attitude_control,
)
from quadrotor_math.run_configuration import (
    RigidBodyInitialState,
    RigidBodyParameters,
    RotorParameters,
    WorldParameters,
)
from quadrotor_math.run_manifest import capture_software_provenance

PARTITIONS = ("smoke", "fixed", "development", "validation")
FIXED = (
    "hover",
    "roll_positive",
    "roll_negative",
    "pitch_positive",
    "pitch_negative",
    "yaw_positive",
    "yaw_negative",
    "coupled",
    "pulse",
    "saturation",
    "persistent",
)
REFINED = ("coupled", "pulse", "saturation")
IDENTITY = np.array([1.0, 0, 0, 0])


def canonical_json(value: object) -> bytes:
    """Finite deterministic encoding; arrays must already have explicit list form."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    return value


def source_sha256(root: Path) -> str:
    """Hash ordered names, lengths and bytes, including untracked execution sources."""
    digest = hashlib.sha256()
    for path in sorted(
        [
            *root.glob("src/quadrotor_math/*.py"),
            *root.glob("experiments/*.py"),
            root / "pyproject.toml",
            root / "uv.lock",
        ]
    ):
        data = path.read_bytes()
        digest.update(path.relative_to(root).as_posix().encode() + b"\0")
        digest.update(str(len(data)).encode() + b"\0" + data)
    return digest.hexdigest()


def baseline_parameters() -> AttitudeControllerParameters:
    """Illustrative SI parameters, not a hardware identification."""
    return AttitudeControllerParameters(
        np.array([3.0, 3, 2]),
        np.array([12.0, 12, 8]),
        np.array([2.0, 2, 1.5]),
        np.array([0.8, 0.8, 0.3]),
        np.pi / 2,
        np.diag([0.02, 0.025, 0.04]),
        RotorParameters(
            np.array([[0.15, 0.15, 0], [0.15, -0.15, 0], [-0.15, -0.15, 0], [-0.15, 0.15, 0]]),
            np.array([1.0, -1, 1, -1]),
            1e-5,
            2e-7,
            0.0,
            900.0,
            0.025,
        ),
    )


def planned_jobs(partition: str) -> list[dict[str, Any]]:
    """Stable identities and order; held-out seeds do not overlap development/smoke."""
    if partition not in PARTITIONS:
        raise ValueError("unknown attitude validation partition")
    if partition == "fixed":
        return [{"case": case, "seed": None, "refinement": 1} for case in FIXED] + [
            {"case": case, "seed": None, "refinement": 2} for case in REFINED
        ]
    seeds = {
        "smoke": range(10, 12),
        "development": range(1000, 1005),
        "validation": range(60000, 60030),
    }[partition]
    return [{"case": "random_recovery", "seed": seed, "refinement": 1} for seed in seeds]


def validation_protocol(partition: str) -> dict[str, Any]:
    """Complete protocol fingerprint excludes source, date and worker count."""
    return {
        "name": "baseline_attitude_control",
        "version": 1,
        "partition": partition,
        "jobs": planned_jobs(partition),
        "controller": _plain(asdict(baseline_parameters())),
        "truth": "matched nominal inertia/rotors; mass=1 kg; g=9.81 m/s^2; calm, zero drag",
        "plant_time_step_s": 0.0025,
        "control_period_s": 0.01,
        "duration_s": 0.04 if partition == "smoke" else 4.0,
        "initial": "p=v=0; equal actual motors sqrt(9.81/4e-5); default q=identity, omega=0",
        "collective_thrust_N": 9.81,
        "fixed_cases": {
            "signed_axis_steps": {"angle_deg": 20.0, "time_s": 0.5},
            "coupled": {"axis": [1.0, -2, 3], "angle_deg": 30.0, "omega_B": [0.25, -0.2, 0.15]},
            "pulse": {"interval_s": [1.0, 1.2], "moment_B_N_m": [0.12, -0.1, 0.06]},
            "saturation": {
                "axis": [1.0, 0, 0],
                "angle_deg": 30.0,
                "interval_s": [0.5, 1.5],
                "maximum_moment_B_N_m": [0.05, 0.05, 0.02],
            },
            "persistent": {"start_s": 0.5, "moment_B_N_m": [0.04, 0.0, 0.0]},
        },
        "randomness": {
            "generator": "PCG64",
            "SeedSequence_entropy": [0x41545443, 1, "seed"],
            "draw_order": [
                "axis=normalized standard_normal(3)",
                "angle=uniform(5,30) deg",
                "omega_B=uniform(-.3,.3,3) rad/s",
            ],
        },
        "targets": {
            "final_error_deg": 0.5,
            "final_rate_rad_s": 0.02,
            "settling_error_deg": 1.0,
            "settling_rate_rad_s": 0.05,
            "fixed_step_pulse_settling_s": 2.0,
            "final_moment_limit_free_s": 1.0,
            "domain_rad": float(np.pi / 2),
            "hover_state_absolute_tolerance": 1e-11,
            "persistent_offset_error_deg": 0.1,
            "refinement_attitude_deg": 0.02,
            "refinement_rate_rad_s": 0.002,
        },
        "semantics": (
            "Intervals left closed/right open; settling first grid epoch after last band "
            "violation after final event; equality passes. Metrics use complete histories. "
            "Persistent-torque diagnostic tests its nonzero predicted offset. Smoke only "
            "tests finite execution. Failures remain in ledger and suppress aggregate pass."
        ),
    }


def axis_quaternion(axis: NDArray[np.float64], angle: float) -> NDArray[np.float64]:
    return np.asarray(
        np.r_[np.cos(angle / 2), axis / np.linalg.norm(axis) * np.sin(angle / 2)],
        dtype=np.float64,
    )


def make_case(
    job: dict[str, Any], duration_s: float = 4.0
) -> tuple[RigidBodyInitialState, AttitudeControllerParameters, AttitudeControlSchedule]:
    """Construct deterministic inputs only; never inspect outputs to alter a case."""
    case, factor = job["case"], job["refinement"]
    if case not in (*FIXED, "random_recovery") or factor not in (1, 2):
        raise ValueError("unknown case or refinement")
    dt, stride = 0.0025 / factor, 4 * factor
    n, m = round(duration_s / dt), round(duration_s / 0.01)
    reference = np.tile(IDENTITY, (m, 1))
    disturbance = np.zeros((n, 3))
    q_WB, omega = IDENTITY.copy(), np.zeros(3)
    controller = baseline_parameters()
    control_time, plant_time = np.arange(m) * 0.01, np.arange(n) * dt
    if case.startswith(("roll_", "pitch_", "yaw_")):
        axis = np.eye(3)[{"roll": 0, "pitch": 1, "yaw": 2}[case.split("_")[0]]]
        sign = 1 if case.endswith("positive") else -1
        reference[control_time >= 0.5] = axis_quaternion(axis, sign * np.deg2rad(20))
    elif case == "coupled":
        q_WB = axis_quaternion(np.array([1.0, -2, 3]), np.deg2rad(30))
        omega = np.array([0.25, -0.2, 0.15])
    elif case == "pulse":
        disturbance[(plant_time >= 1) & (plant_time < 1.2)] = [0.12, -0.1, 0.06]
    elif case == "saturation":
        controller = replace(controller, maximum_moment_B=np.array([0.05, 0.05, 0.02]))
        reference[(control_time >= 0.5) & (control_time < 1.5)] = axis_quaternion(
            np.array([1.0, 0, 0]), np.deg2rad(30)
        )
    elif case == "persistent":
        disturbance[plant_time >= 0.5] = [0.04, 0, 0]
    elif case == "random_recovery":
        rng = np.random.Generator(
            np.random.PCG64(np.random.SeedSequence([0x41545443, 1, job["seed"]]))
        )
        axis = rng.standard_normal(3)
        q_WB = axis_quaternion(axis, np.deg2rad(rng.uniform(5, 30)))
        omega = rng.uniform(-0.3, 0.3, 3)
    return (
        RigidBodyInitialState(np.zeros(3), np.zeros(3), q_WB, omega),
        controller,
        AttitudeControlSchedule(dt, stride, reference, np.full(m, 9.81), disturbance),
    )


def attitude_errors(result: AttitudeSimulationResult) -> NDArray[np.float64]:
    return np.array(
        [
            attitude_error_body(q_WB, reference)
            for q_WB, reference in zip(result.q_WB, result.reference_q_WB, strict=True)
        ]
    )


def settling_time(
    time_s: NDArray[np.float64],
    error_rad: NDArray[np.float64],
    rate_rad_s: NDArray[np.float64],
    after_s: float,
) -> float | None:
    """First epoch after the last simultaneous-band violation, or None if unsettled."""
    selected = np.flatnonzero(time_s >= after_s)
    if not len(selected):
        return None
    outside = selected[(error_rad[selected] > np.deg2rad(1)) | (rate_rad_s[selected] > 0.05)]
    index = int(outside[-1] + 1) if len(outside) else int(selected[0])
    return None if index >= len(time_s) else float(time_s[index] - after_s)


def score_case(job: dict[str, Any], result: AttitudeSimulationResult) -> dict[str, Any]:
    """Descriptive finite-grid physical metrics and frozen case-specific acceptance."""
    errors = attitude_errors(result)
    angles, rates = np.linalg.norm(errors, axis=1), np.linalg.norm(result.omega_B, axis=1)
    time = result.time_s
    widths = np.diff(np.r_[result.control_time_s, time[-1]])
    case = job["case"]
    last_event = (
        1.2
        if case == "pulse"
        else (
            1.5
            if case == "saturation"
            else (
                0.5 if case == "persistent" or case.startswith(("roll_", "pitch_", "yaw_")) else 0.0
            )
        )
    )
    settled = settling_time(time, angles, rates, last_event)
    final_second = result.control_time_s >= time[-1] - 1
    last_moment_limited = bool(np.any(result.limit_flags[final_second, 1:]))
    metrics: dict[str, Any] = {
        "final_error_deg": float(np.rad2deg(angles[-1])),
        "peak_error_deg": float(np.rad2deg(np.max(angles))),
        "rms_error_deg": float(np.rad2deg(np.sqrt(np.trapezoid(angles**2, time) / time[-1]))),
        "final_rate_rad_s": float(rates[-1]),
        "peak_rate_rad_s": float(np.max(rates)),
        "settling_after_event_s": settled,
        "last_event_s": last_event,
        "limit_duration_s": (result.limit_flags.T @ widths).tolist(),
        "rotor_bound_duration_s": float(
            np.dot(
                np.diff(time),
                np.any(
                    (result.actual_rotor_omega[:-1] <= 1e-10)
                    | (result.actual_rotor_omega[:-1] >= 900 - 1e-10),
                    axis=1,
                ),
            )
        ),
        "squared_actual_moment_integral_N2_m2_s": float(
            np.trapezoid(np.sum(result.actual_moment_B**2, axis=1), time)
        ),
        "final_position_displacement_m": float(np.linalg.norm(result.position_W[-1])),
        "final_moment_limiting": last_moment_limited,
    }
    conditions = {
        "local_domain": bool(np.all(angles <= np.pi / 2)),
        "final_error": angles[-1] <= np.deg2rad(0.5),
        "final_rate": rates[-1] <= 0.02,
        "no_final_moment_limiting": not last_moment_limited,
    }
    if case.startswith(("roll_", "pitch_", "yaw_")):
        axis = {"roll": 0, "pitch": 1, "yaw": 2}[case.split("_")[0]]
        sign = 1 if case.endswith("positive") else -1
        response = (
            np.array([attitude_error_body(IDENTITY, q_WB)[axis] for q_WB in result.q_WB])
            * sign
            / np.deg2rad(20)
        )
        selected = time >= 0.5
        crossings = [np.flatnonzero(selected & (response >= bound)) for bound in (0.1, 0.9)]
        metrics["rise_10_90_s"] = (
            float(time[crossings[1][0]] - time[crossings[0][0]])
            if all(len(value) for value in crossings)
            else None
        )
        metrics["overshoot_percent"] = float(max(0.0, np.max(response[selected]) - 1) * 100)
        conditions["settling"] = settled is not None and settled <= 2.0
    elif case == "pulse":
        conditions["settling"] = settled is not None and settled <= 2.0
    elif case == "saturation":
        conditions["exercises_moment_limit"] = bool(np.any(result.limit_flags[:, 1]))
    elif case == "hover":
        conditions["equilibrium"] = bool(
            all(
                np.max(np.abs(value)) <= 1e-11
                for value in (
                    result.position_W,
                    result.velocity_W,
                    result.omega_B,
                    result.q_WB - IDENTITY,
                )
            )
        )
    elif case == "persistent":
        expected = 0.04 / (0.02 * 12 * 3)
        actual = attitude_error_body(IDENTITY, result.q_WB[-1])[0]
        metrics["predicted_roll_offset_deg"] = float(np.rad2deg(expected))
        metrics["measured_roll_offset_deg"] = float(np.rad2deg(actual))
        del conditions["final_error"]
        conditions["expected_nonzero_offset"] = abs(actual - expected) <= np.deg2rad(0.1)
    if time[-1] < 1:
        conditions = {"smoke_finite_execution_only": True}
    metrics["conditions"] = {name: bool(value) for name, value in conditions.items()}
    metrics["passed"] = all(conditions.values())
    return metrics


def _execute(job_and_duration: tuple[dict[str, Any], float]) -> dict[str, Any]:
    job, duration = job_and_duration
    try:
        state, model, schedule = make_case(job, duration)
        result = simulate_attitude_control(
            state,
            np.full(4, np.sqrt(9.81 / 4e-5)),
            RigidBodyParameters(1.0, model.nominal_inertia_B),
            model.nominal_rotors,
            WorldParameters(9.81),
            model,
            schedule,
        )
        return {
            "job": job,
            "status": "ok",
            "metrics": score_case(job, result),
            "history": _plain(asdict(result)),
        }
    except Exception as error:
        # Keep a failed trial, including its identity; never count partial successes as complete.
        return {
            "job": job,
            "status": "failed",
            "error_type": type(error).__name__,
            "error": str(error),
        }


def _result(history: dict[str, Any]) -> AttitudeSimulationResult:
    names = {field.name for field in fields(AttitudeSimulationResult)}
    if set(history) != names:
        raise ValueError("unexpected history fields")
    return AttitudeSimulationResult(**{name: np.asarray(history[name]) for name in names})


def summarize(protocol: dict[str, Any], rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Require the exact ledger, recompute metrics and validate protocol-aligned clocks."""
    if [row["job"] for row in rows] != protocol["jobs"]:
        raise ValueError("missing, duplicate or reordered trial identity")
    failures = 0
    results: dict[str, AttitudeSimulationResult] = {}
    for row in rows:
        if row["status"] == "failed":
            if not isinstance(row.get("error"), str) or not isinstance(row.get("error_type"), str):
                raise ValueError("failed trial requires a diagnostic")
            failures += 1
            continue
        if row["status"] != "ok":
            raise ValueError("unknown trial status")
        result = _result(row["history"])
        state, _, schedule = make_case(row["job"], protocol["duration_s"])
        n, stride = len(schedule.disturbance_moment_B), schedule.controller_stride
        expected_time = np.arange(n + 1) * schedule.time_step_s
        if (
            not np.array_equal(result.time_s, expected_time)
            or not np.array_equal(result.control_time_s, expected_time[:-1:stride])
            or not np.array_equal(
                result.reference_q_WB,
                schedule.reference_q_WB[
                    np.minimum(np.arange(n + 1) // stride, len(schedule.reference_q_WB) - 1)
                ],
            )
            or not np.array_equal(result.q_WB[0], state.q_WB)
            or not np.array_equal(result.omega_B[0], state.omega_B)
        ):
            raise ValueError("history does not match declared case/clock")
        computed = score_case(row["job"], result)
        if canonical_json(computed) != canonical_json(row["metrics"]):
            raise ValueError("stored metrics do not match histories")
        results[f"{row['job']['case']}/{row['job']['refinement']}"] = result
    refinement: dict[str, Any] = {}
    if protocol["partition"] == "fixed":
        for case in REFINED:
            if f"{case}/1" not in results or f"{case}/2" not in results:
                refinement[case] = {"complete": False, "passed": False}
                continue
            coarse, fine = results[f"{case}/1"], results[f"{case}/2"]
            angle = float(
                np.rad2deg(np.linalg.norm(attitude_error_body(coarse.q_WB[-1], fine.q_WB[-1])))
            )
            rate = float(np.linalg.norm(coarse.omega_B[-1] - fine.omega_B[-1]))
            refinement[case] = {
                "complete": True,
                "attitude_difference_deg": angle,
                "rate_difference_rad_s": rate,
                "passed": angle < 0.02 and rate < 0.002,
            }
    passed = (
        failures == 0
        and all(row["metrics"]["passed"] for row in rows if row["status"] == "ok")
        and all(value["passed"] for value in refinement.values())
    )
    return {
        "planned": len(rows),
        "numerical_failures": failures,
        "acceptance_failures": sum(
            not row["metrics"]["passed"] for row in rows if row["status"] == "ok"
        ),
        "refinement": refinement,
        "passed": passed,
    }


def run_validation(partition: str, workers: int = 1) -> dict[str, Any]:
    if isinstance(workers, bool) or not isinstance(workers, int) or workers < 1:
        raise ValueError("workers must be a positive integer")
    protocol = validation_protocol(partition)
    jobs = [(job, protocol["duration_s"]) for job in protocol["jobs"]]
    if workers == 1:
        rows = [_execute(job) for job in jobs]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(_execute, jobs))
    return {
        "protocol": protocol,
        "protocol_sha256": hashlib.sha256(canonical_json(protocol)).hexdigest(),
        "trials": rows,
        "summary": summarize(protocol, rows),
    }


def validate_report(report: dict[str, Any]) -> dict[str, Any]:
    """Reject incomplete, modified or nonfinite evidence before plotting."""
    canonical_json(report)
    protocol = validation_protocol(report["protocol"]["partition"])
    if (
        canonical_json(protocol) != canonical_json(report["protocol"])
        or hashlib.sha256(canonical_json(protocol)).hexdigest() != report["protocol_sha256"]
    ):
        raise ValueError("protocol does not match frozen definition")
    summary = summarize(protocol, report["trials"])
    if canonical_json(summary) != canonical_json(report["summary"]):
        raise ValueError("stored summary does not match trial ledger")
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", choices=PARTITIONS, default="smoke")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    root = Path(__file__).resolve().parents[1]
    provenance, source_hash = asdict(capture_software_provenance(root)), source_sha256(root)
    protocol = validation_protocol(args.partition)
    print(
        "Frozen protocol SHA256: " + hashlib.sha256(canonical_json(protocol)).hexdigest(),
        file=sys.stderr,
        flush=True,
    )
    report = run_validation(args.partition, args.workers)
    if source_hash != source_sha256(root):
        raise RuntimeError("execution sources changed during the campaign")
    report.update(
        {
            "software_provenance": provenance,
            "source_sha256": source_hash,
            "generated_at_utc": datetime.now(UTC).isoformat(),
            "workers": args.workers,
        }
    )
    with args.output.open("xb") as stream:
        stream.write(canonical_json(report) + b"\n")
    print(json.dumps(report["summary"], sort_keys=True), file=sys.stderr)
    return int(not report["summary"]["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
