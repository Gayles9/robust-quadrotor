"""Frozen Week 8 true-state mission evidence; complete histories, no hidden trials.

CLI: python -m experiments.position_control_validation --partition fixed --output DIR
DIR must be new. report.json is published last, binding separate compressed NPZ
histories by exact-byte SHA-256. No sensor-run schema is changed (ADR 0012).
"""

import argparse
import hashlib
import io
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, fields
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import (
    axis_quaternion,
    baseline_parameters,
    canonical_json,
    source_sha256,
)
from quadrotor_math.attitude_control import AttitudeControllerParameters, attitude_error_body
from quadrotor_math.mission_simulation import MissionNumerics, MissionResult, simulate_mission
from quadrotor_math.missions import (
    MissionPhase,
    MissionPlan,
    MissionSafetyLimits,
    MissionSegment,
    MissionState,
    ReferenceKind,
    advance_mission,
    mission_guard_reason,
    mission_reference,
)
from quadrotor_math.position_control import PositionControllerParameters, compute_position_control
from quadrotor_math.run_configuration import (
    RigidBodyInitialState,
    RigidBodyParameters,
    WorldParameters,
)
from quadrotor_math.run_manifest import capture_software_provenance

PARTITIONS = ("smoke", "fixed", "development", "validation")


def plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def position_parameters() -> PositionControllerParameters:
    return PositionControllerParameters(
        np.array([1.0, 1, 2.25]),
        np.array([1.8, 1.8, 3]),
        np.full(3, 2.0),
        1.0,
        9.81,
        float(np.deg2rad(20)),
        2.0,
        18.0,
    )


def safety_limits() -> MissionSafetyLimits:
    return MissionSafetyLimits(
        np.array([-3.0, -3, -3]), np.array([3.0, 3, 0.5]), float(np.deg2rad(35))
    )


def baseline_mission(case: str) -> MissionPlan:
    """Explicit takeoff, hover/waypoint/step and landing reference primitives."""
    if case not in ("smoke", "hover", "square", "vertical_step", "mild_wind"):
        raise ValueError("unknown mission case")
    point = np.zeros(3)
    segments: list[MissionSegment] = []

    def add(phase: MissionPhase, duration: float, target: list[float], kind: ReferenceKind) -> None:
        nonlocal point
        endpoint = np.array(target, dtype=np.float64)
        segments.append(MissionSegment(phase, duration, point, endpoint, kind))
        point = endpoint

    if case == "smoke":
        for phase in (
            MissionPhase.INITIALIZE,
            MissionPhase.TAKEOFF,
            MissionPhase.TRACK,
            MissionPhase.LAND,
        ):
            add(phase, 0.01, [0, 0, 0], ReferenceKind.HOLD)
        return MissionPlan(tuple(segments), 0.0, 0.08, 0.08, 0.01, 0.04)
    add(MissionPhase.INITIALIZE, 1.0, [0, 0, 0], ReferenceKind.HOLD)
    add(MissionPhase.TAKEOFF, 4.0, [0, 0, -1], ReferenceKind.SMOOTH)
    add(MissionPhase.TRACK, 60.0 if case == "hover" else 2.0, [0, 0, -1], ReferenceKind.HOLD)
    if case in ("square", "mild_wind"):
        for target in ([1.0, 0, -1], [1.0, 1, -1], [0.0, 1, -1], [0.0, 0, -1]):
            add(MissionPhase.TRACK, 6.0, target, ReferenceKind.SMOOTH)
            add(MissionPhase.TRACK, 1.0, target, ReferenceKind.HOLD)
    elif case == "vertical_step":
        add(MissionPhase.TRACK, 8.0, [0, 0, -1.5], ReferenceKind.STEP)
    add(MissionPhase.LAND, 4.0, [0, 0, 0], ReferenceKind.SMOOTH)
    return MissionPlan(tuple(segments), 0.0, 0.08, 0.08, 0.5, 8.0)


def planned_jobs(partition: str) -> list[dict[str, Any]]:
    if partition not in PARTITIONS:
        raise ValueError("unknown position validation partition")
    if partition == "fixed":
        return [
            {"case": case, "seed": None, "refinement": 1}
            for case in ("hover", "square", "vertical_step", "mild_wind")
        ] + [{"case": "square", "seed": None, "refinement": 2}]
    seeds = {
        "smoke": range(20, 22),
        "development": range(2000, 2003),
        "validation": range(70000, 70010),
    }[partition]
    return [
        {"case": "smoke" if partition == "smoke" else "square", "seed": seed, "refinement": 1}
        for seed in seeds
    ]


def make_case(
    job: dict[str, Any],
) -> tuple[
    RigidBodyInitialState,
    RigidBodyParameters,
    WorldParameters,
    PositionControllerParameters,
    AttitudeControllerParameters,
    MissionPlan,
    MissionSafetyLimits,
    MissionNumerics,
]:
    case, factor, seed = job["case"], job["refinement"], job["seed"]
    if type(factor) is not int or factor not in (1, 2):
        raise ValueError("refinement must be one or two")
    plan = baseline_mission(case)
    attitude = baseline_parameters()
    position, velocity, q_WB, omega = (
        np.zeros(3),
        np.zeros(3),
        np.array([1.0, 0, 0, 0]),
        np.zeros(3),
    )
    if seed is not None:
        if type(seed) is not int or seed < 0:
            raise ValueError("seed must be a nonnegative integer or None")
        rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([0x504F5343, 1, seed])))
        position, velocity = rng.uniform(-0.02, 0.02, 3), rng.uniform(-0.02, 0.02, 3)
        rotation = np.deg2rad(rng.uniform(-2, 2, 3))
        angle = float(np.linalg.norm(rotation))
        q_WB = axis_quaternion(rotation, angle) if angle else q_WB
        omega = rng.uniform(-0.02, 0.02, 3)
    drag = np.array([0.1, 0.1, 0.15]) if case == "mild_wind" else np.zeros(3)
    wind = np.array([0.5, -0.3, 0]) if case == "mild_wind" else np.zeros(3)
    return (
        RigidBodyInitialState(position, velocity, q_WB, omega),
        RigidBodyParameters(1.0, attitude.nominal_inertia_B, drag),
        WorldParameters(9.81, wind),
        position_parameters(),
        attitude,
        plan,
        safety_limits(),
        MissionNumerics(0.0025 / factor, 4 * factor, 8 * factor, 100000),
    )


def validation_protocol(partition: str) -> dict[str, Any]:
    return {
        "name": "true_state_position_missions",
        "version": 1,
        "partition": partition,
        "jobs": planned_jobs(partition),
        "position_controller": plain(asdict(position_parameters())),
        "attitude_controller": plain(asdict(baseline_parameters())),
        "safety": plain(asdict(safety_limits())),
        "plans": {
            case: plain(asdict(baseline_mission(case)))
            for case in ("smoke", "hover", "square", "vertical_step", "mild_wind")
        },
        "truth": {
            "mass_kg": 1.0,
            "gravity_m_s2": 9.81,
            "inertia_rotors": "matched inner nominal",
            "wind_W_m_s": [0.0, 0, 0],
            "drag_B_kg_m": [0.0, 0, 0],
            "mild_wind_W_m_s": [0.5, -0.3, 0],
            "mild_wind_drag_B_kg_m": [0.1, 0.1, 0.15],
        },
        "clock": {
            "plant_dt_s": 0.0025,
            "attitude_period_s": 0.01,
            "position_period_s": 0.02,
            "maximum_steps": 100000,
        },
        "initial": {
            "p_W": [0.0, 0, 0],
            "v_W": [0.0, 0, 0],
            "q_WB": [1.0, 0, 0, 0],
            "omega_B": [0.0, 0, 0],
            "actual_rotor_omega": "sqrt(9.81/4e-5), each",
        },
        "randomness": {
            "generator": "PCG64",
            "SeedSequence_entropy": [0x504F5343, 1, "seed"],
            "draw_order": [
                "p_W uniform(-.02,.02,3) m",
                "v_W uniform(-.02,.02,3) m/s",
                "rotation vector uniform(-2,2,3) degrees",
                "omega_B uniform(-.02,.02,3) rad/s",
            ],
        },
        "targets": {
            "position_rmse_m": 0.15,
            "hover_peak_error_m": 0.08,
            "step_settling_s": 6.0,
            "longest_actuator_limiting_s": 0.5,
            "final_limit_free_s": 1.0,
            "refinement_position_m": 0.005,
            "refinement_attitude_deg": 0.05,
        },
        "semantics": (
            "Complete full-grid histories; sampled guards only. Actuator limiting is moment "
            "clipping or allocation scaling or any commanded rotor at a speed bound. Smoke "
            "checks execution, not performance. All failures retained. Landing is virtual; "
            "no contact, disarming, sensors or ESKF."
        ),
    }


def longest_active_duration(flags: NDArray[np.bool_], widths: NDArray[np.float64]) -> float:
    current = longest = 0.0
    for active, width in zip(flags, widths, strict=True):
        current = current + float(width) if active else 0.0
        longest = max(longest, current)
    return longest


def score_case(job: dict[str, Any], result: MissionResult) -> dict[str, Any]:
    time = result.time_s
    position_error = np.linalg.norm(result.position_W - result.reference_position_W, axis=1)
    velocity_error = np.linalg.norm(result.velocity_W - result.reference_velocity_W, axis=1)
    complete = result.phase[-1] == MissionPhase.COMPLETE
    c = len(result.control_time_s)
    widths = np.diff(np.r_[result.control_time_s, time[-1]]) if c else np.zeros(0)
    outer_widths = np.diff(np.r_[result.position_control_time_s, time[-1]]) if c else np.zeros(0)
    attitude = np.zeros(len(time))
    if c:
        held = np.searchsorted(result.control_time_s, time, side="right") - 1
        attitude = np.array(
            [
                np.linalg.norm(attitude_error_body(q_WB, result.q_reference_WB[j]))
                for q_WB, j in zip(result.q_WB, held, strict=True)
            ]
        )
    bound = np.any(
        (result.commanded_rotor_omega <= 1e-10) | (result.commanded_rotor_omega >= 900 - 1e-10),
        axis=1,
    )
    limited = np.any(result.inner_limit_flags[:, 1:], axis=1) | bound
    longest = longest_active_duration(limited, widths)
    final_limiting = bool(np.any(limited & (result.control_time_s + widths > time[-1] - 1)))

    def rms(values: NDArray[np.float64]) -> float:
        return (
            float(np.sqrt(np.trapezoid(values**2, time) / time[-1]))
            if time[-1]
            else float(values[0])
        )

    plan = baseline_mission(job["case"])
    endpoints: list[dict[str, float]] = []
    elapsed = 0.0
    for segment in plan.segments:
        elapsed += segment.duration_s
        if segment.phase == MissionPhase.TRACK:
            index = int(np.searchsorted(time, elapsed))
            if index < len(time):
                endpoints.append(
                    {
                        "time_s": elapsed,
                        "error_m": float(
                            np.linalg.norm(result.position_W[index] - segment.end_position_W)
                        ),
                    }
                )
    metrics: dict[str, Any] = {
        "completed": bool(complete),
        "abort_reason": result.abort_reason,
        "duration_s": float(time[-1]),
        "position_rmse_m": rms(position_error),
        "peak_position_error_m": float(np.max(position_error)),
        "final_position_error_m": float(position_error[-1]),
        "velocity_rmse_m_s": rms(velocity_error),
        "final_speed_m_s": float(np.linalg.norm(result.velocity_W[-1])),
        "attitude_rmse_deg": float(np.rad2deg(rms(attitude))),
        "peak_attitude_error_deg": float(np.rad2deg(np.max(attitude))),
        "peak_actual_tilt_deg": float(
            np.rad2deg(
                np.max(np.arccos(np.clip(1 - 2 * np.sum(result.q_WB[:, 1:3] ** 2, axis=1), -1, 1)))
            )
        ),
        "inner_limit_duration_s": (result.inner_limit_flags.T @ widths).tolist(),
        "outer_limit_duration_s": (result.outer_limit_flags.T @ outer_widths).tolist(),
        "longest_actuator_limiting_s": longest,
        "final_actuator_limiting": final_limiting,
        "squared_command_thrust_integral_N2_s": float(np.dot(result.collective_thrust**2, widths)),
        "squared_actual_moment_integral_N2_m2_s": float(
            np.trapezoid(np.sum(result.actual_moment_B**2, axis=1), time)
        ),
        "waypoint_endpoint_errors": endpoints,
        "transitions": [
            {"time_s": float(time[i]), "phase": MissionPhase(int(result.phase[i])).name}
            for i in range(len(time))
            if i == 0 or result.phase[i] != result.phase[i - 1]
        ],
    }
    conditions = {
        "completed": bool(complete),
        "position_rmse": metrics["position_rmse_m"] < 0.15,
        "no_sustained_actuator_limiting": longest <= 0.5,
        "final_limit_free": not final_limiting,
    }
    if job["case"] == "hover":
        selected = (time >= 5) & (time <= 65)
        peak = float(np.max(position_error[selected])) if np.any(selected) else None
        metrics["hover_segment_peak_error_m"] = peak
        conditions["60_second_hover"] = bool(time[-1] >= 65 and peak is not None and peak <= 0.08)
    if job["case"] == "vertical_step":
        selected_indices = np.flatnonzero((time >= 7) & (time < 15))
        outside = selected_indices[
            (position_error[selected_indices] > 0.08) | (velocity_error[selected_indices] > 0.08)
        ]
        settled_index = (
            int(outside[-1] + 1)
            if len(outside)
            else (int(selected_indices[0]) if len(selected_indices) else len(time))
        )
        settling = (
            float(time[settled_index] - 7)
            if len(selected_indices) and settled_index <= selected_indices[-1]
            else None
        )
        metrics["vertical_step_settling_s"] = settling
        conditions["step_settling"] = settling is not None and settling <= 6
    if job["case"] == "smoke":
        conditions = {"smoke_execution_complete": bool(complete)}
    metrics["conditions"] = conditions
    metrics["passed"] = all(conditions.values())
    return metrics


def result_from_history(history: dict[str, Any]) -> MissionResult:
    names = {field.name for field in fields(MissionResult)}
    if set(history) != names:
        raise ValueError("unexpected history fields")
    return MissionResult(**history)


def _execute(job: dict[str, Any]) -> dict[str, Any]:
    try:
        state, body, world, outer, inner, plan, safety, numerics = make_case(job)
        result = simulate_mission(
            state,
            np.full(4, np.sqrt(9.81 / 4e-5)),
            body,
            inner.nominal_rotors,
            world,
            outer,
            inner,
            plan,
            safety,
            numerics,
        )
        return {
            "job": job,
            "status": "ok",
            "metrics": score_case(job, result),
            "history": {field.name: getattr(result, field.name) for field in fields(result)},
        }
    except Exception as error:
        return {
            "job": job,
            "status": "failed",
            "error_type": type(error).__name__,
            "error": str(error),
        }


def _validate_history(job: dict[str, Any], result: MissionResult) -> None:
    initial, _, _, outer, inner, plan, safety, numerics = make_case(job)
    n = len(result.time_s)
    expected = np.arange(n) * numerics.time_step_s
    if (
        not np.array_equal(result.time_s, expected)
        or not np.array_equal(result.control_time_s, expected[: -1 : numerics.attitude_stride])
        or not np.array_equal(
            result.position_control_time_s, expected[: -1 : numerics.position_stride]
        )
    ):
        raise ValueError("history clocks do not match protocol")
    for name in ("position_W", "velocity_W", "q_WB", "omega_B"):
        if not np.array_equal(getattr(result, name)[0], getattr(initial, name)):
            raise ValueError("history initial state does not match protocol")
    if not np.array_equal(result.actual_rotor_omega[0], np.full(4, np.sqrt(9.81 / 4e-5))):
        raise ValueError("initial motors do not match protocol")
    if (
        result.time_s[-1]
        > plan.reference_duration_s + plan.completion_timeout_s + numerics.time_step_s
    ):
        raise ValueError("history exceeds protocol horizon")
    supervisor = MissionState()
    if (
        np.any(result.actual_rotor_omega > 900)
        or np.any(result.commanded_rotor_omega > 900)
        or np.any(result.collective_thrust < outer.minimum_collective_thrust)
        or np.any(result.collective_thrust > outer.maximum_collective_thrust)
    ):
        raise ValueError("history violates protocol actuator bounds")
    for index, time in enumerate(result.time_s):
        reference, _ = mission_reference(plan, float(time))
        for name in ("position_W", "velocity_W", "acceleration_W"):
            if not np.array_equal(
                getattr(result, "reference_" + name)[index], getattr(reference, name)
            ):
                raise ValueError("history references do not match protocol")
        reason = mission_guard_reason(result.position_W[index], result.q_WB[index], safety)
        supervisor = advance_mission(
            supervisor,
            plan,
            float(time),
            result.position_W[index],
            result.velocity_W[index],
            reason,
        )
        if supervisor.phase not in (MissionPhase.ABORT, MissionPhase.COMPLETE):
            if index % numerics.position_stride == 0:
                held = compute_position_control(
                    result.position_W[index], result.velocity_W[index], reference, outer
                )
            if (
                np.linalg.norm(attitude_error_body(result.q_WB[index], held.q_reference_WB))
                > inner.maximum_attitude_error_rad
            ):
                supervisor = MissionState(MissionPhase.ABORT, float(time), reason="attitude_domain")
            elif index < n - 1 and index % numerics.attitude_stride == 0:
                j = index // numerics.attitude_stride
                if (
                    not np.array_equal(result.q_reference_WB[j], held.q_reference_WB)
                    or result.collective_thrust[j] != held.collective_thrust
                ):
                    raise ValueError("stored held attitude/thrust does not match outer law")
            if (
                supervisor.phase != MissionPhase.ABORT
                and index < n - 1
                and index % numerics.position_stride == 0
            ):
                j = index // numerics.position_stride
                expected_flags = [held.acceleration_limited, held.tilt_limited, held.thrust_limited]
                if (
                    not np.array_equal(
                        result.requested_acceleration_W[j], held.requested_acceleration_W
                    )
                    or not np.array_equal(
                        result.feasible_acceleration_W[j], held.feasible_acceleration_W
                    )
                    or not np.array_equal(result.outer_limit_flags[j], expected_flags)
                ):
                    raise ValueError("stored outer diagnostics do not match outer law")
        if result.phase[index] != supervisor.phase or (
            supervisor.phase in (MissionPhase.COMPLETE, MissionPhase.ABORT) and index != n - 1
        ):
            raise ValueError("history mission phases do not match protocol")
    if supervisor.reason != result.abort_reason:
        raise ValueError("history abort reason does not match supervisor")


def summarize(protocol: dict[str, Any], rows: list[dict[str, Any]]) -> dict[str, Any]:
    if [row["job"] for row in rows] != protocol["jobs"]:
        raise ValueError("missing, duplicate or reordered trial identity")
    results: dict[str, MissionResult] = {}
    failures = 0
    for row in rows:
        if row["status"] == "failed":
            if not isinstance(row.get("error"), str) or not isinstance(row.get("error_type"), str):
                raise ValueError("failed trial needs a diagnostic")
            failures += 1
            continue
        if row["status"] != "ok":
            raise ValueError("unknown trial status")
        result = result_from_history(row["history"])
        _validate_history(row["job"], result)
        if canonical_json(score_case(row["job"], result)) != canonical_json(row["metrics"]):
            raise ValueError("stored metrics do not match histories")
        results[f"{row['job']['case']}/{row['job']['refinement']}"] = result
    refinement: dict[str, Any] = {}
    if protocol["partition"] == "fixed":
        if "square/1" in results and "square/2" in results:
            coarse, fine = results["square/1"], results["square/2"]
            count = min(len(coarse.time_s), len(fine.time_s[::2]))
            position = float(
                np.max(
                    np.linalg.norm(
                        coarse.position_W[:count] - fine.position_W[: 2 * count : 2], axis=1
                    )
                )
            )
            angle = float(
                np.rad2deg(
                    max(
                        np.linalg.norm(attitude_error_body(a, b))
                        for a, b in zip(
                            coarse.q_WB[:count], fine.q_WB[: 2 * count : 2], strict=True
                        )
                    )
                )
            )
            refinement = {
                "complete": True,
                "maximum_position_difference_m": position,
                "maximum_attitude_difference_deg": angle,
                "passed": position < 0.005 and angle < 0.05,
            }
        else:
            refinement = {"complete": False, "passed": False}
    acceptance_failures = sum(not row["metrics"]["passed"] for row in rows if row["status"] == "ok")
    return {
        "planned": len(rows),
        "numerical_failures": failures,
        "acceptance_failures": acceptance_failures,
        "refinement": refinement,
        "passed": not (failures or acceptance_failures)
        and (not refinement or refinement["passed"]),
    }


def run_validation(partition: str, workers: int = 1) -> dict[str, Any]:
    if type(workers) is not int or workers < 1:
        raise ValueError("workers must be a positive integer")
    protocol = validation_protocol(partition)
    if workers == 1:
        rows = [_execute(job) for job in protocol["jobs"]]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(_execute, protocol["jobs"]))
    return {
        "protocol": protocol,
        "protocol_sha256": hashlib.sha256(canonical_json(protocol)).hexdigest(),
        "trials": rows,
        "summary": summarize(protocol, rows),
    }


def validate_report(report: dict[str, Any]) -> dict[str, Any]:
    protocol = validation_protocol(report["protocol"]["partition"])
    if (
        canonical_json(report["protocol"]) != canonical_json(protocol)
        or report["protocol_sha256"] != hashlib.sha256(canonical_json(protocol)).hexdigest()
    ):
        raise ValueError("protocol does not match frozen definition")
    if canonical_json(report["summary"]) != canonical_json(summarize(protocol, report["trials"])):
        raise ValueError("stored summary does not match trial ledger")
    return report


def save_report(report: dict[str, Any], output: Path) -> None:
    """New directory only; NPZ byte digests; report.json last is the completion marker.

    No crash-durability/authenticity claim; an interrupted directory without its
    report is incomplete. Existing destinations are never repaired/overwritten.
    """
    validate_report(report)
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    for index, row in enumerate(report["trials"]):
        saved = {key: value for key, value in row.items() if key != "history"}
        if row["status"] == "ok":
            history = row["history"]
            path = output / f"trial-{index:03d}.npz"
            with path.open("xb") as stream:
                np.savez_compressed(
                    stream,
                    **{name: value for name, value in history.items() if name != "abort_reason"},
                )
            saved.update(
                {
                    "history_file": path.name,
                    "history_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "abort_reason": history["abort_reason"],
                }
            )
        rows.append(saved)
    saved_report = {**report, "format_version": 1, "trials": rows}
    with (output / "report.json").open("xb") as stream:
        stream.write(canonical_json(saved_report) + b"\n")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def load_report(directory: Path) -> dict[str, Any]:
    """Check byte binding before allow_pickle=False NPZ decoding and full validation."""
    report = json.loads((directory / "report.json").read_bytes(), object_pairs_hook=_unique_object)
    canonical_json(report)
    if report.get("format_version") != 1 or type(report["format_version"]) is not int:
        raise ValueError("unsupported mission evidence format")
    for index, row in enumerate(report["trials"]):
        if row["status"] != "ok":
            continue
        name = f"trial-{index:03d}.npz"
        if row["history_file"] != name:
            raise ValueError("unexpected history filename")
        path = directory / name
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() != row["history_sha256"]:
            raise ValueError("history byte digest mismatch")
        with np.load(io.BytesIO(data), allow_pickle=False) as archive:
            if len(archive.files) != len(set(archive.files)):
                raise ValueError("duplicate NPZ array")
            row["history"] = {name: archive[name] for name in archive.files}
        row["history"]["abort_reason"] = row["abort_reason"]
    return validate_report(report)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", choices=PARTITIONS, default="smoke")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    root = Path(__file__).resolve().parents[1]
    provenance, source = asdict(capture_software_provenance(root)), source_sha256(root)
    print(
        "Protocol SHA256: "
        + hashlib.sha256(canonical_json(validation_protocol(args.partition))).hexdigest(),
        file=sys.stderr,
        flush=True,
    )
    report = run_validation(args.partition, args.workers)
    if source != source_sha256(root):
        raise RuntimeError("execution source changed during campaign")
    report.update(
        {
            "software_provenance": provenance,
            "source_sha256": source,
            "workers": args.workers,
            "generated_at_utc": datetime.now(UTC).isoformat(),
        }
    )
    save_report(report, args.output)
    print(json.dumps(report["summary"], sort_keys=True), file=sys.stderr)
    return int(not report["summary"]["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
