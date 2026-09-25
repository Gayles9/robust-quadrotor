"""Frozen ADR 0016 polynomial mission evidence; retain every declared case.

python -m experiments.trajectory_mission_validation --output NEW_DIR --workers 3
Nominal reference bounds and closed-loop evidence have different meanings.
"""

import argparse
import hashlib
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, fields, replace
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.position_control_validation import make_case, plain, safety_limits
from quadrotor_math.minimum_snap import minimum_snap_trajectory
from quadrotor_math.mission_simulation import MissionResult, simulate_mission
from quadrotor_math.missions import (
    MinimumSnapMissionSegment,
    MissionPlan,
    MissionSegment,
)
from quadrotor_math.missions import (
    MissionPhase as P,
)
from quadrotor_math.missions import (
    ReferenceKind as K,
)
from quadrotor_math.trajectory_feasibility import (
    TrajectoryLimits,
    check_trajectory_feasibility,
    retime_trajectory,
)

CASES = ("nominal", "refined", "offset_positive", "offset_negative", "mild_wind")
TARGETS = {
    "position_rmse_m": 0.15,
    "position_peak_m": 0.25,
    "final_position_error_m": 0.08,
    "final_speed_m_s": 0.08,
    "any_limiting": False,
    "refinement_position_m": 0.005,
    "refinement_attitude_deg": 0.05,
}


def planning_limits() -> TrajectoryLimits:
    safety = safety_limits()
    return TrajectoryLimits(
        safety.minimum_position_W,
        safety.maximum_position_W,
        0.6,
        np.full(3, 0.6),
        1.0,
        9.81,
        7.0,
        13.0,
        float(np.deg2rad(10)),
        0.4,
    )


def build_mission() -> tuple[MissionPlan, dict[str, Any]]:
    """Construct the frozen path and choose timing from reference bounds alone."""
    points = np.array([[0.0, 0, -1], [1.0, 0, -1.2], [1.0, 1, -0.8], [0.0, 1, -1], [0.0, 0, -1]])
    limits = planning_limits()
    initial = minimum_snap_trajectory(points, np.ones(4))
    timed = retime_trajectory(initial, limits)
    if timed.trajectory is None:
        raise RuntimeError("frozen reference timing failed: " + str(timed.reason))
    zero = np.zeros(3)
    takeoff = minimum_snap_trajectory(np.array([zero, points[0]]), np.array([4.0]))
    landing = minimum_snap_trajectory(np.array([points[-1], zero]), np.array([4.0]))
    reports = {
        "takeoff": check_trajectory_feasibility(takeoff, limits),
        "track": timed.attempts[-1].feasibility,
        "landing": check_trajectory_feasibility(landing, limits),
    }
    if not all(r.accepted for r in reports.values()):
        raise RuntimeError("frozen mission phase fails reference planning bounds")
    plan = MissionPlan(
        (
            MissionSegment(P.INITIALIZE, 1.0, zero, zero, K.HOLD),
            MinimumSnapMissionSegment(P.TAKEOFF, 4.0, zero, points[0], K.MINIMUM_SNAP, takeoff),
            MinimumSnapMissionSegment(
                P.TRACK,
                float(timed.trajectory.knot_times_s[-1]),
                points[0],
                points[-1],
                K.MINIMUM_SNAP,
                timed.trajectory,
            ),
            MissionSegment(P.TRACK, 1.0, points[-1], points[-1], K.HOLD),
            MinimumSnapMissionSegment(P.LAND, 4.0, points[-1], zero, K.MINIMUM_SNAP, landing),
        ),
        0.0,
        0.08,
        0.08,
        0.5,
        8.0,
    )
    return plan, {
        "planning_limits": plain(asdict(limits)),
        "scale": timed.scale,
        "attempts": [plain(asdict(a)) for a in timed.attempts],
        "phase_bounds": {name: plain(asdict(report)) for name, report in reports.items()},
        "initial_snap_cost_m2_s7": initial.snap_cost,
        "retimed_snap_cost_m2_s7": timed.trajectory.snap_cost,
    }


def score_result(result: MissionResult) -> dict[str, Any]:
    """Full-mission metrics, including failures and initialization/landing."""
    error = np.linalg.norm(result.position_W - result.reference_position_W, axis=1)
    duration = float(result.time_s[-1])
    rmse = float(np.sqrt(np.trapezoid(error**2, result.time_s) / duration)) if duration else None
    peak, final = float(np.max(error)), float(error[-1])
    final_speed = float(np.linalg.norm(result.velocity_W[-1]))
    inner, outer = bool(np.any(result.inner_limit_flags)), bool(np.any(result.outer_limit_flags))
    complete = result.phase[-1] == P.COMPLETE
    rotor = result.commanded_rotor_omega
    at_rotor_bound = bool(np.any((rotor <= 0) | (rotor >= 900)))
    return {
        "complete": bool(complete),
        "abort_reason": result.abort_reason,
        "duration_s": duration,
        "position_rmse_m": rmse,
        "position_peak_m": peak,
        "final_position_error_m": final,
        "final_speed_m_s": final_speed,
        "any_inner_limiting": inner,
        "any_outer_limiting": outer,
        "any_rotor_at_bound": at_rotor_bound,
        "maximum_actual_body_rate_rad_s": float(np.max(np.linalg.norm(result.omega_B, axis=1))),
        "maximum_actual_tilt_deg": float(
            np.rad2deg(
                np.max(np.arccos(np.clip(1 - 2 * np.sum(result.q_WB[:, 1:3] ** 2, axis=1), -1, 1)))
            )
        ),
        "passed": bool(
            complete
            and rmse is not None
            and rmse <= TARGETS["position_rmse_m"]
            and peak <= TARGETS["position_peak_m"]
            and final <= TARGETS["final_position_error_m"]
            and final_speed <= TARGETS["final_speed_m_s"]
            and not inner
            and not outer
            and not at_rotor_bound
        ),
    }


def run_case(job: tuple[str, str]) -> dict[str, Any]:
    name, directory = job
    try:
        initial, body, world, outer, inner, _, safety, numerics = make_case(
            {
                "case": "mild_wind" if name == "mild_wind" else "square",
                "seed": None,
                "refinement": 2 if name == "refined" else 1,
            }
        )
        plan, _ = build_mission()
        if name.startswith("offset_"):
            sign = 1 if name == "offset_positive" else -1
            angle = sign * np.deg2rad(1.0)
            initial = replace(
                initial,
                position_W=sign * np.array([0.02, -0.01, 0.015]),
                velocity_W=sign * np.array([0.01, 0.0, -0.01]),
                q_WB=np.array([np.cos(angle / 2), np.sin(angle / 2), 0.0, 0.0]),
                omega_B=sign * np.array([0.01, -0.01, 0.005]),
            )
        result = simulate_mission(
            initial,
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
        path = Path(directory) / (name + ".npz")
        with path.open("xb") as stream:
            np.savez_compressed(
                stream,
                **{
                    field.name: getattr(result, field.name)
                    for field in fields(result)
                    if isinstance(getattr(result, field.name), np.ndarray)
                },
            )
        return {
            "name": name,
            **score_result(result),
            "archive": path.name,
            "archive_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    except Exception as error:
        return {"name": name, "passed": False, "error": f"{type(error).__name__}: {error}"}


def refinement_report(directory: Path) -> dict[str, Any]:
    with (
        np.load(directory / "nominal.npz", allow_pickle=False) as coarse,
        np.load(directory / "refined.npz", allow_pickle=False) as fine,
    ):
        count = min(len(coarse["time_s"]), len(fine["time_s"][::2]))
        assert np.array_equal(coarse["time_s"][:count], fine["time_s"][::2][:count])
        position = float(
            np.max(
                np.linalg.norm(
                    coarse["position_W"][:count] - fine["position_W"][::2][:count], axis=1
                )
            )
        )
        dots = np.sum(coarse["q_WB"][:count] * fine["q_WB"][::2][:count], axis=1)
        angle = float(np.rad2deg(np.max(2 * np.arccos(np.clip(np.abs(dots), 0, 1)))))
    return {
        "shared_epochs": count,
        "maximum_position_difference_m": position,
        "maximum_attitude_difference_deg": angle,
        "passed": position <= TARGETS["refinement_position_m"]
        and angle <= TARGETS["refinement_attitude_deg"],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=1, choices=range(1, 6))
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    execution_source = source_sha256(root)
    plan, planning = build_mission()
    protocol = {
        "name": "minimum_snap_true_state_missions",
        "version": 1,
        "cases": CASES,
        "targets": TARGETS,
        "plan": plain(asdict(plan)),
        "planning": planning,
        "controllers_and_truth": "unchanged experiments.position_control_validation.make_case",
        "offsets": {
            "position_W_m": [0.02, -0.01, 0.015],
            "velocity_W_m_s": [0.01, 0, -0.01],
            "roll_deg": 1.0,
            "omega_B_rad_s": [0.01, -0.01, 0.005],
            "signs": [1, -1],
        },
        "semantics": "Five bounded deterministic true-state cases; not Monte Carlo qualification. "
        "Reference feasibility is nominal and drag-free. No estimated-feedback or gain changes.",
    }
    args.output.mkdir(parents=True, exist_ok=False)
    protocol_bytes = canonical_json(protocol)
    (args.output / "protocol.json").write_bytes(protocol_bytes + b"\n")
    jobs = [(name, str(args.output)) for name in CASES]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        cases = list(pool.map(run_case, jobs))
    refinement = (
        refinement_report(args.output)
        if all((args.output / (name + ".npz")).is_file() for name in ("nominal", "refined"))
        else {"passed": False, "error": "required histories missing"}
    )
    report = {
        "protocol_sha256": hashlib.sha256(protocol_bytes).hexdigest(),
        "source_sha256": execution_source,
        "source_unchanged_during_run": execution_source == source_sha256(root),
        "cases": cases,
        "refinement": refinement,
        "all_passed": all(case["passed"] for case in cases)
        and refinement["passed"]
        and execution_source == source_sha256(root),
    }
    (args.output / "report.json").write_bytes(canonical_json(report) + b"\n")
    print(canonical_json(report).decode(), flush=True)
    return 0 if report["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
