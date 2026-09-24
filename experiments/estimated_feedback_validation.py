"""Frozen paired true/estimated-state mission campaign (ADR 0013).

CLI: python -m experiments.estimated_feedback_validation --partition smoke --output DIR
DIR must be new. Full histories and covariances are lossless compressed chunks;
report.json is published last, after replay/dataflow/metric validation.
"""

import argparse
import hashlib
import json
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, fields
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.estimated_feedback_evidence import (
    load_history,
    pack_history,
    plain,
    save_history,
    unpack_history,
)
from experiments.position_control_validation import (
    _unique_object,
    _validate_history,
    make_case,
    score_case,
)
from experiments.position_control_validation import (
    validation_protocol as baseline_protocol,
)
from quadrotor_math.attitude_control import attitude_error_body, compute_attitude_control
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_endpoint import EskfSampledImuNoise
from quadrotor_math.eskf_innovation import chi_square_99_percent_eskf_innovation_policy
from quadrotor_math.eskf_replay import EskfReplayConfiguration, replay_eskf
from quadrotor_math.estimated_mission import (
    EstimatedMissionResult,
    MissionSensors,
    simulate_estimated_mission,
)
from quadrotor_math.mission_simulation import MissionResult, simulate_mission
from quadrotor_math.missions import (
    MissionPhase,
    MissionState,
    advance_mission,
    mission_guard_reason,
    mission_reference,
)
from quadrotor_math.position_control import compute_position_control
from quadrotor_math.run_configuration import ImuParameters, PositionSensorParameters, SensorSchedule
from quadrotor_math.run_manifest import capture_software_provenance

PARTITIONS = ("smoke", "fixed", "development", "validation")


def planned_jobs(partition: str) -> list[dict[str, Any]]:
    if partition == "fixed":
        return [{"case": "square", "seed": None, "noiseless": True}] + [
            {"case": case, "seed": 30 + i, "noiseless": False}
            for i, case in enumerate(("hover", "square", "vertical_step", "mild_wind"))
        ]
    if partition not in ("smoke", "development", "validation"):
        raise ValueError("unknown estimated-feedback partition")
    seeds = {
        "smoke": range(40, 42),
        "development": range(8100, 8103),
        "validation": range(90000, 90010),
    }[partition]
    return [
        {"case": "smoke" if partition == "smoke" else "square", "seed": seed, "noiseless": False}
        for seed in seeds
    ]


def make_configuration(job: dict[str, Any]) -> dict[str, Any]:
    """Separate true initialization/noise, independent fixed prior, and nominal gains."""
    initial, body, world, outer, inner, plan, safety, numerics = make_case(
        {"case": job["case"], "seed": job["seed"], "refinement": 1}
    )
    noiseless = job["noiseless"]
    a_bias = g_bias = np.zeros(3)
    if not noiseless:
        generator = np.random.Generator(
            np.random.PCG64(np.random.SeedSequence([0x45535443, 1, job["seed"]]))
        )
        a_bias = generator.uniform(-0.02, 0.02, 3)
        g_bias = generator.uniform(-0.003, 0.003, 3)
    sigma_a, sigma_g = (0.0, 0.0) if noiseless else (0.04, 0.002)
    walk_a, walk_g = (0.0, 0.0) if noiseless else (0.0002, 0.00002)
    imu = ImuParameters(
        a_bias,
        np.full(3, sigma_a),
        np.full(3, walk_a),
        g_bias,
        np.full(3, sigma_g),
        np.full(3, walk_g),
    )
    position = PositionSensorParameters(
        np.zeros(3), np.full(3, 0 if noiseless else 0.02), 0.0, 0.0, 0 if noiseless else 0.03
    )
    sensors = MissionSensors(
        imu,
        position,
        SensorSchedule(0.2, 0),
        SensorSchedule(0.04, 0),
        0 if job["seed"] is None else job["seed"],
    )
    prior = EskfNominalState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )
    prior_std = np.array([0.03] * 6 + [np.deg2rad(3)] * 3 + [0.03] * 3 + [0.005] * 3)
    config = EskfReplayConfiguration(
        0.0,
        prior,
        np.zeros((15, 15)) if noiseless else np.diag(prior_std**2),
        9.81,
        np.zeros((12, 12)),
        np.zeros(3),
        np.eye(3) * 0.02**2,
        0.0,
        0.0,
        0.03**2,
        innovation_policy=chi_square_99_percent_eskf_innovation_policy(),
        sampled_imu_noise=EskfSampledImuNoise(
            np.diag([sigma_a**2] * 3 + [sigma_g**2] * 3),
            np.diag([walk_a**2] * 3 + [walk_g**2] * 3),
        ),
    )
    return dict(
        initial_state=initial,
        initial_actual_rotor_omega=np.full(4, np.sqrt(9.81 / 4e-5)),
        truth_body=body,
        truth_rotors=inner.nominal_rotors,
        truth_world=world,
        position_controller=outer,
        attitude_controller=inner,
        plan=plan,
        safety=safety,
        numerics=numerics,
        sensors=sensors,
        estimator_configuration=config,
    )


def validation_protocol(partition: str) -> dict[str, Any]:
    jobs = planned_jobs(partition)
    return {
        "name": "estimated_state_mission_feedback",
        "version": 1,
        "partition": partition,
        "jobs": jobs,
        "configurations": [
            plain(
                {
                    key: asdict(value) if hasattr(value, "__dataclass_fields__") else value
                    for key, value in make_configuration(job).items()
                }
            )
            for job in jobs
        ],
        "baseline_definition": baseline_protocol("fixed"),
        "initialization": (
            "Fixed zero p/v/bias and identity q prior, independent of truth and measurements"
        ),
        "true_initial_randomness": (
            "Existing position-mission PCG64/SeedSequence [0x504F5343,1,seed]"
        ),
        "true_bias_randomness": {
            "SeedSequence": [0x45535443, 1, "seed"],
            "draws": ["uniform(-.02,.02,3) m/s2", "uniform(-.003,.003,3) rad/s"],
        },
        "sensor_randomness": (
            "Existing six named PCG64 streams, derivation version 1; no draw at zero for walks"
        ),
        "targets": {
            "position_rmse_m": 0.15,
            "final_true_position_m": 0.15,
            "final_true_speed_m_s": 0.15,
            "estimation_position_rmse_m": 0.10,
            "peak_attitude_estimation_deg": 15.0,
            "longest_actuator_limiting_s": 0.5,
            "final_limit_free_s": 1.0,
            "noiseless_pair_position_m": 0.005,
            "noiseless_pair_attitude_deg": 0.05,
        },
        "semantics": "Full-grid, paired separate closed loops; endpoint IMU 400Hz including t=0; "
        "slow first positive period; posterior estimate feedback and completion; "
        "separate truth safety "
        "oracle; full offline replay/dataflow audit; no hardware, rewind, or calibration claim. "
        "Noiseless uses zero prior/sample/walk covariance but positive nominal observation R "
        "to keep innovation solves defined; this is a deterministic limiting case, "
        "not matched-noise calibration.",
    }


def score_trial(
    job: dict[str, Any], result: EstimatedMissionResult, baseline: MissionResult
) -> dict[str, Any]:
    generic_job = {"case": job["case"], "seed": job["seed"], "refinement": 1}
    controlled, reference = (
        score_case(generic_job, result.mission),
        score_case(generic_job, baseline),
    )
    time = result.mission.time_s
    estimate_position = np.array([s.position_W for s in result.estimates.states])
    estimate_velocity = np.array([s.velocity_W for s in result.estimates.states])
    error = np.linalg.norm(estimate_position - result.mission.position_W, axis=1)
    velocity_error = np.linalg.norm(estimate_velocity - result.mission.velocity_W, axis=1)
    angle = np.rad2deg(
        [
            np.linalg.norm(attitude_error_body(s.q_WB, q_WB))
            for s, q_WB in zip(result.estimates.states, result.mission.q_WB, strict=True)
        ]
    )

    def rms(values: Any) -> float:
        return (
            float(np.sqrt(np.trapezoid(values**2, time) / time[-1]))
            if time[-1]
            else float(values[0])
        )

    common = min(len(time), len(baseline.time_s))
    differences = np.linalg.norm(
        result.mission.position_W[:common] - baseline.position_W[:common], axis=1
    )
    attitude_difference = np.rad2deg(
        [
            np.linalg.norm(attitude_error_body(a, b))
            for a, b in zip(result.mission.q_WB[:common], baseline.q_WB[:common], strict=True)
        ]
    )
    counts = dict(Counter(e.status.value for e in result.estimates.events))
    nis = {
        kind: [
            e.innovation.normalized_innovation_squared
            for e in result.estimates.events
            if e.observation.kind.name == kind and e.innovation is not None
        ]
        for kind in ("LOCAL_POSITION", "BAROMETRIC_ALTITUDE")
    }
    metrics = {
        "estimated_feedback": controlled,
        "true_feedback": reference,
        "estimation_position_rmse_m": rms(error),
        "estimation_velocity_rmse_m_s": rms(velocity_error),
        "peak_attitude_estimation_deg": float(np.max(angle)),
        "event_counts": counts,
        "nis_mean": {
            kind: float(np.mean(values)) if values else None for kind, values in nis.items()
        },
        "common_horizon_s": float(time[common - 1]),
        "maximum_pair_position_difference_m": float(np.max(differences)),
        "maximum_pair_attitude_difference_deg": float(np.max(attitude_difference)),
        "position_rmse_change_m": controlled["position_rmse_m"] - reference["position_rmse_m"],
    }
    conditions = {
        "estimated_mission": controlled["passed"],
        "paired_true_mission": reference["passed"],
    }
    if job["case"] != "smoke":
        conditions.update(
            {
                "true_landing_position": controlled["final_position_error_m"] < 0.15,
                "true_landing_speed": controlled["final_speed_m_s"] < 0.15,
                "estimation_position": metrics["estimation_position_rmse_m"] < 0.10,
                "estimation_attitude": metrics["peak_attitude_estimation_deg"] < 15,
            }
        )
    if job["noiseless"]:
        conditions.update(
            {
                "noiseless_position_parity": metrics["maximum_pair_position_difference_m"] < 0.005,
                "noiseless_attitude_parity": metrics["maximum_pair_attitude_difference_deg"] < 0.05,
            }
        )
    metrics["conditions"], metrics["passed"] = conditions, all(conditions.values())
    return metrics


def audit_history(
    job: dict[str, Any],
    result: EstimatedMissionResult,
    baseline: MissionResult,
    *,
    configuration: dict[str, Any] | None = None,
) -> None:
    """Recompute offline ESKF, feedback commands, phases, clocks and reference values."""
    kwargs = make_configuration(job) if configuration is None else configuration
    generic_job = {"case": job["case"], "seed": job["seed"], "refinement": 1}
    _validate_history(
        generic_job,
        baseline,
        position_controller=kwargs["position_controller"],
        attitude_controller=kwargs["attitude_controller"],
    )
    r, estimates = result.mission, result.estimates
    dt = kwargs["numerics"].time_step_s
    if not np.array_equal(r.time_s, np.arange(len(r.time_s)) * dt):
        raise ValueError("noncanonical estimated mission clock")
    for name in ("position_W", "velocity_W", "q_WB", "omega_B"):
        if not np.array_equal(getattr(r, name)[0], getattr(kwargs["initial_state"], name)):
            raise ValueError("wrong true initialization")
    if not np.array_equal(r.actual_rotor_omega[0], kwargs["initial_actual_rotor_omega"]):
        raise ValueError("wrong initial motors")
    for clock, stride in (
        (r.control_time_s, kwargs["numerics"].attitude_stride),
        (r.position_control_time_s, kwargs["numerics"].position_stride),
    ):
        if not np.array_equal(clock, r.time_s[:-1:stride]):
            raise ValueError("feedback command clock mismatch")
    replay = replay_eskf(result.measurements, kwargs["estimator_configuration"])
    if not np.array_equal(replay.covariances, estimates.covariances):
        raise ValueError("online covariance differs from measurement-only offline replay")
    for actual, expected_state in zip(estimates.states, replay.states, strict=True):
        if any(
            not np.array_equal(getattr(actual, f.name), getattr(expected_state, f.name))
            for f in fields(EskfNominalState)
        ):
            raise ValueError("online state differs from measurement-only offline replay")
    if canonical_json(plain([asdict(e) for e in estimates.events])) != canonical_json(
        plain([asdict(e) for e in replay.events])
    ):
        raise ValueError("online event ledger differs from replay")
    for o, scheduled in zip(
        result.measurements.observations, result.scheduled_observation_delivery_time_s, strict=True
    ):
        schedule = (
            kwargs["sensors"].local_position_schedule
            if o.kind.value == 3
            else kwargs["sensors"].barometric_altitude_schedule
        )
        stride = round(schedule.sample_period_s / dt)
        if o.acquisition_index != (o.observation_index + 1) * stride or (
            scheduled != r.time_s[o.acquisition_index] + schedule.delivery_delay_s
        ):
            raise ValueError("observation acquisition/schedule differs from configuration")
    supervisor = MissionState()
    outer_row = inner_row = 0
    for k, t in enumerate(r.time_s):
        state = estimates.states[k]
        ref, _ = mission_reference(kwargs["plan"], float(t))
        for field, expected in (
            ("reference_position_W", ref.position_W),
            ("reference_velocity_W", ref.velocity_W),
            ("reference_acceleration_W", ref.acceleration_W),
        ):
            if not np.array_equal(getattr(r, field)[k], expected):
                raise ValueError("analytic reference mismatch")
        reason = mission_guard_reason(r.position_W[k], r.q_WB[k], kwargs["safety"])
        reason = "truth_" + reason if reason is not None else None
        observed = mission_guard_reason(state.position_W, state.q_WB, kwargs["safety"])
        if reason is None and observed is not None:
            reason = "estimate_" + observed
        supervisor = advance_mission(
            supervisor, kwargs["plan"], float(t), state.position_W, state.velocity_W, reason
        )
        if supervisor.phase not in (MissionPhase.COMPLETE, MissionPhase.ABORT):
            if k % kwargs["numerics"].position_stride == 0:
                held = compute_position_control(
                    state.position_W, state.velocity_W, ref, kwargs["position_controller"]
                )
            if np.linalg.norm(attitude_error_body(state.q_WB, held.q_reference_WB)) > (
                kwargs["attitude_controller"].maximum_attitude_error_rad
            ):
                supervisor = MissionState(MissionPhase.ABORT, float(t), reason="attitude_domain")
        if r.phase[k] != supervisor.phase:
            raise ValueError("mission phase did not use estimated feedback")
        if supervisor.phase in (MissionPhase.COMPLETE, MissionPhase.ABORT):
            if k != len(r.time_s) - 1 or supervisor.reason != r.abort_reason:
                raise ValueError("incorrect terminal disposition")
            break
        if k % kwargs["numerics"].position_stride == 0:
            outer_values = (
                held.requested_acceleration_W,
                held.feasible_acceleration_W,
                np.array([held.acceleration_limited, held.tilt_limited, held.thrust_limited]),
            )
            for field, value in zip(
                ("requested_acceleration_W", "feasible_acceleration_W", "outer_limit_flags"),
                outer_values,
                strict=True,
            ):
                if not np.array_equal(getattr(r, field)[outer_row], value):
                    raise ValueError("outer controller did not use its recorded estimate")
            outer_row += 1
        if k % kwargs["numerics"].attitude_stride == 0:
            command = compute_attitude_control(
                state.q_WB,
                result.angular_velocity_estimate_B[k],
                held.q_reference_WB,
                held.collective_thrust,
                kwargs["attitude_controller"],
            )
            inner_values: dict[str, Any] = {
                "q_reference_WB": held.q_reference_WB,
                "collective_thrust": held.collective_thrust,
                "commanded_rotor_omega": command.allocation.commanded_rotor_omega,
                "moment_requested_B": command.moment_requested_B,
                "moment_limited_B": command.moment_limited_B,
                "allocated_moment_B": command.allocation.allocated_moment_B,
                "desired_omega_B": command.desired_omega_B,
                "moment_scale": command.allocation.moment_scale,
                "inner_limit_flags": [
                    command.rate_limited,
                    command.moment_limited,
                    command.allocation.moment_scale < 1,
                ],
            }
            if any(
                not np.array_equal(getattr(r, key)[inner_row], value)
                for key, value in inner_values.items()
            ):
                raise ValueError("inner controller did not use its recorded estimate")
            inner_row += 1


def _execute(job: dict[str, Any]) -> dict[str, Any]:
    try:
        kwargs = make_configuration(job)
        result = simulate_estimated_mission(**kwargs)
        baseline = simulate_mission(
            **{k: v for k, v in kwargs.items() if k not in ("sensors", "estimator_configuration")}
        )
        return {
            "job": job,
            "status": "ok",
            "metrics": score_trial(job, result, baseline),
            "history": pack_history(result, baseline),
        }
    except (ValueError, FloatingPointError, np.linalg.LinAlgError, RuntimeError) as error:
        return {"job": job, "status": "error", "error": str(error)}


def summarize(protocol: dict[str, Any], rows: list[dict[str, Any]]) -> dict[str, Any]:
    if canonical_json([r["job"] for r in rows]) != canonical_json(protocol["jobs"]):
        raise ValueError("trial ledger must exactly match every planned identity in order")
    errors = failures = 0
    for row in rows:
        if row["status"] == "error":
            if (
                set(row) != {"job", "status", "error"}
                or not isinstance(row["error"], str)
                or not row["error"]
            ):
                raise ValueError("numerical failure requires its retained diagnostic")
            errors += 1
        elif row["status"] == "ok":
            failures += not row["metrics"]["passed"]
        else:
            raise ValueError("unknown trial status")
    return {
        "planned": len(rows),
        "numerical_failures": errors,
        "acceptance_failures": failures,
        "passed": errors == failures == 0,
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
    if canonical_json(protocol) != canonical_json(report["protocol"]) or (
        report["protocol_sha256"] != hashlib.sha256(canonical_json(protocol)).hexdigest()
    ):
        raise ValueError("protocol differs from its frozen definition")
    summary = summarize(protocol, report["trials"])
    for row in report["trials"]:
        if row["status"] == "ok":
            result, baseline = unpack_history(row["history"])
            audit_history(row["job"], result, baseline)
            if canonical_json(row["metrics"]) != canonical_json(
                score_trial(row["job"], result, baseline)
            ):
                raise ValueError("metrics do not match complete histories")
    if canonical_json(summary) != canonical_json(report["summary"]):
        raise ValueError("summary does not match the full trial ledger")
    return report


def save_report(report: dict[str, Any], output: Path) -> None:
    validate_report(report)
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    for index, row in enumerate(report["trials"]):
        saved = {key: value for key, value in row.items() if key != "history"}
        if row["status"] == "ok":
            saved["history_parts"] = save_history(output, index, row["history"])
        rows.append(saved)
    with (output / "report.json").open("xb") as stream:
        stream.write(canonical_json({**report, "format_version": 1, "trials": rows}) + b"\n")


def load_report(directory: Path) -> dict[str, Any]:
    report = json.loads((directory / "report.json").read_bytes(), object_pairs_hook=_unique_object)
    canonical_json(report)
    if type(report.get("format_version")) is not int or report["format_version"] != 1:
        raise ValueError("unsupported estimated-feedback format")
    for index, row in enumerate(report["trials"]):
        if row["status"] == "ok":
            row["history"] = load_history(directory, index, row["history_parts"])
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
    source = source_sha256(root)
    provenance = asdict(capture_software_provenance(root))
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
            "source_sha256": source,
            "software_provenance": provenance,
            "generated_at_utc": datetime.now(UTC).isoformat(),
            "workers": args.workers,
        }
    )
    save_report(report, args.output)
    print(json.dumps(report["summary"], sort_keys=True), file=sys.stderr)
    return int(not report["summary"]["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
