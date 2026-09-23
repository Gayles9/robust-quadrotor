"""Run frozen ESKF protocols: original ADR 0009 or endpoint ADR 0010.

python -m experiments.eskf_validation --partition smoke --output /tmp/new.json
Generated JSON is an analysis report, not a persisted estimator/run-artifact schema.
"""

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
from dataclasses import asdict, replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

from experiments.eskf_consistency import (
    BLOCK_UNITS,
    _score_summary,
    _source_sha256,
    canonical_json,
    error_metrics,
    innovation_series,
    protocol_sha256,
)
from quadrotor_math.eskf_consistency import evaluate_eskf_replay
from quadrotor_math.eskf_endpoint import EskfSampledImuNoise
from quadrotor_math.eskf_faults import (
    EskfFaultInjection,
    EskfObservationFault,
    inject_eskf_observation_faults,
)
from quadrotor_math.eskf_innovation import chi_square_99_percent_eskf_innovation_policy
from quadrotor_math.eskf_replay import (
    EskfObservationKind,
    EskfReplayInput,
    EskfReplayResult,
    EskfReplayStatus,
    _time_vector,
    replay_eskf,
)
from quadrotor_math.eskf_synthetic import (
    ACCEL_STD,
    ACCEL_WALK,
    ALTITUDE_DATUM,
    ALTITUDE_STD,
    GRAVITY,
    GYRO_STD,
    GYRO_WALK,
    MOTIONS,
    POSITION_STD,
    PRIOR_STD,
    EskfSyntheticCase,
    make_eskf_synthetic_case,
    synthetic_eskf_rng,
)
from quadrotor_math.run_manifest import capture_software_provenance

PARTITIONS = {
    "smoke": (range(10, 12), 40),
    "development": (range(4000, 4005), 3000),
    "validation": (range(40000, 40100), 3000),
}
ENDPOINT_PARTITIONS = {
    "smoke": (range(20, 22), 40),
    "development": (range(5000, 5005), 3000),
    "validation": (range(50000, 50100), 3000),
}
SENSITIVITY = {
    "q_low": (0.25, 1.0),
    "q_high": (4.0, 1.0),
    "r_low": (1.0, 0.25),
    "r_high": (1.0, 4.0),
}


def planned_jobs(
    partition: str, propagation: str = "first_order"
) -> list[tuple[str, int, int, tuple[str, ...], bool]]:
    """Freeze paired variants; each job has isolated RNGs, independent of worker order."""
    if propagation not in ("first_order", "endpoint"):
        raise ValueError(f"unknown propagation: {propagation}")
    partitions = ENDPOINT_PARTITIONS if propagation == "endpoint" else PARTITIONS
    if partition not in partitions:
        raise ValueError(f"unknown partition: {partition}")
    seeds, steps = partitions[partition]
    jobs = []
    for family in MOTIONS if partition == "development" else ("excited",):
        for index, seed in enumerate(seeds):
            variants = ["nominal", "dead_reckoning"]
            if propagation == "endpoint":
                variants.append("first_order")
            if family == "excited":
                if index < 20:
                    variants.extend(("fault_gated", "fault_ungated"))
                if index < 10:
                    variants.extend(SENSITIVITY)
            jobs.append((family, seed, steps, tuple(variants), index == 0))
    return jobs


def validation_protocol(partition: str, propagation: str = "first_order") -> dict[str, Any]:
    """Complete finite canonical metadata; no time, source or worker count in its hash."""
    jobs = planned_jobs(partition, propagation)
    protocol: dict[str, Any] = {
        "name": "eskf_completion",
        "version": 1,
        "partition": partition,
        "jobs": [
            {"motion": family, "seed": seed, "number_of_steps": steps, "variants": list(variants)}
            for family, seed, steps, variants, _ in jobs
        ],
        "time_step_s": 0.01,
        "position_period_s": 0.1,
        "altitude_period_s": 0.2,
        "gravity_m_per_s2": GRAVITY,
        "altitude_datum_m": ALTITUDE_DATUM,
        "sample_standard_deviations": {
            "accelerometer": ACCEL_STD,
            "gyroscope": GYRO_STD,
            "position": POSITION_STD,
            "altitude": ALTITUDE_STD,
        },
        "bias_walk_densities": [ACCEL_WALK, GYRO_WALK],
        "prior_block_standard_deviations": list(PRIOR_STD),
        "prior": (
            "independent pose/velocity error; zero nominal biases, "
            "independent true initial biases; no initial walk"
        ),
        "motion_distribution": {
            "position_amplitude_m": [1, 2],
            "position_frequency_rad_per_s": [0.5, 1],
            "roll_pitch_amplitude_rad": [0.15, 0.3],
            "yaw_amplitude_rad": [0.3, 0.6],
            "euler_frequency_rad_per_s": [0.4, 0.8],
            "phase_rad": [-float(np.pi), float(np.pi)],
        },
        "randomness": {
            "generator": "PCG64",
            "seed_sequence_entropy": [0x45534B43, 1, "stream_id", "seed"],
            "pool_size": 4,
            "streams": [
                "motion",
                "prior",
                "initial_bias",
                "accelerometer_white",
                "gyroscope_white",
                "accelerometer_walk",
                "gyroscope_walk",
                "position_white",
                "altitude_white",
                "faults",
            ],
        },
        "noise_conversion": "diag(sigma_a^2*dt,sigma_g^2*dt,eta_a^2,eta_g^2)",
        "gate": asdict(chi_square_99_percent_eskf_innovation_policy()),
        "faults": {
            "outlier_single_s": 5.0,
            "outlier_bursts_s": [[12, 14], [22, 24]],
            "outlier_magnitude_m": [2, 3],
            "outlier_direction": (
                "normalized 3D Gaussian for position; fair random sign for altitude"
            ),
            "position_dropout_s": [8, 10],
            "delay_window_s": [15, 17],
            "additional_delay_s": 0.1,
            "final_acquisitions_delayed": True,
            "interval_convention": "left closed, right open",
        },
        "sensitivity_covariance_multipliers": SENSITIVITY.copy(),
        "targets": {
            "position_rmse_reduction": 0.5,
            "bias_final_to_initial_mean_norm_max": 0.5,
            "precision_strict_min": 0.90,
            "recall_strict_min": 0.85,
            "coverage_investigation_band": [0.90, 0.98],
            "divergence_position_m": 10,
            "divergence_attitude_rad": float(np.pi / 2),
        },
        "dropout_epochs_s": [7.9, 9.99, 11.0],
        "statistics": (
            "independent seeds at a common epoch; temporal summaries descriptive; "
            "paired variants correlated; retain every scored NIS including rejections"
        ),
        "failure_policy": (
            "retain every variant/seed; suppress complete-group summary "
            "on any failure; retain finite divergent cases"
        ),
    }
    if propagation == "endpoint":
        protocol["version"] = 2
        protocol["propagation"] = "endpoint"
        protocol["noise_conversion"] = (
            "sample Sigma=diag(sigma_a^2,sigma_g^2); "
            "bias endpoint increment covariance=diag(eta_a^2,eta_g^2)*dt; "
            "retain conditional sample mean and 21x21 joint covariance"
        )
        protocol["targets"]["paired_position_rmse_max_ratio"] = 1.05
    return protocol


def endpoint_case(case: EskfSyntheticCase) -> EskfSyntheticCase:
    """Select explicit sample units, preserving measurements, prior and noise levels."""
    return replace(
        case,
        configuration=replace(
            case.configuration,
            continuous_noise_covariance=np.zeros((12, 12)),
            sampled_imu_noise=EskfSampledImuNoise(
                np.diag(np.repeat([ACCEL_STD**2, GYRO_STD**2], 3)),
                np.diag(np.repeat([ACCEL_WALK**2, GYRO_WALK**2], 3)),
            ),
        ),
    )


def validation_fault_plan(data: EskfReplayInput, seed: int) -> tuple[EskfObservationFault, ...]:
    """Predeclared faults; offsets drawn in source (kind,index) order, independent of noise."""
    rng = synthetic_eskf_rng(seed, 9)
    faults = []
    if len(data.time_s) < 2:
        raise ValueError("fault protocol requires at least two fixed-clock epochs")
    dt = float(data.time_s[1] - data.time_s[0])
    if not np.array_equal(data.time_s, np.arange(len(data.time_s)) * dt):
        raise ValueError("fault protocol requires a uniform clock starting at zero")
    delay_steps = round(0.1 / dt)
    if not np.isclose(delay_steps * dt, 0.1, rtol=1e-12, atol=0):
        raise ValueError("fault protocol needs a clock resolving .1 s")
    last = {
        kind: max((o.observation_index for o in data.observations if o.kind is kind), default=-1)
        for kind in EskfObservationKind
    }
    for observation in sorted(data.observations, key=lambda o: (o.kind, o.observation_index)):
        time = float(data.time_s[observation.acquisition_index])
        position = observation.kind is EskfObservationKind.LOCAL_POSITION
        dropout = position and 8 <= time < 10
        offset = None
        if time == 5.0 or 12 <= time < 14 or 22 <= time < 24:
            direction = rng.standard_normal(3 if position else 1)
            length = float(np.linalg.norm(direction))
            if length == 0:
                raise ValueError("zero random outlier direction")
            offset = direction / length * rng.uniform(2, 3)
        delay = (
            delay_steps
            if 15 <= time < 17 or observation.observation_index == last[observation.kind]
            else 0
        )
        if dropout or delay or offset is not None:
            faults.append(
                EskfObservationFault(
                    observation.kind, observation.observation_index, dropout, delay, offset
                )
            )
    return tuple(faults)


def fault_detection_metrics(
    injection: EskfFaultInjection, result: EskfReplayResult
) -> dict[str, Any]:
    """Confusion matrices for eligible observations; never count missing data as TN.

    The ledger is consumed only after filtering. Verify exact observation identity,
    measurements and time alignment before pairing labels with estimator decisions.
    """
    if not np.array_equal(injection.measurements.time_s, result.time_s):
        raise ValueError("fault metrics require exactly aligned times")
    events = {(e.observation.kind, e.observation.observation_index): e for e in result.events}
    if len(events) != len(injection.measurements.observations):
        raise ValueError("fault metrics require every supplied observation event")
    metrics: dict[str, Any] = {}
    for kind in EskfObservationKind:
        counts = dict.fromkeys(
            (
                "true_positive",
                "false_positive",
                "true_negative",
                "false_negative",
                "dropped",
                "stale",
                "pending",
                "disabled",
                "unscored",
                "ineligible_outliers",
                "source_count",
            ),
            0,
        )
        first_detection: list[float | None] = []
        rejected_times = []
        for record in injection.records:
            if record.source.kind is not kind:
                continue
            counts["source_count"] += 1
            outlier = record.fault.offset is not None and bool(np.any(record.fault.offset))
            if record.output is None:
                counts["dropped"] += 1
                continue
            event = events.get((kind, record.output.observation_index))
            if (
                event is None
                or (event.observation.acquisition_index, event.observation.delivery_index)
                != (record.output.acquisition_index, record.output.delivery_index)
                or not np.array_equal(event.observation.measurement, record.output.measurement)
            ):
                raise ValueError("fault metric observation identity/value mismatch")
            if event.status not in (EskfReplayStatus.FUSED, EskfReplayStatus.REJECTED):
                counts[event.status.value] += 1
                counts["ineligible_outliers"] += int(outlier)
            elif event.innovation is None:
                counts["unscored"] += 1
                counts["ineligible_outliers"] += int(outlier)
            else:
                rejected = event.status is EskfReplayStatus.REJECTED
                key = (
                    ("true_positive" if outlier else "false_positive")
                    if rejected
                    else ("false_negative" if outlier else "true_negative")
                )
                counts[key] += 1
                if outlier and rejected:
                    rejected_times.append(float(result.time_s[event.observation.delivery_index]))
        for start, end in ((12.0, 14.0), (22.0, 24.0)):
            detections = [t for t in rejected_times if start <= t < end]
            first_detection.append(min(detections) - start if detections else None)
        tp, fp, tn, fn = (
            counts[name]
            for name in ("true_positive", "false_positive", "true_negative", "false_negative")
        )
        metrics[kind.name.lower()] = {
            **counts,
            "precision": tp / (tp + fp) if tp + fp else None,
            "recall": tp / (tp + fn) if tp + fn else None,
            "false_positive_rate": fp / (fp + tn) if fp + tn else None,
            "burst_detection_delay_s": first_detection,
        }
    return metrics


def _evaluate_variant(
    case: EskfSyntheticCase, variant: str, injected: EskfFaultInjection, keep_history: bool
) -> dict[str, Any]:
    if variant not in (
        "nominal",
        "dead_reckoning",
        "first_order",
        "fault_gated",
        "fault_ungated",
        *SENSITIVITY,
    ):
        raise ValueError(f"unknown variant {variant}")
    configuration = case.configuration
    data = case.measurements
    if variant == "first_order":
        if configuration.sampled_imu_noise is None:
            raise ValueError("first_order comparison requires an endpoint case")
        dt = float(data.time_s[1] - data.time_s[0])
        configuration = replace(
            configuration,
            sampled_imu_noise=None,
            continuous_noise_covariance=np.diag(
                np.repeat([ACCEL_STD**2 * dt, GYRO_STD**2 * dt, ACCEL_WALK**2, GYRO_WALK**2], 3)
            ),
        )
    if variant == "dead_reckoning":
        configuration = replace(
            configuration, fuse_local_position=False, fuse_barometric_altitude=False
        )
    elif variant in ("fault_gated", "fault_ungated"):
        data = injected.measurements
        if variant == "fault_gated":
            configuration = replace(
                configuration, innovation_policy=chi_square_99_percent_eskf_innovation_policy()
            )
    elif variant in SENSITIVITY:
        q_scale, r_scale = SENSITIVITY[variant]
        configuration = replace(
            configuration,
            continuous_noise_covariance=configuration.continuous_noise_covariance * q_scale,
            local_position_noise_covariance_W=configuration.local_position_noise_covariance_W
            * r_scale,
            barometric_altitude_noise_variance=configuration.barometric_altitude_noise_variance
            * r_scale,
            sampled_imu_noise=(
                EskfSampledImuNoise(
                    configuration.sampled_imu_noise.sample_covariance_B * q_scale,
                    configuration.sampled_imu_noise.bias_walk_spectral_density_B * q_scale,
                )
                if configuration.sampled_imu_noise is not None
                else None
            ),
        )
    # Only measured data and nominal assumptions cross the estimator boundary.
    result = replay_eskf(data, configuration)
    consistency = evaluate_eskf_replay(result, case.reference)
    metrics = error_metrics(consistency.error_states)
    scales = np.sqrt(np.diagonal(result.covariances, axis1=1, axis2=2))
    normalized = result.covariances / scales[:, :, None] / scales[:, None, :]
    min_eigenvalue = float(np.linalg.eigvalsh(normalized).min())
    sample = np.arange(0, len(result.time_s), 10)
    norms = np.linalg.norm(consistency.error_states.reshape(-1, 5, 3), axis=2)
    report: dict[str, Any] = {
        "status": "completed",
        "time_s": result.time_s.tolist(),
        "metrics": metrics,
        "nees": consistency.nees.tolist(),
        "nis": {name: series.to_mapping() for name, series in innovation_series(result).items()},
        "minimum_normalized_covariance_eigenvalue": min_eigenvalue,
        "max_quaternion_squared_norm_defect": max(
            abs(float(s.q_WB @ s.q_WB) - 1) for s in result.states
        ),
        "initial_bias_error_norms": norms[0, 3:].tolist(),
        "final_bias_error_norms": norms[-1, 3:].tolist(),
        "sample_time_s": result.time_s[sample].tolist(),
        "error_block_norms": norms[sample].tolist(),
        "horizontal_position_variance_m2": (
            result.covariances[sample, 0, 0] + result.covariances[sample, 1, 1]
        ).tolist(),
    }
    if result.time_s[0] <= 7.9 and result.time_s[-1] >= 11.0:
        # Use actual epochs, including on the step-refinement diagnostic clocks.
        # If a requested instant is absent, report the last epoch at/before it.
        epochs = np.searchsorted(result.time_s, [7.9, 9.99, 11.0], side="right") - 1
        report["dropout_time_s"] = result.time_s[epochs].tolist()
        report["dropout_horizontal_variance_m2"] = (
            result.covariances[epochs, 0, 0] + result.covariances[epochs, 1, 1]
        ).tolist()
        report["dropout_position_error_m"] = norms[epochs, 0].tolist()
    if variant.startswith("fault_"):
        report["fault_detection"] = fault_detection_metrics(injected, result)
        labels = {
            (record.output.kind, record.output.observation_index): record
            for record in injected.records
            if record.output is not None
        }
        report["fault_events"] = [
            {
                "kind": event.observation.kind.name.lower(),
                "source_index": labels[
                    (event.observation.kind, event.observation.observation_index)
                ].source.observation_index,
                "output_index": event.observation.observation_index,
                "acquisition_index": event.observation.acquisition_index,
                "delivery_index": event.observation.delivery_index,
                "status": event.status.value,
                "nis": (
                    event.innovation.normalized_innovation_squared
                    if event.innovation is not None
                    else None
                ),
                "threshold": event.nis_threshold,
                "offset": (
                    record.fault.offset.tolist() if record.fault.offset is not None else None
                ),
            }
            for event in result.events
            for record in [labels[(event.observation.kind, event.observation.observation_index)]]
        ]
    if keep_history:
        report["representative"] = {
            "error_states": consistency.error_states[sample].tolist(),
            "standard_deviations": scales[sample].tolist(),
            "position_W": np.array([s.position_W for s in case.reference.states])[sample].tolist(),
        }
    return report


def run_validation_trial(
    job: tuple[str, int, int, tuple[str, ...], bool], propagation: str = "first_order"
) -> dict[str, Any]:
    """Execute all declared variants; preserve any failed seed and continue the others."""
    if propagation not in ("first_order", "endpoint"):
        raise ValueError(f"unknown propagation: {propagation}")
    family, seed, steps, variants, keep_history = job
    report: dict[str, Any] = {"family": family, "seed": seed, "variants": {}}
    try:
        case = make_eskf_synthetic_case(seed, family=family, number_of_steps=steps)
        if propagation == "endpoint":
            case = endpoint_case(case)
        injected = inject_eskf_observation_faults(
            case.measurements, validation_fault_plan(case.measurements, seed)
        )
        report.update(
            {
                "motion_parameters": {
                    name: getattr(case.motion, name).tolist()
                    for name in case.motion.__dataclass_fields__
                },
                "initial_state": {
                    name: getattr(case.configuration.initial_state, name).tolist()
                    for name in case.configuration.initial_state.__dataclass_fields__
                },
                "initial_covariance": case.configuration.initial_covariance.tolist(),
                "continuous_noise_covariance": (
                    case.configuration.continuous_noise_covariance.tolist()
                ),
            }
        )
        if case.configuration.sampled_imu_noise is not None:
            model = case.configuration.sampled_imu_noise
            report["sampled_imu_noise"] = {
                "sample_covariance_B": model.sample_covariance_B.tolist(),
                "bias_walk_spectral_density_B": model.bias_walk_spectral_density_B.tolist(),
            }
    except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
        report["variants"] = {
            name: {
                "status": "numerical_failure",
                "stage": "generation",
                "error": f"{type(error).__name__}: {error}",
            }
            for name in variants
        }
        return report
    for variant in variants:
        try:
            report["variants"][variant] = _evaluate_variant(case, variant, injected, keep_history)
        except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
            report["variants"][variant] = {
                "status": "numerical_failure",
                "stage": "replay_or_evaluation",
                "error": f"{type(error).__name__}: {error}",
            }
    return report


def _validate_trial_plan(
    trials: list[dict[str, Any]], protocol: dict[str, Any] | None = None
) -> None:
    """Reject duplicate replicates, unknown variants and incomplete declared jobs."""
    if not trials:
        raise ValueError("trials must contain the declared jobs")
    seen: set[tuple[str, int]] = set()
    for trial in trials:
        identity = (trial["family"], trial["seed"])
        if trial["family"] not in MOTIONS or type(trial["seed"]) is not int:
            raise ValueError("invalid trial family or seed")
        if identity in seen:
            raise ValueError("duplicate trial family/seed cannot represent independent replicates")
        seen.add(identity)
        if not trial["variants"] or any(
            name
            not in (
                "nominal",
                "dead_reckoning",
                "first_order",
                "fault_gated",
                "fault_ungated",
                *SENSITIVITY,
            )
            for name in trial["variants"]
        ):
            raise ValueError("unknown or empty trial variants")
        for record in trial["variants"].values():
            if record["status"] == "numerical_failure":
                continue
            if record["status"] != "completed":
                raise ValueError("invalid variant status")
            times = _time_vector(np.asarray(record["time_s"], dtype=np.float64))
            scores = np.asarray(record["nees"], dtype=np.float64)
            if scores.shape != times.shape or not np.all(np.isfinite(scores)) or np.any(scores < 0):
                raise ValueError("NEES scores must be finite and match their epochs")
    if protocol is None:
        return
    jobs = {(job["motion"], job["seed"]): job for job in protocol["jobs"]}
    if len(jobs) != len(protocol["jobs"]) or seen != set(jobs):
        raise ValueError("trial identities must match every declared job exactly once")
    for trial in trials:
        job = jobs[(trial["family"], trial["seed"])]
        if set(trial["variants"]) != set(job["variants"]):
            raise ValueError("trial variants must match the declared job")
        times = np.arange(job["number_of_steps"] + 1) * protocol["time_step_s"]
        for record in trial["variants"].values():
            if record["status"] == "completed" and (
                record["time_s"] != times.tolist()
                or record["sample_time_s"] != times[::10].tolist()
            ):
                raise ValueError("completed variant epochs must match the declared job")
            if record["status"] == "completed" and times[-1] >= 11.0:
                epochs = np.searchsorted(times, protocol["dropout_epochs_s"], side="right") - 1
                if record["dropout_time_s"] != times[epochs].tolist():
                    raise ValueError("dropout diagnostic epochs must match the declared clock")


def summarize_validation(
    trials: list[dict[str, Any]], *, protocol: dict[str, Any] | None = None
) -> dict[str, Any]:
    """Report complete per-family/variant ensembles, never survivors-only estimates."""
    _validate_trial_plan(trials, protocol)
    groups: dict[str, Any] = {}
    identities = sorted(
        {(trial["family"], variant) for trial in trials for variant in trial["variants"]}
    )
    for family, variant in identities:
        records = [
            t["variants"][variant]
            for t in trials
            if t["family"] == family and variant in t["variants"]
        ]
        failures = sum(r["status"] != "completed" for r in records)
        group: dict[str, Any] = {
            "planned_count": len(records),
            "numerical_failure_count": failures,
            "ensemble": None,
        }
        groups[family + "/" + variant] = group
        if failures:
            continue
        if any(
            r["time_s"] != records[0]["time_s"] or r["sample_time_s"] != records[0]["sample_time_s"]
            for r in records
        ):
            raise ValueError("ensemble NEES/sample epochs must match exactly")
        time_s = np.asarray(records[0]["time_s"], dtype=np.float64)
        ensemble: dict[str, Any] = {
            "divergence_count": sum(r["metrics"]["diverged"] for r in records),
            "nees": _score_summary(np.array([r["nees"] for r in records]), 15, time_s),
            "rmse_distribution": {
                unit: {
                    "mean": float(np.mean(values)),
                    "median": float(np.median(values)),
                    "p95": float(np.quantile(values, 0.95)),
                    "maximum": float(max(values)),
                }
                for unit in BLOCK_UNITS
                for values in [[r["metrics"]["rmse"][unit] for r in records]]
            },
            "mean_initial_bias_error_norms": np.mean(
                [r["initial_bias_error_norms"] for r in records], axis=0
            ).tolist(),
            "mean_final_bias_error_norms": np.mean(
                [r["final_bias_error_norms"] for r in records], axis=0
            ).tolist(),
            "minimum_normalized_covariance_eigenvalue": min(
                r["minimum_normalized_covariance_eigenvalue"] for r in records
            ),
            "max_quaternion_squared_norm_defect": max(
                r["max_quaternion_squared_norm_defect"] for r in records
            ),
            "sample_time_s": records[0]["sample_time_s"],
            "mean_error_block_norms": np.mean(
                [r["error_block_norms"] for r in records], axis=0
            ).tolist(),
            "mean_horizontal_position_variance_m2": np.mean(
                [r["horizontal_position_variance_m2"] for r in records], axis=0
            ).tolist(),
        }
        for name, degrees in (("local_position", 3), ("barometric_altitude", 1)):
            series = [r["nis"][name] for r in records]
            if any(s["time_s"] != series[0]["time_s"] for s in series):
                raise ValueError(
                    "ensemble NIS epochs differ; refusing selection-conditioned summary"
                )
            ensemble[name + "_nis"] = (
                _score_summary(
                    np.array([s["scores"] for s in series]), degrees, np.array(series[0]["time_s"])
                )
                if series[0]["scores"]
                else None
            )
        if "dropout_horizontal_variance_m2" in records[0]:
            if any(r["dropout_time_s"] != records[0]["dropout_time_s"] for r in records):
                raise ValueError("ensemble dropout epochs must match exactly")
            values = np.array([r["dropout_horizontal_variance_m2"] for r in records])
            ensemble["dropout"] = {
                "time_s": records[0]["dropout_time_s"],
                "mean_variance_m2": values.mean(axis=0).tolist(),
                "growth_count": int(np.sum(values[:, 1] > values[:, 0])),
                "recovery_count": int(np.sum(values[:, 2] < values[:, 1])),
                "mean_position_error_m": np.mean(
                    [r["dropout_position_error_m"] for r in records], axis=0
                ).tolist(),
            }
        if "fault_detection" in records[0]:
            detections = {}
            for sensor in ("local_position", "barometric_altitude"):
                source = [r["fault_detection"][sensor] for r in records]
                counts = {
                    key: sum(s[key] for s in source)
                    for key in source[0]
                    if key
                    not in ("precision", "recall", "false_positive_rate", "burst_detection_delay_s")
                }
                tp, fp, tn, fn = (
                    counts[k]
                    for k in ("true_positive", "false_positive", "true_negative", "false_negative")
                )
                detections[sensor] = {
                    **counts,
                    "precision": tp / (tp + fp) if tp + fp else None,
                    "recall": tp / (tp + fn) if tp + fn else None,
                    "false_positive_rate": fp / (fp + tn) if fp + tn else None,
                    "burst_detection_delay_s": [s["burst_detection_delay_s"] for s in source],
                }
            ensemble["fault_detection"] = detections
        group["ensemble"] = ensemble
    return groups


def run_validation(
    partition: str, workers: int = 1, propagation: str = "first_order"
) -> dict[str, Any]:
    """Run deterministic jobs; worker count changes throughput, never seed assignment."""
    if type(workers) is not int or not 1 <= workers <= 8:
        raise ValueError("workers must be an integer in [1,8]")
    protocol = validation_protocol(partition, propagation)
    jobs = planned_jobs(partition, propagation)
    if workers == 1:
        trials = [run_validation_trial(job, propagation) for job in jobs]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            trials = list(pool.map(run_validation_trial, jobs, [propagation] * len(jobs)))
    return _assemble_report(protocol, trials)


def _assemble_report(protocol: dict[str, Any], trials: list[dict[str, Any]]) -> dict[str, Any]:
    """Derive every summary/count/assessment from the complete declared trial ledger."""
    variants = [(name, r) for trial in trials for name, r in trial["variants"].items()]
    report = {
        "report_schema": {"name": "eskf_completion_evaluation", "version": 2},
        "protocol": protocol,
        "protocol_sha256": protocol_sha256(protocol),
        "trials": trials,
        "groups": summarize_validation(trials, protocol=protocol),
        "numerical_failure_count": sum(r["status"] != "completed" for _, r in variants),
        "nominal_or_gated_divergence_count": sum(
            r["metrics"]["diverged"]
            for name, r in variants
            if name in ("nominal", "fault_gated") and r["status"] == "completed"
        ),
    }
    report["assessment"] = assess_validation(report)
    return report


def validate_validation_report(report: dict[str, Any]) -> dict[str, Any]:
    """Validate stored evidence before use and return an owned schema-v2 view.

    Check the frozen protocol/hash, complete unique job/variant identities and epochs,
    then recompute groups, failure counts and assessments. This detects accidental
    truncation/stale summaries; it cannot authenticate the underlying execution.
    Legacy v1 reports used an implicit fixed clock. Recover that clock only from the
    verified original protocol, preserving their numerical values and source metadata.
    """
    try:
        canonical_json(report)
        checked = deepcopy(report)
        schema = checked["report_schema"]
        if (
            not isinstance(schema, dict)
            or schema.get("name") != "eskf_completion_evaluation"
            or type(schema.get("version")) is not int
            or schema["version"] not in (1, 2)
        ):
            raise ValueError("unsupported estimator completion report schema")
        protocol = checked["protocol"]
        propagation = "endpoint" if protocol["version"] == 2 else "first_order"
        if canonical_json(protocol) != canonical_json(
            validation_protocol(protocol["partition"], propagation)
        ) or checked["protocol_sha256"] != protocol_sha256(protocol):
            raise ValueError("report must match the frozen protocol and its hash")
        if schema["version"] == 1:
            jobs = {(job["motion"], job["seed"]): job for job in protocol["jobs"]}
            for trial in checked["trials"]:
                job = jobs[(trial["family"], trial["seed"])]
                times = (np.arange(job["number_of_steps"] + 1) * protocol["time_step_s"]).tolist()
                for record in trial["variants"].values():
                    if record["status"] == "completed":
                        record.setdefault("time_s", times.copy())
                        if "dropout_horizontal_variance_m2" in record:
                            record.setdefault("dropout_time_s", list(protocol["dropout_epochs_s"]))
            for group in checked["groups"].values():
                ensemble = group["ensemble"]
                if ensemble is not None and "dropout" in ensemble:
                    ensemble["dropout"].setdefault("time_s", list(protocol["dropout_epochs_s"]))
            # The v1 assessment also embeds the dropout summary.
            fault_targets = checked["assessment"]["fault_targets"]
            if fault_targets is not None and fault_targets["dropout"] is not None:
                fault_targets["dropout"].setdefault("time_s", list(protocol["dropout_epochs_s"]))
            schema["version"] = 2
        expected = _assemble_report(protocol, checked["trials"])
        for key in (
            "groups",
            "numerical_failure_count",
            "nominal_or_gated_divergence_count",
            "assessment",
        ):
            if canonical_json(checked[key]) != canonical_json(expected[key]):
                raise ValueError(f"report {key} does not match its declared trials")
        return checked
    except (KeyError, TypeError, OverflowError) as error:
        raise ValueError("malformed estimator completion evidence") from error


def assess_validation(report: dict[str, Any]) -> dict[str, Any]:
    """Expose target findings separately from execution success; never tune on a miss."""
    groups = report["groups"]
    nominal = groups.get("excited/nominal", {}).get("ensemble")
    dead = groups.get("excited/dead_reckoning", {}).get("ensemble")
    result: dict[str, Any] = {
        "acceptance_partition": report["protocol"]["partition"] == "validation",
        "execution_passed": not (
            report["numerical_failure_count"] or report["nominal_or_gated_divergence_count"]
        ),
        "nominal_targets": None,
        "fault_targets": None,
        "paired_sensitivity": {},
    }
    if nominal is not None and dead is not None:
        reduction = (
            1
            - nominal["rmse_distribution"]["position_m"]["mean"]
            / dead["rmse_distribution"]["position_m"]["mean"]
        )
        ratios = (
            np.array(nominal["mean_final_bias_error_norms"])
            / nominal["mean_initial_bias_error_norms"]
        )
        coverage = {
            name: nominal[name]["descriptive_coverage"]
            for name in ("nees", "local_position_nis", "barometric_altitude_nis")
        }
        result["nominal_targets"] = {
            "position_rmse_reduction": reduction,
            "position_target_met": reduction >= 0.5,
            "bias_final_to_initial_mean_norm": ratios.tolist(),
            "bias_targets_met": bool(np.all(ratios <= 0.5)),
            "coverage": coverage,
            "coverage_investigation_needed": any(not 0.90 <= c <= 0.98 for c in coverage.values()),
        }
    fault = groups.get("excited/fault_gated", {}).get("ensemble")
    if fault is not None:
        result["fault_targets"] = {
            sensor: {
                "precision_target_met": values["precision"] is not None
                and values["precision"] > 0.9,
                "recall_target_met": values["recall"] is not None and values["recall"] > 0.85,
            }
            for sensor, values in fault["fault_detection"].items()
        }
        result["fault_targets"]["dropout"] = fault.get("dropout")
    for name in SENSITIVITY:
        paired = [trial for trial in report["trials"] if name in trial["variants"]]
        if paired and all(
            trial["variants"][variant]["status"] == "completed"
            for trial in paired
            for variant in ("nominal", name)
        ):
            result["paired_sensitivity"][name] = {
                "seed_count": len(paired),
                "position_rmse_means": {
                    variant: float(
                        np.mean(
                            [
                                trial["variants"][variant]["metrics"]["rmse"]["position_m"]
                                for trial in paired
                            ]
                        )
                    )
                    for variant in ("nominal", name)
                },
                "nees_means": {
                    variant: float(
                        np.mean([trial["variants"][variant]["nees"] for trial in paired])
                    )
                    for variant in ("nominal", name)
                },
            }
        else:
            result["paired_sensitivity"][name] = None
    if report["protocol"].get("propagation") == "endpoint":
        original = groups.get("excited/first_order", {}).get("ensemble")
        result["paired_first_order"] = None
        if nominal is not None and original is not None:
            ratio = (
                nominal["rmse_distribution"]["position_m"]["mean"]
                / original["rmse_distribution"]["position_m"]["mean"]
            )
            result["paired_first_order"] = {
                "position_rmse_ratio": ratio,
                "accuracy_target_met": ratio <= 1.05,
                "nominal_nees_mean": float(np.mean(nominal["nees"]["mean_by_epoch"])),
                "first_order_nees_mean": float(np.mean(original["nees"]["mean_by_epoch"])),
            }
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", choices=tuple(PARTITIONS), default="smoke")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--propagation", choices=("first_order", "endpoint"), default="first_order")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    root = Path(__file__).resolve().parents[1]
    provenance = capture_software_provenance(root)
    source_hash = _source_sha256(root)
    print(
        "Frozen protocol SHA256: "
        + protocol_sha256(validation_protocol(args.partition, args.propagation)),
        file=sys.stderr,
        flush=True,
    )
    report = run_validation(args.partition, args.workers, args.propagation)
    if source_hash != _source_sha256(root):
        raise RuntimeError("source files changed during evaluation")
    report.update(
        {
            "software_provenance": asdict(provenance),
            "source_sha256": source_hash,
            "generated_at_utc": datetime.now(UTC).isoformat(),
            "workers": args.workers,
        }
    )
    with args.output.open("xb") as output:
        output.write(canonical_json(report) + b"\n")
    print(
        f"Numerical failures: {report['numerical_failure_count']}; "
        f"nominal/gated divergence: {report['nominal_or_gated_divergence_count']}; {args.output}",
        file=sys.stderr,
    )
    return int(
        bool(report["numerical_failure_count"] or report["nominal_or_gated_divergence_count"])
    )


if __name__ == "__main__":
    raise SystemExit(main())
