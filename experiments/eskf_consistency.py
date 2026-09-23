"""Frozen nominal ESKF ensemble; run with --partition smoke|development|validation.

See ADR 0008. Generated JSON is an evaluation report, not a run-artifact schema.
No settings are fitted to this ensemble. Execute from a real Git checkout using
the project's locked environment. Results belong outside version control.
"""

import argparse
import hashlib
import json
import sys
from dataclasses import asdict, dataclass, replace
from datetime import UTC, datetime
from math import cos, hypot, pi, sin, sqrt
from pathlib import Path
from typing import cast

import numpy as np
from numpy.typing import NDArray

from quadrotor_math.consistency_statistics import NormalizedErrorEnsemble
from quadrotor_math.eskf import EskfNominalState, inject_eskf_error_state
from quadrotor_math.eskf_consistency import (
    EskfConsistencyHistory,
    eskf_reference_from_run_artifact,
    evaluate_eskf_replay,
    sampled_imu_continuous_noise_covariance,
)
from quadrotor_math.eskf_innovation import EskfInnovationPolicy
from quadrotor_math.eskf_replay import (
    EskfObservationKind,
    EskfReplayConfiguration,
    EskfReplayResult,
    EskfReplayStatus,
    replay_eskf,
)
from quadrotor_math.eskf_run_replay import (
    eskf_replay_configuration_from_nominal,
    eskf_replay_input_from_run_artifact,
)
from quadrotor_math.run_configuration import (
    ConstantRotorSpeedInput,
    ImuParameters,
    IntegrationMethod,
    NominalConfiguration,
    PositionSensorParameters,
    RigidBodyInitialState,
    RigidBodyParameters,
    RotorParameters,
    RunConfiguration,
    RunNumerics,
    SensorSchedule,
    SensorSchedules,
    TruthConfiguration,
    WorldParameters,
)
from quadrotor_math.run_generation import generate_run_artifact_data
from quadrotor_math.run_manifest import (
    RunManifest,
    SoftwareProvenance,
    capture_software_provenance,
    encode_run_manifest,
)

SCENARIOS = ("hover", "translating_yaw")
PARTITIONS = {
    "smoke": (range(2), 21),
    "development": (range(1000, 1010), 201),
    "validation": (range(20000, 20100), 201),
}
PRIOR_BLOCK_STANDARD_DEVIATIONS = (0.2, 0.1, 0.01, 0.02, 0.002)
BLOCK_UNITS = (
    "position_m",
    "velocity_m_per_s",
    "attitude_rad",
    "accelerometer_bias_m_per_s2",
    "gyroscope_bias_rad_per_s",
)
POSITION_DIVERGENCE_M = 10.0
ATTITUDE_DIVERGENCE_RAD = pi / 2
KINDS = {
    "local_position": EskfObservationKind.LOCAL_POSITION,
    "barometric_altitude": EskfObservationKind.BAROMETRIC_ALTITUDE,
}


def canonical_json(value: object) -> bytes:
    """Deterministic finite JSON used for protocol hashing and result serialization."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def protocol_sha256(protocol: dict[str, object]) -> str:
    return hashlib.sha256(canonical_json(protocol)).hexdigest()


def _motion(scenario: str) -> tuple[NDArray[np.float64], float]:
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario: {scenario}")
    return (
        (np.array([0.5, -0.25, 0.1]), 0.4) if scenario == "translating_yaw" else (np.zeros(3), 0.0)
    )


def run_configuration(scenario: str, seed: int, number_of_steps: int) -> RunConfiguration:
    """Construct one matched-parameter fixed input case; units are SI/NED/FRD."""
    velocity_W, yaw_rate = _motion(scenario)
    body = RigidBodyParameters(1.0, np.diag([0.01, 0.01, 0.02]))
    rotors = RotorParameters(
        np.array([[0.1, 0.1, 0], [0.1, -0.1, 0], [-0.1, -0.1, 0], [-0.1, 0.1, 0]]),
        np.array([1.0, -1, 1, -1]),
        1e-5,
        1e-7,
    )
    world = WorldParameters(9.81)
    imu = ImuParameters(
        np.zeros(3),
        np.full(3, 0.03),
        np.full(3, 0.001),
        np.zeros(3),
        np.full(3, 0.002),
        np.full(3, 0.0001),
    )
    sensors = PositionSensorParameters(np.zeros(3), np.full(3, 0.15), 100.0, 0.0, 0.1)
    return RunConfiguration(
        TruthConfiguration(body, rotors, world, imu, sensors),
        NominalConfiguration(body, rotors, world, imu, sensors),
        RigidBodyInitialState(
            np.zeros(3), velocity_W, np.array([1.0, 0, 0, 0]), np.array([0.0, 0, yaw_rate])
        ),
        RunNumerics(IntegrationMethod.PROJECTED_RK4, 0.01, number_of_steps),
        SensorSchedules(
            SensorSchedule(0.01, 0),
            SensorSchedule(0.01, 0),
            SensorSchedule(0.05, 0),
            SensorSchedule(0.1, 0),
        ),
        ConstantRotorSpeedInput(np.full(4, sqrt(9.81 / 4e-5))),
        seed,
        (),
    )


def replay_configuration(scenario: str, run: RunConfiguration) -> EskfReplayConfiguration:
    """Build an independent prior from the declared analytic case, never a truth row."""
    velocity_W, rate = _motion(scenario)
    dt = run.numerics.truth_time_step_s
    imu = run.nominal.imu
    anchor = EskfNominalState(
        velocity_W * dt,
        velocity_W,
        np.array([cos(rate * dt / 2), 0, 0, sin(rate * dt / 2)]),
        imu.initial_accelerometer_bias_B,
        imu.initial_gyroscope_bias_B,
    )
    rng = np.random.Generator(
        np.random.PCG64(
            np.random.SeedSequence(
                [0x45534B46, 1, SCENARIOS.index(scenario), run.root_seed], pool_size=4
            )
        )
    )
    standard_deviations = np.repeat(PRIOR_BLOCK_STANDARD_DEVIATIONS, 3)
    error = rng.standard_normal(15) * standard_deviations
    prior_variances = standard_deviations**2
    # The generator advances biases once before the first acquisition at dt.
    prior_variances[9:12] += imu.accelerometer_bias_random_walk_density_B**2 * dt
    prior_variances[12:15] += imu.gyroscope_bias_random_walk_density_B**2 * dt
    return eskf_replay_configuration_from_nominal(
        run.nominal,
        initial_time_s=dt,
        initial_state=inject_eskf_error_state(anchor, -error),
        initial_covariance=np.diag(prior_variances),
        continuous_noise_covariance=sampled_imu_continuous_noise_covariance(imu, dt),
        innovation_policy=EskfInnovationPolicy(),
    )


def _manifest(run: RunConfiguration, provenance: SoftwareProvenance) -> dict[str, object]:
    return cast(dict[str, object], json.loads(encode_run_manifest(RunManifest(run, provenance))))


def study_protocol(partition: str, provenance: SoftwareProvenance) -> dict[str, object]:
    """Return the entire predeclared protocol, excluding machine/time/source metadata."""
    if partition not in PARTITIONS:
        raise ValueError(f"unknown partition: {partition}")
    seeds, steps = PARTITIONS[partition]
    configs = {
        name: _manifest(run_configuration(name, 0, steps), provenance)["run_configuration"]
        for name in SCENARIOS
    }
    return {
        "name": "nominal_eskf_consistency",
        "version": 1,
        "partition": partition,
        "seeds": list(seeds),
        "number_of_steps": steps,
        "scenarios": list(SCENARIOS),
        "configuration_templates_root_seed_0": configs,
        "prior_block_standard_deviations": list(PRIOR_BLOCK_STANDARD_DEVIATIONS),
        "prior_block_units": list(BLOCK_UNITS),
        "prior_randomness": {
            "generator": "PCG64",
            "seed_sequence_pool_size": 4,
            "entropy": [0x45534B46, 1, "scenario_index", "root_seed"],
            "normal_draws": 15,
        },
        "prior_epoch": (
            "dt; analytic pose/velocity and initial bias anchors minus independent Gaussian error"
        ),
        "initial_bias_covariance": "prior_stddev^2 + random_walk_density^2 * dt",
        "continuous_noise_diagonal": "[sigma_a^2*dt, sigma_g^2*dt, rw_a^2, rw_g^2]",
        "innovation_policy": {
            "local_position_nis_threshold": None,
            "barometric_altitude_nis_threshold": None,
        },
        "reference_alignment": (
            "truth/bias rows 1..N exactly paired to post-update replay rows 0..N-1"
        ),
        "error_order": "[p_W, v_W, Log(R_est_WB.T R_true_WB), b_a_B, b_g_B]",
        "nees_degrees_of_freedom": 15,
        "nis_degrees_of_freedom": {"local_position": 3, "barometric_altitude": 1},
        "central_probability": 0.95,
        "coverage_investigation_band": [0.90, 0.98],
        "divergence_limits": {
            "position_m": POSITION_DIVERGENCE_M,
            "attitude_rad": ATTITUDE_DIVERGENCE_RAD,
            "comparison": "strictly greater at any epoch",
        },
        "dead_reckoning": "same prior and input; both observation fusion flags false",
        "failure_policy": (
            "retain every seed; suppress ensemble statistics if any numerical trial fails"
        ),
        "interpretation": (
            "independent seeds at each epoch; temporal summaries descriptive; "
            "scenarios not independent of each other"
        ),
    }


def error_metrics(error_states: NDArray[np.float64]) -> dict[str, object]:
    """Physical three-vector RMS norms; no sum across incompatible physical units."""
    if (
        error_states.ndim != 2
        or error_states.shape[1] != 15
        or not error_states.shape[0]
        or not np.all(np.isfinite(error_states))
    ):
        raise ValueError("error_states must be finite nonempty (epochs,15)")
    norms = np.array([[hypot(*row[i : i + 3]) for i in range(0, 15, 3)] for row in error_states])
    if not np.all(np.isfinite(norms)):
        raise ValueError("error norms must remain finite")
    maxima = norms.max(axis=0)
    normalized = np.divide(norms, maxima, out=np.zeros_like(norms), where=maxima != 0)
    rmse = maxima * np.sqrt(np.mean(normalized**2, axis=0))
    return {
        "rmse": {name: float(value) for name, value in zip(BLOCK_UNITS, rmse, strict=True)},
        "final_accelerometer_bias_error_m_per_s2": float(norms[-1, 3]),
        "final_gyroscope_bias_error_rad_per_s": float(norms[-1, 4]),
        "max_position_m": float(maxima[0]),
        "max_attitude_rad": float(maxima[2]),
        "diverged": bool(maxima[0] > POSITION_DIVERGENCE_M or maxima[2] > ATTITUDE_DIVERGENCE_RAD),
    }


@dataclass(frozen=True)
class _InnovationSeries:
    time_s: NDArray[np.float64]
    scores: NDArray[np.float64]
    counts: dict[str, int]

    def to_mapping(self) -> dict[str, object]:
        return {
            "time_s": self.time_s.tolist(),
            "scores": self.scores.tolist(),
            "counts": self.counts,
        }


def innovation_series(result: EskfReplayResult) -> dict[str, _InnovationSeries]:
    """Retain all scored pre-update NIS, including REJECTED; count every disposition."""
    series: dict[str, _InnovationSeries] = {}
    for name, kind in KINDS.items():
        events = [e for e in result.events if e.observation.kind is kind]
        scored = [e for e in events if e.innovation is not None]
        counts = {
            status.value: sum(e.status is status for e in events) for status in EskfReplayStatus
        }
        counts["unscored"] = len(events) - len(scored)
        counts["total"] = len(events)
        series[name] = _InnovationSeries(
            np.array([result.time_s[e.observation.acquisition_index] for e in scored]),
            np.array(
                [
                    e.innovation.normalized_innovation_squared
                    for e in scored
                    if e.innovation is not None
                ]
            ),
            counts,
        )
    return series


@dataclass(frozen=True)
class _Trial:
    report: dict[str, object]
    consistency: EskfConsistencyHistory | None = None
    nis: dict[str, _InnovationSeries] | None = None


def _run_trial(scenario: str, seed: int, steps: int, provenance: SoftwareProvenance) -> _Trial:
    run = run_configuration(scenario, seed, steps)
    config = replay_configuration(scenario, run)
    report: dict[str, object] = {
        "seed": seed,
        "run_manifest": _manifest(run, provenance),
        "initial_state": {
            name: getattr(config.initial_state, name).tolist()
            for name in (
                "position_W",
                "velocity_W",
                "q_WB",
                "accelerometer_bias_B",
                "gyroscope_bias_B",
            )
        },
        "initial_covariance": config.initial_covariance.tolist(),
        "continuous_noise_covariance": config.continuous_noise_covariance.tolist(),
    }
    stage = "generation"
    try:
        artifact = generate_run_artifact_data(run)
        stage = "measurement_adapter"
        measured = eskf_replay_input_from_run_artifact(artifact)
        stage = "fused_replay"
        fused = replay_eskf(measured, config)
        stage = "dead_reckoning_replay"
        dead = replay_eskf(
            measured, replace(config, fuse_local_position=False, fuse_barometric_altitude=False)
        )
        # Truth access for evaluation begins only after both measurement-only replays.
        stage = "reference_and_consistency"
        reference = eskf_reference_from_run_artifact(artifact)
        consistency = evaluate_eskf_replay(fused, reference)
        dead_consistency = evaluate_eskf_replay(dead, reference)
        nis = innovation_series(fused)
        stage = "metrics"
        fused_metrics, dead_metrics = (
            error_metrics(consistency.error_states),
            error_metrics(dead_consistency.error_states),
        )
        report.update(
            {
                "status": "completed",
                "time_s": consistency.time_s.tolist(),
                "nees": consistency.nees.tolist(),
                "fused_metrics": fused_metrics,
                "dead_reckoning_metrics": dead_metrics,
                "nis": {name: series.to_mapping() for name, series in nis.items()},
            }
        )
        return _Trial(report, consistency, nis)
    except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
        report.update(
            {
                "status": "numerical_failure",
                "failure_stage": stage,
                "error": f"{type(error).__name__}: {error}",
            }
        )
        return _Trial(report)


def _score_summary(
    samples: NDArray[np.float64], degrees: int, time_s: NDArray[np.float64]
) -> dict[str, object]:
    scores = NormalizedErrorEnsemble(samples, degrees)
    lower, upper = scores.mean_interval_95
    return {
        "degrees_of_freedom": degrees,
        "seed_count": samples.shape[0],
        "time_s": time_s.tolist(),
        "individual_interval_95": list(scores.individual_interval_95),
        "pointwise_mean_interval_95": list(scores.mean_interval_95),
        "mean_by_epoch": scores.mean_by_epoch.tolist(),
        "coverage_by_epoch": scores.coverage_by_epoch.tolist(),
        "descriptive_mean": scores.descriptive_mean,
        "descriptive_coverage": scores.descriptive_coverage,
        "descriptive_fraction_of_epoch_means_in_band": float(
            np.mean((scores.mean_by_epoch >= lower) & (scores.mean_by_epoch <= upper))
        ),
        "coverage_in_investigation_band": 0.90 <= scores.descriptive_coverage <= 0.98,
    }


def _ensemble(trials: list[_Trial]) -> dict[str, object] | None:
    if any(t.consistency is None or t.nis is None for t in trials):
        return None
    histories = [t.consistency for t in trials if t.consistency is not None]
    first = histories[0]
    if any(not np.array_equal(h.time_s, first.time_s) for h in histories):
        raise ValueError("ensemble NEES epochs must match exactly")
    result: dict[str, object] = {
        "nees": _score_summary(np.stack([h.nees for h in histories]), 15, first.time_s)
    }
    for name in KINDS:
        series = [t.nis[name] for t in trials if t.nis is not None]
        if any(not np.array_equal(s.time_s, series[0].time_s) for s in series):
            raise ValueError(
                "ensemble NIS epochs must match exactly, without acceptance conditioning"
            )
        result[name + "_nis"] = (
            None
            if not series[0].scores.size
            else _score_summary(
                np.stack([s.scores for s in series]),
                3 if name == "local_position" else 1,
                series[0].time_s,
            )
        )
    for mode in ("fused", "dead_reckoning"):
        metrics = [cast(dict[str, object], t.report[mode + "_metrics"]) for t in trials]
        rmses = [cast(dict[str, float], m["rmse"]) for m in metrics]
        result[mode + "_rmse_distribution"] = {
            name: {
                "mean": float(np.mean([r[name] for r in rmses])),
                "median": float(np.median([r[name] for r in rmses])),
                "p95": float(np.quantile([r[name] for r in rmses], 0.95)),
                "maximum": max(r[name] for r in rmses),
            }
            for name in BLOCK_UNITS
        }
    return result


def run_study(partition: str, provenance: SoftwareProvenance) -> dict[str, object]:
    """Run every fixed seed; report failures without omitting them from the record."""
    protocol = study_protocol(partition, provenance)
    seeds, steps = PARTITIONS[partition]
    scenarios: list[dict[str, object]] = []
    failure_count = divergence_count = dead_divergence_count = 0
    for name in SCENARIOS:
        trials = [_run_trial(name, seed, steps, provenance) for seed in seeds]
        failures = sum(t.consistency is None for t in trials)
        failure_count += failures
        for t in trials:
            if t.consistency is not None:
                divergence_count += bool(
                    cast(dict[str, object], t.report["fused_metrics"])["diverged"]
                )
                dead_divergence_count += bool(
                    cast(dict[str, object], t.report["dead_reckoning_metrics"])["diverged"]
                )
        scenarios.append(
            {
                "name": name,
                "planned_trial_count": len(seeds),
                "numerical_failure_count": failures,
                "trials": [t.report for t in trials],
                "ensemble": _ensemble(trials),
            }
        )
    return {
        "report_schema": {"name": "eskf_consistency_evaluation", "version": 1},
        "protocol": protocol,
        "protocol_sha256": protocol_sha256(protocol),
        "software_provenance": asdict(provenance),
        "numerical_failure_count": failure_count,
        "divergence_count": divergence_count,
        "dead_reckoning_divergence_count": dead_divergence_count,
        "scenarios": scenarios,
    }


def _source_sha256(root: Path) -> str:
    paths = sorted(
        [
            *root.glob("src/quadrotor_math/*.py"),
            *root.glob("experiments/*.py"),
            root / "pyproject.toml",
            root / "uv.lock",
        ]
    )
    digest = hashlib.sha256()
    for path in paths:
        content = path.read_bytes()
        digest.update(path.relative_to(root).as_posix().encode() + b"\0")
        digest.update(str(len(content)).encode() + b"\0" + content)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", choices=tuple(PARTITIONS), default="smoke")
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="new JSON path; existing files are never overwritten",
    )
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    root = Path(__file__).resolve().parents[1]
    provenance = capture_software_provenance(root)
    protocol = study_protocol(args.partition, provenance)
    source_hash = _source_sha256(root)
    print(f"Frozen protocol SHA256: {protocol_sha256(protocol)}", file=sys.stderr, flush=True)
    report = run_study(args.partition, provenance)
    if source_hash != _source_sha256(root):
        raise RuntimeError("source files changed during evaluation")
    report.update({"generated_at_utc": datetime.now(UTC).isoformat(), "source_sha256": source_hash})
    with args.output.open("xb") as output:
        output.write(canonical_json(report) + b"\n")
    print(
        f"Numerical failures: {report['numerical_failure_count']}; "
        f"fused divergence flags: {report['divergence_count']}; report: {args.output}",
        file=sys.stderr,
    )
    return int(bool(report["numerical_failure_count"] or report["divergence_count"]))


if __name__ == "__main__":
    raise SystemExit(main())
