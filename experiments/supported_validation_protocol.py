"""ADR 0031 fixed independent seeds, causal pair types and inherited scoring."""

from collections import defaultdict
from dataclasses import dataclass, replace
from typing import Any
from unittest.mock import patch

import numpy as np

from experiments import robustness_validation
from experiments.estimated_feedback_validation import make_configuration
from experiments.robustness_evidence import ensure
from experiments.robustness_protocol import configuration as original_configuration
from experiments.robustness_protocol import faults, policies
from experiments.supported_start import PreparedStart, configuration_for, fly, prepare_configured
from quadrotor_math.eskf_replay import EskfReplayEvent
from quadrotor_math.estimated_mission import EstimatedMissionResult
from quadrotor_math.observation_health import ObservationHealthMonitor

CLEAN = ("nominal_hover", "nominal_tracking", "wind_tracking")
FAULTS = (
    "position_dropout",
    "position_rejection",
    "position_delay",
    "altitude_dropout",
    "altitude_rejection",
    "altitude_delay",
    "position_recovery",
    "landing_position_dropout",
)


@dataclass(frozen=True)
class Job:
    case: str
    seed: int

    def __post_init__(self) -> None:
        if self.case not in CLEAN + FAULTS or type(self.seed) is not int or self.seed < 0:
            raise ValueError("invalid independent supported-start job")

    @property
    def name(self) -> str:
        return f"{self.case}-{self.seed}"


def jobs() -> tuple[Job, ...]:
    return tuple(Job(case, seed) for seed in (47001, 47002, 47003) for case in CLEAN) + tuple(
        Job(case, 47004) for case in FAULTS
    )


def modes(job: Job) -> tuple[str, str]:
    return ("unaligned", "aligned") if job.case in CLEAN else ("off", "on")


def configuration(job: Job, partition: str) -> dict[str, Any]:
    args = original_configuration(job.case, partition)
    seeded = make_configuration(
        dict(case="smoke" if partition == "smoke" else "hover", seed=job.seed, noiseless=False)
    )
    args["initial_state"] = seeded["initial_state"]
    args["sensors"] = replace(args["sensors"], imu=seeded["sensors"].imu, root_seed=job.seed)
    return args


def prepare_job(job: Job, partition: str = "campaign") -> PreparedStart:
    return prepare_configured(job.case, configuration(job, partition), partition)


def alignment_mode(mode: str) -> str:
    ensure(mode in ("unaligned", "aligned", "off", "on"), "unknown pair mode")
    return mode if mode in ("unaligned", "aligned") else "aligned"


def execute(prepared: PreparedStart, mode: str) -> tuple[EstimatedMissionResult, dict[str, Any]]:
    expected = ("unaligned", "aligned") if prepared.case in CLEAN else ("off", "on")
    ensure(mode in expected, "mode does not match causal pair")
    args = configuration_for(prepared, alignment_mode(mode))
    return fly(
        prepared,
        alignment_mode(mode),
        faults=faults(prepared.case, prepared.partition, args),
        supervised=mode != "off",
    )


def clean_comparison(
    case: str, unaligned: dict[str, Any], aligned: dict[str, Any]
) -> dict[str, Any]:
    conditions = {"aligned_flight": aligned["flight_passed"]}
    conditions.update(
        {
            "retains_" + key: aligned["flight_conditions"].get(key, False)
            for key, passed in unaligned["flight_conditions"].items()
            if passed
        }
    )
    if case == "nominal_hover":
        peak, reference = aligned.get("hover_peak_m"), unaligned.get("hover_peak_m")
        conditions["hover_no_regression"] = (
            peak is not None and reference is not None and peak <= reference + 1e-12
        )
    return dict(conditions=conditions, passed=all(conditions.values()))


def response_metrics(
    prepared: PreparedStart,
    results: dict[str, EstimatedMissionResult],
    diagnostics: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    args = configuration_for(prepared, "aligned")
    health = {}
    for mode, result in results.items():
        monitor = ObservationHealthMonitor(policies(args, prepared.partition)[0])
        by_epoch: dict[int, list[EskfReplayEvent]] = defaultdict(list)
        for event in result.estimates.events:
            if event.observation.delivery_index != -1:
                by_epoch[event.observation.delivery_index].append(event)
        for k, time in enumerate(result.mission.time_s):
            monitor.step(float(time), tuple(by_epoch[k]))
        health[mode] = monitor
    with patch.object(robustness_validation, "configuration", lambda _c, _p: args):
        return robustness_validation.score_pair(
            prepared.case, prepared.partition, results, health, diagnostics
        )


def paired_initialization(prepared: PreparedStart) -> None:
    """Check causal isolation before either clean arm runs."""
    from experiments.supported_start import plain_configuration

    first, second = (configuration_for(prepared, m) for m in ("unaligned", "aligned"))
    for key in first:
        if key != "estimator_configuration":
            ensure(
                plain_configuration(first[key]) == plain_configuration(second[key]),
                "pair physics differ",
            )
    for field in ("position_W", "velocity_W"):
        ensure(
            np.array_equal(
                getattr(first["estimator_configuration"].initial_state, field),
                getattr(second["estimator_configuration"].initial_state, field),
            ),
            "navigation mean differs",
        )
    ensure(
        np.array_equal(
            first["estimator_configuration"].initial_covariance[:6],
            second["estimator_configuration"].initial_covariance[:6],
        ),
        "navigation covariance differs",
    )


def common_bias_prefix(results: dict[str, EstimatedMissionResult]) -> None:
    first, second = results.values()
    n = min(len(first.mission.time_s), len(second.mission.time_s))
    for field in ("true_accelerometer_bias_B", "true_gyroscope_bias_B"):
        ensure(
            np.array_equal(getattr(first, field)[:n], getattr(second, field)[:n]),
            "paired bias draws differ",
        )
    # Each arm's white noise is independently reconstructed against identical
    # generators by the boundary audit, including the exhaustive slow-sensor ledger.
    for result in results.values():
        ensure(len(result.mission.time_s) > 1, "empty flight history")
