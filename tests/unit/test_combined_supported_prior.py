"""One supported conditioning step, causal first prediction, and two controls."""

from dataclasses import replace
from unittest.mock import patch

import numpy as np
import pytest

from experiments import release_prediction
from experiments.combined_supported_prior import (
    combined_comparison,
    combined_prediction,
    condition_start,
    summarize,
    validate_initialization,
)
from experiments.nonlinear_release_campaign import modes
from experiments.nonlinear_release_validation import initial_posterior
from experiments.supported_start import audit, configuration_for, fly, plain_configuration, prepare
from experiments.supported_validation_protocol import jobs
from quadrotor_math import eskf_online, eskf_replay


@pytest.fixture(scope="module")
def original():
    return prepare("nominal_hover", "smoke")


def test_conditioning_changes_only_initial_velocity_covariance(original):
    saved = plain_configuration(original.release)
    candidate = condition_start(original)
    validate_initialization(original, candidate)
    assert plain_configuration(original.release) == saved
    before, after = original.release.endpoint, candidate.release.endpoint
    expected = before.joint_covariance.copy()
    expected[3:6, :] = expected[:, 3:6] = 0
    np.testing.assert_array_equal(after.joint_covariance, expected)
    assert plain_configuration(before.nominal_state) == plain_configuration(after.nominal_state)
    assert plain_configuration(candidate.release.fresh_sample) == plain_configuration(
        original.release.fresh_sample
    )
    assert candidate.evidence is original.evidence
    with pytest.raises(ValueError, match="positive definite"):
        condition_start(candidate)


@pytest.mark.parametrize("fault", ["missing", "moving", "revoked", "stale"])
def test_invalid_support_never_enters_a_flight(original, fault):
    broken = replace(original)
    if fault == "missing":
        broken.release = None
    elif fault == "moving":
        broken.evidence = {
            **original.evidence,
            "velocity_W": np.ones_like(original.evidence["velocity_W"]),
        }
    elif fault == "revoked":
        broken.release = replace(
            original.release, support=replace(original.release.support, revoked=True)
        )
    else:
        broken.release = replace(
            original.release, fresh_sample=replace(original.release.fresh_sample, time_s=9.0)
        )
    with pytest.raises(ValueError):
        condition_start(broken)


def test_prior_or_configuration_mixup_rejected_before_prediction(original):
    with pytest.raises(ValueError, match="conditioned release"):
        validate_initialization(original, original)
    candidate = condition_start(original)
    candidate.original = {
        **candidate.original,
        "initial_state": replace(candidate.original["initial_state"], position_W=np.ones(3)),
    }
    with pytest.raises(ValueError, match="configuration"):
        validate_initialization(original, candidate)


def test_complete_online_replay_preserves_causal_prior_and_new_sample_memory(original):
    candidate = condition_start(original)
    ordinary = release_prediction.predict_eskf_endpoint
    with patch.object(release_prediction, "predict_eskf_endpoint", wraps=ordinary) as later:
        with combined_prediction(original, candidate, "online") as online:
            result, diagnostic = fly(candidate, "aligned")
    assert later.call_count == len(result.mission.time_s) - 2
    with combined_prediction(original, candidate, "replay") as replay:
        audit(candidate, "aligned", result, diagnostic)
    assert online == replay
    assert online["zero_velocity_conditioning"]
    assert online["routing"]["release_predictions"] == 1
    inputs = dict(
        observations=tuple(
            e.observation for e in result.estimates.events if e.observation.delivery_index == 0
        ),
        initial_estimate={
            key: getattr(result.estimates.states[0], key)
            for key in original.release.endpoint.nominal_state.__dataclass_fields__
        },
    )
    expected = initial_posterior(original, configuration_for(original, "aligned"), inputs, True)
    assert online["input"] == plain_configuration(expected)
    output_covariance = np.array(online["output"]["joint_covariance"])
    assert np.linalg.matrix_rank(output_covariance) == 21
    assert np.any(output_covariance[3:6, 15:])
    assert np.linalg.norm(online["output"]["nominal_state"]["velocity_W"]) > 0


@pytest.mark.parametrize("target,module", [("online", eskf_online), ("replay", eskf_replay)])
def test_prediction_context_restores_on_exception(original, target, module):
    before = module.predict_eskf_endpoint
    candidate = condition_start(original)
    with pytest.raises(RuntimeError, match="injected"):
        with combined_prediction(original, candidate, target):
            raise RuntimeError("injected")
    assert module.predict_eskf_endpoint is before


def test_acceptance_requires_both_controls_and_every_absolute_limit():
    control = dict(flight_passed=True, tracking_rmse_m=0.06, hover_peak_m=0.07)
    better = {**control, "tracking_rmse_m": 0.05, "hover_peak_m": 0.06}
    assert combined_comparison("nominal_hover", control, control, better)["passed"]
    for index in (0, 1):
        controls = [control, control]
        controls[index] = {**control, "hover_peak_m": 0.055}
        result = combined_comparison("nominal_hover", *controls, better)
        assert not result["passed"]
        controls[index] = {**control, "tracking_rmse_m": 0.045}
        assert not combined_comparison("nominal_hover", *controls, better)["passed"]
    assert not combined_comparison(
        "nominal_hover", control, control, {**better, "flight_passed": False}
    )["passed"]


def test_25_flight_ledger_never_means_default_adoption():
    rows = [
        dict(
            name=j.name,
            case=j.case,
            modes={m: dict(candidate_metrics=dict(flight_passed=True)) for m in modes(j)},
            comparison=dict(passed=True),
        )
        for j in jobs()
    ]
    summary = summarize(rows)
    assert summary["scientific_flights"] == 25 and summary["zero_velocity_conditioning"]
    assert summary["frozen_comparison_passed"]
    assert not summary["adopted_as_default"] and not summary["fresh_validation_complete"]
    rows[0]["comparison"]["passed"] = False
    assert not summarize(rows)["frozen_comparison_passed"]
    with pytest.raises(ValueError):
        summarize(rows[:-1])
