"""Frozen joint calibration and flight no-regression decisions, without campaigns."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.nonlinear_release import structural_factor
from experiments.nonlinear_release_campaign import clean_comparison, modes, summarize
from experiments.nonlinear_release_validation import accuracy, calibration
from experiments.supported_validation_protocol import jobs
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_endpoint import EskfEndpointState


def endpoint():
    state = EskfNominalState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )
    return EskfEndpointState(state, np.zeros(6), np.eye(21))


def test_accuracy_compares_full_covariance_and_mean():
    old = endpoint()
    assert accuracy(old, old)["passed"]
    assert not accuracy(replace(old, joint_covariance=old.joint_covariance * 1.01), old)["passed"]
    moved = replace(
        old, nominal_state=replace(old.nominal_state, velocity_W=np.array([1e-5, 0, 0]))
    )
    assert not accuracy(moved, old)["passed"]


def test_joint_calibration_rejects_hidden_cross_correlation():
    errors = np.r_[np.eye(21), -np.eye(21)] * np.sqrt(41 / 2)
    assert calibration(errors, np.eye(21))["passed"]
    wrong = np.eye(21)
    wrong[0, 1] = wrong[1, 0] = 0.2
    assert not calibration(errors, wrong)["passed"]
    assert not calibration(errors + 0.1, np.eye(21))["passed"]
    with pytest.raises(np.linalg.LinAlgError):
        calibration(errors, np.diag([0.0] + [1.0] * 20))


def test_tiny_positive_latent_variance_is_never_dropped():
    root, active = structural_factor(np.diag([0.0, 1e-30, 1.0]))
    assert active.tolist() == [1, 2]
    np.testing.assert_allclose(root @ root.T, np.diag([0.0, 1e-30, 1.0]), rtol=1e-15, atol=0)
    with pytest.raises(ValueError, match="singularity"):
        structural_factor(np.ones((2, 2)))


def test_any_flight_or_no_regression_failure_rejects_comparison():
    old = dict(flight_passed=True, tracking_rmse_m=0.06, hover_peak_m=0.07)
    assert clean_comparison("nominal_hover", old, old)["passed"]
    for field, value in [
        ("flight_passed", False),
        ("tracking_rmse_m", 0.060001),
        ("hover_peak_m", 0.070001),
    ]:
        assert not clean_comparison("nominal_hover", old, {**old, field: value})["passed"]


def test_exact_25_flights_are_separate_from_adoption():
    rows = []
    for job in jobs():
        rows.append(
            dict(
                name=job.name,
                case=job.case,
                modes={
                    mode: dict(candidate_metrics=dict(flight_passed=True)) for mode in modes(job)
                },
                comparison=dict(passed=True),
            )
        )
    result = summarize(rows)
    assert result["scientific_flights"] == 25 and result["frozen_comparison_passed"]
    assert not result["adopted_as_default"] and not result["zero_velocity_conditioning"]
    rows[0]["comparison"]["passed"] = False
    assert not summarize(rows)["frozen_comparison_passed"]
    with pytest.raises(ValueError):
        summarize(rows[:-1])
