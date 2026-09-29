"""Exact supported conditioning and an independent ballistic release check."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.supported_start import configuration_for
from experiments.supported_validation_protocol import Job, prepare_job
from experiments.supported_velocity_prior import (
    SupportedVelocityConditioner,
    VelocitySupport,
    boundary_screen,
    fixture_velocity_support,
)
from quadrotor_math.eskf_endpoint import EskfEndpointState, EskfSampledImuNoise
from quadrotor_math.rotations import rotation_matrix_body_to_world, skew_symmetric


@pytest.fixture(scope="module")
def prepared():
    return prepare_job(Job("nominal_hover", 47831), "smoke")


def condition(prepared, release=None, support=None):
    return SupportedVelocityConditioner().condition(
        prepared.release if release is None else release,
        fixture_velocity_support(prepared) if support is None else support,
        now_s=0.5025,
    )


def test_exact_conditioning_preserves_every_other_joint_block_and_sample(prepared):
    old = prepared.release
    new = condition(prepared)
    C = old.endpoint.joint_covariance
    H = np.eye(21)[3:6]
    expected = C - C @ H.T @ np.linalg.solve(H @ C @ H.T, H @ C)
    np.testing.assert_array_equal(new.endpoint.joint_covariance, expected)
    assert np.linalg.matrix_rank(C) == 21
    assert np.linalg.matrix_rank(new.endpoint.joint_covariance) == 18
    assert np.linalg.eigvalsh(new.endpoint.joint_covariance).min() >= -1e-15
    keep = np.r_[0:3, 6:21]
    np.testing.assert_array_equal(
        new.endpoint.joint_covariance[np.ix_(keep, keep)], C[np.ix_(keep, keep)]
    )
    for field in old.endpoint.nominal_state.__dataclass_fields__:
        np.testing.assert_array_equal(
            getattr(new.endpoint.nominal_state, field), getattr(old.endpoint.nominal_state, field)
        )
    np.testing.assert_array_equal(new.endpoint.imu_noise_mean_B, old.endpoint.imu_noise_mean_B)
    np.testing.assert_array_equal(
        new.fresh_sample.specific_force_B, old.fresh_sample.specific_force_B
    )
    assert new.identity == old.identity and new.support == old.support
    assert new.aligned_sample_ids == old.aligned_sample_ids
    assert not np.shares_memory(new.endpoint.joint_covariance, C)
    assert not new.endpoint.joint_covariance.flags.writeable
    assert C[3, 3] == pytest.approx(0.03**2)


@pytest.mark.parametrize("valid", [False, True])
def test_conditioner_is_one_shot_even_after_rejection(prepared, valid):
    once = SupportedVelocityConditioner()
    support = fixture_velocity_support(prepared) if valid else None
    if valid:
        once.condition(prepared.release, support, now_s=0.5025)
    else:
        with pytest.raises(ValueError):
            once.condition(prepared.release, support, now_s=0.5025)
    with pytest.raises(ValueError, match="consumed"):
        once.condition(prepared.release, fixture_velocity_support(prepared), now_s=0.5025)


@pytest.mark.parametrize(
    "change",
    [
        "moving",
        "revoked",
        "stale",
        "identity",
        "motors",
        "acceleration",
        "rotation",
        "mechanical",
        "clock",
        "sample",
        "window",
        "profile",
        "units",
    ],
)
def test_invalid_or_unsupported_release_is_rejected(prepared, change):
    release = prepared.release
    support = fixture_velocity_support(prepared)
    if change == "moving":
        support = replace(support, zero_world_velocity=False)
    elif change in (
        "revoked",
        "stale",
        "identity",
        "motors",
        "acceleration",
        "rotation",
        "mechanical",
    ):
        edits = {
            "revoked": dict(revoked=True),
            "stale": dict(observed_through_s=0.5),
            "identity": dict(acquisition_id="different"),
            "motors": dict(motors_off=False),
            "acceleration": dict(zero_world_acceleration=False),
            "rotation": dict(zero_angular_velocity=False),
            "mechanical": dict(mechanically_supported=False),
        }
        changed = replace(release.support, **edits[change])
        release = replace(release, support=changed)
        support = VelocitySupport(changed, True)
    elif change == "window":
        release = replace(release, aligned_sample_ids=(0, 199))
    else:
        edits = {
            "clock": dict(clock_id="different"),
            "sample": dict(sample_id=200),
            "profile": dict(profile_id="different"),
            "units": dict(units="wrong"),
        }
        release = replace(release, fresh_sample=replace(release.fresh_sample, **edits[change]))
    with pytest.raises(ValueError):
        condition(prepared, release, support)


@pytest.mark.parametrize("now", [0.5, 0.5026, np.nan, np.inf, True])
def test_stale_future_or_invalid_clock_is_rejected(prepared, now):
    with pytest.raises(ValueError):
        SupportedVelocityConditioner().condition(
            prepared.release, fixture_velocity_support(prepared), now_s=now
        )


@pytest.mark.parametrize(
    "change",
    ["mean", "position_cross", "bias_cross", "sample_cross", "noise_mean", "already_conditioned"],
)
def test_unsupported_prior_is_rejected(prepared, change):
    release = prepared.release
    e = release.endpoint
    C = e.joint_covariance.copy()
    if change == "mean":
        e = replace(e, nominal_state=replace(e.nominal_state, velocity_W=np.ones(3)))
    elif change == "noise_mean":
        e = replace(e, imu_noise_mean_B=np.ones(6))
    else:
        if change == "already_conditioned":
            C[3:6, 3:6] = 0
        else:
            j = {"position_cross": 0, "bias_cross": 9, "sample_cross": 15}[change]
            C[3, j] = C[j, 3] = 1e-9
        e = replace(e, joint_covariance=C)
    with pytest.raises(ValueError):
        condition(prepared, replace(release, endpoint=e))


def test_fixture_assertion_rejects_world_motion_even_with_zero_acceleration(prepared):
    evidence = {k: v.copy() for k, v in prepared.evidence.items()}
    evidence["velocity_W"][:, 0] = 0.1
    support = fixture_velocity_support(replace(prepared, evidence=evidence))
    assert not support.zero_world_velocity
    with pytest.raises(ValueError):
        condition(prepared, support=support)


def test_ballistic_counterexample_and_variance_have_independent_closed_forms(prepared):
    noise = configuration_for(prepared, "aligned")["estimator_configuration"].sampled_imu_noise
    old, new = prepared.release, condition(prepared)
    baseline, candidate = (boundary_screen(x, noise) for x in (old, new))
    h, g = 0.0025, 9.81
    for result in (baseline, candidate):
        np.testing.assert_allclose(result["velocity_error_W"], [0, 0, -g * h / 2], atol=1e-15)
        np.testing.assert_allclose(result["position_error_W"], [0, 0, -g * h * h / 3], atol=1e-15)
        assert result["down_variance"] == pytest.approx(
            result["independent_down_variance"], abs=1e-18
        )
    assert baseline["bias_inside_99_radius"]
    assert not candidate["bias_inside_99_radius"]
    assert candidate["down_bias_sigma_ratio"] > 100
    assert baseline["down_variance"] - candidate["down_variance"] == pytest.approx(
        0.03**2, abs=1e-18
    )


def test_zero_uncertainty_cannot_hide_deterministic_release_error(prepared):
    release = prepared.release
    exact = EskfEndpointState(release.endpoint.nominal_state, np.zeros(6), np.zeros((21, 21)))
    result = boundary_screen(
        replace(release, endpoint=exact), EskfSampledImuNoise(np.zeros((6, 6)), np.zeros((6, 6)))
    )
    assert result["down_variance"] == 0
    assert result["down_bias_sigma_ratio"] is None
    assert not result["bias_inside_99_radius"]


def test_complete_ballistic_covariance_from_independent_specialized_derivatives(prepared):
    release = condition(prepared)
    noise = configuration_for(prepared, "aligned")["estimator_configuration"].sampled_imu_noise
    result = boundary_screen(release, noise)
    h = 0.0025
    R = rotation_matrix_body_to_world(release.endpoint.nominal_state.q_WB)
    L = -R @ skew_symmetric(-R.T @ np.array([0.0, 0.0, 9.81]))
    A, B = np.zeros((21, 21)), np.zeros((21, 12))
    A[:15, :15] = np.eye(15)
    A[:3, 3:6] = h * np.eye(3)
    A[:3, 6:9], A[3:6, 6:9] = h**2 / 3 * L, h / 2 * L
    A[:3, 9:12], A[3:6, 9:12] = -(h**2) / 2 * R, -h * R
    A[:3, 15:18], A[3:6, 15:18] = -(h**2) / 3 * R, -h / 2 * R
    A[6:9, 12:15], A[6:9, 18:21] = -h * np.eye(3), -h / 2 * np.eye(3)
    for column in (0, 6):
        B[:3, column : column + 3] = -(h**2) / 6 * R
        B[3:6, column : column + 3] = -h / 2 * R
        B[6:9, column + 3 : column + 6] = -h / 2 * np.eye(3)
    B[9:15, 6:12] = np.eye(6)
    B[15:21, :6] = np.eye(6)
    D = np.zeros((12, 12))
    D[:6, :6] = noise.sample_covariance_B
    D[6:, 6:] = h * noise.bias_walk_spectral_density_B
    expected = A @ release.endpoint.joint_covariance @ A.T + B @ D @ B.T
    np.testing.assert_allclose(
        result["predicted"]["joint_covariance"], expected, atol=1e-18, rtol=1e-12
    )
    assert np.linalg.norm(expected[:15, 15:]) > 0  # New sample memory is retained.
