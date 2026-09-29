"""Analytic finite-noise moments and independent conditional covariance checks."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.nonlinear_release import (
    conditional_inputs,
    conditional_output_covariance,
    nonlinear_release_prediction,
    predict_nonlinear_release_endpoint,
    rotation_matrices,
    rotation_rule,
)
from experiments.release_prediction import predict_release_endpoint
from experiments.release_uncertainty import exp_batch, yaw_noise_counterexample
from experiments.supported_start import audit, configuration_for, fly
from experiments.supported_validation_protocol import Job, prepare_job
from experiments.supported_velocity_prior import (
    SupportedVelocityConditioner,
    fixture_velocity_support,
)
from quadrotor_math import eskf_online
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_endpoint import EskfEndpointState, EskfSampledImuNoise


def state():
    return EskfNominalState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )


def test_missing_yaw_noise_variances_are_recovered_without_a_floor():
    analytic = yaw_noise_counterexample()
    C = np.zeros((21, 21))
    C[8, 8] = analytic["yaw_sigma_rad"] ** 2
    R = np.zeros((6, 6))
    R[0, 0] = analytic["force_sigma_m_s2"] ** 2
    noise = EskfSampledImuNoise(R, np.zeros((6, 6)))
    old = EskfEndpointState(state(), np.zeros(6), C)
    h = 0.0025
    out = predict_nonlinear_release_endpoint(
        old, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), 9.81, noise, h
    )
    H = np.zeros((2, 21))
    H[0, 3] = H[1, 4] = 1
    H[0, 15] = h
    np.testing.assert_allclose(
        np.diag(H @ out.joint_covariance @ H.T), analytic["exact_variance"], rtol=1e-8, atol=1e-22
    )
    np.testing.assert_allclose(out.nominal_state.velocity_W, [0, 0, h * 9.81], atol=1e-16)
    assert out.joint_covariance[3, 15] == pytest.approx(
        -h * R[0, 0] * np.exp(-C[8, 8] / 2), rel=1e-12
    )


@pytest.mark.parametrize("sign", [-1.0, 1.0])
def test_no_rotation_uncertainty_recovers_full_linear_map(sign):
    C = np.zeros((21, 21))
    C[:6, :6] = np.diag(np.linspace(0.01, 0.06, 6))
    C[9:12, 9:12] = np.eye(3) * 0.001
    R = np.diag([0.002] * 3 + [0.0] * 3)
    Q = R * 0.01
    old = EskfEndpointState(replace(state(), q_WB=sign * state().q_WB), np.zeros(6), C)
    args = (
        old,
        np.array([0.0, 0, -9.81]),
        np.array([0.0, 0, 0.2]),
        np.array([0.2, -0.3, -0.1]),
        np.array([0.0, 0, 0.2]),
        9.81,
        EskfSampledImuNoise(R, Q),
        0.0025,
    )
    actual = predict_nonlinear_release_endpoint(*args)
    expected = predict_release_endpoint(*args)
    np.testing.assert_allclose(
        actual.joint_covariance, expected.joint_covariance, atol=1e-17, rtol=1e-12
    )
    for key in state().__dataclass_fields__:
        np.testing.assert_allclose(
            getattr(actual.nominal_state, key), getattr(expected.nominal_state, key), atol=1e-16
        )


def test_conditional_decomposition_preserves_correlated_covariance():
    rng = np.random.default_rng(36001)
    L = rng.normal(size=(21, 21)) * 0.01
    C = L @ L.T
    Lr = rng.normal(size=(6, 6)) * 0.02
    Lq = rng.normal(size=(6, 6)) * 0.0001
    noise = EskfSampledImuNoise(Lr @ Lr.T, Lq @ Lq.T)
    model = conditional_inputs(EskfEndpointState(state(), np.zeros(6), C), noise, 0.0025)
    Sigma = np.zeros((33, 33))
    Sigma[:21, :21] = C
    Sigma[21:27, 21:27] = noise.sample_covariance_B
    Sigma[27:, 27:] = 0.0025 * noise.bias_walk_spectral_density_B
    V = model.rotation_selector @ Sigma @ model.rotation_selector.T
    X = model.linear_selector @ Sigma @ model.rotation_selector.T
    Z = model.linear_selector @ Sigma @ model.linear_selector.T
    np.testing.assert_allclose(model.gain @ V, X, atol=1e-18)
    np.testing.assert_allclose(
        model.residual_covariance + model.gain @ V @ model.gain.T, Z, atol=1e-17
    )
    assert np.linalg.eigvalsh(model.residual_covariance).min() > 0


def test_optimized_total_covariance_matches_explicit_positive_sum():
    rng = np.random.default_rng(36002)
    L = rng.normal(size=(18, 18)) * 0.01
    D = L @ L.T
    matrices = rotation_matrices(exp_batch(rng.normal(size=(13, 3)) * 0.1))
    weights = np.arange(1.0, 14.0)
    weights /= weights.sum()
    F, G, E = rng.normal(size=(18, 18)), rng.normal(size=(18, 3)), rng.normal(size=(3, 18))
    explicit = np.zeros((18, 18))
    for w, R in zip(weights, matrices, strict=True):
        M = F - G @ R @ E
        explicit += w * M @ D @ M.T
    np.testing.assert_allclose(
        conditional_output_covariance(D, matrices, weights, F, G, E),
        explicit,
        atol=1e-15,
        rtol=1e-13,
    )


@pytest.fixture(scope="module")
def prepared():
    return prepare_job(Job("nominal_hover", 47831), "smoke")


@pytest.mark.parametrize("conditioned", [False, True])
def test_supported_moments_are_positive_and_recover_full_rank(prepared, conditioned):
    release = prepared.release
    if conditioned:
        release = SupportedVelocityConditioner().condition(
            release, fixture_velocity_support(prepared), now_s=release.fresh_sample.time_s
        )
    s = release.endpoint.nominal_state
    noise = configuration_for(prepared, "aligned")["estimator_configuration"].sampled_imu_noise
    out = predict_nonlinear_release_endpoint(
        release.endpoint,
        release.fresh_sample.specific_force_B,
        s.gyroscope_bias_B,
        s.accelerometer_bias_B,
        s.gyroscope_bias_B,
        9.81,
        noise,
        0.0025,
    )
    np.linalg.cholesky(out.joint_covariance)
    assert np.linalg.matrix_rank(out.joint_covariance) == 21
    np.testing.assert_array_equal(out.imu_noise_mean_B, np.zeros(6))
    np.testing.assert_allclose(
        out.joint_covariance[15:, 15:], noise.sample_covariance_B, atol=1e-17
    )
    assert not out.joint_covariance.flags.writeable


def test_positive_rotation_rule_integrates_standard_normal_moments():
    x, w = rotation_rule(5, 3)
    assert len(w) == 125 and np.all(w > 0)
    assert sum(w) == pytest.approx(1)
    np.testing.assert_allclose(w @ x, 0, atol=1e-16)
    np.testing.assert_allclose(x.T @ (w[:, None] * x), np.eye(3), atol=1e-15)
    np.testing.assert_allclose(w @ (x**4), np.full(3, 3.0), atol=1e-14)


@pytest.mark.parametrize("order", [True, 0, 3, 6])
def test_unfrozen_rule_rejected(order):
    with pytest.raises(ValueError):
        rotation_rule(order, 3)


def test_online_replay_and_original_prior_are_preserved(prepared):
    initial = prepared.release.endpoint.joint_covariance.copy()
    original = eskf_online.predict_eskf_endpoint
    with nonlinear_release_prediction(prepared, "online") as online:
        result, diagnostic = fly(prepared, "aligned")
    with nonlinear_release_prediction(prepared, "replay") as replay:
        audit(prepared, "aligned", result, diagnostic)
    assert online == replay
    assert online["routing"] == {
        "predictions": len(result.mission.time_s) - 1,
        "release_predictions": 1,
    }
    np.testing.assert_array_equal(
        np.array(online["input"]["joint_covariance"])[3:6, 3:6], initial[3:6, 3:6]
    )
    np.testing.assert_array_equal(prepared.release.endpoint.joint_covariance, initial)
    assert eskf_online.predict_eskf_endpoint is original


def test_context_restores_when_mission_raises(prepared):
    original = eskf_online.predict_eskf_endpoint
    with pytest.raises(RuntimeError):
        with nonlinear_release_prediction(prepared, "online"):
            raise RuntimeError("fixture failure")
    assert eskf_online.predict_eskf_endpoint is original
