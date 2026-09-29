"""Independent equation, uncertainty, shared-noise and explicit routing checks."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.release_prediction import (
    ReleasePrediction,
    predict_release_endpoint,
    release_endpoint_map,
)
from experiments.release_uncertainty import numerical_derivative, yaw_noise_counterexample
from experiments.supported_start import audit, configuration_for, fly
from experiments.supported_validation_protocol import Job, prepare_job
from quadrotor_math import eskf_online, eskf_replay
from quadrotor_math.eskf import EskfNominalState, inject_eskf_error_state
from quadrotor_math.eskf_endpoint import (
    EskfEndpointState,
    EskfSampledImuNoise,
    initialize_eskf_endpoint,
    predict_eskf_endpoint,
    update_eskf_endpoint,
)


def state():
    return EskfNominalState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )


@pytest.mark.parametrize("sign", [-1.0, 1.0])
@pytest.mark.parametrize("acceleration", [[0.0, 0, 9.81], [1.0, -2.0, 4.0]])
def test_constant_world_acceleration_ignores_supported_force(sign, acceleration):
    old = replace(state(), q_WB=sign * state().q_WB, velocity_W=np.array([1.0, -2.0, 0.3]))
    a = np.array(acceleration)
    h = 0.0025
    out, A, B = release_endpoint_map(
        old,
        np.array([6.0, 2.0, -90.0]),
        np.zeros(3),
        a - [0, 0, 9.81],
        np.zeros(3),
        np.zeros(6),
        9.81,
        h,
    )
    np.testing.assert_allclose(out.velocity_W, old.velocity_W + h * a, atol=1e-16)
    np.testing.assert_allclose(out.position_W, h * old.velocity_W + h * h / 2 * a, atol=1e-16)
    np.testing.assert_array_equal(A[:, 15:18], 0)
    np.testing.assert_array_equal(B[15:, :6], np.eye(6))


def test_all_33_columns_and_full_covariance_use_independent_finite_map():
    old = inject_eskf_error_state(state(), np.linspace(-0.3, 0.4, 15))
    f0, f1 = np.array([5.0, -8.0, -9]), np.array([1.4, -1.3, -8.7])
    w0, w1 = np.array([0.3, -0.5, 0.2]), np.array([0.2, -0.1, 0.4])
    mean = np.linspace(-0.03, 0.04, 6)
    _, A, B = release_endpoint_map(old, f0, w0, f1, w1, mean, 9.81, 0.13)
    numerical = numerical_derivative(old, w0, f1, w1, mean, 9.81, 0.13)
    np.testing.assert_allclose(np.c_[A, B], numerical, atol=5e-10, rtol=2e-7)
    rng = np.random.default_rng(35001)
    factors = [rng.normal(size=(n, n)) * 0.005 for n in (21, 6, 6)]
    C, R, Q = [L @ L.T for L in factors]
    out = predict_release_endpoint(
        EskfEndpointState(old, mean, C), f0, w0, f1, w1, 9.81, EskfSampledImuNoise(R, Q), 0.13
    )
    latent = np.zeros((33, 33))
    latent[:21, :21] = C
    latent[21:27, 21:27] = R
    latent[27:, 27:] = 0.13 * Q
    np.testing.assert_allclose(out.joint_covariance, numerical @ latent @ numerical.T, atol=1e-12)
    assert np.linalg.eigvalsh(out.joint_covariance).min() > 0


def test_linear_force_has_the_declared_error_bound_and_halving_rate():
    jerk = np.array([3.0, -2.0, 1.0])
    a0 = np.array([1.0, 2.0, -3.0])
    errors = []
    for h in (0.01, 0.005, 0.0025):
        out, _, _ = release_endpoint_map(
            state(), np.zeros(3), np.zeros(3), a0 + h * jerk, np.zeros(3), np.zeros(6), 0.0, h
        )
        ev = out.velocity_W - (h * a0 + h * h / 2 * jerk)
        ep = out.position_W - (h * h / 2 * a0 + h**3 / 6 * jerk)
        np.testing.assert_allclose(ev, h * h / 2 * jerk, atol=1e-17)
        np.testing.assert_allclose(ep, h**3 / 3 * jerk, atol=1e-19)
        errors.append([np.linalg.norm(ev), np.linalg.norm(ep)])
    np.testing.assert_allclose(
        np.array(errors[:-1]) / np.array(errors[1:]), [[4, 8], [4, 8]], rtol=1e-11
    )


def test_constant_rate_rotating_body_force_matches_analytic_endpoint_and_bound():
    rate, h = 0.4, 0.0025
    out, _, _ = release_endpoint_map(
        state(),
        np.zeros(3),
        np.array([0.0, 0.0, rate]),
        np.array([1.0, 0, 0]),
        np.array([0.0, 0.0, rate]),
        np.zeros(6),
        0.0,
        h,
    )
    exact_v = np.array([np.sin(rate * h) / rate, (1 - np.cos(rate * h)) / rate, 0.0])
    exact_p = np.array(
        [(1 - np.cos(rate * h)) / rate**2, h / rate - np.sin(rate * h) / rate**2, 0.0]
    )
    np.testing.assert_allclose(
        out.q_WB, [np.cos(rate * h / 2), 0, 0, np.sin(rate * h / 2)], atol=1e-15
    )
    assert np.linalg.norm(out.velocity_W - exact_v) <= rate * h * h / 2 * (1 + 1e-8)
    assert np.linalg.norm(out.position_W - exact_p) <= rate * h**3 / 3 * (1 + 1e-7)


def test_release_then_ordinary_intervals_and_updates_match_batch_latent_gaussian():
    N, h, var, obsvar = 6, 0.1, 0.04, 0.002
    P = np.zeros((15, 15))
    P[0, 0] = 0.1
    noise = EskfSampledImuNoise(np.diag([var, 0, 0, 0, 0, 0]), np.zeros((6, 6)))
    current = initialize_eskf_endpoint(state(), P, noise)
    C = np.diag([0.1] + [var] * (N + 1))
    mean = np.zeros(N + 2)
    p = np.eye(N + 2)[0]
    v = np.zeros(N + 2)
    H = np.eye(15)[[0]]
    for k in range(1, N + 1):
        p = p + h * v
        if k == 1:
            p[k + 1] -= h * h / 2
            v = v.copy()
            v[k + 1] -= h
        else:
            p[k] -= h * h / 3
            p[k + 1] -= h * h / 6
            v = v.copy()
            v[k : k + 2] -= h / 2
        predictor = predict_release_endpoint if k == 1 else predict_eskf_endpoint
        current = predictor(
            current, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), 0.0, noise, h
        )
        z = 0.05 * np.sin(k)
        gain = C @ p / (p @ C @ p + obsvar)
        mean += gain * (z - p @ mean)
        C -= np.outer(gain, p @ C)
        current, _ = update_eskf_endpoint(
            current, np.array([z]), current.nominal_state.position_W[:1], H, np.array([[obsvar]])
        )
        rows = np.vstack((p, v, np.eye(N + 2)[k + 1]))
        np.testing.assert_allclose(
            [
                current.nominal_state.position_W[0],
                current.nominal_state.velocity_W[0],
                current.imu_noise_mean_B[0],
            ],
            rows @ mean,
            atol=1e-15,
        )
        np.testing.assert_allclose(
            current.joint_covariance[np.ix_([0, 3, 15], [0, 3, 15])], rows @ C @ rows.T, atol=1e-16
        )


def test_nonlinear_yaw_noise_invalidates_exact_tangent_zero_variance():
    r = yaw_noise_counterexample()
    assert r["maximum_quadrature_error"] < 1e-22
    assert min(r["exact_variance"]) > 0
    assert not r["exact_zero_variance_claim_valid"]


@pytest.mark.parametrize("h", [0.0, -1.0, True, np.nan, np.inf])
def test_bad_interval(h):
    with pytest.raises(ValueError):
        release_endpoint_map(
            state(), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(6), 9.81, h
        )


@pytest.mark.parametrize("index", range(5))
def test_invalid_vector_rejected(index):
    args = [np.zeros(3) for _ in range(4)] + [np.zeros(6)]
    args[index][0] = np.nan
    with pytest.raises(ValueError):
        release_endpoint_map(state(), *args, 9.81, 0.0025)


@pytest.fixture(scope="module")
def prepared():
    return prepare_job(Job("nominal_hover", 47831), "smoke")


def arguments(prepared):
    r = prepared.release
    return (
        r.endpoint,
        r.fresh_sample.specific_force_B,
        r.fresh_sample.angular_velocity_B,
        np.zeros(3),
        np.zeros(3),
        9.81,
        configuration_for(prepared, "aligned")["estimator_configuration"].sampled_imu_noise,
        0.0025,
    )


@pytest.mark.parametrize("target", ["online", "replay"])
def test_adapter_changes_first_prediction_only_and_restores(prepared, target):
    module = eskf_online if target == "online" else eskf_replay
    original = module.predict_eskf_endpoint
    adapter = ReleasePrediction(prepared)
    args = arguments(prepared)
    with adapter.installed(target) as trace:
        first = module.predict_eskf_endpoint(*args)
        second = module.predict_eskf_endpoint(*((first,) + args[1:]))
    expected = predict_eskf_endpoint(*((first,) + args[1:]))
    np.testing.assert_array_equal(second.joint_covariance, expected.joint_covariance)
    assert trace == {"predictions": 2, "release_predictions": 1}
    assert module.predict_eskf_endpoint is original
    with pytest.raises(ValueError, match="consumed"):
        with adapter.installed(target):
            pass


@pytest.mark.parametrize("failure", ["sample", "clock", "noise", "empty", "exception"])
def test_adapter_rejects_and_restores_on_all_exit_paths(prepared, failure):
    adapter = ReleasePrediction(prepared)
    args = list(arguments(prepared))
    original = eskf_online.predict_eskf_endpoint
    if failure == "sample":
        args[1] = np.zeros(3)
    if failure == "clock":
        args[-1] = 0.005
    if failure == "noise":
        args[-2] = EskfSampledImuNoise(np.eye(6), np.eye(6))
    with pytest.raises((ValueError, RuntimeError)):
        with adapter.installed("online"):
            if failure == "exception":
                raise RuntimeError("test exit")
            if failure != "empty":
                eskf_online.predict_eskf_endpoint(*args)
    assert eskf_online.predict_eskf_endpoint is original
    with pytest.raises(ValueError, match="consumed"):
        with adapter.installed("online"):
            pass


def test_short_online_mission_and_complete_saved_replay_agree(prepared):
    with ReleasePrediction(prepared).installed("online") as online:
        result, diagnostic = fly(prepared, "aligned")
    with ReleasePrediction(prepared).installed("replay") as replay:
        audit(prepared, "aligned", result, diagnostic)
    assert (
        online
        == replay
        == {"predictions": len(result.mission.time_s) - 1, "release_predictions": 1}
    )
