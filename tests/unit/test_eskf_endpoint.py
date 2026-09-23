"""Independent discrete derivatives, noise memory and analytic inertial cases."""

from dataclasses import replace

import numpy as np
import pytest

from quadrotor_math.eskf import EskfNominalState, inject_eskf_error_state
from quadrotor_math.eskf_consistency import eskf_right_local_error
from quadrotor_math.eskf_endpoint import (
    EskfEndpointState,
    EskfSampledImuNoise,
    eskf_endpoint_map,
    initialize_eskf_endpoint,
    predict_eskf_endpoint,
    update_eskf_endpoint,
)


def state():
    return EskfNominalState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )


def noise(sample=0.0, walk=0.0):
    return EskfSampledImuNoise(np.eye(6) * sample, np.eye(6) * walk)


def test_linear_world_acceleration_is_integrated_analytically():
    old = replace(state(), velocity_W=np.array([2.0, -1, 3]))
    a0, jerk = np.array([1.0, 2, -3]), np.array([3.0, -2, 1])
    h = 0.3
    out, _, _ = eskf_endpoint_map(
        old, a0, np.zeros(3), a0 + h * jerk, np.zeros(3), np.zeros(6), 0.0, h
    )
    np.testing.assert_allclose(out.velocity_W, old.velocity_W + h * a0 + h * h / 2 * jerk)
    np.testing.assert_allclose(
        out.position_W, h * old.velocity_W + h * h / 2 * a0 + h**3 / 6 * jerk
    )


def test_all_discrete_prior_and_noise_jacobian_columns():
    old = inject_eskf_error_state(state(), np.linspace(-0.3, 0.4, 15))
    f0, f1 = np.array([1.0, -2, -9]), np.array([1.4, -1.3, -8.7])
    w0, w1 = np.array([0.3, -0.5, 0.2]), np.array([0.2, -0.1, 0.4])
    mean = np.linspace(-0.03, 0.04, 6)
    h = 0.13
    nominal, A, B = eskf_endpoint_map(old, f0, w0, f1, w1, mean, 9.81, h)
    step = 1e-6
    numerical = np.zeros((21, 33))
    for col in range(33):
        outputs = []
        for sign in (1, -1):
            delta = np.zeros(33)
            delta[col] = sign * step
            perturbed = inject_eskf_error_state(old, delta[:15])
            # Perturb the true latent endpoint samples and final bias. The
            # nominal map subtracts the initial bias at both ends, so subtract
            # the independent final bias increment explicitly at endpoint 1.
            out, _, _ = eskf_endpoint_map(
                perturbed,
                f0,
                w0,
                f1 - delta[21:24] - delta[27:30],
                w1 - delta[24:27] - delta[30:33],
                mean + delta[15:21],
                9.81,
                h,
            )
            out = replace(
                out,
                accelerometer_bias_B=out.accelerometer_bias_B + delta[27:30],
                gyroscope_bias_B=out.gyroscope_bias_B + delta[30:33],
            )
            outputs.append(np.r_[eskf_right_local_error(nominal, out), delta[21:27]])
        numerical[:, col] = (outputs[0] - outputs[1]) / (2 * step)
    np.testing.assert_allclose(np.c_[A, B], numerical, atol=5e-10, rtol=2e-7)


def test_shared_endpoint_noise_accumulates_exact_variance():
    sample = np.diag([0.04, 0, 0, 0, 0, 0])
    model = EskfSampledImuNoise(sample, np.zeros((6, 6)))
    current = initialize_eskf_endpoint(state(), np.zeros((15, 15)), model)
    h = 0.1
    for n in range(1, 9):
        current = predict_eskf_endpoint(
            current, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), 0.0, model, h
        )
        assert current.joint_covariance[3, 3] == pytest.approx(h * h * 0.04 * (n - 0.5))
        assert current.joint_covariance[3, 15] == pytest.approx(-h * 0.04 / 2)


def test_joint_correction_matches_independent_gaussian_conditioning():
    rng = np.random.default_rng(732)
    L = rng.normal(size=(21, 21)) * 0.01
    C = L @ L.T
    current = EskfEndpointState(state(), np.linspace(-0.02, 0.03, 6), C)
    H = np.zeros((3, 15))
    H[:, :3] = np.eye(3)
    Hj = np.c_[H, np.zeros((3, 6))]
    R = np.eye(3) * 0.04
    z = np.array([0.2, -0.1, 0.3])
    gain = np.linalg.solve(Hj @ C @ Hj.T + R, Hj @ C).T
    corrected, diagnostic = update_eskf_endpoint(current, z, np.zeros(3), H, R)
    np.testing.assert_allclose(corrected.imu_noise_mean_B, current.imu_noise_mean_B + gain[15:] @ z)
    from quadrotor_math.eskf import eskf_reset_jacobian

    reset = np.eye(21)
    reset[:15, :15] = eskf_reset_jacobian(gain[:15] @ z)
    expected = reset @ (C - gain @ Hj @ C) @ reset.T
    np.testing.assert_allclose(corrected.joint_covariance, expected, atol=2e-18)
    np.testing.assert_array_equal(diagnostic.covariance, corrected.joint_covariance[:15, :15])
    assert np.linalg.norm(gain[15:] @ z) > 0


@pytest.mark.parametrize("h", [0.0, -1.0, np.nan, np.inf, True])
def test_invalid_interval_is_rejected(h):
    with pytest.raises(ValueError, match="time_step_s"):
        eskf_endpoint_map(
            state(), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(6), 9.81, h
        )


def test_arrays_are_independent_and_read_only():
    raw = np.eye(6)
    model = EskfSampledImuNoise(raw, raw)
    current = initialize_eskf_endpoint(state(), np.eye(15), model)
    raw[:] = 4
    np.testing.assert_array_equal(model.sample_covariance_B, np.eye(6))
    for array in (
        model.sample_covariance_B,
        model.bias_walk_spectral_density_B,
        current.imu_noise_mean_B,
        current.joint_covariance,
    ):
        assert array.flags.owndata and array.flags.c_contiguous and not array.flags.writeable


def test_multiple_intervals_and_corrections_match_batch_latent_gaussian():
    """A separate batch regression over every sample verifies memory and means."""
    N, h, sigma2, obs_variance = 6, 0.1, 0.04, 0.002
    P = np.zeros((15, 15))
    P[0, 0], P[3, 3] = 0.1, 0.2
    model = EskfSampledImuNoise(np.diag([sigma2, 0, 0, 0, 0, 0]), np.zeros((6, 6)))
    current = initialize_eskf_endpoint(state(), P, model)
    # Latent vector is p0, v0, then all independent acceleration sample errors.
    covariance = np.diag([0.1, 0.2] + [sigma2] * (N + 1))
    mean = np.zeros(N + 3)
    p_row, v_row = np.eye(N + 3)[0], np.eye(N + 3)[1]
    H = np.zeros((1, 15))
    H[0, 0] = 1
    for k in range(1, N + 1):
        p_row = p_row + h * v_row
        p_row[k + 1] -= h * h / 3
        p_row[k + 2] -= h * h / 6
        v_row = v_row.copy()
        v_row[k + 1 : k + 3] -= h / 2
        current = predict_eskf_endpoint(
            current, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), 0.0, model, h
        )
        z = 0.05 * np.sin(k)
        variance = p_row @ covariance @ p_row + obs_variance
        gain = covariance @ p_row / variance
        mean = mean + gain * (z - p_row @ mean)
        covariance = covariance - np.outer(gain, p_row @ covariance)
        current, _ = update_eskf_endpoint(
            current,
            np.array([z]),
            current.nominal_state.position_W[:1],
            H,
            np.array([[obs_variance]]),
        )
        rows = np.vstack((p_row, v_row, np.eye(N + 3)[k + 2]))
        np.testing.assert_allclose(
            [
                current.nominal_state.position_W[0],
                current.nominal_state.velocity_W[0],
                current.imu_noise_mean_B[0],
            ],
            rows @ mean,
            atol=2e-15,
        )
        np.testing.assert_allclose(
            current.joint_covariance[np.ix_([0, 3, 15], [0, 3, 15])],
            rows @ covariance @ rows.T,
            atol=2e-16,
        )


def test_smooth_analytic_motion_has_second_order_global_convergence():
    from quadrotor_math.eskf_synthetic import randomized_eskf_motion, sample_eskf_analytic_motion

    errors = []
    for h in (0.04, 0.02, 0.01):
        times = np.arange(round(2 / h) + 1) * h
        exact = sample_eskf_analytic_motion(randomized_eskf_motion(5000, "excited"), times)
        old = replace(
            state(),
            position_W=exact.position_W[0],
            velocity_W=exact.velocity_W[0],
            q_WB=exact.q_WB[0],
        )
        for k in range(len(times) - 1):
            old, _, _ = eskf_endpoint_map(
                old,
                exact.specific_force_B[k],
                exact.angular_velocity_B[k],
                exact.specific_force_B[k + 1],
                exact.angular_velocity_B[k + 1],
                np.zeros(6),
                9.81,
                h,
            )
        truth = replace(
            state(),
            position_W=exact.position_W[-1],
            velocity_W=exact.velocity_W[-1],
            q_WB=exact.q_WB[-1],
        )
        error = eskf_right_local_error(old, truth)
        errors.append([np.linalg.norm(error[s : s + 3]) for s in (0, 3, 6)])
    ratios = np.array(errors[:-1]) / np.array(errors[1:])
    assert np.all((ratios > 3.8) & (ratios < 4.2))


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_stationary_and_constant_yaw_have_analytic_solution(sign):
    old = replace(state(), q_WB=np.array([sign, 0.0, 0, 0]))
    for yaw_rate in (0.0, 0.4):
        out, _, _ = eskf_endpoint_map(
            old,
            np.array([0.0, 0, -9.81]),
            np.array([0.0, 0, yaw_rate]),
            np.array([0.0, 0, -9.81]),
            np.array([0.0, 0, yaw_rate]),
            np.zeros(6),
            9.81,
            0.5,
        )
        np.testing.assert_allclose(out.position_W, 0, atol=1e-15)
        np.testing.assert_allclose(out.velocity_W, 0, atol=1e-15)
        np.testing.assert_allclose(
            out.q_WB, sign * np.array([np.cos(yaw_rate * 0.25), 0, 0, np.sin(yaw_rate * 0.25)])
        )


@pytest.mark.parametrize("field", ["sample_covariance_B", "bias_walk_spectral_density_B"])
@pytest.mark.parametrize(
    "bad", ["shape", "nonfinite", "complex", "negative", "asymmetric", "indefinite"]
)
def test_noise_covariance_invalid_domains(field, bad):
    raw = np.eye(6)
    if bad == "shape":
        raw = np.eye(5)
    elif bad == "nonfinite":
        raw[0, 0] = np.inf
    elif bad == "complex":
        raw = raw.astype(complex)
    elif bad == "negative":
        raw[0, 0] = -1e-30
    elif bad == "asymmetric":
        raw[0, 1] = 0.1
    else:
        raw[0, 1] = raw[1, 0] = 1.1
    with pytest.raises(ValueError, match=field):
        replace(noise(), **{field: raw})


@pytest.mark.parametrize("index", range(5))
@pytest.mark.parametrize("bad", ["shape", "nonfinite", "complex"])
def test_endpoint_vectors_invalid_domains(index, bad):
    args = [np.zeros(3) for _ in range(4)] + [np.zeros(6)]
    if bad == "shape":
        args[index] = np.zeros(2)
    elif bad == "nonfinite":
        args[index][0] = np.nan
    else:
        args[index] = args[index].astype(complex)
    with pytest.raises(ValueError):
        eskf_endpoint_map(state(), *args, 9.81, 0.1)


@pytest.mark.parametrize("g", [True, -1.0, np.nan, np.inf, 10**1000])
def test_gravity_invalid_domains(g):
    with pytest.raises(ValueError, match="gravity_acceleration"):
        eskf_endpoint_map(
            state(), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(6), g, 0.1
        )


def test_overflow_and_singular_update_are_atomic():
    model = noise()
    current = initialize_eskf_endpoint(state(), np.zeros((15, 15)), model)
    with pytest.raises(ValueError):
        predict_eskf_endpoint(
            current,
            np.full(3, 1e308),
            np.zeros(3),
            np.full(3, 1e308),
            np.zeros(3),
            0.0,
            model,
            100.0,
        )
    with pytest.raises(ValueError, match="positive definite"):
        update_eskf_endpoint(current, np.zeros(1), np.zeros(1), np.zeros((1, 15)), np.zeros((1, 1)))
    np.testing.assert_array_equal(current.joint_covariance, 0)
    np.testing.assert_array_equal(current.nominal_state.position_W, 0)


def test_correlated_noise_prediction_is_psd_and_matches_sampled_nonlinear_moments():
    rng = np.random.default_rng(98712)
    L = rng.normal(size=(6, 6)) * 1e-4
    model = EskfSampledImuNoise(L @ L.T, np.eye(6) * 1e-8)
    P = np.eye(15) * 1e-8
    current = initialize_eskf_endpoint(
        inject_eskf_error_state(state(), np.linspace(-0.3, 0.4, 15)), P, model
    )
    args = (
        np.array([1.0, -2, -9]),
        np.array([0.3, -0.5, 0.2]),
        np.array([1.4, -1.3, -8.7]),
        np.array([0.2, -0.1, 0.4]),
    )
    h = 0.13
    predicted = predict_eskf_endpoint(current, *args, 9.81, model, h)
    assert np.linalg.eigvalsh(predicted.joint_covariance).min() > 0
    D = np.zeros((33, 33))
    D[:21, :21] = current.joint_covariance
    D[21:27, 21:27] = model.sample_covariance_B
    D[27:, 27:] = model.bias_walk_spectral_density_B * h
    samples = rng.multivariate_normal(np.zeros(33), D, size=3000)
    errors = []
    for delta in samples:
        perturbed = inject_eskf_error_state(current.nominal_state, delta[:15])
        out, _, _ = eskf_endpoint_map(
            perturbed,
            args[0],
            args[1],
            args[2] - delta[21:24] - delta[27:30],
            args[3] - delta[24:27] - delta[30:33],
            delta[15:21],
            9.81,
            h,
        )
        out = replace(
            out,
            accelerometer_bias_B=out.accelerometer_bias_B + delta[27:30],
            gyroscope_bias_B=out.gyroscope_bias_B + delta[30:33],
        )
        errors.append(np.r_[eskf_right_local_error(predicted.nominal_state, out), delta[21:27]])
    values = np.array(errors)
    expected = predicted.joint_covariance
    standard_error = np.sqrt(
        (expected**2 + np.outer(expected.diagonal(), expected.diagonal())) / 2999
    )
    assert np.max(np.abs(np.cov(values.T) - expected) / standard_error) < 6
    assert np.max(np.abs(values.mean(axis=0)) / np.sqrt(expected.diagonal() / 3000)) < 6
