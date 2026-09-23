"""Independent continuous motion oracle and seeded stochastic fixture validation."""

from dataclasses import replace

import numpy as np
import pytest

from quadrotor_math.eskf import propagate_eskf_nominal_state
from quadrotor_math.eskf_consistency import eskf_right_local_error
from quadrotor_math.eskf_synthetic import (
    ACCEL_STD,
    ACCEL_WALK,
    GYRO_STD,
    GYRO_WALK,
    make_eskf_synthetic_case,
    randomized_eskf_motion,
    sample_eskf_analytic_motion,
    synthetic_eskf_rng,
)
from quadrotor_math.rotations import rotation_matrix_body_to_world, skew_symmetric


@pytest.mark.parametrize("seed", [0, 7, 123])
def test_analytic_derivatives_and_frame_direction_against_central_differences(seed):
    motion = randomized_eskf_motion(seed, "excited")
    t, h = 1.37, 1e-5
    exact = sample_eskf_analytic_motion(motion, np.array([t - h, t, t + h]))
    np.testing.assert_allclose(
        (exact.position_W[2] - exact.position_W[0]) / (2 * h), exact.velocity_W[1], atol=4e-10
    )
    np.testing.assert_allclose(
        (exact.velocity_W[2] - exact.velocity_W[0]) / (2 * h), exact.acceleration_W[1], atol=4e-10
    )
    rotations = [rotation_matrix_body_to_world(q_WB) for q_WB in exact.q_WB]
    np.testing.assert_allclose(
        (rotations[2] - rotations[0]) / (2 * h),
        rotations[1] @ skew_symmetric(exact.angular_velocity_B[1]),
        atol=5e-10,
    )
    np.testing.assert_allclose(
        rotations[1] @ exact.specific_force_B[1] + [0, 0, 9.81], exact.acceleration_W[1], atol=1e-14
    )


@pytest.mark.parametrize("family", ["stationary", "translating_yaw"])
def test_simple_exact_physics(family):
    history = sample_eskf_analytic_motion(randomized_eskf_motion(0, family), np.array([0.0, 2.0]))
    velocity = np.zeros(3) if family == "stationary" else np.array([0.5, -0.25, 0.1])
    rate = 0 if family == "stationary" else 0.2
    np.testing.assert_allclose(history.position_W, np.array([0, 2])[:, None] * velocity)
    np.testing.assert_allclose(history.specific_force_B, np.tile([0, 0, -9.81], (2, 1)), atol=2e-14)
    np.testing.assert_allclose(history.angular_velocity_B, np.tile([0, 0, rate], (2, 1)))
    np.testing.assert_allclose(history.q_WB[-1], [np.cos(rate), 0, 0, np.sin(rate)])


def test_noise_free_strapdown_error_decreases_under_step_refinement():
    errors = []
    for dt in (0.02, 0.01, 0.005):
        case = make_eskf_synthetic_case(
            7, number_of_steps=round(2 / dt), time_step_s=dt, stochastic=False
        )
        state = case.configuration.initial_state
        for f_B, omega_B in zip(
            case.measurements.specific_force_measurements_B[:-1],
            case.measurements.angular_velocity_measurements_B[:-1],
            strict=True,
        ):
            state = propagate_eskf_nominal_state(state, f_B, omega_B, 9.81, dt)
        errors.append(np.linalg.norm(eskf_right_local_error(state, case.reference.states[-1])[:9]))
    assert errors[2] < errors[1] < errors[0]
    assert 1.8 < errors[0] / errors[1] < 2.2
    assert 1.8 < errors[1] / errors[2] < 2.2


def test_initial_epoch_prior_and_walk_independence_are_exact():
    case = make_eskf_synthetic_case(7, number_of_steps=40)
    assert case.measurements.time_s[0] == case.configuration.initial_time_s == 0
    expected_bias = synthetic_eskf_rng(7, 2).standard_normal(6) * np.repeat([0.08, 0.008], 3)
    first = case.reference.states[0]
    np.testing.assert_array_equal(first.accelerometer_bias_B, expected_bias[:3])
    np.testing.assert_array_equal(first.gyroscope_bias_B, expected_bias[3:])
    error = eskf_right_local_error(case.configuration.initial_state, first)
    expected = synthetic_eskf_rng(7, 1).standard_normal(15) * np.repeat(
        [0.2, 0.1, 0.015, 0.08, 0.008], 3
    )
    expected[9:] = expected_bias
    np.testing.assert_allclose(error, expected, atol=1e-15)
    for stream, density, field in [
        (5, ACCEL_WALK, "accelerometer_bias_B"),
        (6, GYRO_WALK, "gyroscope_bias_B"),
    ]:
        bias = np.array([getattr(s, field) for s in case.reference.states])
        increments = (
            synthetic_eskf_rng(7, stream).standard_normal((40, 3)) * density * np.sqrt(0.01)
        )
        np.testing.assert_allclose(np.diff(bias, axis=0), increments, atol=4e-17)


def test_sensor_statistics_sample_units_and_repeatability():
    case = make_eskf_synthetic_case(12, number_of_steps=2000)
    exact = sample_eskf_analytic_motion(case.motion, case.measurements.time_s)
    for field, signal, biasfield, sigma in [
        (
            "specific_force_measurements_B",
            exact.specific_force_B,
            "accelerometer_bias_B",
            ACCEL_STD,
        ),
        ("angular_velocity_measurements_B", exact.angular_velocity_B, "gyroscope_bias_B", GYRO_STD),
    ]:
        bias = np.array([getattr(s, biasfield) for s in case.reference.states])
        noise = (getattr(case.measurements, field) - signal - bias) / sigma
        assert abs(noise.mean()) < 0.05
        assert 0.94 < noise.std() < 1.06
    again = make_eskf_synthetic_case(12, number_of_steps=2000)
    np.testing.assert_array_equal(
        case.measurements.specific_force_measurements_B,
        again.measurements.specific_force_measurements_B,
    )
    assert not np.shares_memory(case.measurements.time_s, case.reference.time_s)
    assert not case.motion.position_amplitude_W.flags.writeable


@pytest.mark.parametrize(
    "change",
    [
        dict(seed=True),
        dict(seed=-1),
        dict(seed=2**32),
        dict(family="invalid"),
        dict(number_of_steps=0),
        dict(number_of_steps=True),
        dict(time_step_s=0),
        dict(time_step_s=1e-320),
        dict(time_step_s=True),
        dict(time_step_s=np.nan),
        dict(time_step_s=0.03),
        dict(stochastic=1),
    ],
)
def test_fixture_rejects_invalid_domain(change):
    kwargs = dict(seed=0, number_of_steps=10)
    kwargs.update(change)
    with pytest.raises((TypeError, ValueError)):
        make_eskf_synthetic_case(**kwargs)


def test_analytic_arrays_are_owned_and_nonfinite_or_overflow_is_rejected():
    motion = randomized_eskf_motion(0, "excited")
    with pytest.raises(ValueError):
        replace(motion, euler_amplitude=np.full(3, np.inf))
    with pytest.raises(ValueError, match="finite"):
        sample_eskf_analytic_motion(
            replace(motion, position_frequency=np.full(3, 1e308)), np.array([0.0, 2.0])
        )
    with pytest.raises(ValueError):
        synthetic_eskf_rng(0, True)
