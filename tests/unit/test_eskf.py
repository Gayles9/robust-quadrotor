"""Tests for the 15-state ESKF prediction core."""

import re
import warnings
from dataclasses import fields

import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.eskf import (
    EskfNominalState,
    discretize_eskf_error_dynamics_first_order,
    eskf_continuous_error_dynamics_matrices,
    inject_eskf_error_state,
    predict_eskf,
    propagate_eskf_covariance,
    propagate_eskf_nominal_state,
)
from quadrotor_math.rotations import rotation_matrix_body_to_world


def _state(
    *,
    position_W: NDArray[np.float64] | None = None,
    velocity_W: NDArray[np.float64] | None = None,
    q_WB: NDArray[np.float64] | None = None,
    accelerometer_bias_B: NDArray[np.float64] | None = None,
    gyroscope_bias_B: NDArray[np.float64] | None = None,
) -> EskfNominalState:
    return EskfNominalState(
        position_W=(np.array([1.0, -2.0, 3.0]) if position_W is None else position_W),
        velocity_W=(np.array([0.5, -0.25, 1.5]) if velocity_W is None else velocity_W),
        q_WB=(np.array([1.0, 0.0, 0.0, 0.0]) if q_WB is None else q_WB),
        accelerometer_bias_B=(
            np.array([0.1, -0.2, 0.3]) if accelerometer_bias_B is None else accelerometer_bias_B
        ),
        gyroscope_bias_B=(
            np.array([0.01, -0.02, 0.03]) if gyroscope_bias_B is None else gyroscope_bias_B
        ),
    )


def _quaternion_product(
    q_left_WB: NDArray[np.float64], q_right_BB: NDArray[np.float64]
) -> NDArray[np.float64]:
    lw, lx, ly, lz = q_left_WB
    rw, rx, ry, rz = q_right_BB
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )


def _rotation_vector_quaternion(phi_B: NDArray[np.float64]) -> NDArray[np.float64]:
    angle = float(np.linalg.norm(phi_B))
    if angle == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.concatenate(
        (
            np.array([np.cos(0.5 * angle)]),
            (np.sin(0.5 * angle) / angle) * phi_B,
        )
    )


def _independent_skew(vector: NDArray[np.float64]) -> NDArray[np.float64]:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _right_local_error(
    nominal: EskfNominalState, perturbed: EskfNominalState
) -> NDArray[np.float64]:
    error = np.empty(15, dtype=np.float64)
    error[0:3] = perturbed.position_W - nominal.position_W
    error[3:6] = perturbed.velocity_W - nominal.velocity_W
    R_nominal_WB = rotation_matrix_body_to_world(nominal.q_WB)
    R_perturbed_WB = rotation_matrix_body_to_world(perturbed.q_WB)
    relative_R_BB = R_nominal_WB.T @ R_perturbed_WB
    antisymmetric_vector = np.array(
        [
            relative_R_BB[2, 1] - relative_R_BB[1, 2],
            relative_R_BB[0, 2] - relative_R_BB[2, 0],
            relative_R_BB[1, 0] - relative_R_BB[0, 1],
        ]
    )
    cosine_angle = float(np.clip(0.5 * (np.trace(relative_R_BB) - 1.0), -1.0, 1.0))
    angle = float(np.arccos(cosine_angle))
    if angle < 1.0e-7:
        scale = 0.5 + angle**2 / 12.0
    else:
        scale = angle / (2.0 * np.sin(angle))
    error[6:9] = scale * antisymmetric_vector
    error[9:12] = perturbed.accelerometer_bias_B - nominal.accelerometer_bias_B
    error[12:15] = perturbed.gyroscope_bias_B - nominal.gyroscope_bias_B
    return error


@pytest.mark.parametrize(
    ("field_name", "invalid", "expected_shape"),
    [
        ("position_W", np.zeros(2), "(3,)"),
        ("velocity_W", np.zeros((3, 1)), "(3,)"),
        ("q_WB", np.zeros(3), "(4,)"),
        ("accelerometer_bias_B", np.zeros(4), "(3,)"),
        ("gyroscope_bias_B", np.zeros(2), "(3,)"),
    ],
)
def test_nominal_state_rejects_invalid_shapes(
    field_name: str, invalid: NDArray[np.float64], expected_shape: str
) -> None:
    arguments = {
        "position_W": np.zeros(3),
        "velocity_W": np.zeros(3),
        "q_WB": np.array([1.0, 0.0, 0.0, 0.0]),
        "accelerometer_bias_B": np.zeros(3),
        "gyroscope_bias_B": np.zeros(3),
    }
    arguments[field_name] = invalid

    with pytest.raises(
        ValueError,
        match=rf"^{field_name} must have shape {re.escape(expected_shape)}$",
    ):
        EskfNominalState(**arguments)


def test_nominal_state_checks_all_shapes_before_finiteness() -> None:
    with pytest.raises(ValueError, match=r"^gyroscope_bias_B must have shape \(3,\)$"):
        EskfNominalState(
            np.array([np.nan, 0.0, 0.0]),
            np.zeros(3),
            np.array([1.0, 0.0, 0.0, 0.0]),
            np.zeros(3),
            np.zeros(2),
        )


@pytest.mark.parametrize(
    "field_name",
    [
        "position_W",
        "velocity_W",
        "q_WB",
        "accelerometer_bias_B",
        "gyroscope_bias_B",
    ],
)
def test_nominal_state_rejects_nonfinite_arrays(field_name: str) -> None:
    arguments = {
        "position_W": np.zeros(3),
        "velocity_W": np.zeros(3),
        "q_WB": np.array([1.0, 0.0, 0.0, 0.0]),
        "accelerometer_bias_B": np.zeros(3),
        "gyroscope_bias_B": np.zeros(3),
    }
    arguments[field_name] = arguments[field_name].copy()
    arguments[field_name][0] = np.nan

    with pytest.raises(ValueError, match=rf"^{field_name} must contain only finite values$"):
        EskfNominalState(**arguments)


@pytest.mark.parametrize(
    "q_WB",
    [np.zeros(4), np.array([1.0 + 3.0e-12, 0.0, 0.0, 0.0])],
)
def test_nominal_state_requires_strict_unit_quaternion(q_WB: NDArray[np.float64]) -> None:
    with pytest.raises(ValueError, match="^q_WB must have unit norm$"):
        _state(q_WB=q_WB)


def test_nominal_state_rejects_finite_quaternion_norm_overflow_without_warning() -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="^q_WB must have unit norm$"):
            _state(q_WB=np.full(4, np.finfo(np.float64).max))


def test_nominal_state_owns_read_only_float64_arrays_without_sign_canonicalization() -> None:
    caller_arrays = [
        np.array([1, 2, 3]),
        np.array([4, 5, 6]),
        np.array([-1, 0, 0, 0]),
        np.array([7, 8, 9]),
        np.array([10, 11, 12]),
    ]
    snapshots = [values.copy() for values in caller_arrays]
    state = EskfNominalState(*caller_arrays)
    for values in caller_arrays:
        values[...] = 99

    assert state.q_WB[0] == -1.0
    assert fields(state)[0].name == "position_W"
    for stored, caller, expected in zip(
        (
            state.position_W,
            state.velocity_W,
            state.q_WB,
            state.accelerometer_bias_B,
            state.gyroscope_bias_B,
        ),
        caller_arrays,
        snapshots,
        strict=True,
    ):
        np.testing.assert_array_equal(stored, expected)
        assert stored.dtype == np.float64
        assert stored.flags.owndata and stored.flags.c_contiguous
        assert not stored.flags.writeable
        assert not np.shares_memory(stored, caller)


def test_zero_error_injection_preserves_values_with_independent_arrays() -> None:
    nominal = _state()
    error = np.zeros(15)
    injected = inject_eskf_error_state(nominal, error)

    assert injected is not nominal
    for field in fields(EskfNominalState):
        actual = getattr(injected, field.name)
        expected = getattr(nominal, field.name)
        np.testing.assert_array_equal(actual, expected)
        assert not np.shares_memory(actual, expected)


@pytest.mark.parametrize(
    ("error_slice", "field_name"),
    [
        (slice(0, 3), "position_W"),
        (slice(3, 6), "velocity_W"),
        (slice(9, 12), "accelerometer_bias_B"),
        (slice(12, 15), "gyroscope_bias_B"),
    ],
)
def test_injection_adds_each_vector_error_block(error_slice: slice, field_name: str) -> None:
    nominal = _state()
    error = np.zeros(15)
    error[error_slice] = np.array([0.2, -0.3, 0.4])

    injected = inject_eskf_error_state(nominal, error)

    np.testing.assert_allclose(
        getattr(injected, field_name),
        getattr(nominal, field_name) + error[error_slice],
        rtol=0.0,
        atol=0.0,
    )


def test_attitude_error_is_right_multiplicative_at_identity() -> None:
    phi_B = np.array([0.2, -0.3, 0.4])
    error = np.zeros(15)
    error[6:9] = phi_B

    injected = inject_eskf_error_state(_state(), error)

    np.testing.assert_allclose(injected.q_WB, _rotation_vector_quaternion(phi_B), atol=1e-15)


def test_attitude_error_is_body_local_at_nontrivial_orientation() -> None:
    nominal_q_WB = np.array([0.5, 0.5, 0.5, 0.5])
    phi_B = np.array([0.15, -0.1, 0.05])
    error = np.zeros(15)
    error[6:9] = phi_B
    expected_q_WB = _quaternion_product(nominal_q_WB, _rotation_vector_quaternion(phi_B))
    left_product = _quaternion_product(_rotation_vector_quaternion(phi_B), nominal_q_WB)

    injected = inject_eskf_error_state(_state(q_WB=nominal_q_WB), error)

    np.testing.assert_allclose(injected.q_WB, expected_q_WB, rtol=0.0, atol=1e-15)
    assert not np.allclose(injected.q_WB, left_product, rtol=0.0, atol=1e-6)


def test_small_attitude_error_uses_accurate_exponential() -> None:
    phi_B = np.array([1.0e-12, -2.0e-12, 3.0e-12])
    error = np.zeros(15)
    error[6:9] = phi_B

    injected = inject_eskf_error_state(_state(), error)

    np.testing.assert_allclose(injected.q_WB[1:], 0.5 * phi_B, rtol=1e-15, atol=0.0)
    assert injected.q_WB[0] == pytest.approx(1.0, abs=1e-15)


def test_large_finite_attitude_rotation_is_supported() -> None:
    error = np.zeros(15)
    error[6:9] = np.array([1000.0, -2000.0, 3000.0])

    injected = inject_eskf_error_state(_state(), error)

    assert np.all(np.isfinite(injected.q_WB))
    assert np.linalg.norm(injected.q_WB) == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_allclose(
        rotation_matrix_body_to_world(injected.q_WB),
        rotation_matrix_body_to_world(_rotation_vector_quaternion(error[6:9])),
        atol=1e-12,
    )


def test_injection_does_not_mutate_inputs_and_is_caller_mutation_independent() -> None:
    nominal = _state()
    error = np.linspace(-0.1, 0.1, 15)
    error_snapshot = error.copy()
    nominal_snapshots = {
        field.name: getattr(nominal, field.name).copy() for field in fields(EskfNominalState)
    }

    injected = inject_eskf_error_state(nominal, error)
    np.testing.assert_array_equal(error, error_snapshot)
    error[...] = 99.0

    for field in fields(EskfNominalState):
        np.testing.assert_array_equal(getattr(nominal, field.name), nominal_snapshots[field.name])
        assert not np.shares_memory(getattr(injected, field.name), getattr(nominal, field.name))
        assert not np.shares_memory(getattr(injected, field.name), error)


@pytest.mark.parametrize("invalid", [np.zeros(14), np.zeros((15, 1))])
def test_injection_rejects_invalid_error_shape(invalid: NDArray[np.float64]) -> None:
    with pytest.raises(ValueError, match=r"^error_state must have shape \(15,\)$"):
        inject_eskf_error_state(_state(), invalid)


def test_injection_rejects_nonfinite_error() -> None:
    error = np.zeros(15)
    error[7] = np.inf
    with pytest.raises(ValueError, match="^error_state must contain only finite values$"):
        inject_eskf_error_state(_state(), error)


def test_injection_rejects_finite_overflow_without_warning() -> None:
    nominal = _state(position_W=np.full(3, np.finfo(np.float64).max))
    error = np.zeros(15)
    error[0:3] = np.finfo(np.float64).max

    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="^ESKF error injection must remain finite$"):
            inject_eskf_error_state(nominal, error)


def test_nominal_propagation_keeps_stationary_level_ned_state() -> None:
    nominal = _state(
        position_W=np.array([4.0, -3.0, 2.0]),
        velocity_W=np.zeros(3),
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )

    propagated = propagate_eskf_nominal_state(
        nominal,
        np.array([0.0, 0.0, -9.81]),
        np.zeros(3),
        9.81,
        0.1,
    )

    np.testing.assert_array_equal(propagated.position_W, nominal.position_W)
    np.testing.assert_array_equal(propagated.velocity_W, nominal.velocity_W)
    np.testing.assert_array_equal(propagated.q_WB, nominal.q_WB)
    np.testing.assert_array_equal(propagated.accelerometer_bias_B, nominal.accelerometer_bias_B)
    np.testing.assert_array_equal(propagated.gyroscope_bias_B, nominal.gyroscope_bias_B)


def test_nominal_propagation_matches_constant_acceleration_analytic_solution() -> None:
    nominal = _state(
        position_W=np.array([1.0, 2.0, 3.0]),
        velocity_W=np.array([0.5, -1.0, 2.0]),
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )
    acceleration_W = np.array([2.0, -3.0, 4.0])
    time_step_s = 0.2

    propagated = propagate_eskf_nominal_state(
        nominal,
        acceleration_W,
        np.zeros(3),
        0.0,
        time_step_s,
    )

    np.testing.assert_allclose(
        propagated.position_W,
        nominal.position_W
        + nominal.velocity_W * time_step_s
        + 0.5 * acceleration_W * time_step_s**2,
        rtol=0.0,
        atol=1e-15,
    )
    np.testing.assert_allclose(
        propagated.velocity_W,
        nominal.velocity_W + acceleration_W * time_step_s,
        rtol=0.0,
        atol=1e-15,
    )


def test_nominal_propagation_matches_constant_body_yaw_rate() -> None:
    nominal = _state(
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )
    angular_velocity_B = np.array([0.0, 0.0, 2.0])
    time_step_s = 0.25

    propagated = propagate_eskf_nominal_state(
        nominal,
        np.zeros(3),
        angular_velocity_B,
        0.0,
        time_step_s,
    )

    expected = np.array([np.cos(0.25), 0.0, 0.0, np.sin(0.25)])
    np.testing.assert_allclose(propagated.q_WB, expected, rtol=0.0, atol=1e-15)


def test_nominal_propagation_subtracts_nonzero_imu_biases() -> None:
    accelerometer_bias_B = np.array([0.4, -0.3, 0.2])
    gyroscope_bias_B = np.array([0.1, 0.2, -0.1])
    nominal = _state(
        position_W=np.zeros(3),
        velocity_W=np.zeros(3),
        accelerometer_bias_B=accelerometer_bias_B,
        gyroscope_bias_B=gyroscope_bias_B,
    )
    corrected_specific_force_B = np.array([1.0, 2.0, -3.0])
    corrected_angular_velocity_B = np.array([0.2, -0.1, 0.3])

    propagated = propagate_eskf_nominal_state(
        nominal,
        corrected_specific_force_B + accelerometer_bias_B,
        corrected_angular_velocity_B + gyroscope_bias_B,
        0.0,
        0.1,
    )

    np.testing.assert_allclose(
        propagated.velocity_W,
        0.1 * corrected_specific_force_B,
        rtol=0.0,
        atol=1e-15,
    )
    np.testing.assert_allclose(
        propagated.q_WB,
        _rotation_vector_quaternion(0.1 * corrected_angular_velocity_B),
        rtol=0.0,
        atol=1e-15,
    )
    np.testing.assert_array_equal(propagated.accelerometer_bias_B, accelerometer_bias_B)
    np.testing.assert_array_equal(propagated.gyroscope_bias_B, gyroscope_bias_B)


def test_nominal_propagation_zero_rate_preserves_nonidentity_attitude() -> None:
    q_WB = np.array([0.5, 0.5, 0.5, 0.5])
    nominal = _state(
        q_WB=q_WB,
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )

    propagated = propagate_eskf_nominal_state(nominal, np.zeros(3), np.zeros(3), 0.0, 0.1)

    np.testing.assert_array_equal(propagated.q_WB, q_WB)


def test_nominal_propagation_rotates_specific_force_from_body_to_world() -> None:
    half_sqrt_two = np.sqrt(0.5)
    nominal = _state(
        position_W=np.zeros(3),
        velocity_W=np.zeros(3),
        q_WB=np.array([half_sqrt_two, 0.0, 0.0, half_sqrt_two]),
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )

    propagated = propagate_eskf_nominal_state(
        nominal, np.array([1.0, 0.0, 0.0]), np.zeros(3), 0.0, 0.5
    )

    np.testing.assert_allclose(
        propagated.velocity_W, np.array([0.0, 0.5, 0.0]), rtol=0.0, atol=1e-15
    )
    np.testing.assert_allclose(
        propagated.position_W, np.array([0.0, 0.125, 0.0]), rtol=0.0, atol=1e-15
    )


def test_nominal_propagation_preserves_quaternion_norm_over_many_steps() -> None:
    nominal = _state(
        q_WB=np.array([0.5, 0.5, 0.5, 0.5]),
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )
    for _ in range(5000):
        nominal = propagate_eskf_nominal_state(
            nominal,
            np.zeros(3),
            np.array([0.7, -0.4, 0.2]),
            0.0,
            0.002,
        )

    assert np.linalg.norm(nominal.q_WB) == pytest.approx(1.0, abs=1e-12)
    assert np.all(np.isfinite(nominal.q_WB))


def test_nominal_propagation_owns_outputs_and_does_not_mutate_inputs() -> None:
    nominal = _state()
    specific_force_measurement_B = np.array([1.0, 2.0, 3.0])
    angular_velocity_measurement_B = np.array([0.1, 0.2, 0.3])
    force_snapshot = specific_force_measurement_B.copy()
    angular_snapshot = angular_velocity_measurement_B.copy()
    nominal_snapshots = {
        field.name: getattr(nominal, field.name).copy() for field in fields(EskfNominalState)
    }

    propagated = propagate_eskf_nominal_state(
        nominal,
        specific_force_measurement_B,
        angular_velocity_measurement_B,
        9.81,
        0.01,
    )
    np.testing.assert_array_equal(specific_force_measurement_B, force_snapshot)
    np.testing.assert_array_equal(angular_velocity_measurement_B, angular_snapshot)
    specific_force_measurement_B[...] = 99.0
    angular_velocity_measurement_B[...] = 99.0

    for field in fields(EskfNominalState):
        np.testing.assert_array_equal(getattr(nominal, field.name), nominal_snapshots[field.name])
        output = getattr(propagated, field.name)
        assert output.flags.owndata and output.flags.c_contiguous
        assert not output.flags.writeable
        assert not np.shares_memory(output, getattr(nominal, field.name))
        assert not np.shares_memory(output, specific_force_measurement_B)
        assert not np.shares_memory(output, angular_velocity_measurement_B)


@pytest.mark.parametrize(
    ("specific_force_measurement_B", "angular_velocity_measurement_B", "expected_message"),
    [
        (np.zeros(2), np.zeros(3), "specific_force_measurement_B must have shape (3,)"),
        (np.zeros(3), np.zeros(4), "angular_velocity_measurement_B must have shape (3,)"),
        (
            np.array([np.nan, 0.0, 0.0]),
            np.zeros(2),
            "angular_velocity_measurement_B must have shape (3,)",
        ),
        (
            np.array([np.inf, 0.0, 0.0]),
            np.zeros(3),
            "specific_force_measurement_B must contain only finite values",
        ),
        (
            np.zeros(3),
            np.array([0.0, -np.inf, 0.0]),
            "angular_velocity_measurement_B must contain only finite values",
        ),
    ],
)
def test_nominal_propagation_validates_measurements(
    specific_force_measurement_B: NDArray[np.float64],
    angular_velocity_measurement_B: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=rf"^{re.escape(expected_message)}$"):
        propagate_eskf_nominal_state(
            _state(),
            specific_force_measurement_B,
            angular_velocity_measurement_B,
            9.81,
            0.01,
        )


@pytest.mark.parametrize("gravity_acceleration", [np.nan, np.inf, -0.1])
def test_nominal_propagation_rejects_invalid_gravity(gravity_acceleration: float) -> None:
    with pytest.raises(ValueError, match="^gravity_acceleration must be finite and nonnegative$"):
        propagate_eskf_nominal_state(_state(), np.zeros(3), np.zeros(3), gravity_acceleration, 0.01)


@pytest.mark.parametrize("time_step_s", [np.nan, np.inf, -0.1])
def test_nominal_propagation_rejects_invalid_time_step(time_step_s: float) -> None:
    expected = (
        "time_step_s must be finite"
        if not np.isfinite(time_step_s)
        else "time_step_s must be nonnegative"
    )
    with pytest.raises(ValueError, match=rf"^{expected}$"):
        propagate_eskf_nominal_state(_state(), np.zeros(3), np.zeros(3), 9.81, time_step_s)


def test_nominal_propagation_zero_time_is_exact_and_independently_owned() -> None:
    nominal = _state(q_WB=np.array([1.0 + 5.0e-13, 0.0, 0.0, 0.0]))

    propagated = propagate_eskf_nominal_state(
        nominal,
        np.array([2.0, -3.0, 4.0]),
        np.array([0.5, -0.6, 0.7]),
        9.81,
        0.0,
    )

    assert propagated is not nominal
    for field in fields(EskfNominalState):
        actual = getattr(propagated, field.name)
        expected = getattr(nominal, field.name)
        np.testing.assert_array_equal(actual, expected)
        assert not np.shares_memory(actual, expected)


def test_nominal_propagation_rejects_finite_overflow_without_warning() -> None:
    nominal = _state(
        position_W=np.full(3, np.finfo(np.float64).max),
        velocity_W=np.full(3, np.finfo(np.float64).max),
    )
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="^ESKF nominal propagation must remain finite$"):
            propagate_eskf_nominal_state(nominal, np.zeros(3), np.zeros(3), 0.0, 2.0)


def test_continuous_error_matrices_match_every_exact_block() -> None:
    q_WB = np.array([0.5, 0.5, 0.5, 0.5])
    accelerometer_bias_B = np.array([0.1, -0.2, 0.3])
    gyroscope_bias_B = np.array([0.01, 0.02, -0.03])
    specific_force_measurement_B = np.array([2.0, -1.0, 0.5])
    angular_velocity_measurement_B = np.array([0.4, -0.3, 0.2])
    nominal = _state(
        q_WB=q_WB,
        accelerometer_bias_B=accelerometer_bias_B,
        gyroscope_bias_B=gyroscope_bias_B,
    )
    R_WB = rotation_matrix_body_to_world(q_WB)
    corrected_force_B = specific_force_measurement_B - accelerometer_bias_B
    corrected_angular_velocity_B = angular_velocity_measurement_B - gyroscope_bias_B
    expected_F = np.zeros((15, 15))
    expected_F[0:3, 3:6] = np.eye(3)
    expected_F[3:6, 6:9] = -R_WB @ _independent_skew(corrected_force_B)
    expected_F[3:6, 9:12] = -R_WB
    expected_F[6:9, 6:9] = -_independent_skew(corrected_angular_velocity_B)
    expected_F[6:9, 12:15] = -np.eye(3)
    expected_G = np.zeros((15, 12))
    expected_G[3:6, 0:3] = -R_WB
    expected_G[6:9, 3:6] = -np.eye(3)
    expected_G[9:12, 6:9] = np.eye(3)
    expected_G[12:15, 9:12] = np.eye(3)

    F, G = eskf_continuous_error_dynamics_matrices(
        nominal, specific_force_measurement_B, angular_velocity_measurement_B
    )

    np.testing.assert_array_equal(F, expected_F)
    np.testing.assert_array_equal(G, expected_G)
    for matrix, shape in ((F, (15, 15)), (G, (15, 12))):
        assert matrix.shape == shape
        assert matrix.dtype == np.float64
        assert matrix.flags.owndata and matrix.flags.c_contiguous


def test_continuous_error_matrices_have_required_coupling_signs() -> None:
    nominal = _state(
        position_W=np.zeros(3),
        velocity_W=np.zeros(3),
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )
    force_B = np.array([1.0, 2.0, 3.0])
    angular_velocity_B = np.array([0.4, -0.5, 0.6])

    F, G = eskf_continuous_error_dynamics_matrices(nominal, force_B, angular_velocity_B)

    np.testing.assert_array_equal(F[3:6, 6:9], -_independent_skew(force_B))
    np.testing.assert_array_equal(F[3:6, 9:12], -np.eye(3))
    np.testing.assert_array_equal(F[6:9, 6:9], -_independent_skew(angular_velocity_B))
    np.testing.assert_array_equal(F[6:9, 12:15], -np.eye(3))
    np.testing.assert_array_equal(G[3:6, 0:3], -np.eye(3))
    np.testing.assert_array_equal(G[6:9, 3:6], -np.eye(3))


def test_continuous_error_matrices_are_invariant_to_position_and_velocity() -> None:
    first = _state(position_W=np.zeros(3), velocity_W=np.zeros(3))
    second = _state(
        position_W=np.array([100.0, -200.0, 300.0]),
        velocity_W=np.array([-4.0, 5.0, -6.0]),
    )
    measurements = (np.array([1.0, -2.0, 3.0]), np.array([0.4, 0.5, -0.6]))

    first_matrices = eskf_continuous_error_dynamics_matrices(first, *measurements)
    second_matrices = eskf_continuous_error_dynamics_matrices(second, *measurements)

    for first_matrix, second_matrix in zip(first_matrices, second_matrices, strict=True):
        np.testing.assert_array_equal(first_matrix, second_matrix)


def test_continuous_error_matrices_do_not_mutate_or_alias_measurements() -> None:
    nominal = _state()
    specific_force_measurement_B = np.array([1.0, 2.0, 3.0])
    angular_velocity_measurement_B = np.array([0.1, 0.2, 0.3])
    force_snapshot = specific_force_measurement_B.copy()
    angular_snapshot = angular_velocity_measurement_B.copy()

    F, G = eskf_continuous_error_dynamics_matrices(
        nominal, specific_force_measurement_B, angular_velocity_measurement_B
    )
    np.testing.assert_array_equal(specific_force_measurement_B, force_snapshot)
    np.testing.assert_array_equal(angular_velocity_measurement_B, angular_snapshot)
    specific_force_measurement_B[...] = 99.0
    angular_velocity_measurement_B[...] = 99.0

    assert not np.shares_memory(F, specific_force_measurement_B)
    assert not np.shares_memory(G, angular_velocity_measurement_B)


def test_continuous_error_matrices_use_nominal_measurement_validation() -> None:
    with pytest.raises(
        ValueError, match=r"^angular_velocity_measurement_B must have shape \(3,\)$"
    ):
        eskf_continuous_error_dynamics_matrices(_state(), np.array([np.nan, 0.0, 0.0]), np.zeros(2))


def test_continuous_error_matrices_reject_finite_overflow_without_warning() -> None:
    half_sqrt_two = np.sqrt(0.5)
    nominal = _state(q_WB=np.array([half_sqrt_two, half_sqrt_two, 0.0, 0.0]))
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="^ESKF continuous error dynamics must remain finite$"):
            eskf_continuous_error_dynamics_matrices(
                nominal,
                np.full(3, np.finfo(np.float64).max),
                np.zeros(3),
            )


def test_continuous_error_jacobian_matches_first_order_discretization() -> None:
    q_WB = _rotation_vector_quaternion(np.array([0.4, -0.3, 0.2]))
    nominal = _state(
        position_W=np.array([2.0, -1.0, 0.5]),
        velocity_W=np.array([1.2, -0.7, 0.3]),
        q_WB=q_WB,
        accelerometer_bias_B=np.array([0.12, -0.08, 0.05]),
        gyroscope_bias_B=np.array([0.03, -0.02, 0.04]),
    )
    specific_force_measurement_B = np.array([1.7, -2.2, -8.4])
    angular_velocity_measurement_B = np.array([0.6, -0.4, 0.3])
    F, _ = eskf_continuous_error_dynamics_matrices(
        nominal, specific_force_measurement_B, angular_velocity_measurement_B
    )
    perturbation = 1.0e-6
    step_sizes = np.array([0.04, 0.02, 0.01, 0.005])
    errors = []

    for time_step_s in step_sizes:
        nominal_next = propagate_eskf_nominal_state(
            nominal,
            specific_force_measurement_B,
            angular_velocity_measurement_B,
            9.81,
            float(time_step_s),
        )
        numerical_jacobian = np.empty((15, 15))
        for column in range(15):
            positive_error = np.zeros(15)
            positive_error[column] = perturbation
            negative_error = -positive_error
            positive_state = inject_eskf_error_state(nominal, positive_error)
            negative_state = inject_eskf_error_state(nominal, negative_error)
            positive_next = propagate_eskf_nominal_state(
                positive_state,
                specific_force_measurement_B,
                angular_velocity_measurement_B,
                9.81,
                float(time_step_s),
            )
            negative_next = propagate_eskf_nominal_state(
                negative_state,
                specific_force_measurement_B,
                angular_velocity_measurement_B,
                9.81,
                float(time_step_s),
            )
            numerical_jacobian[:, column] = (
                _right_local_error(nominal_next, positive_next)
                - _right_local_error(nominal_next, negative_next)
            ) / (2.0 * perturbation)

        first_order_transition = np.eye(15) + F * time_step_s
        error = float(np.linalg.norm(numerical_jacobian - first_order_transition, ord="fro"))
        errors.append(error)

    ratios = np.asarray(errors[:-1]) / np.asarray(errors[1:])
    assert np.all(np.diff(errors) < 0.0)
    assert np.all((ratios > 3.5) & (ratios < 4.5)), (errors, ratios)


def test_first_order_discretization_matches_hand_computable_identity_case() -> None:
    nominal = _state(
        position_W=np.zeros(3),
        velocity_W=np.zeros(3),
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )
    force_B = np.array([1.0, -2.0, 3.0])
    angular_velocity_B = np.array([0.2, 0.3, -0.4])
    F, G = eskf_continuous_error_dynamics_matrices(nominal, force_B, angular_velocity_B)
    Q_c = np.diag(np.repeat([2.0, 3.0, 5.0, 7.0], 3))
    time_step_s = 0.01
    expected_transition = np.eye(15)
    expected_transition[0:3, 3:6] = np.eye(3) * time_step_s
    expected_transition[3:6, 6:9] = -_independent_skew(force_B) * time_step_s
    expected_transition[3:6, 9:12] = -np.eye(3) * time_step_s
    expected_transition[6:9, 6:9] += -_independent_skew(angular_velocity_B) * time_step_s
    expected_transition[6:9, 12:15] = -np.eye(3) * time_step_s
    expected_discrete_noise = np.zeros((15, 15))
    expected_discrete_noise[3:6, 3:6] = 2.0 * time_step_s * np.eye(3)
    expected_discrete_noise[6:9, 6:9] = 3.0 * time_step_s * np.eye(3)
    expected_discrete_noise[9:12, 9:12] = 5.0 * time_step_s * np.eye(3)
    expected_discrete_noise[12:15, 12:15] = 7.0 * time_step_s * np.eye(3)

    transition, discrete_noise = discretize_eskf_error_dynamics_first_order(F, G, Q_c, time_step_s)

    np.testing.assert_array_equal(transition, expected_transition)
    np.testing.assert_allclose(discrete_noise, expected_discrete_noise, atol=1.0e-17)


def test_first_order_discretization_supports_correlated_psd_noise() -> None:
    rng = np.random.default_rng(827)
    F = rng.normal(size=(15, 15))
    G = rng.normal(size=(15, 12))
    noise_factor = rng.normal(size=(12, 7))
    Q_c = noise_factor @ noise_factor.T

    transition, discrete_noise = discretize_eskf_error_dynamics_first_order(F, G, Q_c, 0.025)

    np.testing.assert_allclose(transition, np.eye(15) + F * 0.025)
    np.testing.assert_allclose(discrete_noise, G @ Q_c @ G.T * 0.025)
    np.testing.assert_array_equal(discrete_noise, discrete_noise.T)
    assert np.linalg.eigvalsh(discrete_noise)[0] >= -1.0e-12


def test_first_order_discretization_preserves_smallest_positive_subnormal_noise() -> None:
    smallest_positive = np.nextafter(0.0, 1.0)
    G = np.zeros((15, 12))
    G[4, 0] = 1.0
    G[5, 1] = 1.0
    Q_c = np.zeros((12, 12))
    Q_c[0:2, 0:2] = smallest_positive

    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        _, discrete_noise = discretize_eskf_error_dynamics_first_order(
            np.zeros((15, 15)), G, Q_c, 1.0
        )

    assert discrete_noise[4, 4] == smallest_positive
    assert discrete_noise[4, 5] == smallest_positive
    assert discrete_noise[5, 4] == smallest_positive
    assert discrete_noise[5, 5] == smallest_positive
    np.testing.assert_array_equal(discrete_noise, discrete_noise.T)
    assert np.all(np.isfinite(discrete_noise))
    assert np.linalg.eigvalsh(discrete_noise)[0] >= 0.0


@pytest.mark.parametrize(
    ("argument_name", "invalid", "message"),
    [
        (
            "continuous_state_matrix",
            np.zeros((14, 15)),
            "continuous_state_matrix must have shape (15, 15)",
        ),
        (
            "continuous_noise_input_matrix",
            np.zeros((15, 11)),
            "continuous_noise_input_matrix must have shape (15, 12)",
        ),
        (
            "continuous_noise_covariance",
            np.zeros((11, 12)),
            "continuous_noise_covariance must have shape (12, 12)",
        ),
    ],
)
def test_first_order_discretization_rejects_matrix_shapes(
    argument_name: str, invalid: NDArray[np.float64], message: str
) -> None:
    arguments = {
        "continuous_state_matrix": np.zeros((15, 15)),
        "continuous_noise_input_matrix": np.zeros((15, 12)),
        "continuous_noise_covariance": np.eye(12),
    }
    arguments[argument_name] = invalid
    with pytest.raises(ValueError, match=f"^{re.escape(message)}$"):
        discretize_eskf_error_dynamics_first_order(**arguments, time_step_s=0.01)


@pytest.mark.parametrize(
    "argument_name",
    [
        "continuous_state_matrix",
        "continuous_noise_input_matrix",
        "continuous_noise_covariance",
    ],
)
def test_first_order_discretization_rejects_nonfinite_matrices(
    argument_name: str,
) -> None:
    arguments = {
        "continuous_state_matrix": np.zeros((15, 15)),
        "continuous_noise_input_matrix": np.zeros((15, 12)),
        "continuous_noise_covariance": np.eye(12),
    }
    arguments[argument_name][0, 0] = np.nan
    with pytest.raises(
        ValueError,
        match=f"^{argument_name} must contain only finite values$",
    ):
        discretize_eskf_error_dynamics_first_order(**arguments, time_step_s=0.01)


def test_first_order_discretization_rejects_nonsymmetric_or_indefinite_noise() -> None:
    nonsymmetric = np.eye(12)
    nonsymmetric[0, 1] = 0.1
    with pytest.raises(ValueError, match="^continuous_noise_covariance must be symmetric$"):
        discretize_eskf_error_dynamics_first_order(
            np.zeros((15, 15)), np.zeros((15, 12)), nonsymmetric, 0.01
        )

    indefinite = np.eye(12)
    indefinite[4, 4] = -1.0e-4
    with pytest.raises(
        ValueError,
        match="^continuous_noise_covariance must be positive semidefinite$",
    ):
        discretize_eskf_error_dynamics_first_order(
            np.zeros((15, 15)), np.zeros((15, 12)), indefinite, 0.01
        )


def test_matrix_validation_rejects_local_asymmetry_masked_by_large_diagonal() -> None:
    locally_asymmetric = np.eye(12)
    locally_asymmetric[0, 0] = 1.0e24
    locally_asymmetric[1, 2] = 1.0e-8

    with pytest.raises(ValueError, match="^continuous_noise_covariance must be symmetric$"):
        discretize_eskf_error_dynamics_first_order(
            np.zeros((15, 15)),
            np.zeros((15, 12)),
            locally_asymmetric,
            0.01,
        )


def test_matrix_validation_reports_smallest_subnormal_local_asymmetry() -> None:
    smallest_positive = np.nextafter(0.0, 1.0)
    covariance = np.eye(15)
    covariance[0, 0] = smallest_positive
    covariance[1, 1] = smallest_positive
    covariance[0, 1] = smallest_positive
    covariance[1, 0] = 0.0

    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="^covariance must be symmetric$"):
            propagate_eskf_covariance(covariance, np.eye(15), np.zeros((15, 15)))


@pytest.mark.parametrize(
    "locally_indefinite",
    [
        np.diag([1.0e24, -1.0, *([1.0] * 10)]),
        np.block(
            [
                [np.array([[1.0e24]]), np.zeros((1, 2)), np.zeros((1, 9))],
                [
                    np.zeros((2, 1)),
                    np.array([[1.0e-6, 2.0e-6], [2.0e-6, 1.0e-6]]),
                    np.zeros((2, 9)),
                ],
                [np.zeros((9, 1)), np.zeros((9, 2)), np.eye(9)],
            ]
        ),
        np.diag([0.0, *([1.0] * 11)])
        + np.diag([np.nextafter(0.0, 1.0), *([0.0] * 10)], k=1)
        + np.diag([np.nextafter(0.0, 1.0), *([0.0] * 10)], k=-1),
    ],
)
def test_matrix_validation_rejects_local_indefiniteness_masked_by_large_eigenvalue(
    locally_indefinite: NDArray[np.float64],
) -> None:
    with pytest.raises(
        ValueError,
        match="^continuous_noise_covariance must be positive semidefinite$",
    ):
        discretize_eskf_error_dynamics_first_order(
            np.zeros((15, 15)),
            np.zeros((15, 12)),
            locally_indefinite,
            0.01,
        )


def test_matrix_validation_accepts_psd_scales_and_local_roundoff_asymmetry() -> None:
    scaled_psd = np.diag(np.logspace(-24, 24, 12))
    scaled_psd[5, 6] = 1.0e-12
    scaled_psd[6, 5] = np.nextafter(1.0e-12, np.inf)

    _, discrete_noise = discretize_eskf_error_dynamics_first_order(
        np.zeros((15, 15)), np.zeros((15, 12)), scaled_psd, 0.01
    )

    np.testing.assert_array_equal(discrete_noise, np.zeros((15, 15)))


@pytest.mark.parametrize(
    ("time_step_s", "message"),
    [
        (np.nan, "time_step_s must be finite"),
        (np.inf, "time_step_s must be finite"),
        (-0.01, "time_step_s must be nonnegative"),
    ],
)
def test_first_order_discretization_rejects_invalid_time_step(
    time_step_s: float, message: str
) -> None:
    with pytest.raises(ValueError, match=f"^{message}$"):
        discretize_eskf_error_dynamics_first_order(
            np.zeros((15, 15)), np.zeros((15, 12)), np.eye(12), time_step_s
        )


def test_first_order_discretization_zero_time_is_exact_without_arithmetic() -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        transition, discrete_noise = discretize_eskf_error_dynamics_first_order(
            np.full((15, 15), np.finfo(np.float64).max),
            np.full((15, 12), np.finfo(np.float64).max),
            np.eye(12),
            0.0,
        )

    np.testing.assert_array_equal(transition, np.eye(15))
    np.testing.assert_array_equal(discrete_noise, np.zeros((15, 15)))


def test_first_order_discretization_owns_outputs_and_preserves_inputs() -> None:
    F = np.arange(225.0).reshape(15, 15) / 1000.0
    G = np.arange(180.0).reshape(15, 12) / 1000.0
    Q_c = np.eye(12)
    snapshots = tuple(matrix.copy() for matrix in (F, G, Q_c))

    transition, discrete_noise = discretize_eskf_error_dynamics_first_order(F, G, Q_c, 0.01)

    for matrix, snapshot in zip((F, G, Q_c), snapshots, strict=True):
        np.testing.assert_array_equal(matrix, snapshot)
    for output in (transition, discrete_noise):
        assert output.dtype == np.float64
        assert output.flags.owndata and output.flags.c_contiguous
        assert not any(np.shares_memory(output, matrix) for matrix in (F, G, Q_c))


def test_first_order_discretization_rejects_overflow_without_warning() -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(
            ValueError, match="^ESKF first-order discretization must remain finite$"
        ):
            discretize_eskf_error_dynamics_first_order(
                np.full((15, 15), np.finfo(np.float64).max),
                np.zeros((15, 12)),
                np.eye(12),
                2.0,
            )


@pytest.mark.parametrize(
    ("argument_name", "invalid", "message"),
    [
        ("covariance", np.zeros((14, 15)), "covariance must have shape (15, 15)"),
        (
            "transition_matrix",
            np.zeros((15, 14)),
            "transition_matrix must have shape (15, 15)",
        ),
        (
            "discrete_process_noise_covariance",
            np.zeros((14, 15)),
            "discrete_process_noise_covariance must have shape (15, 15)",
        ),
    ],
)
def test_covariance_propagation_rejects_matrix_shapes(
    argument_name: str, invalid: NDArray[np.float64], message: str
) -> None:
    arguments = {
        "covariance": np.eye(15),
        "transition_matrix": np.eye(15),
        "discrete_process_noise_covariance": np.eye(15),
    }
    arguments[argument_name] = invalid
    with pytest.raises(ValueError, match=f"^{re.escape(message)}$"):
        propagate_eskf_covariance(**arguments)


@pytest.mark.parametrize(
    "argument_name",
    ["covariance", "transition_matrix", "discrete_process_noise_covariance"],
)
def test_covariance_propagation_rejects_nonfinite_matrices(
    argument_name: str,
) -> None:
    arguments = {
        "covariance": np.eye(15),
        "transition_matrix": np.eye(15),
        "discrete_process_noise_covariance": np.eye(15),
    }
    arguments[argument_name][0, 0] = np.inf
    with pytest.raises(ValueError, match=f"^{argument_name} must contain only finite values$"):
        propagate_eskf_covariance(**arguments)


@pytest.mark.parametrize("argument_name", ["covariance", "discrete_process_noise_covariance"])
def test_covariance_propagation_rejects_nonsymmetric_matrices(
    argument_name: str,
) -> None:
    arguments = {
        "covariance": np.eye(15),
        "transition_matrix": np.eye(15),
        "discrete_process_noise_covariance": np.eye(15),
    }
    arguments[argument_name][0, 1] = 0.1
    with pytest.raises(ValueError, match=f"^{argument_name} must be symmetric$"):
        propagate_eskf_covariance(**arguments)


@pytest.mark.parametrize("argument_name", ["covariance", "discrete_process_noise_covariance"])
def test_covariance_propagation_rejects_materially_indefinite_matrices(
    argument_name: str,
) -> None:
    arguments = {
        "covariance": np.eye(15),
        "transition_matrix": np.eye(15),
        "discrete_process_noise_covariance": np.eye(15),
    }
    arguments[argument_name][3, 3] = -1.0e-3
    with pytest.raises(ValueError, match=f"^{argument_name} must be positive semidefinite$"):
        propagate_eskf_covariance(**arguments)


def test_covariance_propagation_accepts_smallest_positive_subnormal_variance() -> None:
    smallest_positive = np.nextafter(0.0, 1.0)
    covariance = np.eye(15)
    covariance[0, 0] = smallest_positive

    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        propagated = propagate_eskf_covariance(covariance, np.eye(15), np.zeros((15, 15)))

    np.testing.assert_array_equal(propagated, covariance)
    assert not np.shares_memory(propagated, covariance)


def test_covariance_propagation_returns_bit_symmetric_signed_zero_entries() -> None:
    covariance = np.eye(15)
    covariance[0, 1] = 0.0
    covariance[1, 0] = -0.0

    propagated = propagate_eskf_covariance(covariance, np.eye(15), np.zeros((15, 15)))

    np.testing.assert_array_equal(propagated.view(np.uint64), propagated.T.copy().view(np.uint64))


def test_covariance_propagation_preserves_smallest_subnormal_after_permutation() -> None:
    smallest_positive = np.nextafter(0.0, 1.0)
    covariance = np.eye(15)
    covariance[0, 0] = smallest_positive
    transition = np.eye(15)
    transition[[0, 1]] = transition[[1, 0]]

    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        propagated = propagate_eskf_covariance(covariance, transition, np.zeros((15, 15)))

    assert propagated[1, 1] == smallest_positive
    np.testing.assert_array_equal(propagated, propagated.T)
    assert np.all(np.isfinite(propagated))
    assert np.linalg.eigvalsh(propagated)[0] >= 0.0


def test_covariance_propagation_preserves_symmetry_psd_and_ownership() -> None:
    rng = np.random.default_rng(3301)
    covariance_factor = rng.normal(size=(15, 15))
    noise_factor = rng.normal(size=(15, 8))
    covariance = covariance_factor @ covariance_factor.T
    transition = np.eye(15) + 0.01 * rng.normal(size=(15, 15))
    discrete_noise = 1.0e-4 * (noise_factor @ noise_factor.T)
    snapshots = tuple(matrix.copy() for matrix in (covariance, transition, discrete_noise))
    expected = transition @ covariance @ transition.T + discrete_noise
    expected = 0.5 * (expected + expected.T)

    propagated = propagate_eskf_covariance(covariance, transition, discrete_noise)

    np.testing.assert_allclose(propagated, expected)
    np.testing.assert_array_equal(propagated, propagated.T)
    assert np.linalg.eigvalsh(propagated)[0] >= -1.0e-12
    assert propagated.dtype == np.float64
    assert propagated.flags.owndata and propagated.flags.c_contiguous
    for matrix, snapshot in zip((covariance, transition, discrete_noise), snapshots, strict=True):
        np.testing.assert_array_equal(matrix, snapshot)
        assert not np.shares_memory(propagated, matrix)


def test_covariance_propagation_rejects_overflow_without_warning() -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="^ESKF covariance propagation must remain finite$"):
            propagate_eskf_covariance(
                np.full((15, 15), np.finfo(np.float64).max),
                np.eye(15) * 2.0,
                np.zeros((15, 15)),
            )


def test_predict_matches_separately_composed_public_functions() -> None:
    nominal = _state(q_WB=_rotation_vector_quaternion(np.array([0.2, -0.1, 0.3])))
    covariance = np.diag(np.linspace(0.1, 1.5, 15))
    force_B = np.array([0.8, -0.4, -9.0])
    angular_velocity_B = np.array([0.2, -0.3, 0.1])
    Q_c = np.diag(np.linspace(1.0e-5, 1.2e-4, 12))
    F, G = eskf_continuous_error_dynamics_matrices(nominal, force_B, angular_velocity_B)
    transition, discrete_noise = discretize_eskf_error_dynamics_first_order(F, G, Q_c, 0.005)
    expected_nominal = propagate_eskf_nominal_state(
        nominal, force_B, angular_velocity_B, 9.81, 0.005
    )
    expected_covariance = propagate_eskf_covariance(covariance, transition, discrete_noise)

    predicted_nominal, predicted_covariance = predict_eskf(
        nominal,
        covariance,
        force_B,
        angular_velocity_B,
        9.81,
        Q_c,
        0.005,
    )

    for field in fields(EskfNominalState):
        np.testing.assert_array_equal(
            getattr(predicted_nominal, field.name), getattr(expected_nominal, field.name)
        )
    np.testing.assert_array_equal(predicted_covariance, expected_covariance)


@pytest.mark.parametrize("seed", [7, 404, 9103])
def test_predict_preserves_covariance_psd_for_fixed_seed_spd_cases(seed: int) -> None:
    rng = np.random.default_rng(seed)
    covariance_factor = rng.normal(size=(15, 15))
    noise_factor = rng.normal(size=(12, 12))
    covariance = covariance_factor @ covariance_factor.T + 0.1 * np.eye(15)
    Q_c = 1.0e-5 * (noise_factor @ noise_factor.T)

    _, predicted_covariance = predict_eskf(
        _state(q_WB=_rotation_vector_quaternion(rng.normal(size=3))),
        covariance,
        rng.normal(size=3),
        rng.normal(size=3),
        9.81,
        Q_c,
        0.002,
    )

    np.testing.assert_array_equal(predicted_covariance, predicted_covariance.T)
    assert np.linalg.eigvalsh(predicted_covariance)[0] >= -1.0e-11


def test_predict_does_not_mutate_or_alias_inputs() -> None:
    nominal = _state()
    covariance = np.eye(15)
    force_B = np.array([0.0, 0.0, -9.81])
    angular_velocity_B = np.zeros(3)
    Q_c = np.eye(12) * 1.0e-6
    snapshots = tuple(matrix.copy() for matrix in (covariance, force_B, angular_velocity_B, Q_c))

    predicted_nominal, predicted_covariance = predict_eskf(
        nominal, covariance, force_B, angular_velocity_B, 9.81, Q_c, 0.01
    )

    for matrix, snapshot in zip(
        (covariance, force_B, angular_velocity_B, Q_c), snapshots, strict=True
    ):
        np.testing.assert_array_equal(matrix, snapshot)
        assert not np.shares_memory(predicted_covariance, matrix)
        for field in fields(EskfNominalState):
            assert not np.shares_memory(getattr(predicted_nominal, field.name), matrix)


def test_predict_supports_zero_process_noise() -> None:
    nominal = _state(accelerometer_bias_B=np.zeros(3), gyroscope_bias_B=np.zeros(3))
    covariance = np.eye(15)
    F, G = eskf_continuous_error_dynamics_matrices(
        nominal, np.array([0.0, 0.0, -9.81]), np.zeros(3)
    )
    transition, discrete_noise = discretize_eskf_error_dynamics_first_order(
        F, G, np.zeros((12, 12)), 0.01
    )

    _, predicted_covariance = predict_eskf(
        nominal,
        covariance,
        np.array([0.0, 0.0, -9.81]),
        np.zeros(3),
        9.81,
        np.zeros((12, 12)),
        0.01,
    )

    np.testing.assert_array_equal(discrete_noise, np.zeros((15, 15)))
    np.testing.assert_allclose(predicted_covariance, transition @ covariance @ transition.T)


def test_predict_zero_time_preserves_exact_values_with_independent_outputs() -> None:
    nominal = _state(q_WB=np.array([1.0 + 5.0e-13, 0.0, 0.0, 0.0]))
    covariance = np.diag(np.logspace(-12, 12, 15))
    Q_c = np.diag(np.logspace(-18, 6, 12))

    predicted_nominal, predicted_covariance = predict_eskf(
        nominal,
        covariance,
        np.array([0.2, -0.3, -9.4]),
        np.array([0.1, 0.2, -0.4]),
        9.81,
        Q_c,
        0.0,
    )

    for field in fields(EskfNominalState):
        actual = getattr(predicted_nominal, field.name)
        expected = getattr(nominal, field.name)
        np.testing.assert_array_equal(actual, expected)
        assert not np.shares_memory(actual, expected)
    np.testing.assert_array_equal(predicted_covariance, covariance)
    assert not np.shares_memory(predicted_covariance, covariance)


def test_predict_zero_time_avoids_extreme_bias_subtraction_without_warning() -> None:
    maximum = np.finfo(np.float64).max
    nominal = _state(
        accelerometer_bias_B=np.full(3, maximum),
        gyroscope_bias_B=np.full(3, maximum),
    )
    covariance = np.diag(np.linspace(0.1, 1.5, 15))

    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        predicted_nominal, predicted_covariance = predict_eskf(
            nominal,
            covariance,
            np.full(3, -maximum),
            np.full(3, -maximum),
            9.81,
            np.eye(12),
            0.0,
        )

    for field in fields(EskfNominalState):
        actual = getattr(predicted_nominal, field.name)
        expected = getattr(nominal, field.name)
        np.testing.assert_array_equal(actual, expected)
        assert not np.shares_memory(actual, expected)
    np.testing.assert_array_equal(predicted_covariance, covariance)
    assert not np.shares_memory(predicted_covariance, covariance)


@pytest.mark.parametrize(
    ("argument_name", "invalid", "message"),
    [
        (
            "specific_force_measurement_B",
            np.zeros(2),
            "specific_force_measurement_B must have shape (3,)",
        ),
        (
            "angular_velocity_measurement_B",
            np.array([np.nan, 0.0, 0.0]),
            "angular_velocity_measurement_B must contain only finite values",
        ),
        ("gravity_acceleration", -0.1, "gravity_acceleration must be finite and nonnegative"),
        ("covariance", np.diag([-1.0, *([1.0] * 14)]), "covariance must be positive semidefinite"),
        (
            "continuous_noise_covariance",
            np.diag([-1.0, *([1.0] * 11)]),
            "continuous_noise_covariance must be positive semidefinite",
        ),
    ],
)
def test_predict_zero_time_validates_every_public_input_boundary(
    argument_name: str, invalid: NDArray[np.float64] | float, message: str
) -> None:
    arguments: dict[str, NDArray[np.float64] | float | EskfNominalState] = {
        "nominal_state": _state(),
        "covariance": np.eye(15),
        "specific_force_measurement_B": np.zeros(3),
        "angular_velocity_measurement_B": np.zeros(3),
        "gravity_acceleration": 9.81,
        "continuous_noise_covariance": np.eye(12),
        "time_step_s": 0.0,
    }
    arguments[argument_name] = invalid

    with pytest.raises(ValueError, match=f"^{re.escape(message)}$"):
        predict_eskf(**arguments)  # type: ignore[arg-type]


def test_predict_remains_finite_symmetric_and_psd_over_long_sequence() -> None:
    nominal = _state(
        position_W=np.zeros(3),
        velocity_W=np.zeros(3),
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )
    covariance = np.eye(15) * 1.0e-3
    Q_c = np.diag(np.repeat([1.0e-4, 1.0e-5, 1.0e-7, 1.0e-8], 3))

    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        for _ in range(750):
            nominal, covariance = predict_eskf(
                nominal,
                covariance,
                np.array([0.2, -0.1, -9.81]),
                np.array([0.01, -0.02, 0.015]),
                9.81,
                Q_c,
                0.002,
            )

    for field in fields(EskfNominalState):
        assert np.all(np.isfinite(getattr(nominal, field.name)))
    assert np.isclose(np.linalg.norm(nominal.q_WB), 1.0, rtol=1.0e-12, atol=1.0e-12)
    assert np.all(np.isfinite(covariance))
    np.testing.assert_array_equal(covariance, covariance.T)
    assert np.linalg.eigvalsh(covariance)[0] >= -1.0e-12


@pytest.mark.parametrize("norm_error", [1.5e-12, -1.5e-12])
def test_nominal_state_rejects_quaternion_outside_downstream_rotation_contract(
    norm_error: float,
) -> None:
    q_WB = np.array([1.0 + norm_error, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError, match="unit norm"):
        rotation_matrix_body_to_world(q_WB)
    with pytest.raises(ValueError, match="unit norm"):
        _state(q_WB=q_WB)
