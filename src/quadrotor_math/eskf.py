"""Error-state Kalman filter prediction mathematics."""

from dataclasses import dataclass
from math import cos, sin

import numpy as np
from numpy.typing import NDArray

from .rotations import (
    normalize_quaternion_body_to_world,
    rotation_matrix_body_to_world,
    skew_symmetric,
)


def _symmetrized_float64_matrix(values: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return a bit-symmetric copy while preserving equal entries exactly."""
    symmetric_values = np.array(values, dtype=np.float64, order="C", copy=True)
    upper_rows, upper_columns = np.triu_indices(values.shape[0], k=1)
    upper_values = values[upper_rows, upper_columns]
    lower_values = values[upper_columns, upper_rows]
    unequal_entries = upper_values != lower_values
    symmetric_values[upper_columns, upper_rows] = upper_values
    if not np.any(unequal_entries):
        return symmetric_values

    unequal_upper_rows = upper_rows[unequal_entries]
    unequal_upper_columns = upper_columns[unequal_entries]
    averaged_values = 0.5 * upper_values[unequal_entries] + 0.5 * lower_values[unequal_entries]
    symmetric_values[unequal_upper_rows, unequal_upper_columns] = averaged_values
    symmetric_values[unequal_upper_columns, unequal_upper_rows] = averaged_values
    return symmetric_values


def _validated_float64_matrix(
    name: str,
    values: NDArray[np.float64],
    shape: tuple[int, int],
    *,
    symmetric_positive_semidefinite: bool = False,
) -> NDArray[np.float64]:
    """Return an owned finite matrix with optional symmetry and PSD checks."""
    if values.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    owned_values = np.array(values, dtype=np.float64, order="C", copy=True)
    if not np.all(np.isfinite(owned_values)):
        raise ValueError(f"{name} must contain only finite values")
    if not symmetric_positive_semidefinite:
        return owned_values

    upper_rows, upper_columns = np.triu_indices(owned_values.shape[0], k=1)
    upper_values = owned_values[upper_rows, upper_columns]
    lower_values = owned_values[upper_columns, upper_rows]
    unequal_entries = upper_values != lower_values
    if np.any(unequal_entries):
        unequal_upper_values = upper_values[unequal_entries]
        unequal_lower_values = lower_values[unequal_entries]
        mirrored_scale = np.maximum(np.abs(unequal_upper_values), np.abs(unequal_lower_values))
        with np.errstate(over="raise", divide="raise", invalid="raise"):
            normalized_difference = np.abs(
                unequal_upper_values / mirrored_scale - unequal_lower_values / mirrored_scale
            )
        symmetry_roundoff_relative_tolerance = (
            16.0 * float(owned_values.shape[0]) * np.finfo(np.float64).eps
        )
        if np.any(normalized_difference > symmetry_roundoff_relative_tolerance):
            raise ValueError(f"{name} must be symmetric")
    input_diagonal = np.diag(owned_values)
    if np.any(input_diagonal < 0.0):
        raise ValueError(f"{name} must be positive semidefinite")
    zero_diagonal = input_diagonal == 0.0
    if np.any(owned_values[zero_diagonal, :] != 0.0) or np.any(
        owned_values[:, zero_diagonal] != 0.0
    ):
        raise ValueError(f"{name} must be positive semidefinite")

    owned_values = _symmetrized_float64_matrix(owned_values)
    diagonal = np.diag(owned_values)
    positive_diagonal = ~zero_diagonal
    if not np.any(positive_diagonal):
        return owned_values
    active_values = owned_values[np.ix_(positive_diagonal, positive_diagonal)]
    active_standard_deviations = np.sqrt(diagonal[positive_diagonal])
    scale_products = np.outer(active_standard_deviations, active_standard_deviations)
    try:
        with np.errstate(over="raise", divide="raise", invalid="raise"):
            scale_normalized_values = active_values / scale_products
        eigenvalues = np.linalg.eigvalsh(scale_normalized_values)
    except (FloatingPointError, np.linalg.LinAlgError):
        raise ValueError(f"{name} must be positive semidefinite") from None
    if not np.all(np.isfinite(eigenvalues)):
        raise ValueError(f"{name} must be positive semidefinite")
    eigenvalue_scale = float(np.max(np.abs(eigenvalues), initial=0.0))
    roundoff_tolerance = (
        64.0 * np.finfo(np.float64).eps * float(owned_values.shape[0]) * eigenvalue_scale
    )
    if float(np.min(eigenvalues, initial=0.0)) < -roundoff_tolerance:
        raise ValueError(f"{name} must be positive semidefinite")
    return owned_values


def _rotation_vector_quaternion(phi_B: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the Hamilton quaternion exponential of one body-local rotation vector."""
    angle = float(np.linalg.norm(phi_B))
    if not np.isfinite(angle):
        raise FloatingPointError

    angle_squared = angle * angle
    if angle < 1.0e-8:
        scalar = 1.0 - angle_squared / 8.0 + angle_squared * angle_squared / 384.0
        vector_scale = 0.5 - angle_squared / 48.0 + angle_squared * angle_squared / 3840.0
    else:
        scalar = cos(0.5 * angle)
        vector_scale = sin(0.5 * angle) / angle

    delta_q_BB = np.empty(4, dtype=np.float64)
    delta_q_BB[0] = scalar
    delta_q_BB[1:] = vector_scale * phi_B
    if not np.all(np.isfinite(delta_q_BB)):
        raise FloatingPointError
    return delta_q_BB


def _quaternion_product(
    q_left_WB: NDArray[np.float64],
    q_right_BB: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return the scalar-first Hamilton product ``q_left_WB ⊗ q_right_BB``."""
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


def _validated_imu_measurements(
    specific_force_measurement_B: NDArray[np.float64],
    angular_velocity_measurement_B: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return owned finite IMU vectors after shape-first validation."""
    if specific_force_measurement_B.shape != (3,):
        raise ValueError("specific_force_measurement_B must have shape (3,)")
    if angular_velocity_measurement_B.shape != (3,):
        raise ValueError("angular_velocity_measurement_B must have shape (3,)")

    owned_specific_force_measurement_B = np.array(
        specific_force_measurement_B, dtype=np.float64, order="C", copy=True
    )
    owned_angular_velocity_measurement_B = np.array(
        angular_velocity_measurement_B, dtype=np.float64, order="C", copy=True
    )
    if not np.all(np.isfinite(owned_specific_force_measurement_B)):
        raise ValueError("specific_force_measurement_B must contain only finite values")
    if not np.all(np.isfinite(owned_angular_velocity_measurement_B)):
        raise ValueError("angular_velocity_measurement_B must contain only finite values")
    return owned_specific_force_measurement_B, owned_angular_velocity_measurement_B


@dataclass(frozen=True, slots=True, eq=False)
class EskfNominalState:
    """Store the immutable nominal state used by the 15-state ESKF."""

    position_W: NDArray[np.float64]
    velocity_W: NDArray[np.float64]
    q_WB: NDArray[np.float64]
    accelerometer_bias_B: NDArray[np.float64]
    gyroscope_bias_B: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Validate and own nominal-state arrays."""
        arrays = (
            ("position_W", self.position_W, (3,)),
            ("velocity_W", self.velocity_W, (3,)),
            ("q_WB", self.q_WB, (4,)),
            ("accelerometer_bias_B", self.accelerometer_bias_B, (3,)),
            ("gyroscope_bias_B", self.gyroscope_bias_B, (3,)),
        )
        for field_name, values, expected_shape in arrays:
            if values.shape != expected_shape:
                raise ValueError(f"{field_name} must have shape {expected_shape}")

        owned_arrays: dict[str, NDArray[np.float64]] = {}
        for field_name, values, _ in arrays:
            owned_values = np.array(values, dtype=np.float64, order="C", copy=True)
            if not np.all(np.isfinite(owned_values)):
                raise ValueError(f"{field_name} must contain only finite values")
            owned_arrays[field_name] = owned_values

        try:
            with np.errstate(over="raise", invalid="raise"):
                quaternion_norm = np.linalg.norm(owned_arrays["q_WB"])
        except FloatingPointError:
            raise ValueError("q_WB must have unit norm") from None
        if not np.isclose(
            quaternion_norm,
            1.0,
            rtol=1.0e-12,
            atol=1.0e-12,
        ):
            raise ValueError("q_WB must have unit norm")

        for field_name, _, _ in arrays:
            owned_values = owned_arrays[field_name]
            owned_values.flags.writeable = False
            object.__setattr__(self, field_name, owned_values)


def inject_eskf_error_state(
    nominal_state: EskfNominalState,
    error_state: NDArray[np.float64],
) -> EskfNominalState:
    """Inject one right-local 15-state error into a nominal state."""
    if error_state.shape != (15,):
        raise ValueError("error_state must have shape (15,)")
    owned_error_state = np.array(error_state, dtype=np.float64, order="C", copy=True)
    if not np.all(np.isfinite(owned_error_state)):
        raise ValueError("error_state must contain only finite values")

    try:
        with np.errstate(over="raise", invalid="raise"):
            position_W = nominal_state.position_W + owned_error_state[0:3]
            velocity_W = nominal_state.velocity_W + owned_error_state[3:6]
            delta_q_BB = _rotation_vector_quaternion(owned_error_state[6:9])
            q_WB = _quaternion_product(nominal_state.q_WB, delta_q_BB)
            accelerometer_bias_B = nominal_state.accelerometer_bias_B + owned_error_state[9:12]
            gyroscope_bias_B = nominal_state.gyroscope_bias_B + owned_error_state[12:15]
            candidate_arrays = (
                position_W,
                velocity_W,
                q_WB,
                accelerometer_bias_B,
                gyroscope_bias_B,
            )
            if not all(np.all(np.isfinite(values)) for values in candidate_arrays):
                raise FloatingPointError
            q_WB = normalize_quaternion_body_to_world(q_WB)
    except FloatingPointError:
        raise ValueError("ESKF error injection must remain finite") from None

    return EskfNominalState(
        position_W=position_W,
        velocity_W=velocity_W,
        q_WB=q_WB,
        accelerometer_bias_B=accelerometer_bias_B,
        gyroscope_bias_B=gyroscope_bias_B,
    )


def propagate_eskf_nominal_state(
    nominal_state: EskfNominalState,
    specific_force_measurement_B: NDArray[np.float64],
    angular_velocity_measurement_B: NDArray[np.float64],
    gravity_acceleration: float,
    time_step_s: float,
) -> EskfNominalState:
    """Propagate the nominal ESKF state over one constant-IMU interval."""
    (
        owned_specific_force_measurement_B,
        owned_angular_velocity_measurement_B,
    ) = _validated_imu_measurements(
        specific_force_measurement_B,
        angular_velocity_measurement_B,
    )
    if not np.isfinite(gravity_acceleration) or gravity_acceleration < 0.0:
        raise ValueError("gravity_acceleration must be finite and nonnegative")
    if not np.isfinite(time_step_s):
        raise ValueError("time_step_s must be finite")
    if time_step_s < 0.0:
        raise ValueError("time_step_s must be nonnegative")
    if time_step_s == 0.0:
        return EskfNominalState(
            position_W=nominal_state.position_W,
            velocity_W=nominal_state.velocity_W,
            q_WB=nominal_state.q_WB,
            accelerometer_bias_B=nominal_state.accelerometer_bias_B,
            gyroscope_bias_B=nominal_state.gyroscope_bias_B,
        )

    try:
        with np.errstate(over="raise", invalid="raise"):
            specific_force_corrected_B = (
                owned_specific_force_measurement_B - nominal_state.accelerometer_bias_B
            )
            angular_velocity_corrected_B = (
                owned_angular_velocity_measurement_B - nominal_state.gyroscope_bias_B
            )
            gravity_W = np.array([0.0, 0.0, gravity_acceleration], dtype=np.float64)
            R_WB = rotation_matrix_body_to_world(nominal_state.q_WB)
            acceleration_W = gravity_W + R_WB @ specific_force_corrected_B
            time_step_squared_s2 = time_step_s * time_step_s
            position_W = (
                nominal_state.position_W
                + nominal_state.velocity_W * time_step_s
                + 0.5 * acceleration_W * time_step_squared_s2
            )
            velocity_W = nominal_state.velocity_W + acceleration_W * time_step_s
            delta_q_BB = _rotation_vector_quaternion(angular_velocity_corrected_B * time_step_s)
            q_WB = normalize_quaternion_body_to_world(
                _quaternion_product(nominal_state.q_WB, delta_q_BB)
            )
            candidate_arrays = (position_W, velocity_W, q_WB)
            if not all(np.all(np.isfinite(values)) for values in candidate_arrays):
                raise FloatingPointError
    except FloatingPointError:
        raise ValueError("ESKF nominal propagation must remain finite") from None

    return EskfNominalState(
        position_W=position_W,
        velocity_W=velocity_W,
        q_WB=q_WB,
        accelerometer_bias_B=nominal_state.accelerometer_bias_B,
        gyroscope_bias_B=nominal_state.gyroscope_bias_B,
    )


def eskf_continuous_error_dynamics_matrices(
    nominal_state: EskfNominalState,
    specific_force_measurement_B: NDArray[np.float64],
    angular_velocity_measurement_B: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return the continuous 15-state right-local ESKF matrices ``F`` and ``G``."""
    (
        owned_specific_force_measurement_B,
        owned_angular_velocity_measurement_B,
    ) = _validated_imu_measurements(
        specific_force_measurement_B,
        angular_velocity_measurement_B,
    )

    try:
        with np.errstate(over="raise", invalid="raise"):
            specific_force_corrected_B = (
                owned_specific_force_measurement_B - nominal_state.accelerometer_bias_B
            )
            angular_velocity_corrected_B = (
                owned_angular_velocity_measurement_B - nominal_state.gyroscope_bias_B
            )
            R_WB = rotation_matrix_body_to_world(nominal_state.q_WB)

            F = np.zeros((15, 15), dtype=np.float64)
            F[0:3, 3:6] = np.eye(3)
            F[3:6, 6:9] = -R_WB @ skew_symmetric(specific_force_corrected_B)
            F[3:6, 9:12] = -R_WB
            F[6:9, 6:9] = -skew_symmetric(angular_velocity_corrected_B)
            F[6:9, 12:15] = -np.eye(3)

            G = np.zeros((15, 12), dtype=np.float64)
            G[3:6, 0:3] = -R_WB
            G[6:9, 3:6] = -np.eye(3)
            G[9:12, 6:9] = np.eye(3)
            G[12:15, 9:12] = np.eye(3)

            if not np.all(np.isfinite(F)) or not np.all(np.isfinite(G)):
                raise FloatingPointError
    except FloatingPointError:
        raise ValueError("ESKF continuous error dynamics must remain finite") from None

    return F, G


def discretize_eskf_error_dynamics_first_order(
    continuous_state_matrix: NDArray[np.float64],
    continuous_noise_input_matrix: NDArray[np.float64],
    continuous_noise_covariance: NDArray[np.float64],
    time_step_s: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Discretize ESKF error dynamics with the first-order high-rate baseline."""
    matrix_shapes = (
        ("continuous_state_matrix", continuous_state_matrix, (15, 15)),
        (
            "continuous_noise_input_matrix",
            continuous_noise_input_matrix,
            (15, 12),
        ),
        (
            "continuous_noise_covariance",
            continuous_noise_covariance,
            (12, 12),
        ),
    )
    for name, values, shape in matrix_shapes:
        if values.shape != shape:
            raise ValueError(f"{name} must have shape {shape}")

    owned_continuous_state_matrix = _validated_float64_matrix(
        "continuous_state_matrix", continuous_state_matrix, (15, 15)
    )
    owned_continuous_noise_input_matrix = _validated_float64_matrix(
        "continuous_noise_input_matrix", continuous_noise_input_matrix, (15, 12)
    )
    owned_continuous_noise_covariance = _validated_float64_matrix(
        "continuous_noise_covariance",
        continuous_noise_covariance,
        (12, 12),
        symmetric_positive_semidefinite=True,
    )
    if not np.isfinite(time_step_s):
        raise ValueError("time_step_s must be finite")
    if time_step_s < 0.0:
        raise ValueError("time_step_s must be nonnegative")
    if time_step_s == 0.0:
        return np.eye(15, dtype=np.float64), np.zeros((15, 15), dtype=np.float64)

    try:
        with np.errstate(over="raise", invalid="raise"):
            transition_matrix = (
                np.eye(15, dtype=np.float64) + owned_continuous_state_matrix * time_step_s
            )
            raw_discrete_process_noise_covariance = (
                owned_continuous_noise_input_matrix
                @ owned_continuous_noise_covariance
                @ owned_continuous_noise_input_matrix.T
                * time_step_s
            )
            discrete_process_noise_covariance = _symmetrized_float64_matrix(
                raw_discrete_process_noise_covariance
            )
            if not np.all(np.isfinite(transition_matrix)) or not np.all(
                np.isfinite(discrete_process_noise_covariance)
            ):
                raise FloatingPointError
    except FloatingPointError:
        raise ValueError("ESKF first-order discretization must remain finite") from None

    return (
        np.array(transition_matrix, dtype=np.float64, order="C", copy=True),
        np.array(
            discrete_process_noise_covariance,
            dtype=np.float64,
            order="C",
            copy=True,
        ),
    )


def propagate_eskf_covariance(
    covariance: NDArray[np.float64],
    transition_matrix: NDArray[np.float64],
    discrete_process_noise_covariance: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Propagate one ESKF covariance with a discrete transition and process noise."""
    matrix_shapes = (
        ("covariance", covariance, (15, 15)),
        ("transition_matrix", transition_matrix, (15, 15)),
        (
            "discrete_process_noise_covariance",
            discrete_process_noise_covariance,
            (15, 15),
        ),
    )
    for name, values, shape in matrix_shapes:
        if values.shape != shape:
            raise ValueError(f"{name} must have shape {shape}")

    owned_covariance = _validated_float64_matrix(
        "covariance",
        covariance,
        (15, 15),
        symmetric_positive_semidefinite=True,
    )
    owned_transition_matrix = _validated_float64_matrix(
        "transition_matrix", transition_matrix, (15, 15)
    )
    owned_discrete_process_noise_covariance = _validated_float64_matrix(
        "discrete_process_noise_covariance",
        discrete_process_noise_covariance,
        (15, 15),
        symmetric_positive_semidefinite=True,
    )
    if np.array_equal(owned_transition_matrix, np.eye(15)) and not np.any(
        owned_discrete_process_noise_covariance
    ):
        return np.array(owned_covariance, dtype=np.float64, order="C", copy=True)

    try:
        with np.errstate(over="raise", invalid="raise"):
            propagated_covariance = (
                owned_transition_matrix @ owned_covariance @ owned_transition_matrix.T
                + owned_discrete_process_noise_covariance
            )
            propagated_covariance = _symmetrized_float64_matrix(propagated_covariance)
            if not np.all(np.isfinite(propagated_covariance)):
                raise FloatingPointError
    except FloatingPointError:
        raise ValueError("ESKF covariance propagation must remain finite") from None

    return _validated_float64_matrix(
        "propagated_covariance",
        propagated_covariance,
        (15, 15),
        symmetric_positive_semidefinite=True,
    )


def predict_eskf(
    nominal_state: EskfNominalState,
    covariance: NDArray[np.float64],
    specific_force_measurement_B: NDArray[np.float64],
    angular_velocity_measurement_B: NDArray[np.float64],
    gravity_acceleration: float,
    continuous_noise_covariance: NDArray[np.float64],
    time_step_s: float,
) -> tuple[EskfNominalState, NDArray[np.float64]]:
    """Predict the nominal ESKF state and covariance over one IMU interval."""
    if time_step_s == 0.0:
        propagated_nominal_state = propagate_eskf_nominal_state(
            nominal_state,
            specific_force_measurement_B,
            angular_velocity_measurement_B,
            gravity_acceleration,
            time_step_s,
        )
        transition_matrix, discrete_process_noise_covariance = (
            discretize_eskf_error_dynamics_first_order(
                np.zeros((15, 15), dtype=np.float64),
                np.zeros((15, 12), dtype=np.float64),
                continuous_noise_covariance,
                time_step_s,
            )
        )
        propagated_covariance = propagate_eskf_covariance(
            covariance,
            transition_matrix,
            discrete_process_noise_covariance,
        )
        return propagated_nominal_state, propagated_covariance

    continuous_state_matrix, continuous_noise_input_matrix = (
        eskf_continuous_error_dynamics_matrices(
            nominal_state,
            specific_force_measurement_B,
            angular_velocity_measurement_B,
        )
    )
    transition_matrix, discrete_process_noise_covariance = (
        discretize_eskf_error_dynamics_first_order(
            continuous_state_matrix,
            continuous_noise_input_matrix,
            continuous_noise_covariance,
            time_step_s,
        )
    )
    propagated_nominal_state = propagate_eskf_nominal_state(
        nominal_state,
        specific_force_measurement_B,
        angular_velocity_measurement_B,
        gravity_acceleration,
        time_step_s,
    )
    propagated_covariance = propagate_eskf_covariance(
        covariance,
        transition_matrix,
        discrete_process_noise_covariance,
    )
    return propagated_nominal_state, propagated_covariance
