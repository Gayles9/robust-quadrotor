"""Second-order endpoint IMU integration with conditional sample-noise memory.

There are 15 physical error states and six temporary IMU noise variables. Adjacent
intervals share a sample, so its conditional mean and cross-covariance must survive
prediction and correction. See ADR 0010 for the discrete map and its derivatives.
"""

from dataclasses import dataclass, replace

import numpy as np
from numpy.typing import NDArray

from .eskf import (
    EskfMeasurementUpdate,
    EskfNominalState,
    _quaternion_product,
    _rotation_vector_quaternion,
    _scaled_innovation_cholesky,
    _symmetrized_float64_matrix,
    _validated_float64_matrix,
    eskf_reset_jacobian,
    update_eskf_linear_measurement,
)
from .rotations import (
    normalize_quaternion_body_to_world,
    rotation_matrix_body_to_world,
    skew_symmetric,
)


def _array(
    name: str, values: NDArray[np.float64], shape: tuple[int, ...], *, covariance: bool = False
) -> NDArray[np.float64]:
    if not isinstance(values, np.ndarray) or values.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    if values.dtype.kind not in "fiu":
        raise ValueError(f"{name} must contain real numeric values")
    with np.errstate(over="ignore", invalid="ignore"):
        owned = np.array(values, dtype=np.float64, order="C", copy=True)
    if not np.all(np.isfinite(owned)):
        raise ValueError(f"{name} must contain only finite values")
    if covariance:
        owned = _validated_float64_matrix(
            name, owned, (shape[0], shape[0]), symmetric_positive_semidefinite=True
        )
    owned.flags.writeable = False
    return owned


def _nonnegative_scalar(name: str, value: float, *, positive: bool = False) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError(f"{name} must be a finite real scalar")
    try:
        result = float(value)
    except OverflowError:
        raise ValueError(f"{name} must be finite") from None
    if not np.isfinite(result) or result < 0 or (positive and result == 0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return result


@dataclass(frozen=True, slots=True, eq=False)
class EskfSampledImuNoise:
    """Independent endpoint noise and bias-walk increment assumptions.

    Both matrices are (6,6), in FRD accelerometer-then-gyro order. Sample
    covariance uses squared m/s² and rad/s (and their cross units). Bias-walk
    spectral density uses those covariance units per second. Samples are
    independent across epochs and independent of bias increments and the prior.
    """

    sample_covariance_B: NDArray[np.float64]
    bias_walk_spectral_density_B: NDArray[np.float64]

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            object.__setattr__(
                self, name, _array(name, getattr(self, name), (6, 6), covariance=True)
            )


@dataclass(frozen=True, slots=True, eq=False)
class EskfEndpointState:
    """Physical estimate plus current sample's conditional noise information.

    joint_covariance is (21,21), ordered [physical error (15), sample noise (6)].
    imu_noise_mean_B is (6,) in accelerometer-then-gyro order. Its mean and all
    cross terms are required for continuation, even after an observation update.
    """

    nominal_state: EskfNominalState
    imu_noise_mean_B: NDArray[np.float64]
    joint_covariance: NDArray[np.float64]

    def __post_init__(self) -> None:
        if not isinstance(self.nominal_state, EskfNominalState):
            raise TypeError("nominal_state must be an EskfNominalState")
        object.__setattr__(self, "nominal_state", replace(self.nominal_state))
        object.__setattr__(
            self, "imu_noise_mean_B", _array("imu_noise_mean_B", self.imu_noise_mean_B, (6,))
        )
        object.__setattr__(
            self,
            "joint_covariance",
            _array("joint_covariance", self.joint_covariance, (21, 21), covariance=True),
        )


def initialize_eskf_endpoint(
    nominal_state: EskfNominalState, covariance: NDArray[np.float64], noise: EskfSampledImuNoise
) -> EskfEndpointState:
    """Initialize once with an independent physical prior and first IMU sample."""
    if not isinstance(noise, EskfSampledImuNoise):
        raise TypeError("noise must be an EskfSampledImuNoise")
    noise = replace(noise)
    C = np.zeros((21, 21))
    C[:15, :15] = _array("covariance", covariance, (15, 15), covariance=True)
    C[15:, 15:] = noise.sample_covariance_B
    return EskfEndpointState(nominal_state, np.zeros(6), C)


def eskf_endpoint_map(
    nominal_state: EskfNominalState,
    specific_force_start_B: NDArray[np.float64],
    angular_velocity_start_B: NDArray[np.float64],
    specific_force_end_B: NDArray[np.float64],
    angular_velocity_end_B: NDArray[np.float64],
    imu_noise_mean_B: NDArray[np.float64],
    gravity_acceleration: float,
    time_step_s: float,
) -> tuple[EskfNominalState, NDArray[np.float64], NDArray[np.float64]]:
    """Return nominal endpoint, A (21,21) and B (21,12) for the actual discrete map.

    Inputs are instantaneous FRD specific force [m/s²] and rate [rad/s] at
    interval endpoints. A acts on [physical error, old sample-noise error]; B
    acts on [new sample noise, bias endpoint increment]. h must be positive.
    New sample noise and bias increment have zero nominal means. NED gravity
    is [0,0,g]. The new noise mean is zero before any same-epoch observations.
    """
    if not isinstance(nominal_state, EskfNominalState):
        raise TypeError("nominal_state must be an EskfNominalState")
    state = replace(nominal_state)
    measured = [
        _array(name, value, (3,))
        for name, value in (
            ("specific_force_start_B", specific_force_start_B),
            ("angular_velocity_start_B", angular_velocity_start_B),
            ("specific_force_end_B", specific_force_end_B),
            ("angular_velocity_end_B", angular_velocity_end_B),
        )
    ]
    mean = _array("imu_noise_mean_B", imu_noise_mean_B, (6,))
    g = _nonnegative_scalar("gravity_acceleration", gravity_acceleration)
    h = _nonnegative_scalar("time_step_s", time_step_s, positive=True)
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            f0 = measured[0] - state.accelerometer_bias_B - mean[:3]
            w0 = measured[1] - state.gyroscope_bias_B - mean[3:]
            f1 = measured[2] - state.accelerometer_bias_B
            w1 = measured[3] - state.gyroscope_bias_B
            phi = (0.5 * w0 + 0.5 * w1) * h
            delta_q_BB = normalize_quaternion_body_to_world(_rotation_vector_quaternion(phi))
            q_WB = normalize_quaternion_body_to_world(_quaternion_product(state.q_WB, delta_q_BB))
            E = rotation_matrix_body_to_world(delta_q_BB)
            R0 = rotation_matrix_body_to_world(state.q_WB)
            R1 = rotation_matrix_body_to_world(q_WB)
            gravity_W = np.array([0.0, 0.0, g])
            a0, a1 = gravity_W + R0 @ f0, gravity_W + R1 @ f1
            next_state = EskfNominalState(
                state.position_W + h * state.velocity_W + h * h * (a0 / 3 + a1 / 6),
                state.velocity_W + h * (a0 / 2 + a1 / 2),
                q_WB,
                state.accelerometer_bias_B,
                state.gyroscope_bias_B,
            )
            correction = np.zeros(15)
            correction[6:9] = phi
            Jr = eskf_reset_jacobian(correction)[6:9, 6:9]
            # Columns: physical 15, old noise 6, new noise 6, bias increment 6.
            theta = np.zeros((3, 33))
            theta[:, 6:9] = E.T
            theta[:, 12:15] = -h * Jr
            for block in (18, 24, 30):
                theta[:, block : block + 3] = -h / 2 * Jr
            acc0 = np.zeros((3, 33))
            acc0[:, 6:9] = -R0 @ skew_symmetric(f0)
            acc0[:, 9:12] = -R0
            acc0[:, 15:18] = -R0
            acc1 = -R1 @ skew_symmetric(f1) @ theta
            for block in (9, 21, 27):
                acc1[:, block : block + 3] -= R1
            derivative = np.zeros((21, 33))
            derivative[:3] = h * h * (acc0 / 3 + acc1 / 6)
            derivative[:3, :3] += np.eye(3)
            derivative[:3, 3:6] += h * np.eye(3)
            derivative[3:6] = h * (acc0 / 2 + acc1 / 2)
            derivative[3:6, 3:6] += np.eye(3)
            derivative[6:9] = theta
            derivative[9:15, 9:15] = np.eye(6)
            derivative[9:15, 27:33] = np.eye(6)
            derivative[15:21, 21:27] = np.eye(6)
    except (FloatingPointError, OverflowError):
        raise ValueError("ESKF endpoint map must remain finite") from None
    return (
        next_state,
        _array("transition_matrix", derivative[:, :21], (21, 21)),
        _array("noise_input_matrix", derivative[:, 21:], (21, 12)),
    )


def predict_eskf_endpoint(
    current: EskfEndpointState,
    specific_force_start_B: NDArray[np.float64],
    angular_velocity_start_B: NDArray[np.float64],
    specific_force_end_B: NDArray[np.float64],
    angular_velocity_end_B: NDArray[np.float64],
    gravity_acceleration: float,
    noise: EskfSampledImuNoise,
    time_step_s: float,
) -> EskfEndpointState:
    """Predict jointly, then retain the new sample's covariance and cross terms."""
    if not isinstance(current, EskfEndpointState) or not isinstance(noise, EskfSampledImuNoise):
        raise TypeError("prediction requires EskfEndpointState and EskfSampledImuNoise")
    current, noise = replace(current), replace(noise)
    h = _nonnegative_scalar("time_step_s", time_step_s, positive=True)
    nominal, A, B = eskf_endpoint_map(
        current.nominal_state,
        specific_force_start_B,
        angular_velocity_start_B,
        specific_force_end_B,
        angular_velocity_end_B,
        current.imu_noise_mean_B,
        gravity_acceleration,
        h,
    )
    try:
        with np.errstate(over="raise", invalid="raise"):
            D = np.zeros((12, 12))
            D[:6, :6] = noise.sample_covariance_B
            D[6:, 6:] = noise.bias_walk_spectral_density_B * h
            C = _symmetrized_float64_matrix(A @ current.joint_covariance @ A.T + B @ D @ B.T)
    except FloatingPointError:
        raise ValueError("ESKF endpoint covariance must remain finite") from None
    return EskfEndpointState(nominal, np.zeros(6), C)


def update_eskf_endpoint(
    current: EskfEndpointState,
    measurement: NDArray[np.float64],
    predicted_measurement: NDArray[np.float64],
    measurement_jacobian: NDArray[np.float64],
    measurement_noise_covariance: NDArray[np.float64],
) -> tuple[EskfEndpointState, EskfMeasurementUpdate]:
    """Condition physical and shared-noise estimates with one independent observation.

    H has (m,15) physical columns and implicit zero sample-noise columns. Return
    the usual physical diagnostics and the complete conditioned memory. Gating
    belongs upstream and must occur before this function is called.
    """
    if not isinstance(current, EskfEndpointState):
        raise TypeError("current must be an EskfEndpointState")
    current = replace(current)
    update = update_eskf_linear_measurement(
        current.nominal_state,
        current.joint_covariance[:15, :15],
        measurement,
        predicted_measurement,
        measurement_jacobian,
        measurement_noise_covariance,
    )
    size = measurement.size
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            H = np.zeros((size, 21))
            H[:, :15] = measurement_jacobian
            scales, factor = _scaled_innovation_cholesky(update.innovation_covariance)
            cross = current.joint_covariance[15:, :15] @ measurement_jacobian.T
            noise_gain = (
                np.linalg.solve(factor.T, np.linalg.solve(factor, cross.T / scales[:, None]))
                / scales[:, None]
            ).T
            K = np.vstack((update.kalman_gain, noise_gain))
            mean = current.imu_noise_mean_B + noise_gain @ update.innovation
            residual = np.eye(21) - K @ H
            C = (
                residual @ current.joint_covariance @ residual.T
                + K @ measurement_noise_covariance @ K.T
            )
            reset = np.eye(21)
            reset[:15, :15] = eskf_reset_jacobian(update.error_state_correction)
            C = _symmetrized_float64_matrix(reset @ C @ reset.T)
    except (FloatingPointError, np.linalg.LinAlgError):
        raise ValueError("ESKF endpoint correction must remain finite and solvable") from None
    posterior = EskfEndpointState(update.nominal_state, mean, C)
    return posterior, replace(update, covariance=posterior.joint_covariance[:15, :15])
