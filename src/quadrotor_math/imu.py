import numpy as np
from numpy.random import Generator
from numpy.typing import NDArray


def ideal_accelerometer_specific_force_body(
    translational_acceleration_W: NDArray[np.float64],
    R_WB: NDArray[np.float64],
    gravity_acceleration: float,
) -> NDArray[np.float64]:
    """Calculate ideal accelerometer specific force in the FRD body frame.

    The calculation uses ``f_B = R_WB.T @ (a_W - g_W)``.

    Args:
        translational_acceleration_W: World-frame inertial acceleration in
            m/s².
        R_WB: Body-to-world rotation matrix.
        gravity_acceleration: Nonnegative gravity magnitude in m/s².

    Returns:
        Ideal accelerometer specific force in the forward-right-down (FRD)
        body frame in m/s².

    Raises:
        ValueError: If ``translational_acceleration_W`` does not have shape
            ``(3,)``, if ``R_WB`` does not have shape ``(3, 3)``, if either
            array contains a non-finite value, or if ``gravity_acceleration``
            is non-finite or negative.
    """
    if translational_acceleration_W.shape != (3,):
        raise ValueError("translational_acceleration_W must have shape (3,)")

    if R_WB.shape != (3, 3):
        raise ValueError("R_WB must have shape (3, 3)")

    if not np.all(np.isfinite(translational_acceleration_W)):
        raise ValueError("translational_acceleration_W must contain only finite values")

    if not np.all(np.isfinite(R_WB)):
        raise ValueError("R_WB must contain only finite values")

    if not np.isfinite(gravity_acceleration) or gravity_acceleration < 0.0:
        raise ValueError("gravity_acceleration must be finite and nonnegative")

    gravity_W = np.array(
        [0.0, 0.0, gravity_acceleration],
        dtype=np.float64,
    )

    return R_WB.T @ (translational_acceleration_W - gravity_W)


def accelerometer_specific_force_with_bias_body(
    ideal_specific_force_B: NDArray[np.float64],
    accelerometer_bias_B: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return biased accelerometer specific force in the FRD body frame.

    Args:
        ideal_specific_force_B: Ideal body-aligned accelerometer output expected
            to have shape ``(3,)``, expressed in the forward-right-down (FRD)
            body frame in m/s².
        accelerometer_bias_B: Constant additive three-axis bias expected to have
            shape ``(3,)``, expressed in the FRD body frame in m/s².

    Returns:
        Biased body-frame specific force in m/s².

    Raises:
        ValueError: If either input does not have shape ``(3,)`` or contains a
            non-finite value.
    """
    if ideal_specific_force_B.shape != (3,):
        raise ValueError("ideal_specific_force_B must have shape (3,)")

    if accelerometer_bias_B.shape != (3,):
        raise ValueError("accelerometer_bias_B must have shape (3,)")

    if not np.all(np.isfinite(ideal_specific_force_B)):
        raise ValueError("ideal_specific_force_B must contain only finite values")

    if not np.all(np.isfinite(accelerometer_bias_B)):
        raise ValueError("accelerometer_bias_B must contain only finite values")

    return ideal_specific_force_B + accelerometer_bias_B


def ideal_gyroscope_angular_velocity_body(
    omega_B: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return an ideal body-aligned gyroscope measurement in the FRD body frame.

    Args:
        omega_B: Rigid-body angular velocity with shape ``(3,)``, expressed in
            the forward-right-down (FRD) body frame in rad/s.

    Returns:
        Independently owned float64 array containing the ideal body-aligned
        gyroscope measurement in rad/s.

    Raises:
        ValueError: If ``omega_B`` does not have shape ``(3,)`` or contains a
            non-finite value.
    """
    if omega_B.shape != (3,):
        raise ValueError("omega_B must have shape (3,)")

    if not np.all(np.isfinite(omega_B)):
        raise ValueError("omega_B must contain only finite values")

    return np.array(
        omega_B,
        dtype=np.float64,
        copy=True,
    )


def gyroscope_angular_velocity_with_bias_body(
    ideal_angular_velocity_B: NDArray[np.float64],
    gyroscope_bias_B: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return biased gyroscope angular velocity in the FRD body frame.

    Args:
        ideal_angular_velocity_B: Ideal body-aligned gyroscope output expected
            to have shape ``(3,)``, expressed in the forward-right-down (FRD)
            body frame in rad/s.
        gyroscope_bias_B: Constant additive three-axis bias expected to have
            shape ``(3,)``, expressed in the FRD body frame in rad/s.

    Returns:
        Biased body-frame angular velocity in rad/s.

    Raises:
        ValueError: If either input does not have shape ``(3,)`` or contains a
            non-finite value.
    """
    if ideal_angular_velocity_B.shape != (3,):
        raise ValueError("ideal_angular_velocity_B must have shape (3,)")

    if gyroscope_bias_B.shape != (3,):
        raise ValueError("gyroscope_bias_B must have shape (3,)")

    if not np.all(np.isfinite(ideal_angular_velocity_B)):
        raise ValueError("ideal_angular_velocity_B must contain only finite values")

    if not np.all(np.isfinite(gyroscope_bias_B)):
        raise ValueError("gyroscope_bias_B must contain only finite values")

    return ideal_angular_velocity_B + gyroscope_bias_B


def gyroscope_angular_velocity_measurement_body(
    ideal_angular_velocity_B: NDArray[np.float64],
    gyroscope_bias_B: NDArray[np.float64],
    noise_standard_deviation_B: NDArray[np.float64],
    rng: Generator,
) -> NDArray[np.float64]:
    """Return a biased gyroscope measurement with per-axis white noise.

    Args:
        ideal_angular_velocity_B: Ideal body-aligned gyroscope value expected
            to have shape ``(3,)``, expressed along FRD body axes in rad/s.
        gyroscope_bias_B: Constant additive three-axis bias expected to have
            shape ``(3,)``, expressed along FRD body axes in rad/s.
        noise_standard_deviation_B: Per-axis, per-sample white-noise standard
            deviations expected to have shape ``(3,)``, expressed along FRD
            body axes in rad/s. Changing sample rate does not automatically
            rescale these supplied standard deviations.
        rng: Caller-owned generator that supplies the stochastic sample. One
            call consumes three standard-normal variates in one vectorized
            draw.

    Returns:
        Noisy biased angular-velocity measurement along FRD body axes in rad/s.

    Raises:
        ValueError: If any of the three array inputs does not have shape
            ``(3,)``, any contains a non-finite value, or
            ``noise_standard_deviation_B`` contains a negative value.
    """
    if ideal_angular_velocity_B.shape != (3,):
        raise ValueError("ideal_angular_velocity_B must have shape (3,)")

    if gyroscope_bias_B.shape != (3,):
        raise ValueError("gyroscope_bias_B must have shape (3,)")

    if noise_standard_deviation_B.shape != (3,):
        raise ValueError("noise_standard_deviation_B must have shape (3,)")

    if not np.all(np.isfinite(ideal_angular_velocity_B)):
        raise ValueError("ideal_angular_velocity_B must contain only finite values")

    if not np.all(np.isfinite(gyroscope_bias_B)):
        raise ValueError("gyroscope_bias_B must contain only finite values")

    if not np.all(np.isfinite(noise_standard_deviation_B)):
        raise ValueError("noise_standard_deviation_B must contain only finite values")

    if np.any(noise_standard_deviation_B < 0.0):
        raise ValueError("noise_standard_deviation_B must be nonnegative")

    standard_normal_sample_B = rng.standard_normal(3)
    return (
        ideal_angular_velocity_B
        + gyroscope_bias_B
        + noise_standard_deviation_B * standard_normal_sample_B
    )
