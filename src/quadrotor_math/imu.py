import numpy as np
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
