import numpy as np
from numpy.typing import NDArray


def translational_acceleration_world_from_body_force(
    force_B: NDArray[np.float64],
    R_WB: NDArray[np.float64],
    mass: float,
    gravity_acceleration: float,
) -> NDArray[np.float64]:
    """Return translational acceleration expressed in the NED world frame.

    Args:
        force_B: Force with shape ``(3,)``, expressed in the forward-right-down
            (FRD) body frame in newtons.
        R_WB: Active rotation matrix with shape ``(3, 3)`` that maps
            body-coordinate vectors into north-east-down (NED) world
            coordinates.
        mass: Vehicle mass in kilograms.
        gravity_acceleration: Positive gravity magnitude in m/s^2.

    Returns:
        ``acceleration_W`` with shape ``(3,)``, expressed in the NED world
        frame in m/s^2, where positive world z points downward.

    Raises:
        ValueError: If ``force_B`` does not have shape ``(3,)`` or ``R_WB``
            does not have shape ``(3, 3)``, or if ``force_B`` or ``R_WB``
            contains a non-finite value, or if ``mass`` is non-finite or
            non-positive, or if ``gravity_acceleration`` is non-finite or
            non-positive.
    """
    if force_B.shape != (3,):
        raise ValueError("force_B must have shape (3,)")

    if R_WB.shape != (3, 3):
        raise ValueError("R_WB must have shape (3, 3)")

    if not np.all(np.isfinite(force_B)):
        raise ValueError("force_B must contain only finite values")

    if not np.all(np.isfinite(R_WB)):
        raise ValueError("R_WB must contain only finite values")

    if not np.isfinite(mass):
        raise ValueError("mass must be finite")

    if mass <= 0.0:
        raise ValueError("mass must be positive")

    if not np.isfinite(gravity_acceleration):
        raise ValueError("gravity_acceleration must be finite")

    if gravity_acceleration <= 0.0:
        raise ValueError("gravity_acceleration must be positive")

    gravity_W = np.array(
        [0.0, 0.0, gravity_acceleration],
        dtype=np.float64,
    )
    acceleration_W = gravity_W + (R_WB @ force_B) / mass
    return np.asarray(acceleration_W, dtype=np.float64)
