import numpy as np
from numpy.typing import NDArray


def rotor_thrusts_from_speeds(
    rotor_omega: NDArray[np.float64],
    thrust_coefficient: float,
) -> NDArray[np.float64]:
    """Return per-rotor static thrust magnitudes from rotor angular speeds.

    Args:
        rotor_omega: Rotor angular speeds in rad/s.
        thrust_coefficient: Thrust coefficient ``k_f`` in N/(rad/s)^2.

    Returns:
        A float64 array containing nonnegative per-rotor thrust magnitudes in
        newtons.

    Raises:
        ValueError: If ``rotor_omega`` does not have shape ``(4,)``, contains
            a non-finite component, or contains a negative component, or if
            ``thrust_coefficient`` is non-finite or not positive.
    """
    if rotor_omega.shape != (4,):
        raise ValueError(f"rotor_omega must have shape (4,), got {rotor_omega.shape}")

    if not np.all(np.isfinite(rotor_omega)):
        raise ValueError("rotor_omega must contain only finite values")

    if not np.all(rotor_omega >= 0.0):
        raise ValueError("rotor_omega must be nonnegative")

    if not np.isfinite(thrust_coefficient):
        raise ValueError("thrust_coefficient must be finite")

    if thrust_coefficient <= 0.0:
        raise ValueError("thrust_coefficient must be positive")

    return np.asarray(thrust_coefficient * rotor_omega**2, dtype=np.float64)


def thrust_force_body_from_rotor_thrusts(
    rotor_thrusts: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return the body-frame thrust force from positive rotor thrust magnitudes.

    Positive thrust magnitudes produce force along negative body z because the
    body frame uses the forward-right-down (FRD) convention.

    Raises:
        ValueError: If ``rotor_thrusts`` does not have shape ``(4,)``, contains
            a non-finite component, or contains a negative component.
    """
    if rotor_thrusts.shape != (4,):
        raise ValueError("rotor_thrusts must have shape (4,)")

    if not np.all(np.isfinite(rotor_thrusts)):
        raise ValueError("rotor_thrusts must contain only finite values")

    if not np.all(rotor_thrusts >= 0.0):
        raise ValueError("rotor_thrusts must be nonnegative")

    return np.array(
        [0.0, 0.0, -np.sum(rotor_thrusts)],
        dtype=np.float64,
    )
