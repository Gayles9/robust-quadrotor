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
