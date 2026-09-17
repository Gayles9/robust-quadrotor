import numpy as np
from numpy.typing import NDArray


def motor_speed_first_order_step(
    actual_rotor_omega: NDArray[np.float64],
    commanded_rotor_omega: NDArray[np.float64],
    minimum_rotor_omega: float,
    maximum_rotor_omega: float,
    motor_time_constant_s: float,
    time_step_s: float,
) -> NDArray[np.float64]:
    """Advance actual rotor speeds over one constant-command interval.

    Speeds are in rad/s. The commanded speeds are clipped to the rotor limits,
    then the exact finite-time first-order response approaches that saturated
    target. Return the next actual rotor speeds in rad/s. Zero elapsed time
    returns a copy of the unchanged actual rotor speeds.

    Raises:
        ValueError: If either rotor-speed array does not have shape ``(4,)`` or
            contains a non-finite value, if an actual speed is outside the
            supplied rotor-speed limits, or if a commanded speed is negative.
            Also raised if either speed limit, the motor time constant, or the
            time step is non-finite; if the minimum speed or time step is
            negative; if the maximum speed is not greater than the minimum;
            or if the motor time constant is not positive.
    """
    if actual_rotor_omega.shape != (4,):
        raise ValueError("actual_rotor_omega must have shape (4,)")

    if commanded_rotor_omega.shape != (4,):
        raise ValueError("commanded_rotor_omega must have shape (4,)")

    if not np.all(np.isfinite(actual_rotor_omega)):
        raise ValueError("actual_rotor_omega must contain only finite values")

    if not np.all(np.isfinite(commanded_rotor_omega)):
        raise ValueError("commanded_rotor_omega must contain only finite values")

    if not np.isfinite(minimum_rotor_omega):
        raise ValueError("minimum_rotor_omega must be finite")

    if not np.isfinite(maximum_rotor_omega):
        raise ValueError("maximum_rotor_omega must be finite")

    if minimum_rotor_omega < 0.0:
        raise ValueError("minimum_rotor_omega must be nonnegative")

    if maximum_rotor_omega <= minimum_rotor_omega:
        raise ValueError("maximum_rotor_omega must be greater than minimum_rotor_omega")

    if not np.isfinite(motor_time_constant_s):
        raise ValueError("motor_time_constant_s must be finite")

    if motor_time_constant_s <= 0.0:
        raise ValueError("motor_time_constant_s must be positive")

    if not np.isfinite(time_step_s):
        raise ValueError("time_step_s must be finite")

    if time_step_s < 0.0:
        raise ValueError("time_step_s must be nonnegative")

    if not np.all(
        (actual_rotor_omega >= minimum_rotor_omega) & (actual_rotor_omega <= maximum_rotor_omega)
    ):
        raise ValueError("actual_rotor_omega must be within rotor speed limits")

    if not np.all(commanded_rotor_omega >= 0.0):
        raise ValueError("commanded_rotor_omega must be nonnegative")

    if time_step_s == 0.0:
        return actual_rotor_omega.copy()

    saturated_target = np.clip(
        commanded_rotor_omega,
        minimum_rotor_omega,
        maximum_rotor_omega,
    )
    decay = np.exp(-time_step_s / motor_time_constant_s)
    next_actual_rotor_omega = saturated_target + (actual_rotor_omega - saturated_target) * decay
    return np.asarray(next_actual_rotor_omega, dtype=np.float64)


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


def reaction_moment_body_from_rotor_speeds(
    rotor_omega: NDArray[np.float64],
    rotor_spin_directions: NDArray[np.float64],
    moment_coefficient: float,
) -> NDArray[np.float64]:
    """Return the body yaw reaction moment from rotor speeds.

    Args:
        rotor_omega: Four nonnegative rotor-speed magnitudes with shape ``(4,)``
            in rad/s.
        rotor_spin_directions: Rotor spin directions with shape ``(4,)`` that
            contain only ``-1`` or ``+1``. A value of ``+1`` is positive rotor
            rotation about ``+z_B``; the body reaction moment acts in the
            opposite direction.
        moment_coefficient: Moment coefficient ``k_m`` in N·m/(rad/s)^2.

    Returns:
        The body-frame reaction moment with shape ``(3,)`` in N·m.

    Raises:
        ValueError: If ``rotor_omega`` or ``rotor_spin_directions`` does not
            have shape ``(4,)``, either contains a non-finite component,
            ``rotor_omega`` contains a negative component,
            ``rotor_spin_directions`` contains a value other than ``-1`` or
            ``+1``, or ``moment_coefficient`` is non-finite or not positive.
    """
    if rotor_omega.shape != (4,):
        raise ValueError("rotor_omega must have shape (4,)")

    if rotor_spin_directions.shape != (4,):
        raise ValueError("rotor_spin_directions must have shape (4,)")

    if not np.all(np.isfinite(rotor_omega)):
        raise ValueError("rotor_omega must contain only finite values")

    if not np.all(rotor_omega >= 0.0):
        raise ValueError("rotor_omega must be nonnegative")

    if not np.all(np.isfinite(rotor_spin_directions)):
        raise ValueError("rotor_spin_directions must contain only finite values")

    if not np.all((rotor_spin_directions == -1.0) | (rotor_spin_directions == 1.0)):
        raise ValueError("rotor_spin_directions must contain only -1 or +1")

    if not np.isfinite(moment_coefficient):
        raise ValueError("moment_coefficient must be finite")

    if moment_coefficient <= 0.0:
        raise ValueError("moment_coefficient must be positive")

    tau_z = -moment_coefficient * np.sum(rotor_spin_directions * rotor_omega**2)
    return np.array([0.0, 0.0, tau_z], dtype=np.float64)


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


def thrust_moment_body_from_rotor_thrusts(
    rotor_positions_B: NDArray[np.float64],
    rotor_thrusts: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return the moment from offset rotor thrust forces in the FRD body frame.

    Args:
        rotor_positions_B: Rotor positions with shape ``(4, 3)``, expressed in
            the forward-right-down (FRD) body frame in metres.
        rotor_thrusts: Rotor thrust magnitudes with shape ``(4,)`` in newtons.

    Returns:
        The body-frame moment with shape ``(3,)`` in N·m.

    Raises:
        ValueError: If ``rotor_positions_B`` does not have shape ``(4, 3)`` or
            contains a non-finite component, or if ``rotor_thrusts`` does not
            have shape ``(4,)``, contains a non-finite component, or contains a
            negative component.
    """
    if rotor_positions_B.shape != (4, 3):
        raise ValueError("rotor_positions_B must have shape (4, 3)")

    if rotor_thrusts.shape != (4,):
        raise ValueError("rotor_thrusts must have shape (4,)")

    if not np.all(np.isfinite(rotor_positions_B)):
        raise ValueError("rotor_positions_B must contain only finite values")

    if not np.all(np.isfinite(rotor_thrusts)):
        raise ValueError("rotor_thrusts must contain only finite values")

    if not np.all(rotor_thrusts >= 0.0):
        raise ValueError("rotor_thrusts must be nonnegative")

    rotor_forces_B = np.zeros((4, 3), dtype=np.float64)
    rotor_forces_B[:, 2] = -rotor_thrusts
    return np.asarray(
        np.sum(np.cross(rotor_positions_B, rotor_forces_B), axis=0),
        dtype=np.float64,
    )


def force_and_moment_body_from_rotor_speeds(
    rotor_omega: NDArray[np.float64],
    rotor_positions_B: NDArray[np.float64],
    rotor_spin_directions: NDArray[np.float64],
    thrust_coefficient: float,
    moment_coefficient: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return the combined rotor force and moment in the FRD body frame.

    Args:
        rotor_omega: Rotor angular speeds with shape ``(4,)`` in rad/s.
        rotor_positions_B: Rotor positions with shape ``(4, 3)``, expressed in
            the forward-right-down (FRD) body frame in metres.
        rotor_spin_directions: Rotor spin directions with shape ``(4,)`` that
            contain only ``-1`` or ``+1``.
        thrust_coefficient: Thrust coefficient ``k_f`` in N/(rad/s)^2.
        moment_coefficient: Moment coefficient ``k_m`` in N·m/(rad/s)^2.

    Returns:
        A tuple ``(force_B, moment_B)``. ``force_B`` has shape ``(3,)`` in
        newtons, and ``moment_B`` has shape ``(3,)`` in N·m. Both vectors are
        expressed in the forward-right-down (FRD) body frame.

    Raises:
        ValueError: If an input is invalid. Input validation is performed by
            the composed actuation functions.
    """
    rotor_thrusts = rotor_thrusts_from_speeds(rotor_omega, thrust_coefficient)
    force_B = thrust_force_body_from_rotor_thrusts(rotor_thrusts)
    moment_thrust_B = thrust_moment_body_from_rotor_thrusts(
        rotor_positions_B,
        rotor_thrusts,
    )
    moment_reaction_B = reaction_moment_body_from_rotor_speeds(
        rotor_omega,
        rotor_spin_directions,
        moment_coefficient,
    )
    moment_B = moment_thrust_B + moment_reaction_B
    return force_B, moment_B
