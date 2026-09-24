"""Bounded cascaded attitude/rate baseline; SI units and NED/FRD (ADR 0011).

Pure feedback only: no truth configuration, clock, RNG, integrator state, or ESKF.
References are piecewise constant orientations, not angular trajectories.
"""

from dataclasses import dataclass, replace
from math import atan2, hypot, isfinite, pi

import numpy as np
from numpy.typing import NDArray

from .actuation import (
    commanded_rotor_speeds_from_collective_thrust_and_body_moment,
    force_and_moment_body_from_rotor_speeds,
)
from .rotations import rotation_matrix_body_to_world
from .run_configuration import RotorParameters


def _array(name: str, value: NDArray[np.float64], shape: tuple[int, ...]) -> NDArray[np.float64]:
    if not isinstance(value, np.ndarray) or value.shape != shape or value.dtype.kind not in "iuf":
        raise ValueError(f"{name} must be a real numeric array with shape {shape}")
    with np.errstate(over="ignore", invalid="ignore"):
        owned = np.array(value, dtype=np.float64, order="C", copy=True)
    if not np.all(np.isfinite(owned)):
        raise ValueError(f"{name} must contain only finite values")
    owned.flags.writeable = False
    return owned


def _scalar(name: str, value: float, *, positive: bool = False) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError(f"{name} must be a real scalar")
    try:
        value = float(value)
    except OverflowError:
        raise ValueError(f"{name} must fit a finite float64 scalar") from None
    if not isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return value


def _inertia(value: NDArray[np.float64]) -> NDArray[np.float64]:
    matrix = _array("inertia_B", value, (3, 3))
    if not np.array_equal(matrix, matrix.T):
        raise ValueError("inertia_B must be exactly symmetric")
    try:
        np.linalg.cholesky(matrix)
    except np.linalg.LinAlgError:
        raise ValueError("inertia_B must be positive definite") from None
    return matrix


def _quaternion(name: str, value: NDArray[np.float64]) -> NDArray[np.float64]:
    owned = _array(name, value, (4,))
    rotation_matrix_body_to_world(owned)
    return owned


def _rotors(value: RotorParameters) -> RotorParameters:
    if not isinstance(value, RotorParameters):
        raise TypeError("rotors must be RotorParameters")
    if any(
        v is None
        for v in (value.minimum_rotor_omega, value.maximum_rotor_omega, value.motor_time_constant_s)
    ):
        raise ValueError("control requires complete motor parameters")
    assert value.minimum_rotor_omega is not None
    assert value.maximum_rotor_omega is not None
    assert value.motor_time_constant_s is not None
    result = RotorParameters(
        _array("rotor_positions_B", value.rotor_positions_B, (4, 3)),
        _array("rotor_spin_directions", value.rotor_spin_directions, (4,)),
        _scalar("thrust_coefficient", value.thrust_coefficient, positive=True),
        _scalar("moment_coefficient", value.moment_coefficient, positive=True),
        _scalar("minimum_rotor_omega", value.minimum_rotor_omega),
        _scalar("maximum_rotor_omega", value.maximum_rotor_omega, positive=True),
        _scalar("motor_time_constant_s", value.motor_time_constant_s, positive=True),
    )
    return result


def attitude_error_body(
    q_WB: NDArray[np.float64], q_reference_WB: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Owned (3,) Log(R_WB.T R_reference_WB), current-body radians.

    Unit Hamilton scalar-first inputs; either quaternion sign is immaterial.
    Principal angle in [0, pi]; exact-pi axis chooses first nonzero positive.
    The controller separately enforces its declared strictly sub-pi local domain.
    """
    current = _quaternion("q_WB", q_WB)
    reference = _quaternion("q_reference_WB", q_reference_WB)
    real = float(np.dot(current, reference))
    vector = (
        current[0] * reference[1:]
        - reference[0] * current[1:]
        - np.cross(current[1:], reference[1:])
    )
    relative = np.r_[real, vector]
    if next(float(v) for v in relative if v != 0) < 0:
        relative = -relative
    sine_half = hypot(*relative[1:])
    scale = (
        2.0 / hypot(*relative)
        if sine_half < 1e-8
        else (2 * atan2(sine_half, relative[0]) / sine_half)
    )
    return _array("attitude_error_B", relative[1:] * scale, (3,))


def body_rate_moment(
    omega_B: NDArray[np.float64],
    desired_omega_B: NDArray[np.float64],
    nominal_inertia_B: NDArray[np.float64],
    rate_gain_B: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Unbounded FRD moment (N m) for rate P feedback and gyroscopic compensation.

    Rates (3,) are rad/s; gains (3,) positive 1/s; full SPD inertia kg m².
    A constant torque disturbance causes a steady offset: there is no integral.
    """
    rate = _array("omega_B", omega_B, (3,))
    desired = _array("desired_omega_B", desired_omega_B, (3,))
    inertia = _inertia(nominal_inertia_B)
    gain = _array("rate_gain_B", rate_gain_B, (3,))
    if np.any(gain <= 0):
        raise ValueError("rate_gain_B must be positive")
    try:
        with np.errstate(over="raise", invalid="raise"):
            moment = inertia @ (gain * (desired - rate)) + np.cross(rate, inertia @ rate)
    except FloatingPointError:
        raise ValueError("body-rate moment must remain finite") from None
    return _array("moment_B", moment, (3,))


@dataclass(frozen=True, slots=True, eq=False)
class AttitudeControllerParameters:
    """Independent nominal assumptions, positive diagonal gains and bounds.

    Arrays own read-only storage. maximum_attitude_error_rad is in (0,pi).
    No parameters are taken from the simulation's truth configuration.
    """

    attitude_gain_B: NDArray[np.float64]
    rate_gain_B: NDArray[np.float64]
    maximum_body_rate_B: NDArray[np.float64]
    maximum_moment_B: NDArray[np.float64]
    maximum_attitude_error_rad: float
    nominal_inertia_B: NDArray[np.float64]
    nominal_rotors: RotorParameters

    def __post_init__(self) -> None:
        for name in ("attitude_gain_B", "rate_gain_B", "maximum_body_rate_B", "maximum_moment_B"):
            owned = _array(name, getattr(self, name), (3,))
            if np.any(owned <= 0):
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, owned)
        angle = _scalar(
            "maximum_attitude_error_rad", self.maximum_attitude_error_rad, positive=True
        )
        if angle >= pi:
            raise ValueError("maximum_attitude_error_rad must be less than pi")
        object.__setattr__(self, "maximum_attitude_error_rad", angle)
        object.__setattr__(self, "nominal_inertia_B", _inertia(self.nominal_inertia_B))
        object.__setattr__(self, "nominal_rotors", _rotors(self.nominal_rotors))


@dataclass(frozen=True, slots=True, eq=False)
class LimitedMomentAllocation:
    """Nominal command: speeds (4,) rad/s, achieved moment (3,) N m, scale [0,1]."""

    commanded_rotor_omega: NDArray[np.float64]
    allocated_moment_B: NDArray[np.float64]
    moment_scale: float

    def __post_init__(self) -> None:
        speeds = _array("commanded_rotor_omega", self.commanded_rotor_omega, (4,))
        scale = _scalar("moment_scale", self.moment_scale)
        if np.any(speeds < 0) or scale > 1:
            raise ValueError("speeds must be nonnegative and moment_scale at most one")
        object.__setattr__(self, "commanded_rotor_omega", speeds)
        object.__setattr__(
            self, "allocated_moment_B", _array("allocated_moment_B", self.allocated_moment_B, (3,))
        )
        object.__setattr__(self, "moment_scale", scale)


def _strict_allocate(
    collective_thrust: float, moment_B: NDArray[np.float64], model: RotorParameters
) -> NDArray[np.float64]:
    assert model.minimum_rotor_omega is not None and model.maximum_rotor_omega is not None
    return commanded_rotor_speeds_from_collective_thrust_and_body_moment(
        collective_thrust,
        moment_B,
        model.rotor_positions_B,
        model.rotor_spin_directions,
        model.thrust_coefficient,
        model.moment_coefficient,
        model.minimum_rotor_omega,
        model.maximum_rotor_omega,
    )


def allocate_limited_body_moment(
    collective_thrust: float, moment_B: NDArray[np.float64], rotors: RotorParameters
) -> LimitedMomentAllocation:
    """Preserve feasible collective (N), uniformly reduce moment to rotor bounds.

    Require feasible zero-moment collective. Analytically intersect the squared-
    speed ray with all four intervals; no least-squares or independent clipping.
    An active scale uses 64*eps relative backoff before unchanged strict allocation.
    """
    thrust = _scalar("collective_thrust", collective_thrust)
    moment = _array("moment_B", moment_B, (3,))
    model = _rotors(rotors)
    base = _strict_allocate(thrust, np.zeros(3), model) ** 2
    assert model.minimum_rotor_omega is not None and model.maximum_rotor_omega is not None
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            A = np.array(
                [
                    np.full(4, model.thrust_coefficient),
                    -model.rotor_positions_B[:, 1] * model.thrust_coefficient,
                    model.rotor_positions_B[:, 0] * model.thrust_coefficient,
                    -model.rotor_spin_directions * model.moment_coefficient,
                ]
            )
            direction = np.linalg.solve(A, np.r_[0.0, moment])
            if not np.all(np.isfinite(direction)):
                raise FloatingPointError
            lower, upper = model.minimum_rotor_omega**2, model.maximum_rotor_omega**2
            scale = 1.0
            for value, delta in zip(base, direction, strict=True):
                # Divide only when the requested step crosses a bound; ratios <=1.
                if delta > 0 and delta > upper - value:
                    scale = min(scale, float(max(0.0, upper - value) / delta))
                elif delta < 0 and -delta > value - lower:
                    scale = min(scale, float(max(0.0, value - lower) / -delta))
            if scale < 1:
                scale *= 1 - 64 * np.finfo(np.float64).eps
            speeds = _strict_allocate(thrust, scale * moment, model)
            _, actual_moment = force_and_moment_body_from_rotor_speeds(
                speeds,
                model.rotor_positions_B,
                model.rotor_spin_directions,
                model.thrust_coefficient,
                model.moment_coefficient,
            )
    except (FloatingPointError, np.linalg.LinAlgError):
        raise ValueError("limited allocation must remain finite and solvable") from None
    return LimitedMomentAllocation(speeds, actual_moment, scale)


@dataclass(frozen=True, slots=True, eq=False)
class AttitudeControlCommand:
    """Owned diagnostic vectors: radians, rad/s, requested/clipped N m, allocation."""

    attitude_error_B: NDArray[np.float64]
    desired_omega_B: NDArray[np.float64]
    moment_requested_B: NDArray[np.float64]
    moment_limited_B: NDArray[np.float64]
    rate_limited: bool
    moment_limited: bool
    allocation: LimitedMomentAllocation

    def __post_init__(self) -> None:
        for name in (
            "attitude_error_B",
            "desired_omega_B",
            "moment_requested_B",
            "moment_limited_B",
        ):
            object.__setattr__(self, name, _array(name, getattr(self, name), (3,)))
        if type(self.rate_limited) is not bool or type(self.moment_limited) is not bool:
            raise ValueError("limit flags must be bool")
        if not isinstance(self.allocation, LimitedMomentAllocation):
            raise TypeError("allocation must be LimitedMomentAllocation")
        object.__setattr__(self, "allocation", replace(self.allocation))


def compute_attitude_control(
    q_WB: NDArray[np.float64],
    omega_B: NDArray[np.float64],
    q_reference_WB: NDArray[np.float64],
    collective_thrust: float,
    parameters: AttitudeControllerParameters,
) -> AttitudeControlCommand:
    """Pure bounded local attitude P -> body-rate P -> feasible motor command.

    Inputs have shapes (4,), (3,), (4,); rates FRD rad/s, collective N.
    References are static over the control interval. No angular feedforward or
    persistent-disturbance integral is implied. Failure returns no partial command.
    """
    if not isinstance(parameters, AttitudeControllerParameters):
        raise TypeError("parameters must be AttitudeControllerParameters")
    model = replace(parameters)
    error = attitude_error_body(q_WB, q_reference_WB)
    if hypot(*error) > model.maximum_attitude_error_rad:
        raise ValueError("attitude error exceeds the declared local controller domain")
    try:
        with np.errstate(over="raise", invalid="raise"):
            requested_rate = model.attitude_gain_B * error
    except FloatingPointError:
        raise ValueError("attitude rate demand must remain finite") from None
    desired_rate = np.clip(requested_rate, -model.maximum_body_rate_B, model.maximum_body_rate_B)
    moment = body_rate_moment(omega_B, desired_rate, model.nominal_inertia_B, model.rate_gain_B)
    limited = np.clip(moment, -model.maximum_moment_B, model.maximum_moment_B)
    allocation = allocate_limited_body_moment(collective_thrust, limited, model.nominal_rotors)
    return AttitudeControlCommand(
        error,
        desired_rate,
        moment,
        limited,
        bool(np.any(desired_rate != requested_rate)),
        bool(np.any(limited != moment)),
        allocation,
    )
