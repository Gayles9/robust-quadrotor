"""Pure NED position/velocity PD and feasible FRD thrust direction (ADR 0012)."""

from dataclasses import dataclass, replace
from math import cos, hypot, isfinite, pi, sin, tan

import numpy as np
from numpy.typing import NDArray

from .attitude_control import _array, _quaternion, _scalar
from .rotations import normalize_quaternion_body_to_world


def _finite_scalar(name: str, value: float) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError(f"{name} must be a real scalar")
    try:
        result = float(value)
    except OverflowError:
        raise ValueError(f"{name} must be finite") from None
    if not isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True, slots=True, eq=False)
class PositionReference:
    """Owned NED (3,) position m, velocity m/s, acceleration m/s²; yaw radians."""

    position_W: NDArray[np.float64]
    velocity_W: NDArray[np.float64]
    acceleration_W: NDArray[np.float64]
    yaw_rad: float

    def __post_init__(self) -> None:
        for name in ("position_W", "velocity_W", "acceleration_W"):
            object.__setattr__(self, name, _array(name, getattr(self, name), (3,)))
        object.__setattr__(self, "yaw_rad", _finite_scalar("yaw_rad", self.yaw_rad))


@dataclass(frozen=True, slots=True, eq=False)
class PositionControllerParameters:
    """Nominal PD assumptions; no truth access, RNG, clock or integral state.

    Positive (3,) gains Kp [1/s²], Kv [1/s]; acceleration bounds [m/s²].
    The vertical acceleration bound must be less than positive gravity so the
    desired body-down axis always has a positive NED-down component.
    Thrust is magnitude [N], tilt in (0,pi/2) [rad], mass [kg].
    """

    position_gain_W: NDArray[np.float64]
    velocity_gain_W: NDArray[np.float64]
    maximum_acceleration_W: NDArray[np.float64]
    nominal_mass: float
    nominal_gravity_acceleration: float
    maximum_tilt_rad: float
    minimum_collective_thrust: float
    maximum_collective_thrust: float

    def __post_init__(self) -> None:
        for name in ("position_gain_W", "velocity_gain_W", "maximum_acceleration_W"):
            value = _array(name, getattr(self, name), (3,))
            if np.any(value <= 0):
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        for name in (
            "nominal_mass",
            "nominal_gravity_acceleration",
            "maximum_tilt_rad",
            "minimum_collective_thrust",
            "maximum_collective_thrust",
        ):
            object.__setattr__(self, name, _scalar(name, getattr(self, name), positive=True))
        if self.maximum_acceleration_W[2] >= self.nominal_gravity_acceleration:
            raise ValueError("vertical acceleration bound must be below gravity")
        if self.maximum_tilt_rad >= pi / 2:
            raise ValueError("maximum_tilt_rad must be below pi/2")
        if self.minimum_collective_thrust >= self.maximum_collective_thrust:
            raise ValueError("collective thrust bounds must be strictly ordered")
        with np.errstate(over="ignore", invalid="ignore"):
            hover = self.nominal_mass * self.nominal_gravity_acceleration
        if not self.minimum_collective_thrust < hover < self.maximum_collective_thrust:
            raise ValueError("nominal hover thrust must be strictly inside thrust bounds")


@dataclass(frozen=True, slots=True, eq=False)
class PositionControlCommand:
    """Requested/feasible NED acceleration; held orientation and collective demand.

    feasible_acceleration_W assumes instantaneous matched orientation/actuation;
    it is NOT actual vehicle acceleration. Limit flags are command diagnostics.
    """

    requested_acceleration_W: NDArray[np.float64]
    feasible_acceleration_W: NDArray[np.float64]
    q_reference_WB: NDArray[np.float64]
    collective_thrust: float
    acceleration_limited: bool
    tilt_limited: bool
    thrust_limited: bool

    def __post_init__(self) -> None:
        for name in ("requested_acceleration_W", "feasible_acceleration_W"):
            object.__setattr__(self, name, _array(name, getattr(self, name), (3,)))
        object.__setattr__(
            self, "q_reference_WB", _quaternion("q_reference_WB", self.q_reference_WB)
        )
        object.__setattr__(
            self,
            "collective_thrust",
            _scalar("collective_thrust", self.collective_thrust, positive=True),
        )
        for name in ("acceleration_limited", "tilt_limited", "thrust_limited"):
            if type(getattr(self, name)) is not bool:
                raise ValueError("command limit flags must be bool")


def _quaternion_from_axes(R_WB: NDArray[np.float64]) -> NDArray[np.float64]:
    """Largest-component conversion of an internally constructed SO(3) matrix."""
    r = R_WB
    squares = np.array(
        [
            1 + np.trace(r),
            1 + 2 * r[0, 0] - np.trace(r),
            1 + 2 * r[1, 1] - np.trace(r),
            1 + 2 * r[2, 2] - np.trace(r),
        ]
    )
    index = int(np.argmax(squares))
    result = np.zeros(4)
    result[index] = 0.5 * np.sqrt(squares[index])
    divisor = 4 * result[index]
    if index == 0:
        result[1:] = [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]
        result[1:] /= divisor
    else:
        i = index - 1
        j, k = (i + 1) % 3, (i + 2) % 3
        result[0] = (r[k, j] - r[j, k]) / divisor
        result[j + 1] = (r[j, i] + r[i, j]) / divisor
        result[k + 1] = (r[k, i] + r[i, k]) / divisor
    result = normalize_quaternion_body_to_world(result)
    if next(float(value) for value in result if value != 0) < 0:
        result = -result
    return result


def compute_position_control(
    position_W: NDArray[np.float64],
    velocity_W: NDArray[np.float64],
    reference: PositionReference,
    parameters: PositionControllerParameters,
) -> PositionControlCommand:
    """Pure PD + acceleration feedforward -> bounded attitude and thrust.

    State vectors are (3,) NED m and m/s. Arrays are copied, never mutated.
    An invalid input or nonfinite intermediate raises; no partial output.
    Limits act in order: component acceleration, tilt cone, thrust magnitude.
    No compensation using current attitude or true mass/drag is performed.
    """
    if not isinstance(reference, PositionReference) or not isinstance(
        parameters, PositionControllerParameters
    ):
        raise TypeError("reference and parameters must use their dataclasses")
    model, target = replace(parameters), replace(reference)
    position = _array("position_W", position_W, (3,))
    velocity = _array("velocity_W", velocity_W, (3,))
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            requested = (
                target.acceleration_W
                + model.position_gain_W * (target.position_W - position)
                + model.velocity_gain_W * (target.velocity_W - velocity)
            )
            acceleration = np.clip(
                requested, -model.maximum_acceleration_W, model.maximum_acceleration_W
            )
            gravity = np.array([0.0, 0, model.nominal_gravity_acceleration])
            lift = model.nominal_mass * (gravity - acceleration)
            horizontal = hypot(*lift[:2])
            allowed = float(lift[2]) * tan(model.maximum_tilt_rad)
            tilt_limited = horizontal > allowed
            if tilt_limited:
                lift[:2] *= allowed / horizontal
            magnitude = hypot(*lift)
            if not isfinite(magnitude) or magnitude == 0 or not isfinite(allowed):
                raise FloatingPointError
            down_W = lift / magnitude
            thrust = float(
                np.clip(magnitude, model.minimum_collective_thrust, model.maximum_collective_thrust)
            )
            heading_y_W = np.array([-sin(target.yaw_rad), cos(target.yaw_rad), 0.0])
            forward_W = np.cross(heading_y_W, down_W)
            forward_W /= hypot(*forward_W)
            right_W = np.cross(down_W, forward_W)
            q_reference_WB = _quaternion_from_axes(np.column_stack((forward_W, right_W, down_W)))
            feasible = gravity - (thrust / model.nominal_mass) * down_W
    except (FloatingPointError, OverflowError, ZeroDivisionError):
        raise ValueError("position-control arithmetic must remain finite and nonsingular") from None
    return PositionControlCommand(
        requested,
        feasible,
        q_reference_WB,
        thrust,
        bool(np.any(requested != acceleration)),
        bool(tilt_limited),
        bool(thrust != magnitude),
    )
