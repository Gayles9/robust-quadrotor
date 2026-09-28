"""Bounded, opt-in NED vertical integral acceleration; ADR 0024."""

from dataclasses import dataclass, fields, replace

import numpy as np
from numpy.typing import NDArray

from .attitude_control import _array, _scalar
from .position_control import (
    PositionControlCommand,
    PositionControllerParameters,
    PositionReference,
    compute_position_control,
)


@dataclass(frozen=True, slots=True)
class VerticalCompensationPolicy:
    """Positive integral gain [s^-3], acceleration bound [m/s²] and period [s]."""

    integral_gain: float
    maximum_acceleration: float
    update_period_s: float

    def __post_init__(self) -> None:
        for f in fields(self):
            object.__setattr__(self, f.name, _scalar(f.name, getattr(self, f.name), positive=True))


@dataclass(frozen=True, slots=True)
class VerticalCompensationSample:
    """One outer attempt; applied state precedes the causal next-state update."""

    time_s: float
    position_error_down_m: float
    applied_acceleration: float
    next_acceleration: float
    observations_healthy: bool
    previous_inner_limited: bool
    update_reason: str
    bound_limited: bool


class VerticalIntegralCompensator:
    """Own one scalar integral and its diagnostic history, without truth inputs.

    Frozen/unhealthy intervals keep the current correction. They do not accrue
    error for later catch-up. Phase changes retain memory; reset explicitly for
    a new mission. Invalid calls leave all state unchanged. History may include
    a computed outer attempt rejected by a later attitude-domain guard.
    """

    def __init__(
        self, parameters: PositionControllerParameters, policy: VerticalCompensationPolicy
    ) -> None:
        if not isinstance(parameters, PositionControllerParameters) or not isinstance(
            policy, VerticalCompensationPolicy
        ):
            raise TypeError("parameters and policy must use their dataclasses")
        self._parameters, self._policy = replace(parameters), replace(policy)
        if policy.maximum_acceleration > parameters.maximum_acceleration_W[2]:
            raise ValueError("integral bound must fit the existing vertical acceleration limit")
        self.reset()

    @property
    def policy(self) -> VerticalCompensationPolicy:
        return self._policy

    @property
    def acceleration(self) -> float:
        return self._acceleration

    @property
    def history(self) -> tuple[VerticalCompensationSample, ...]:
        return tuple(self._history)

    def matches(self, parameters: PositionControllerParameters) -> bool:
        return isinstance(parameters, PositionControllerParameters) and all(
            np.array_equal(getattr(parameters, f.name), getattr(self._parameters, f.name))
            for f in fields(parameters)
        )

    def reset(self) -> None:
        self._acceleration = 0.0
        self._history: list[VerticalCompensationSample] = []

    def step(
        self,
        time_s: float,
        position_W: NDArray[np.float64],
        velocity_W: NDArray[np.float64],
        reference: PositionReference,
        *,
        observations_healthy: bool,
        previous_inner_limited: bool,
    ) -> PositionControlCommand:
        """Compute the current command, then prepare the bounded next integral."""
        time = _scalar("time_s", time_s)
        if time != len(self._history) * self.policy.update_period_s:
            raise ValueError("compensation clock must start at zero and follow the outer grid")
        if type(observations_healthy) is not bool or type(previous_inner_limited) is not bool:
            raise TypeError("health and previous-limit flags must be bool")
        if not isinstance(reference, PositionReference):
            raise TypeError("reference must be PositionReference")
        position = _array("position_W", position_W, (3,))
        velocity = _array("velocity_W", velocity_W, (3,))
        try:
            with np.errstate(over="raise", invalid="raise"):
                acceleration = reference.acceleration_W.copy()
                acceleration[2] += self.acceleration
                target = (
                    reference
                    if self.acceleration == 0
                    else replace(reference, acceleration_W=acceleration)
                )
                command = compute_position_control(position, velocity, target, self._parameters)
                error = float(reference.position_W[2] - position[2])
                delta = self.policy.update_period_s * self.policy.integral_gain * error
                residual = float(
                    command.requested_acceleration_W[2] - command.feasible_acceleration_W[2]
                )
                reason = (
                    "unhealthy"
                    if not observations_healthy
                    else "inner_limit"
                    if previous_inner_limited
                    else "outer_limit"
                    if (
                        (command.acceleration_limited or command.thrust_limited)
                        and abs(residual) > 1e-12
                        and residual * delta > 0
                    )
                    else "integrating"
                )
                proposed = self.acceleration + (delta if reason == "integrating" else 0.0)
                if not np.isfinite(error) or not np.isfinite(delta) or not np.isfinite(proposed):
                    raise FloatingPointError
                next_acceleration = float(
                    np.clip(
                        proposed,
                        -self.policy.maximum_acceleration,
                        self.policy.maximum_acceleration,
                    )
                )
        except (FloatingPointError, OverflowError):
            raise ValueError("vertical compensation arithmetic must remain finite") from None
        record = VerticalCompensationSample(
            time,
            error,
            self.acceleration,
            next_acceleration,
            observations_healthy,
            previous_inner_limited,
            reason,
            next_acceleration != proposed,
        )
        self._history.append(record)
        self._acceleration = next_acceleration
        return command
