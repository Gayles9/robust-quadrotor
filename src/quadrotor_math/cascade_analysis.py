"""Local hover cascade analysis, not a replacement nonlinear plant (ADR 0014).

Horizontal coordinate eta=-pitch for north, eta=roll for east at zero yaw.
State [p,v,eta,r,a] has units [m,m/s,rad,rad/s,rad/s²]. a is actual angular
acceleration from differential motor thrust at the hover linearization.
Only matched inertia, constant gravity, inactive limits and small angles apply.
"""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .attitude_control import _array, _scalar


def hover_axis_zero_order_hold(
    gravity: float, motor_time_constant_s: float, time_step_s: float
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return owned Phi (5,5), Gamma (5,) for constant angular command u.

    p'=v, v'=g*eta, eta'=r, r'=a, tau*a'=u-a.
    A convergent augmented-matrix exponential avoids cancellation in repeated
    motor integrals and requires no matrix inverse or optional dependency.
    This bounded analysis requires 0 < step <= tau; larger intervals compose
    smaller holds. The result is exact ZOH to floating-point roundoff, not Euler.
    """
    g = _scalar("gravity", gravity, positive=True)
    tau = _scalar("motor_time_constant_s", motor_time_constant_s, positive=True)
    h = _scalar("time_step_s", time_step_s, positive=True)
    if h > tau:
        raise ValueError("time_step_s must not exceed the motor time constant")
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            generator = np.zeros((6, 6))
            generator[0, 1], generator[1, 2] = 1, g
            generator[2, 3], generator[3, 4] = 1, 1
            generator[4, 4], generator[4, 5] = -1 / tau, 1 / tau
            scaled = h * generator
            total, term = np.eye(6), np.eye(6)
            for k in range(1, 81):
                term = (term @ scaled) / k
                total = total + term
                if np.max(np.abs(term)) <= np.finfo(float).eps * np.max(np.abs(total)):
                    break
            else:
                raise ValueError("hold exponential did not converge")
    except (FloatingPointError, OverflowError):
        raise ValueError("hold arithmetic must remain finite") from None
    return _array("Phi", total[:5, :5], (5, 5)), _array("Gamma", total[:5, 5], (5,))


@dataclass(frozen=True, slots=True)
class HorizontalCascade:
    """Independent positive gains and nominal clock for local analysis only.

    position_gain [s^-2]; velocity/attitude/rate gains [s^-1]; gravity [m/s²].
    The outer loop runs every outer_stride inner periods and holds eta_d.
    A lifted state is sampled immediately before an outer update. Analysis
    excludes estimator dynamics: errors may be treated as external forcings,
    but its poles are not poles of the full nonlinear estimator/controller.
    """

    position_gain: float
    velocity_gain: float
    attitude_gain: float
    rate_gain: float
    gravity: float
    motor_time_constant_s: float
    inner_period_s: float
    outer_stride: int

    def __post_init__(self) -> None:
        for name in (
            "position_gain",
            "velocity_gain",
            "attitude_gain",
            "rate_gain",
            "gravity",
            "motor_time_constant_s",
            "inner_period_s",
        ):
            object.__setattr__(self, name, _scalar(name, getattr(self, name), positive=True))
        if type(self.outer_stride) is not int or not 1 <= self.outer_stride <= 10000:
            raise ValueError("outer_stride must be an integer in [1, 10000]")
        if self.inner_period_s > self.motor_time_constant_s:
            raise ValueError("inner period must not exceed the motor time constant")

    def characteristic_polynomial(self) -> NDArray[np.float64]:
        """Descending coefficients of tau*s^5+s^4+kr*s^3+... (not monic)."""
        kp, kv, ka, kr = self.position_gain, self.velocity_gain, self.attitude_gain, self.rate_gain
        return _array(
            "characteristic_polynomial",
            np.array([self.motor_time_constant_s, 1, kr, kr * ka, kr * ka * kv, kr * ka * kp]),
            (6,),
        )

    def continuous_matrix(self) -> NDArray[np.float64]:
        """Owned (5,5) closed-loop generator with continuously updated feedback."""
        kp, kv, ka, kr = self.position_gain, self.velocity_gain, self.attitude_gain, self.rate_gain
        g, tau = self.gravity, self.motor_time_constant_s
        matrix = np.zeros((5, 5))
        matrix[0, 1], matrix[1, 2], matrix[2, 3], matrix[3, 4] = 1, g, 1, 1
        matrix[4] = [
            -kr * ka * kp / g / tau,
            -kr * ka * kv / g / tau,
            -kr * ka / tau,
            -kr / tau,
            -1 / tau,
        ]
        return _array("continuous_matrix", matrix, (5, 5))

    def lifted_transition(self) -> NDArray[np.float64]:
        """Owned (5,5) one-outer-period map with exact inner ZOH and held eta_d.

        Reset the sixth temporary coordinate eta_d=(-kp*p-kv*v)/g only at
        the outer tick. Each inner command is kr*(ka*(eta_d-eta)-r).
        Drop the held coordinate only after the complete outer interval.
        """
        phi, gamma = hover_axis_zero_order_hold(
            self.gravity, self.motor_time_constant_s, self.inner_period_s
        )
        ka, kr = self.attitude_gain, self.rate_gain
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                inner = np.eye(6)
                inner[:5, :5] = phi
                inner[:5] += np.outer(gamma, [0, 0, -kr * ka, -kr, 0, kr * ka])
                reset = np.eye(6)
                reset[5] = [
                    -self.position_gain / self.gravity,
                    -self.velocity_gain / self.gravity,
                    0,
                    0,
                    0,
                    0,
                ]
                transition = np.linalg.matrix_power(inner, self.outer_stride) @ reset
        except (FloatingPointError, OverflowError):
            raise ValueError("lifted arithmetic must remain finite") from None
        return _array("lifted_transition", transition[:5, :5], (5, 5))
