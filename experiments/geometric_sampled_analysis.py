"""Local geometric hover map with bilinear reference memory and motor lag.

State [p,v,eta,r,a,previous,s1,s2,s3]; reference memory is in signed tilt units.
eta=-pitch for north and eta=roll for east. Estimator errors are exogenous
forcing; this is not a linearization of the full stochastic ESKF.
"""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from quadrotor_math.attitude_control import _array, _scalar
from quadrotor_math.cascade_analysis import hover_axis_zero_order_hold


@dataclass(frozen=True, slots=True)
class GeometricHoverMap:
    inertia: float
    pole_rad_s: float = 30.0
    shaped: bool = False

    def __post_init__(self) -> None:
        for key in ("inertia", "pole_rad_s"):
            object.__setattr__(self, key, _scalar(key, getattr(self, key), positive=True))
        if self.pole_rad_s * 0.02 > 1:
            raise ValueError("pole must be resolved by the 20 ms outer period")
        if type(self.shaped) is not bool:
            raise ValueError("shaped must be bool")

    def step(self, state: NDArray[np.float64]) -> NDArray[np.float64]:
        """One 20 ms period, with two held 10 ms moment commands.

        All gains, gravity and clocks are the frozen campaign values. The four
        memory coordinates describe an already initialized filter, not its
        special first-sample constant-prehistory reset.
        """
        x = _array("state", state, (9,))
        physical = x[:5].copy()
        desired = -(x[0] + 1.8 * x[1]) / 9.81
        current, previous = desired, x[5]
        w = self.pole_rad_s
        b = (w * 0.01) / (1 + w * 0.01)
        sections = np.empty(3)
        for i, old in enumerate(x[6:]):
            sections[i] = old + b * ((current - old) + (previous - old))
            current, previous = sections[i], old
        first = w * (sections[1] - sections[2])
        second = w * w * (sections[0] - 2 * sections[1] + sections[2])
        target = sections[2] if self.shaped else desired
        phi, gamma = hover_axis_zero_order_hold(9.81, 0.025, 0.01)
        for _ in range(2):
            u = (0.64 * (target - physical[2]) + 0.32 * (first - physical[3])) / self.inertia
            physical = phi @ physical + gamma * (u + second)
        return _array("next_state", np.concatenate((physical, [desired], sections)), (9,))

    def transition(self) -> NDArray[np.float64]:
        return _array("transition", np.column_stack([self.step(e) for e in np.eye(9)]), (9, 9))
