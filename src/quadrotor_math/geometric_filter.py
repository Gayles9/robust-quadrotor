"""Immutable causal three-section bilinear correction-force differentiator."""

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from .attitude_control import _array, _scalar


@dataclass(frozen=True, slots=True, eq=False)
class FeedbackDerivativeFilter:
    """One ordered, fixed-period force stream. SI inputs N, outputs N/s and N/s².

    step returns a new memory after successful arithmetic; failed calls cannot
    change continuation state. Start a new instance to reset. Only derivative
    outputs are filtered, not the force value used to construct the attitude.
    """

    period_s: float
    pole_rad_s: float = 30.0
    next_index: int = 0
    previous: NDArray[np.float64] = field(default_factory=lambda: np.zeros(3))
    sections: NDArray[np.float64] = field(default_factory=lambda: np.zeros((3, 3)))

    def __post_init__(self) -> None:
        h = _scalar("period_s", self.period_s, positive=True)
        w = _scalar("pole_rad_s", self.pole_rad_s, positive=True)
        if type(self.next_index) is not int or self.next_index < 0:
            raise ValueError("next_index must be a nonnegative integer")
        with np.errstate(over="ignore", invalid="ignore", under="ignore"):
            product = h * w
            a = product / 2
            b = a / (1 + a)
            r = (1 - a) / (1 + a)
            squared = w * w
        if not (0 < product <= 1 and 0 < b < 1 and 0 <= r < 1 and np.isfinite(squared)):
            raise ValueError("filter period/pole must resolve finite coefficients and 0 < w*h <= 1")
        object.__setattr__(self, "period_s", h)
        object.__setattr__(self, "pole_rad_s", w)
        object.__setattr__(self, "previous", _array("previous", self.previous, (3,)))
        object.__setattr__(self, "sections", _array("sections", self.sections, (3, 3)))

    def rebase(self, correction_jump_W: NDArray[np.float64]) -> "FeedbackDerivativeFilter":
        """Translate memory by an estimator's force revision [N], without a tick.

        Apply only independently identified posterior position/velocity jumps.
        Translating every section preserves their differences (the derivatives).
        Physical input changes must still pass through step without rebasing.
        """
        jump = _array("correction_jump_W", correction_jump_W, (3,))
        try:
            with np.errstate(over="raise", invalid="raise"):
                return FeedbackDerivativeFilter(
                    self.period_s,
                    self.pole_rad_s,
                    self.next_index,
                    self.previous + jump,
                    self.sections + jump,
                )
        except FloatingPointError:
            raise ValueError("filter rebase arithmetic must remain finite") from None

    def step(
        self,
        correction_W: NDArray[np.float64],
        sample_index: int,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], "FeedbackDerivativeFilter"]:
        """Use this sample and past memory only; constant prehistory at index zero."""
        if type(sample_index) is not int or sample_index != self.next_index:
            raise ValueError("sample index must equal next_index")
        c = _array("correction_W", correction_W, (3,))
        w, a = self.pole_rad_s, self.pole_rad_s * self.period_s / 2
        b = a / (1 + a)
        if sample_index == 0:
            states = np.tile(c, (3, 1))
            first, second = np.zeros(3), np.zeros(3)
        else:
            try:
                with np.errstate(over="raise", invalid="raise"):
                    states = np.empty((3, 3))
                    current, previous = c, self.previous
                    for i in range(3):
                        old = self.sections[i]
                        states[i] = old + b * ((current - old) + (previous - old))
                        current, previous = states[i], old
                    first = w * (states[1] - states[2])
                    second = w * w * ((states[0] - states[1]) - (states[1] - states[2]))
            except FloatingPointError:
                raise ValueError("filter arithmetic must remain finite") from None
        return (
            _array("correction_rate_W", first, (3,)),
            _array("correction_acceleration_W", second, (3,)),
            FeedbackDerivativeFilter(self.period_s, w, sample_index + 1, c, states),
        )
