"""Central chi-square bounds and pointwise independent-seed consistency summaries.

No filter tuning, temporal-independence assumption or multiple-testing guarantee.
"""

from dataclasses import dataclass, field
from functools import lru_cache
from math import erfc, exp, fsum, lgamma, log, sqrt

import numpy as np
from numpy.typing import NDArray

from .eskf_replay import _owned_array


def _degrees_of_freedom(value: int) -> int:
    if type(value) is not int or not 1 <= value <= 15000:
        raise ValueError("degrees_of_freedom must be a non-Boolean integer in [1, 15000]")
    return value


def _chi_square_survival(x: float, degrees: int) -> float:
    """Finite integer/half-integer gamma recurrence, evaluated in log space."""
    t = x / 2.0
    if t == 0.0:
        return 1.0
    if degrees % 2 == 0:
        terms = (exp(j * log(t) - t - lgamma(j + 1)) for j in range(degrees // 2))
        return fsum(terms)
    terms = (exp((j + 0.5) * log(t) - t - lgamma(j + 1.5)) for j in range(degrees // 2))
    return erfc(sqrt(t)) + fsum(terms)


@lru_cache(maxsize=128)
def _interval(degrees: int) -> tuple[float, float]:
    quantiles: list[float] = []
    for survival_probability in (0.975, 0.025):
        low, high = 0.0, float(degrees)
        while _chi_square_survival(high, degrees) > survival_probability:
            high *= 2.0
        for _ in range(60):
            middle = low + (high - low) / 2.0
            if _chi_square_survival(middle, degrees) > survival_probability:
                low = middle
            else:
                high = middle
        quantiles.append(low + (high - low) / 2.0)
    return quantiles[0], quantiles[1]


def chi_square_central_95_interval(degrees_of_freedom: int) -> tuple[float, float]:
    """Return numerical 2.5% and 97.5% chi-square quantiles, integer DOF 1..15000.

    The bounded domain supports up to 1000 independent 15-state samples. The
    incomplete-gamma recurrence and bisection need only Python's math library;
    this is not a general probability-distribution implementation. See ADR 0008.
    """
    return _interval(_degrees_of_freedom(degrees_of_freedom))


@dataclass(frozen=True, slots=True, eq=False)
class NormalizedErrorEnsemble:
    """Summarize finite scores (independent seeds, common epochs), same DOF.

    Individual central95 bounds use chi²_d; pointwise mean bounds use chi²_(N*d)/N.
    Independence/Gaussian correctness are scientific assumptions, not properties
    this array contract can certify. Temporal means and coverage are descriptive
    only. Never combine accepted-only NIS, mismatched epochs or failed-run subsets.
    """

    samples: NDArray[np.float64]
    degrees_of_freedom: int
    individual_interval_95: tuple[float, float] = field(init=False)
    mean_interval_95: tuple[float, float] = field(init=False)
    mean_by_epoch: NDArray[np.float64] = field(init=False)
    coverage_by_epoch: NDArray[np.float64] = field(init=False)
    descriptive_mean: float = field(init=False)
    descriptive_coverage: float = field(init=False)

    def __post_init__(self) -> None:
        degrees = _degrees_of_freedom(self.degrees_of_freedom)
        if (
            not isinstance(self.samples, np.ndarray)
            or self.samples.ndim != 2
            or not all(self.samples.shape)
        ):
            raise ValueError("samples must be a nonempty (seeds, epochs) matrix")
        samples = _owned_array("samples", self.samples, self.samples.shape)
        if np.any(samples < 0.0):
            raise ValueError("samples must be nonnegative")
        count, epochs = samples.shape
        individual = chi_square_central_95_interval(degrees)
        low, high = chi_square_central_95_interval(count * degrees)
        # Normalize before averaging so large finite observations stay finite.
        scale = np.max(samples, axis=0)
        normalized = np.divide(samples, scale, out=np.zeros_like(samples), where=scale != 0)
        means = scale * np.mean(normalized, axis=0)
        coverage = np.mean((samples >= individual[0]) & (samples <= individual[1]), axis=0)
        maximum = float(np.max(means))
        overall = 0.0 if maximum == 0.0 else maximum * float(np.mean(means / maximum))
        for name, value in (
            ("samples", samples),
            ("individual_interval_95", individual),
            ("mean_interval_95", (low / count, high / count)),
            ("mean_by_epoch", _owned_array("mean_by_epoch", means, (epochs,))),
            ("coverage_by_epoch", _owned_array("coverage_by_epoch", coverage, (epochs,))),
            ("descriptive_mean", overall),
            ("descriptive_coverage", float(np.mean(coverage))),
        ):
            object.__setattr__(self, name, value)
