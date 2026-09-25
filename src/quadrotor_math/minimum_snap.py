"""Fixed-duration C3 NED position splines minimizing integrated squared snap.

ADR 0015: seventh-degree normalized polynomials, fixed waypoints and endpoint
velocity/acceleration/jerk. Internal derivatives are free. No feasibility,
automatic timing, yaw optimization or flight qualification is implied.
"""

from dataclasses import dataclass, field
from math import factorial

import numpy as np
from numpy.typing import NDArray

from .attitude_control import _array, _scalar
from .position_control import PositionReference

MAX_SEGMENTS = 64
MAX_SCALED_CONDITION = 1e8

# Exact Hermite inverse: rows are ascending powers; columns are p,v,a,j at
# s=0, then at s=1. Derivatives here are with respect to normalized s.
_HERMITE = np.array(
    [
        [1, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0.5, 0, 0, 0, 0, 0],
        [0, 0, 0, 1 / 6, 0, 0, 0, 0],
        [-35, -20, -5, -2 / 3, 35, -15, 2.5, -1 / 6],
        [84, 45, 10, 1, -84, 39, -7, 0.5],
        [-70, -36, -7.5, -2 / 3, 70, -34, 6.5, -0.5],
        [20, 10, 2, 1 / 6, -20, 10, -2, 1 / 6],
    ],
    dtype=np.float64,
)
_SNAP_GRAM = np.array(
    [
        [576, 1440, 2880, 5040],
        [1440, 4800, 10800, 20160],
        [2880, 10800, 25920, 50400],
        [5040, 20160, 50400, 100800],
    ],
    dtype=np.float64,
)
# Factor the analytic integrated cost, not a sampled-time approximation.
_SNAP_FACTOR = np.zeros((4, 8))
_SNAP_FACTOR[:, 4:] = np.linalg.cholesky(_SNAP_GRAM).T
for _constant in (_HERMITE, _SNAP_GRAM, _SNAP_FACTOR):
    _constant.flags.writeable = False


def _basis(s: float, order: int) -> NDArray[np.float64]:
    return np.array(
        [
            0.0 if k < order else factorial(k) / factorial(k - order) * s ** (k - order)
            for k in range(8)
        ]
    )


def _durations(value: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    if not isinstance(value, np.ndarray) or value.ndim != 1 or not 1 <= len(value) <= MAX_SEGMENTS:
        raise ValueError(f"segment_durations_s must contain 1..{MAX_SEGMENTS} durations")
    durations = _array("segment_durations_s", value, (len(value),))
    if np.any(durations <= 0):
        raise ValueError("segment durations must be positive")
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise", under="raise"):
            knots = np.concatenate((np.zeros(1), np.cumsum(durations)))
            if np.min(durations) / np.max(durations) < 1e-6:
                raise ValueError("duration ratio must be at most 1e6")
    except FloatingPointError:
        raise ValueError("duration arithmetic must remain representable") from None
    if np.any(np.diff(knots) <= 0):
        raise ValueError("knot times must remain distinguishable")
    return durations, _array("knot_times_s", knots, (len(durations) + 1,))


@dataclass(frozen=True, slots=True, eq=False)
class MinimumSnapTrajectory:
    """Owned normalized coefficients relative to a common NED position origin.

    coefficients_W[i,k] multiplies s**k (metres); position adds position_origin_W.
    The constructor validates numerical representation, not optimality; use
    minimum_snap_trajectory to solve the constrained optimization problem.
    Domain is [0, knot_times_s[-1]] with right-continuous segment selection.
    """

    segment_durations_s: NDArray[np.float64]
    coefficients_W: NDArray[np.float64]
    position_origin_W: NDArray[np.float64]
    scaled_system_condition: float
    relative_stationarity_residual: float
    knot_times_s: NDArray[np.float64] = field(init=False)
    snap_cost: float = field(init=False)

    def __post_init__(self) -> None:
        durations, knots = _durations(self.segment_durations_s)
        coefficients = _array("coefficients_W", self.coefficients_W, (len(durations), 8, 3))
        origin = _array("position_origin_W", self.position_origin_W, (3,))
        condition = _scalar("scaled_system_condition", self.scaled_system_condition, positive=True)
        residual = _scalar("relative_stationarity_residual", self.relative_stationarity_residual)
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise", under="raise"):
                factors = np.array(
                    [
                        _SNAP_FACTOR @ c / t**3.5
                        for c, t in zip(coefficients, durations, strict=True)
                    ]
                )
                cost = float(np.sum(factors**2))
                # The position addition and every supported derivative must be
                # representable, even if the caller first asks only for cost.
                for c, t in zip(coefficients, durations, strict=True):
                    for r in range(5):
                        values = np.array([_basis(s, r) @ c for s in (0.0, 0.5, 1.0)]) / t**r
                        if r == 0:
                            values += origin
                        if not np.all(np.isfinite(values)):
                            raise ValueError("trajectory arithmetic must remain finite")
        except (FloatingPointError, OverflowError):
            raise ValueError("trajectory cost and derivatives must remain representable") from None
        for name, value in (
            ("segment_durations_s", durations),
            ("coefficients_W", coefficients),
            ("position_origin_W", origin),
            ("knot_times_s", knots),
            ("scaled_system_condition", condition),
            ("relative_stationarity_residual", residual),
            ("snap_cost", cost),
        ):
            object.__setattr__(self, name, value)

    def evaluate(self, time_s: float, derivative_order: int = 0) -> NDArray[np.float64]:
        """Return NED derivative 0..4 in m/s**order; no extrapolation or clamping."""
        time = _scalar("time_s", time_s)
        if type(derivative_order) is not int or not 0 <= derivative_order <= 4:
            raise ValueError("derivative_order must be an integer in [0, 4]")
        if time > self.knot_times_s[-1]:
            raise ValueError("time_s lies outside the trajectory")
        index = min(
            int(np.searchsorted(self.knot_times_s, time, side="right")) - 1,
            len(self.segment_durations_s) - 1,
        )
        duration = self.segment_durations_s[index]
        s = 1.0 if time == self.knot_times_s[-1] else (time - self.knot_times_s[index]) / duration
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise", under="raise"):
                result = (
                    _basis(float(s), derivative_order)
                    @ self.coefficients_W[index]
                    / duration**derivative_order
                )
                if derivative_order == 0:
                    result += self.position_origin_W
        except (FloatingPointError, OverflowError):
            raise ValueError("trajectory evaluation must remain representable") from None
        return _array("derivative_W", result, (3,))

    def position_reference(self, time_s: float, yaw_rad: float = 0.0) -> PositionReference:
        """Adapt p/v/a to the existing controller interface with a supplied fixed yaw."""
        return PositionReference(
            self.evaluate(time_s), self.evaluate(time_s, 1), self.evaluate(time_s, 2), yaw_rad
        )


def minimum_snap_trajectory(
    waypoints_W: NDArray[np.float64],
    segment_durations_s: NDArray[np.float64],
    initial_derivatives_W: NDArray[np.float64] | None = None,
    final_derivatives_W: NDArray[np.float64] | None = None,
) -> MinimumSnapTrajectory:
    """Solve the unique fixed-time C3 waypoint spline (ADR 0015).

    Endpoint arrays have shape (3,3): rows velocity, acceleration, jerk;
    columns NED. None means all zero. No bounds on internal derivatives are
    imposed. At most 64 segments and duration ratio <= 1e6 are supported.
    Scaling makes a uniform change of duration units benign; an unresolved
    reduced system, nonfinite or unrepresentable arithmetic raises ValueError.
    """
    durations, _ = _durations(segment_durations_s)
    n = len(durations)
    points = _array("waypoints_W", waypoints_W, (n + 1, 3))
    endpoints = [
        np.zeros((3, 3)) if value is None else _array(name, value, (3, 3))
        for name, value in (
            ("initial_derivatives_W", initial_derivatives_W),
            ("final_derivatives_W", final_derivatives_W),
        )
    ]
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise", under="raise"):
            time_scale = np.max(durations)
            h = durations / time_scale
            centered = points - points[0]
            boundary = np.asarray(endpoints) * time_scale ** np.arange(1, 4)[None, :, None]
            length_scale = max(float(np.max(np.abs(centered))), float(np.max(np.abs(boundary))))
            if length_scale == 0:
                length_scale = 1.0
            # knot derivative rows refer to global dimensionless time t/time_scale.
            data = np.zeros((n + 1, 4, 3))
            data[:, 0] = centered / length_scale
            data[0, 1:], data[-1, 1:] = boundary / length_scale
            free_count = 3 * (n - 1)
            design = np.zeros((4 * n, free_count))
            target = np.empty((4 * n, 3))
            maps = []
            for i, interval in enumerate(h):
                transform = _HERMITE * np.tile(interval ** np.arange(4), 2)[None, :]
                maps.append(transform)
                factor = _SNAP_FACTOR @ transform / interval**3.5
                local = data[i : i + 2].reshape(8, 3)
                target[4 * i : 4 * i + 4] = -(factor @ local)
                for side in (0, 1):
                    knot = i + side
                    if 0 < knot < n:
                        for r in range(1, 4):
                            design[4 * i : 4 * i + 4, 3 * (knot - 1) + r - 1] = factor[
                                :, 4 * side + r
                            ]
            condition, residual = 1.0, 0.0
            if free_count:
                column_scale = np.linalg.norm(design, axis=0)
                scaled = design / column_scale
                solution, _, rank, singular = np.linalg.lstsq(scaled, target, rcond=None)
                condition = float(singular[0] / singular[-1])
                if rank != free_count or condition > MAX_SCALED_CONDITION:
                    raise ValueError("minimum-snap system is ill-conditioned or unresolved")
                gradient = scaled.T @ (scaled @ solution - target)
                residual = float(np.max(np.abs(gradient))) / max(
                    1.0, float(np.linalg.norm(scaled, ord=1) * np.max(np.abs(target)))
                )
                if residual > 2e-10:
                    raise ValueError("minimum-snap stationarity residual is unresolved")
                data[1:-1, 1:] = (solution / column_scale[:, None]).reshape(n - 1, 3, 3)
            coefficients = np.array(
                [transform @ data[i : i + 2].reshape(8, 3) for i, transform in enumerate(maps)]
            )
            # Check every Hermite endpoint in scaled coordinates, independent of
            # the cost minimization residual and before restoring physical units.
            for i, interval in enumerate(h):
                for side in (0, 1):
                    for r in range(4):
                        row = _basis(float(side), r) / interval**r
                        actual = row @ coefficients[i]
                        expected = data[i + side, r]
                        scale = max(1.0, float(np.max(np.abs(expected))))
                        roundoff_bound = (
                            32
                            * np.finfo(float).eps
                            * float(np.max(np.abs(row) @ np.abs(coefficients[i])))
                        )
                        # Large power coefficients can cancel at a knot even
                        # when the reduced least-squares system is well scaled.
                        # Do not use coefficient magnitude to hide that loss.
                        if (
                            roundoff_bound > 2e-8 * scale
                            or np.max(np.abs(actual - expected)) > 2e-9 * scale
                        ):
                            raise ValueError("minimum-snap boundary residual is unresolved")
            coefficients *= length_scale
    except (FloatingPointError, OverflowError, np.linalg.LinAlgError):
        raise ValueError("minimum-snap arithmetic must remain representable and solvable") from None
    return MinimumSnapTrajectory(durations, coefficients, points[0], condition, residual)
