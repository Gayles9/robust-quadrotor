"""Conservative nominal reference bounds and bounded uniform retiming (ADR 0016).

Bernstein convex hulls cover entire polynomial intervals. Floating-point padding
is an engineering guard, not formal interval arithmetic. No motor/torque, drag,
tracking, obstacle or hardware-feasibility guarantee is made.
"""

from dataclasses import dataclass, replace
from math import atan, comb, factorial, pi

import numpy as np
from numpy.typing import NDArray

from .attitude_control import _array, _scalar
from .minimum_snap import MinimumSnapTrajectory

C3_TOLERANCE = 1e-7  # absolute numerical p/v/a/j tolerance in respective SI units


@dataclass(frozen=True, slots=True, eq=False)
class TrajectoryLimits:
    """Nominal NED box, speed/acceleration, mass/gravity, thrust/tilt/rate limits.

    speed=None omits the speed condition for a controller with no speed bound.
    Rate is the norm of the continuous fixed-yaw reference angular velocity,
    not the feedback controller's commanded or actual body rate.
    """

    minimum_position_W: NDArray[np.float64]
    maximum_position_W: NDArray[np.float64]
    maximum_speed_m_s: float | None
    maximum_acceleration_W: NDArray[np.float64]
    nominal_mass: float
    nominal_gravity_m_s2: float
    minimum_thrust_N: float
    maximum_thrust_N: float
    maximum_tilt_rad: float
    maximum_reference_rate_rad_s: float

    def __post_init__(self) -> None:
        for name in ("minimum_position_W", "maximum_position_W", "maximum_acceleration_W"):
            object.__setattr__(self, name, _array(name, getattr(self, name), (3,)))
        if np.any(self.minimum_position_W >= self.maximum_position_W):
            raise ValueError("geofence bounds must be strictly ordered")
        if np.any(self.maximum_acceleration_W <= 0):
            raise ValueError("acceleration bounds must be positive")
        for name in (
            "maximum_speed_m_s",
            "nominal_mass",
            "nominal_gravity_m_s2",
            "minimum_thrust_N",
            "maximum_thrust_N",
            "maximum_tilt_rad",
            "maximum_reference_rate_rad_s",
        ):
            value = getattr(self, name)
            if name != "maximum_speed_m_s" or value is not None:
                object.__setattr__(self, name, _scalar(name, value, positive=True))
        if self.maximum_tilt_rad >= pi / 2:
            raise ValueError("tilt must be below pi/2")
        hover = self.nominal_mass * self.nominal_gravity_m_s2
        if not self.minimum_thrust_N < hover < self.maximum_thrust_N:
            raise ValueError("finite hover thrust must lie strictly inside thrust limits")


def validate_trajectory_continuity(
    trajectory: MinimumSnapTrajectory, *, require_rest: bool = False
) -> None:
    """Reject non-C3 representations and optionally nonzero endpoint v/a/j.

    The solver's public representation constructor does not assert optimality
    or continuity. Consumers must not assume all constructed objects are solves.
    """
    if not isinstance(trajectory, MinimumSnapTrajectory):
        raise TypeError("trajectory must be MinimumSnapTrajectory")
    endpoints = np.empty((len(trajectory.segment_durations_s), 2, 4, 3))
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise", under="raise"):
            for i, (c, duration) in enumerate(
                zip(trajectory.coefficients_W, trajectory.segment_durations_s, strict=True)
            ):
                for r in range(4):
                    derivative = np.polynomial.polynomial.polyder(c, r, axis=0) / duration**r
                    endpoints[i, 0, r] = derivative[0]
                    endpoints[i, 1, r] = np.sum(derivative, axis=0)
    except (FloatingPointError, OverflowError):
        raise ValueError("continuity arithmetic must remain representable") from None
    if np.any(np.abs(endpoints[:-1, 1] - endpoints[1:, 0]) > C3_TOLERANCE):
        raise ValueError("trajectory must have C3 internal joins")
    if require_rest and (
        np.any(np.abs(endpoints[0, 0, 1:]) > C3_TOLERANCE)
        or np.any(np.abs(endpoints[-1, 1, 1:]) > C3_TOLERANCE)
    ):
        raise ValueError("trajectory must be rest-to-rest through jerk")


@dataclass(frozen=True, slots=True, eq=False)
class TrajectoryFeasibility:
    """Sufficient whole-curve bounds; rejection can be conservative.

    Undefined tilt/rate bounds are None when positive down-thrust is unresolved.
    No sampled extrema are substituted for the conservative bounds.
    """

    minimum_position_W: NDArray[np.float64]
    maximum_position_W: NDArray[np.float64]
    maximum_speed_m_s: float
    maximum_acceleration_W: NDArray[np.float64]
    minimum_thrust_N: float
    maximum_thrust_N: float
    minimum_down_specific_thrust_m_s2: float
    maximum_tilt_rad: float | None
    maximum_reference_rate_rad_s: float | None
    violations: tuple[str, ...]
    subdivision_depth: int

    def __post_init__(self) -> None:
        for name in ("minimum_position_W", "maximum_position_W", "maximum_acceleration_W"):
            object.__setattr__(self, name, _array(name, getattr(self, name), (3,)))

    @property
    def accepted(self) -> bool:
        return not self.violations


def _depth(value: int) -> int:
    if type(value) is not int or not 0 <= value <= 8:
        raise ValueError("subdivision_depth must be an integer in [0,8]")
    return value


def _bernstein_leaves(power: NDArray[np.float64], depth: int) -> NDArray[np.float64]:
    """Power -> Bernstein on [0,1], then vectorized midpoint de Casteljau."""
    degree = len(power) - 1
    transform = np.array(
        [
            [comb(i, k) / comb(degree, k) if k <= i else 0.0 for k in range(degree + 1)]
            for i in range(degree + 1)
        ]
    )
    leaves = (transform @ power)[None, :, :]
    for _ in range(depth):
        work = leaves.copy()
        left, right = np.empty_like(leaves), np.empty_like(leaves)
        left[:, 0], right[:, degree] = work[:, 0], work[:, -1]
        for level in range(1, degree + 1):
            work = 0.5 * work[:, :-1] + 0.5 * work[:, 1:]
            left[:, level], right[:, degree - level] = work[:, 0], work[:, -1]
        leaves = np.stack((left, right), axis=1).reshape(-1, degree + 1, 3)
    return leaves


def check_trajectory_feasibility(
    trajectory: MinimumSnapTrajectory,
    limits: TrajectoryLimits,
    *,
    subdivision_depth: int = 6,
) -> TrajectoryFeasibility:
    """Bound C3 reference geometry and nominal fixed-yaw thrust/rate demand.

    For u=g*e3-a with u_z>=z>0, tilt <= atan(||u_xy||/z) and
    ||omega_ref|| <= ||jerk||/z for the existing heading-y attitude construction.
    Bounds use each shared subinterval before reducing over the full trajectory.
    """
    validate_trajectory_continuity(trajectory)
    if not isinstance(limits, TrajectoryLimits):
        raise TypeError("limits must be TrajectoryLimits")
    model, depth = replace(limits), _depth(subdivision_depth)
    pmin, pmax, amax = np.full(3, np.inf), np.full(3, -np.inf), np.zeros(3)
    speed, thrust_max, tilt_max, rate_max = 0.0, 0.0, 0.0, 0.0
    thrust_min, down_min = np.inf, np.inf
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise", under="raise"):
            for coefficients, duration in zip(
                trajectory.coefficients_W, trajectory.segment_durations_s, strict=True
            ):
                leaves, pads = [], []
                for r in range(4):
                    power = np.array(
                        [
                            coefficients[k] * (factorial(k) / factorial(k - r)) / duration**r
                            for k in range(r, 8)
                        ]
                    )
                    absolute = np.sum(np.abs(power), axis=0)
                    if r == 0:
                        absolute += np.abs(trajectory.position_origin_W)
                        power[0] += trajectory.position_origin_W
                    # Covers small-degree conversion, derivatives and <=8 splits
                    # at ordinary float64 scales; do not claim formal interval proof.
                    pad = 512 * np.finfo(float).eps * (1 + absolute)
                    leaves.append(_bernstein_leaves(power, depth))
                    pads.append(pad)
                p, v, a, jerk = leaves
                pmin = np.minimum(pmin, np.min(p, axis=(0, 1)) - pads[0])
                pmax = np.maximum(pmax, np.max(p, axis=(0, 1)) + pads[0])
                speed = max(
                    speed, float(np.max(np.linalg.norm(v, axis=2)) + np.linalg.norm(pads[1]))
                )
                amax = np.maximum(amax, np.max(np.abs(a), axis=(0, 1)) + pads[2])
                u = -a
                u[:, :, 2] += model.nominal_gravity_m_s2
                upad = pads[2] + 512 * np.finfo(float).eps * model.nominal_gravity_m_s2
                z = np.min(u[:, :, 2], axis=1) - upad[2]
                down_min = min(down_min, float(np.min(z)))
                thrust_min = min(thrust_min, model.nominal_mass * max(0.0, float(np.min(z))))
                thrust_max = max(
                    thrust_max,
                    model.nominal_mass
                    * float(np.max(np.linalg.norm(u, axis=2)) + np.linalg.norm(upad)),
                )
                if np.all(z > 0):
                    horizontal = np.max(
                        np.linalg.norm(u[:, :, :2], axis=2), axis=1
                    ) + np.linalg.norm(upad[:2])
                    jnorm = np.max(np.linalg.norm(jerk, axis=2), axis=1) + np.linalg.norm(pads[3])
                    tilt_max = max(tilt_max, atan(float(np.max(horizontal / z))))
                    rate_max = max(rate_max, float(np.max(jnorm / z)))
            scalars = [speed, thrust_min, thrust_max, down_min, tilt_max, rate_max]
            if not np.all(np.isfinite(scalars)):
                raise FloatingPointError
    except (FloatingPointError, OverflowError):
        raise ValueError("feasibility arithmetic must remain representable") from None
    failures: list[str] = []
    if np.any(pmin < model.minimum_position_W) or np.any(pmax > model.maximum_position_W):
        failures.append("geofence")
    if model.maximum_speed_m_s is not None and speed > model.maximum_speed_m_s:
        failures.append("speed")
    if np.any(amax > model.maximum_acceleration_W):
        failures.append("acceleration")
    if thrust_min < model.minimum_thrust_N or thrust_max > model.maximum_thrust_N:
        failures.append("thrust")
    if down_min <= 0:
        failures.append("positive_down_thrust")
    if down_min <= 0 or tilt_max > model.maximum_tilt_rad:
        failures.append("tilt")
    if down_min <= 0 or rate_max > model.maximum_reference_rate_rad_s:
        failures.append("reference_rate")
    return TrajectoryFeasibility(
        pmin,
        pmax,
        speed,
        amax,
        thrust_min,
        thrust_max,
        down_min,
        tilt_max if down_min > 0 else None,
        rate_max if down_min > 0 else None,
        tuple(failures),
        depth,
    )


@dataclass(frozen=True, slots=True)
class RetimingAttempt:
    scale: float
    feasibility: TrajectoryFeasibility


@dataclass(frozen=True, slots=True)
class RetimingResult:
    trajectory: MinimumSnapTrajectory | None
    scale: float
    attempts: tuple[RetimingAttempt, ...]
    reason: str | None


def retime_trajectory(
    trajectory: MinimumSnapTrajectory,
    limits: TrajectoryLimits,
    *,
    maximum_scale: float = 8.0,
    scale_step: float = 1.25,
    maximum_attempts: int = 12,
    subdivision_depth: int = 6,
) -> RetimingResult:
    """Return the first bounded-search acceptance, or None plus every attempt.

    Preserve coefficients, origin and relative durations. Scale zero endpoint
    derivatives with time; no waypoint optimization or controller tuning. Search
    feasibility need not be monotone and minimum feasible time is not claimed.
    """
    validate_trajectory_continuity(trajectory, require_rest=True)
    maximum_scale = _scalar("maximum_scale", maximum_scale, positive=True)
    scale_step = _scalar("scale_step", scale_step, positive=True)
    _depth(subdivision_depth)
    if maximum_scale < 1 or scale_step <= 1:
        raise ValueError("maximum_scale must be >=1 and scale_step >1")
    if type(maximum_attempts) is not int or not 1 <= maximum_attempts <= 64:
        raise ValueError("maximum_attempts must be an integer in [1,64]")
    scale = 1.0
    attempts: list[RetimingAttempt] = []
    for _ in range(maximum_attempts):
        try:
            with np.errstate(over="raise", invalid="raise", under="raise"):
                candidate = replace(
                    trajectory, segment_durations_s=trajectory.segment_durations_s * scale
                )
        except FloatingPointError:
            raise ValueError("retiming arithmetic must remain representable") from None
        report = check_trajectory_feasibility(
            candidate, limits, subdivision_depth=subdivision_depth
        )
        attempts.append(RetimingAttempt(scale, report))
        if report.accepted:
            return RetimingResult(candidate, scale, tuple(attempts), None)
        if "geofence" in report.violations:
            return RetimingResult(None, scale, tuple(attempts), "geofence_not_certified")
        if scale == maximum_scale:
            break
        # Avoid overflowing a product that will immediately be capped anyway.
        following = maximum_scale if scale > maximum_scale / scale_step else scale * scale_step
        if following <= scale:
            break
        scale = following
    return RetimingResult(None, attempts[-1].scale, tuple(attempts), "search_budget_exhausted")
