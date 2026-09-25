"""Reference primitives and a sampled, terminal-latching mission supervisor.

Airborne NED model only: LAND returns to a virtual plane, not ground contact.
ABORT terminates numerical execution; it does not prescribe hardware recovery.
"""

from dataclasses import dataclass, replace
from enum import IntEnum, StrEnum
from math import acos, fsum, hypot, isfinite, pi

import numpy as np
from numpy.typing import NDArray

from .attitude_control import _array, _quaternion, _scalar
from .minimum_snap import MinimumSnapTrajectory
from .position_control import PositionReference, _finite_scalar
from .rotations import rotation_matrix_body_to_world
from .trajectory_feasibility import C3_TOLERANCE, validate_trajectory_continuity


class MissionPhase(IntEnum):
    INITIALIZE = 0
    TAKEOFF = 1
    TRACK = 2
    LAND = 3
    COMPLETE = 4
    ABORT = 5


class ReferenceKind(StrEnum):
    HOLD = "hold"
    SMOOTH = "smooth"
    STEP = "step"
    MINIMUM_SNAP = "minimum_snap"


@dataclass(frozen=True, slots=True, eq=False)
class MissionSegment:
    """Rest-to-rest reference on a positive-duration interval, NED metres.

    HOLD requires equal endpoints. SMOOTH uses a quintic blend. STEP is an
    explicitly discontinuous TRACK diagnostic: end_position_W from its start.
    """

    phase: MissionPhase
    duration_s: float
    start_position_W: NDArray[np.float64]
    end_position_W: NDArray[np.float64]
    kind: ReferenceKind

    def __post_init__(self) -> None:
        if not isinstance(self.phase, MissionPhase) or self.phase.value > MissionPhase.LAND:
            raise ValueError("segment phase must be INITIALIZE, TAKEOFF, TRACK or LAND")
        if not isinstance(self.kind, ReferenceKind):
            raise ValueError("kind must be ReferenceKind")
        object.__setattr__(
            self, "duration_s", _scalar("duration_s", self.duration_s, positive=True)
        )
        for name in ("start_position_W", "end_position_W"):
            object.__setattr__(self, name, _array(name, getattr(self, name), (3,)))
        if self.kind == ReferenceKind.HOLD and not np.array_equal(
            self.start_position_W, self.end_position_W
        ):
            raise ValueError("hold endpoints must be equal")
        if self.kind == ReferenceKind.STEP and self.phase != MissionPhase.TRACK:
            raise ValueError("step references are restricted to TRACK diagnostics")
        if self.phase == MissionPhase.INITIALIZE and self.kind != ReferenceKind.HOLD:
            raise ValueError("INITIALIZE requires a hold")
        if self.kind == ReferenceKind.MINIMUM_SNAP and not isinstance(
            self, MinimumSnapMissionSegment
        ):
            raise ValueError("minimum_snap requires MinimumSnapMissionSegment")


@dataclass(frozen=True, slots=True, eq=False)
class MinimumSnapMissionSegment(MissionSegment):
    """Owned C3 rest-to-rest polynomial, possibly spanning several waypoints.

    This separate subclass preserves the exact historical MissionSegment
    dataclass/serialized schema. Continuity tolerance is 1e-7 in SI units.
    Full-curve nominal feasibility is checked by the true-state runner preflight.
    """

    trajectory: MinimumSnapTrajectory

    def __post_init__(self) -> None:
        MissionSegment.__post_init__(self)
        if self.kind != ReferenceKind.MINIMUM_SNAP:
            raise ValueError("polynomial segments require minimum_snap kind")
        validate_trajectory_continuity(self.trajectory, require_rest=True)
        curve = replace(self.trajectory)
        if self.duration_s != curve.knot_times_s[-1]:
            raise ValueError("segment duration must equal the stored trajectory endpoint")
        for time, point in ((0.0, self.start_position_W), (self.duration_s, self.end_position_W)):
            if np.any(np.abs(curve.evaluate(time) - point) > C3_TOLERANCE):
                raise ValueError("segment endpoints must match the trajectory")
        object.__setattr__(self, "trajectory", curve)


def sample_segment(segment: MissionSegment, elapsed_s: float, yaw_rad: float) -> PositionReference:
    """Analytic p/v/a; elapsed is clamped to the segment's endpoints.

    A step has zero reference velocity/acceleration on either open side of
    its jump; the impulse at the jump is deliberately not represented.
    """
    if not isinstance(segment, MissionSegment):
        raise TypeError("segment must be MissionSegment")
    elapsed = _finite_scalar("elapsed_s", elapsed_s)
    yaw = _finite_scalar("yaw_rad", yaw_rad)
    if elapsed < 0:
        return PositionReference(segment.start_position_W, np.zeros(3), np.zeros(3), yaw)
    if segment.kind == ReferenceKind.MINIMUM_SNAP:
        assert isinstance(segment, MinimumSnapMissionSegment)
        if 0 < elapsed < segment.duration_s:
            return segment.trajectory.position_reference(elapsed, yaw)
        point = segment.start_position_W if elapsed == 0 else segment.end_position_W
        return PositionReference(point, np.zeros(3), np.zeros(3), yaw)
    if segment.kind != ReferenceKind.SMOOTH or elapsed >= segment.duration_s:
        return PositionReference(segment.end_position_W, np.zeros(3), np.zeros(3), yaw)
    if elapsed <= 0:
        return PositionReference(segment.start_position_W, np.zeros(3), np.zeros(3), yaw)
    s = elapsed / segment.duration_s
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            displacement = segment.end_position_W - segment.start_position_W
            blend = s * s * s * (10 + s * (-15 + 6 * s))
            first = 30 * s * s * (1 - s) * (1 - s) / segment.duration_s
            second = 60 * s * (1 - s) * (1 - 2 * s) / segment.duration_s / segment.duration_s
            position = segment.start_position_W + blend * displacement
            velocity, acceleration = first * displacement, second * displacement
    except FloatingPointError:
        raise ValueError("reference arithmetic must remain finite") from None
    return PositionReference(position, velocity, acceleration, yaw)


@dataclass(frozen=True, slots=True, eq=False)
class MissionPlan:
    """Ordered, connected INIT/TAKEOFF/TRACK/LAND segments and terminal contract.

    Transition times are sums of supplied durations, not detection of arrival.
    Completion alone uses measured position/speed and a continuous sampled dwell.
    """

    segments: tuple[MissionSegment, ...]
    yaw_rad: float
    completion_position_tolerance_m: float
    completion_velocity_tolerance_m_s: float
    completion_dwell_s: float
    completion_timeout_s: float

    def __post_init__(self) -> None:
        if (
            not isinstance(self.segments, tuple)
            or not self.segments
            or not all(isinstance(s, MissionSegment) for s in self.segments)
        ):
            raise ValueError("segments must be a nonempty tuple of MissionSegment")
        segments = tuple(replace(segment) for segment in self.segments)
        groups: list[MissionPhase] = []
        end = segments[0].start_position_W
        for segment in segments:
            if not np.array_equal(end, segment.start_position_W):
                raise ValueError("adjacent segment endpoints must be connected")
            end = segment.end_position_W
            if not groups or groups[-1] != segment.phase:
                groups.append(segment.phase)
        if groups != [
            MissionPhase.INITIALIZE,
            MissionPhase.TAKEOFF,
            MissionPhase.TRACK,
            MissionPhase.LAND,
        ]:
            raise ValueError("segments must have ordered INITIALIZE/TAKEOFF/TRACK/LAND phases")
        object.__setattr__(self, "segments", segments)
        object.__setattr__(self, "yaw_rad", _finite_scalar("yaw_rad", self.yaw_rad))
        for name in (
            "completion_position_tolerance_m",
            "completion_velocity_tolerance_m_s",
            "completion_dwell_s",
            "completion_timeout_s",
        ):
            object.__setattr__(self, name, _scalar(name, getattr(self, name), positive=True))
        if self.completion_timeout_s < self.completion_dwell_s:
            raise ValueError("completion timeout must allow a full dwell")
        try:
            total = fsum(s.duration_s for s in segments)
        except OverflowError:
            raise ValueError("mission clock must remain finite") from None
        clock = np.cumsum([0.0, *(s.duration_s for s in segments)])
        if not isfinite(total + self.completion_timeout_s) or np.any(np.diff(clock) <= 0):
            raise ValueError("mission clock must be finite and distinguishable")

    @property
    def reference_duration_s(self) -> float:
        # Match the sequential boundary arithmetic in mission_reference exactly.
        return sum(segment.duration_s for segment in self.segments)


def mission_reference(plan: MissionPlan, time_s: float) -> tuple[PositionReference, MissionPhase]:
    """Right-continuous segment selection; hold final landing target afterward."""
    if not isinstance(plan, MissionPlan):
        raise TypeError("plan must be MissionPlan")
    time = _scalar("time_s", time_s)
    start = 0.0
    for segment in plan.segments:
        end = start + segment.duration_s
        if time < end:
            return sample_segment(segment, time - start, plan.yaw_rad), segment.phase
        start = end
    return sample_segment(
        plan.segments[-1], plan.segments[-1].duration_s, plan.yaw_rad
    ), MissionPhase.LAND


@dataclass(frozen=True, slots=True, eq=False)
class MissionSafetyLimits:
    """Closed NED box [m] and actual body-z tilt bound [rad], checked at epochs."""

    minimum_position_W: NDArray[np.float64]
    maximum_position_W: NDArray[np.float64]
    maximum_tilt_rad: float

    def __post_init__(self) -> None:
        for name in ("minimum_position_W", "maximum_position_W"):
            object.__setattr__(self, name, _array(name, getattr(self, name), (3,)))
        if np.any(self.minimum_position_W >= self.maximum_position_W):
            raise ValueError("geofence bounds must be strictly ordered")
        tilt = _scalar("maximum_tilt_rad", self.maximum_tilt_rad, positive=True)
        if tilt >= pi / 2:
            raise ValueError("safety tilt must be below pi/2")
        object.__setattr__(self, "maximum_tilt_rad", tilt)


def mission_guard_reason(
    position_W: NDArray[np.float64], q_WB: NDArray[np.float64], limits: MissionSafetyLimits
) -> str | None:
    """Geofence has deterministic priority over tilt; malformed inputs raise."""
    if not isinstance(limits, MissionSafetyLimits):
        raise TypeError("limits must be MissionSafetyLimits")
    position = _array("position_W", position_W, (3,))
    attitude = rotation_matrix_body_to_world(_quaternion("q_WB", q_WB))
    if np.any(position < limits.minimum_position_W) or np.any(position > limits.maximum_position_W):
        return "geofence"
    tilt = acos(float(np.clip(attitude[2, 2], -1, 1)))
    return "tilt" if tilt > limits.maximum_tilt_rad else None


@dataclass(frozen=True, slots=True)
class MissionState:
    """Supervisor memory only; terminal states are absorbing. Seconds from origin."""

    phase: MissionPhase = MissionPhase.INITIALIZE
    time_s: float = 0.0
    settled_since_s: float | None = None
    reason: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.phase, MissionPhase):
            raise ValueError("phase must be MissionPhase")
        object.__setattr__(self, "time_s", _scalar("time_s", self.time_s))
        if self.settled_since_s is not None:
            start = _scalar("settled_since_s", self.settled_since_s)
            if start > self.time_s or self.phase not in (MissionPhase.LAND, MissionPhase.COMPLETE):
                raise ValueError("settling memory must belong to landing and precede time")
            object.__setattr__(self, "settled_since_s", start)
        if (self.phase == MissionPhase.ABORT) != (
            isinstance(self.reason, str) and bool(self.reason)
        ):
            raise ValueError("only ABORT requires a nonempty reason")
        if self.phase != MissionPhase.ABORT and self.reason is not None:
            raise ValueError("non-aborted state cannot have a reason")


def advance_mission(
    state: MissionState,
    plan: MissionPlan,
    time_s: float,
    position_W: NDArray[np.float64],
    velocity_W: NDArray[np.float64],
    guard_reason: str | None = None,
) -> MissionState:
    """Guard first, phase schedule second, measured landing dwell/timeout last."""
    if not isinstance(state, MissionState) or not isinstance(plan, MissionPlan):
        raise TypeError("state and plan must use their dataclasses")
    time = _scalar("time_s", time_s)
    position = _array("position_W", position_W, (3,))
    velocity = _array("velocity_W", velocity_W, (3,))
    if time < state.time_s:
        raise ValueError("mission time cannot go backward")
    if guard_reason is not None and (not isinstance(guard_reason, str) or not guard_reason):
        raise ValueError("guard reason must be a nonempty string or None")
    if state.phase in (MissionPhase.COMPLETE, MissionPhase.ABORT):
        return state
    if guard_reason is not None:
        return MissionState(MissionPhase.ABORT, time, reason=guard_reason)
    reference, phase = mission_reference(plan, time)
    if time < plan.reference_duration_s:
        return MissionState(phase, time)
    with np.errstate(over="ignore", invalid="ignore"):
        close = (
            hypot(*(position - reference.position_W)) <= plan.completion_position_tolerance_m
            and hypot(*velocity) <= plan.completion_velocity_tolerance_m_s
        )
    settled = (
        (state.settled_since_s if state.settled_since_s is not None else time) if close else None
    )
    if settled is not None and time - settled >= plan.completion_dwell_s:
        return MissionState(MissionPhase.COMPLETE, time, settled)
    if time >= plan.reference_duration_s + plan.completion_timeout_s:
        return MissionState(MissionPhase.ABORT, time, reason="landing_timeout")
    return MissionState(MissionPhase.LAND, time, settled)
