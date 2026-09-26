"""Fixed-grid cascade or opt-in geometric control with sampled mission guards."""

from collections.abc import Callable
from dataclasses import dataclass, replace
from math import ceil, hypot, isfinite

import numpy as np
from numpy.typing import NDArray

from .attitude_control import (
    AttitudeControllerParameters,
    _array,
    _inertia,
    _quaternion,
    _rotors,
    _scalar,
    allocate_limited_body_moment,
    attitude_error_body,
    compute_attitude_control,
)
from .attitude_simulation import State, _motor, _plant_step, _wrench
from .geometric_control import (
    GeometricControllerParameters,
    GeometricDomainError,
    compute_geometric_control,
)
from .geometric_filter import FeedbackDerivativeFilter
from .geometric_reference import (
    GeometricPositionReference,
    build_geometric_reference,
    projected_collective_thrust,
    reference_jerk_snap,
)
from .missions import (
    MinimumSnapMissionSegment,
    MissionPhase,
    MissionPlan,
    MissionSafetyLimits,
    MissionState,
    ReferenceKind,
    advance_mission,
    mission_guard_reason,
    mission_reference,
)
from .position_control import PositionControllerParameters, compute_position_control
from .run_configuration import (
    RigidBodyInitialState,
    RigidBodyParameters,
    RotorParameters,
    WorldParameters,
)
from .trajectory_feasibility import TrajectoryLimits, check_trajectory_feasibility


@dataclass(frozen=True, slots=True)
class MissionNumerics:
    """Positive plant h [s], integer strides; outer ticks must be inner ticks.

    maximum_steps is an explicit allocation/execution budget, not a timeout
    success criterion. The plan's full timeout horizon must fit the budget.
    """

    time_step_s: float
    attitude_stride: int
    position_stride: int
    maximum_steps: int = 1_000_000

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "time_step_s", _scalar("time_step_s", self.time_step_s, positive=True)
        )
        for name in ("attitude_stride", "position_stride", "maximum_steps"):
            value = getattr(self, name)
            if (
                isinstance(value, (bool, np.bool_))
                or not isinstance(value, (int, np.integer))
                or value < 1
            ):
                raise ValueError(f"{name} must be a positive integer")
            object.__setattr__(self, name, int(value))
        if self.position_stride % self.attitude_stride:
            raise ValueError("position stride must be a multiple of attitude stride")


@dataclass(frozen=True, slots=True, eq=False)
class MissionResult:
    """Owned finite histories, including the initial and terminal truth epochs.

    NED state/reference p/v/a [m,m/s,m/s²], FRD omega [rad/s], rotor speed [rad/s],
    moments [N m], thrust [N]. Per-control fields hold over following intervals;
    no command at the terminal time. phase is an int64 MissionPhase at each
    truth epoch. Abort at t=0 legitimately has one state and no commands.
    Inner flags: rate/moment/allocation. Outer flags: acceleration/tilt/thrust.
    Actual moment is rotor-only; commanded nominal allocation is separate.
    This does not extend the historical sensor/run-artifact persistence schema.
    """

    time_s: NDArray[np.float64]
    position_W: NDArray[np.float64]
    velocity_W: NDArray[np.float64]
    q_WB: NDArray[np.float64]
    omega_B: NDArray[np.float64]
    actual_rotor_omega: NDArray[np.float64]
    actual_moment_B: NDArray[np.float64]
    reference_position_W: NDArray[np.float64]
    reference_velocity_W: NDArray[np.float64]
    reference_acceleration_W: NDArray[np.float64]
    phase: NDArray[np.int64]
    control_time_s: NDArray[np.float64]
    q_reference_WB: NDArray[np.float64]
    collective_thrust: NDArray[np.float64]
    commanded_rotor_omega: NDArray[np.float64]
    moment_requested_B: NDArray[np.float64]
    moment_limited_B: NDArray[np.float64]
    allocated_moment_B: NDArray[np.float64]
    desired_omega_B: NDArray[np.float64]
    moment_scale: NDArray[np.float64]
    inner_limit_flags: NDArray[np.bool_]
    position_control_time_s: NDArray[np.float64]
    requested_acceleration_W: NDArray[np.float64]
    feasible_acceleration_W: NDArray[np.float64]
    outer_limit_flags: NDArray[np.bool_]
    abort_reason: str | None

    def __post_init__(self) -> None:
        for name in ("time_s", "control_time_s", "position_control_time_s"):
            value = getattr(self, name)
            if not isinstance(value, np.ndarray) or value.ndim != 1:
                raise ValueError("result clocks must be vectors")
        n, c, o = len(self.time_s), len(self.control_time_s), len(self.position_control_time_s)
        if n < 1:
            raise ValueError("at least one truth epoch is required")
        shapes: dict[str, tuple[int, ...]] = {
            "time_s": (n,),
            "control_time_s": (c,),
            "position_control_time_s": (o,),
            "q_WB": (n, 4),
            "actual_rotor_omega": (n, 4),
            "q_reference_WB": (c, 4),
            "commanded_rotor_omega": (c, 4),
            "collective_thrust": (c,),
            "moment_scale": (c,),
        }
        for name in (
            "position_W",
            "velocity_W",
            "omega_B",
            "actual_moment_B",
            "reference_position_W",
            "reference_velocity_W",
            "reference_acceleration_W",
        ):
            shapes[name] = (n, 3)
        for name in (
            "moment_requested_B",
            "moment_limited_B",
            "allocated_moment_B",
            "desired_omega_B",
        ):
            shapes[name] = (c, 3)
        for name in ("requested_acceleration_W", "feasible_acceleration_W"):
            shapes[name] = (o, 3)
        for name, shape in shapes.items():
            object.__setattr__(self, name, _array(name, getattr(self, name), shape))
        if self.time_s[0] != 0 or np.any(np.diff(self.time_s) <= 0):
            raise ValueError("truth clock must increase from zero")
        for clock, parent in (
            (self.control_time_s, self.time_s[:-1]),
            (self.position_control_time_s, self.control_time_s),
        ):
            if (
                np.any(np.diff(clock) <= 0)
                or not np.all(np.isin(clock, parent))
                or (len(clock) and clock[0] != 0)
            ):
                raise ValueError("control clocks must increase from zero on the parent grid")
        if (n > 1 and (c == 0 or o == 0)) or (n == 1 and (c != 0 or o != 0)):
            raise ValueError("commands must cover each nonempty execution")
        for name, count in (("inner_limit_flags", c), ("outer_limit_flags", o)):
            value = getattr(self, name)
            if (
                not isinstance(value, np.ndarray)
                or value.shape != (count, 3)
                or value.dtype != np.bool_
            ):
                raise ValueError("limit flags must be bool matrices")
            owned = np.array(value, order="C", copy=True)
            owned.flags.writeable = False
            object.__setattr__(self, name, owned)
        phase = self.phase
        if (
            not isinstance(phase, np.ndarray)
            or phase.shape != (n,)
            or phase.dtype.kind not in "iu"
            or np.any((phase < 0) | (phase > 5))
        ):
            raise ValueError("phase must contain valid integer MissionPhase codes")
        if (
            phase[-1] not in (4, 5)
            or np.any(phase[:-1] > 3)
            or np.any(np.diff(phase.astype(np.int64)) < 0)
        ):
            raise ValueError("phases must be ordered with a single terminal endpoint")
        owned_phase = np.array(phase, dtype=np.int64, copy=True, order="C")
        owned_phase.flags.writeable = False
        object.__setattr__(self, "phase", owned_phase)
        if phase[-1] == MissionPhase.ABORT:
            if not isinstance(self.abort_reason, str) or not self.abort_reason:
                raise ValueError("ABORT requires a reason")
        elif self.abort_reason is not None:
            raise ValueError("COMPLETE cannot have an abort reason")
        if (
            np.any(self.actual_rotor_omega < 0)
            or np.any(self.commanded_rotor_omega < 0)
            or np.any(self.collective_thrust <= 0)
            or np.any((self.moment_scale < 0) | (self.moment_scale > 1))
        ):
            raise ValueError("result commands or rotor speeds outside their domain")
        for history in (self.q_WB, self.q_reference_WB):
            for row in history:
                _quaternion("q_WB history", row)


def simulate_mission(
    initial_state: RigidBodyInitialState,
    initial_actual_rotor_omega: NDArray[np.float64],
    truth_body: RigidBodyParameters,
    truth_rotors: RotorParameters,
    truth_world: WorldParameters,
    position_controller: PositionControllerParameters,
    attitude_controller: AttitudeControllerParameters,
    plan: MissionPlan,
    safety: MissionSafetyLimits,
    numerics: MissionNumerics,
    *,
    geometric_controller: GeometricControllerParameters | None = None,
) -> MissionResult:
    """True-state cascade or opt-in geometric control; independent truth/nominal inputs.

    Guards -> supervisor -> outer -> inner -> existing plant step. Guard aborts
    retain the terminal state; malformed configuration/numerical failures raise
    without returning a partial result. Initial motors are explicit, not inferred
    from truth mass. Every plan endpoint must lie inside the declared geofence.
    """
    arguments = (
        initial_state,
        initial_actual_rotor_omega,
        truth_body,
        truth_rotors,
        truth_world,
        position_controller,
        attitude_controller,
        plan,
        safety,
        numerics,
        None,
    )
    if geometric_controller is None:
        return _simulate_mission(*arguments)
    return _simulate_mission(*arguments, geometric_controller=geometric_controller)


def _simulate_mission(
    initial_state: RigidBodyInitialState,
    initial_actual_rotor_omega: NDArray[np.float64],
    truth_body: RigidBodyParameters,
    truth_rotors: RotorParameters,
    truth_world: WorldParameters,
    position_controller: PositionControllerParameters,
    attitude_controller: AttitudeControllerParameters,
    plan: MissionPlan,
    safety: MissionSafetyLimits,
    numerics: MissionNumerics,
    observer: Callable[[int, float, State, NDArray[np.float64]], State] | None,
    *,
    geometric_controller: GeometricControllerParameters | None = None,
    allow_minimum_snap: bool = False,
    consume_estimator_force_jump: Callable[[], NDArray[np.float64]] | None = None,
) -> MissionResult:
    """Shared private execution; observer is a truth-to-sensor integration boundary.

    None preserves the original true-state path exactly. Otherwise controllers
    and completion use only the returned feedback state; the separate numerical
    truth guard is labelled truth_*, and feedback guards estimate_* (ADR 0013).
    """
    if geometric_controller is not None and not isinstance(
        geometric_controller, GeometricControllerParameters
    ):
        raise TypeError("geometric_controller must be GeometricControllerParameters")
    if type(allow_minimum_snap) is not bool:
        raise ValueError("allow_minimum_snap must be bool")
    for argument, kind in (
        (initial_state, RigidBodyInitialState),
        (truth_body, RigidBodyParameters),
        (truth_world, WorldParameters),
        (position_controller, PositionControllerParameters),
        (attitude_controller, AttitudeControllerParameters),
        (plan, MissionPlan),
        (safety, MissionSafetyLimits),
        (numerics, MissionNumerics),
    ):
        if not isinstance(argument, kind):
            raise TypeError("mission inputs must use their declared dataclasses")
    outer, inner, plan, safety, numerics = (
        replace(position_controller),
        replace(attitude_controller),
        replace(plan),
        replace(safety),
        replace(numerics),
    )
    body = RigidBodyParameters(
        _scalar("mass", truth_body.mass, positive=True),
        _inertia(truth_body.inertia_B),
        _array("drag_B", truth_body.quadratic_drag_coefficient_B, (3,)),
    )
    world = WorldParameters(
        _scalar("gravity", truth_world.gravity_acceleration, positive=True),
        _array("wind_W", truth_world.wind_velocity_W, (3,)),
    )
    rotors = _rotors(truth_rotors)
    nominal = inner.nominal_rotors
    assert rotors.minimum_rotor_omega is not None and rotors.maximum_rotor_omega is not None
    assert nominal.minimum_rotor_omega is not None and nominal.maximum_rotor_omega is not None
    if (
        nominal.minimum_rotor_omega < rotors.minimum_rotor_omega
        or nominal.maximum_rotor_omega > rotors.maximum_rotor_omega
    ):
        raise ValueError("nominal rotor bounds must fit truth bounds")
    for thrust in (outer.minimum_collective_thrust, outer.maximum_collective_thrust):
        allocate_limited_body_moment(thrust, np.zeros(3), nominal)
    if outer.maximum_tilt_rad >= safety.maximum_tilt_rad:
        raise ValueError("command tilt must be below actual tilt guard")
    polynomial_limits = TrajectoryLimits(
        safety.minimum_position_W,
        safety.maximum_position_W,
        None,
        outer.maximum_acceleration_W,
        outer.nominal_mass,
        outer.nominal_gravity_acceleration,
        outer.minimum_collective_thrust,
        outer.maximum_collective_thrust,
        outer.maximum_tilt_rad,
        float(np.min(inner.maximum_body_rate_B)),
    )
    for segment in plan.segments:
        if geometric_controller is not None and segment.kind == ReferenceKind.STEP:
            raise ValueError("geometric control does not support STEP references")
        if isinstance(segment, MinimumSnapMissionSegment):
            if observer is not None and geometric_controller is None and not allow_minimum_snap:
                raise ValueError("minimum-snap missions currently require true-state feedback")
            report = check_trajectory_feasibility(segment.trajectory, polynomial_limits)
            if not report.accepted:
                raise ValueError(
                    "minimum-snap reference not accepted: " + ", ".join(report.violations)
                )
        for point in (segment.start_position_W, segment.end_position_W):
            if np.any(point < safety.minimum_position_W) or np.any(
                point > safety.maximum_position_W
            ):
                raise ValueError("reference endpoint outside geofence")
    dt = numerics.time_step_s
    horizon = plan.reference_duration_s + plan.completion_timeout_s
    quotient = horizon / dt
    if not isfinite(quotient) or quotient > numerics.maximum_steps:
        raise ValueError("mission horizon exceeds maximum_steps")
    n = ceil(quotient)
    times = np.arange(n + 1, dtype=np.float64) * dt
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError("plant clock must remain finite and distinguishable")
    state: State = (
        _array("position_W", initial_state.position_W, (3,)),
        _array("velocity_W", initial_state.velocity_W, (3,)),
        _quaternion("q_WB", initial_state.q_WB),
        _array("omega_B", initial_state.omega_B, (3,)),
    )
    actual = _array("initial_actual_rotor_omega", initial_actual_rotor_omega, (4,))
    _motor(actual, actual, rotors, 0.0)
    history = [np.empty((n + 1, size)) for size in (3, 3, 4, 3, 4, 3, 3, 3, 3)]
    phases = np.empty(n + 1, dtype=np.int64)
    control_indices: list[int] = []
    outer_indices: list[int] = []
    control_values: list[list[NDArray[np.float64]]] = [[] for _ in range(6)]
    outer_values: list[list[NDArray[np.float64]]] = [[], []]
    thrusts: list[float] = []
    scales: list[float] = []
    inner_flags: list[list[bool]] = []
    outer_flags: list[list[bool]] = []
    derivative_memory = (
        None
        if geometric_controller is None
        else FeedbackDerivativeFilter(
            numerics.time_step_s * numerics.position_stride, geometric_controller.filter_pole_rad_s
        )
    )
    geometric_reference: GeometricPositionReference | None = None
    supervisor = MissionState()
    command = np.zeros(4)
    try:
        for k, time in enumerate(times):
            reference, _ = mission_reference(plan, float(time))
            values = (
                *state,
                actual,
                _wrench(actual, rotors)[1],
                reference.position_W,
                reference.velocity_W,
                reference.acceleration_W,
            )
            for target, value in zip(history, values, strict=True):
                target[k] = value
            feedback = state if observer is None else observer(k, float(time), state, actual)
            reason = mission_guard_reason(state[0], state[2], safety)
            if observer is not None:
                reason = f"truth_{reason}" if reason is not None else None
                estimated_reason = mission_guard_reason(feedback[0], feedback[2], safety)
                if reason is None and estimated_reason is not None:
                    reason = f"estimate_{estimated_reason}"
            supervisor = advance_mission(
                supervisor, plan, float(time), feedback[0], feedback[1], reason
            )
            phases[k] = supervisor.phase
            if supervisor.phase in (MissionPhase.COMPLETE, MissionPhase.ABORT):
                break
            if k == n:
                raise RuntimeError("mission timeout was not reached on the plant grid")
            try:
                if k % numerics.position_stride == 0:
                    if geometric_controller is None:
                        held = compute_position_control(feedback[0], feedback[1], reference, outer)
                    else:
                        assert derivative_memory is not None
                        if consume_estimator_force_jump is not None:
                            derivative_memory = derivative_memory.rebase(
                                consume_estimator_force_jump()
                            )
                        jerk, snap = reference_jerk_snap(plan, float(time))
                        geometric_reference, derivative_memory = build_geometric_reference(
                            feedback[0],
                            feedback[1],
                            reference,
                            jerk,
                            snap,
                            outer,
                            derivative_memory,
                            k // numerics.position_stride,
                        )
                        held = geometric_reference.position_command
                if k % numerics.attitude_stride == 0 and geometric_controller is not None:
                    assert geometric_reference is not None
                    projected = projected_collective_thrust(geometric_reference, feedback[2], outer)
                    output = compute_geometric_control(
                        feedback[2],
                        feedback[3],
                        geometric_reference.rotation,
                        projected,
                        inner,
                        geometric_controller,
                    )
            except GeometricDomainError as error:
                supervisor = MissionState(MissionPhase.ABORT, float(time), reason=str(error))
                phases[k] = supervisor.phase
                break
            if (
                hypot(*attitude_error_body(feedback[2], held.q_reference_WB))
                > inner.maximum_attitude_error_rad
            ):
                supervisor = MissionState(MissionPhase.ABORT, float(time), reason="attitude_domain")
                phases[k] = supervisor.phase
                break
            if k % numerics.position_stride == 0:
                outer_indices.append(k)
                outer_values[0].append(held.requested_acceleration_W)
                outer_values[1].append(held.feasible_acceleration_W)
                outer_flags.append(
                    [held.acceleration_limited, held.tilt_limited, held.thrust_limited]
                )
            if k % numerics.attitude_stride == 0:
                if geometric_controller is None:
                    output = compute_attitude_control(
                        feedback[2], feedback[3], held.q_reference_WB, held.collective_thrust, inner
                    )
                command = output.allocation.commanded_rotor_omega
                control_indices.append(k)
                for control_rows, value in zip(
                    control_values,
                    (
                        held.q_reference_WB,
                        command,
                        output.moment_requested_B,
                        output.moment_limited_B,
                        output.allocation.allocated_moment_B,
                        output.desired_omega_B,
                    ),
                    strict=True,
                ):
                    control_rows.append(value)
                thrusts.append(
                    held.collective_thrust if geometric_controller is None else projected
                )
                scales.append(output.allocation.moment_scale)
                inner_flags.append(
                    [output.rate_limited, output.moment_limited, output.allocation.moment_scale < 1]
                )
            state, actual = _plant_step(
                state, actual, command, np.zeros(3), body, rotors, world, dt
            )
    except (FloatingPointError, np.linalg.LinAlgError):
        raise ValueError("mission plant arithmetic must remain finite and solvable") from None
    count = k + 1

    def matrix(rows: list[NDArray[np.float64]], width: int) -> NDArray[np.float64]:
        return np.asarray(rows, dtype=np.float64).reshape((-1, width))

    return MissionResult(
        times[:count],
        history[0][:count],
        history[1][:count],
        history[2][:count],
        history[3][:count],
        history[4][:count],
        history[5][:count],
        history[6][:count],
        history[7][:count],
        history[8][:count],
        phases[:count],
        times[control_indices],
        matrix(control_values[0], 4),
        np.asarray(thrusts),
        matrix(control_values[1], 4),
        matrix(control_values[2], 3),
        matrix(control_values[3], 3),
        matrix(control_values[4], 3),
        matrix(control_values[5], 3),
        np.asarray(scales),
        np.asarray(inner_flags, dtype=np.bool_).reshape((-1, 3)),
        times[outer_indices],
        matrix(outer_values[0], 3),
        matrix(outer_values[1], 3),
        np.asarray(outer_flags, dtype=np.bool_).reshape((-1, 3)),
        supervisor.reason,
    )
