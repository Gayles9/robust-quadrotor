"""Deterministic true-state inner-loop execution, separate from sensor replay.

The existing force/drag/dynamics and exact motor response are evaluated at each
projected RK4 stage. No position controller, ground contact, ESKF, or RNG is used.
"""

from dataclasses import dataclass, replace
from typing import cast

import numpy as np
from numpy.typing import NDArray

from .actuation import force_and_moment_body_from_rotor_speeds, motor_speed_first_order_step
from .attitude_control import (
    AttitudeControllerParameters,
    _array,
    _inertia,
    _quaternion,
    _rotors,
    _scalar,
    compute_attitude_control,
)
from .dynamics import quadratic_drag_force_body, rigid_body_state_derivative_from_body_wrench
from .rotations import normalize_quaternion_body_to_world, rotation_matrix_body_to_world
from .run_configuration import (
    RigidBodyInitialState,
    RigidBodyParameters,
    RotorParameters,
    WorldParameters,
)

State = tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]


@dataclass(frozen=True, slots=True, eq=False)
class AttitudeControlSchedule:
    """Owned fixed-grid input: m=ceil(N/stride) control rows, N>=1 plant rows.

    reference_q_WB (m,4) and collective_thrust (m,) apply from control tick
    j*stride through the next tick, without an extra final update. Disturbance
    moment (N,3), FRD N m, is held over each plant interval. Time starts at zero.
    """

    time_step_s: float
    controller_stride: int
    reference_q_WB: NDArray[np.float64]
    collective_thrust: NDArray[np.float64]
    disturbance_moment_B: NDArray[np.float64]

    def __post_init__(self) -> None:
        dt = _scalar("time_step_s", self.time_step_s, positive=True)
        stride = self.controller_stride
        if (
            isinstance(stride, (bool, np.bool_))
            or not isinstance(stride, (int, np.integer))
            or stride < 1
        ):
            raise ValueError("controller_stride must be a positive integer")
        disturbance = self.disturbance_moment_B
        if not isinstance(disturbance, np.ndarray) or disturbance.ndim != 2 or len(disturbance) < 1:
            raise ValueError("disturbance_moment_B must have N>=1 rows")
        n = len(disturbance)
        m = (n + int(stride) - 1) // int(stride)
        reference = _array("reference_q_WB", self.reference_q_WB, (m, 4))
        for row in reference:
            _quaternion("reference_q_WB", row)
        collective = _array("collective_thrust", self.collective_thrust, (m,))
        if np.any(collective < 0):
            raise ValueError("collective_thrust must be nonnegative")
        with np.errstate(over="ignore", invalid="ignore"):
            times = np.arange(n + 1, dtype=np.float64) * dt
        if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
            raise ValueError("plant clock must remain finite and distinguishable")
        object.__setattr__(self, "time_step_s", dt)
        object.__setattr__(self, "controller_stride", int(stride))
        object.__setattr__(self, "reference_q_WB", reference)
        object.__setattr__(self, "collective_thrust", collective)
        object.__setattr__(
            self, "disturbance_moment_B", _array("disturbance_moment_B", disturbance, (n, 3))
        )


@dataclass(frozen=True, slots=True, eq=False)
class AttitudeSimulationResult:
    """Owned histories; state rows include origin and final endpoint.

    State vectors use SI NED/FRD units. actual_moment_B is rotor moment only,
    before external disturbance, at state epochs. Per-control-row diagnostics
    hold over the following intervals. limit_flags columns are rate, moment
    clipping and allocation scaling; they do not indicate actual motor bounds.
    reference_q_WB is expanded at state epochs (new reference at a control tick).
    This in-memory result is not an extension to the run-artifact schema.
    """

    time_s: NDArray[np.float64]
    position_W: NDArray[np.float64]
    velocity_W: NDArray[np.float64]
    q_WB: NDArray[np.float64]
    omega_B: NDArray[np.float64]
    actual_rotor_omega: NDArray[np.float64]
    actual_moment_B: NDArray[np.float64]
    reference_q_WB: NDArray[np.float64]
    control_time_s: NDArray[np.float64]
    commanded_rotor_omega: NDArray[np.float64]
    desired_omega_B: NDArray[np.float64]
    moment_requested_B: NDArray[np.float64]
    moment_limited_B: NDArray[np.float64]
    allocated_moment_B: NDArray[np.float64]
    moment_scale: NDArray[np.float64]
    limit_flags: NDArray[np.bool_]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.time_s, np.ndarray)
            or not isinstance(self.control_time_s, np.ndarray)
            or self.time_s.ndim != 1
            or len(self.time_s) < 2
            or self.control_time_s.ndim != 1
        ):
            raise ValueError("result clocks must be nonempty vectors with at least two state rows")
        n, m = len(self.time_s), len(self.control_time_s)
        if m < 1:
            raise ValueError("result requires a control row")
        shapes = {
            "time_s": (n,),
            "position_W": (n, 3),
            "velocity_W": (n, 3),
            "q_WB": (n, 4),
            "omega_B": (n, 3),
            "actual_rotor_omega": (n, 4),
            "actual_moment_B": (n, 3),
            "reference_q_WB": (n, 4),
            "control_time_s": (m,),
            "commanded_rotor_omega": (m, 4),
            "desired_omega_B": (m, 3),
            "moment_requested_B": (m, 3),
            "moment_limited_B": (m, 3),
            "allocated_moment_B": (m, 3),
            "moment_scale": (m,),
        }
        for name, shape in shapes.items():
            object.__setattr__(self, name, _array(name, getattr(self, name), shape))
        if (
            self.time_s[0] != 0
            or self.control_time_s[0] != 0
            or np.any(np.diff(self.time_s) <= 0)
            or np.any(np.diff(self.control_time_s) <= 0)
            or not np.all(np.isin(self.control_time_s, self.time_s[:-1]))
        ):
            raise ValueError("result clocks must be increasing with control epochs in plant clock")
        for history in (self.q_WB, self.reference_q_WB):
            for row in history:
                _quaternion("q_WB history", row)
        if (
            np.any(self.actual_rotor_omega < 0)
            or np.any(self.commanded_rotor_omega < 0)
            or np.any((self.moment_scale < 0) | (self.moment_scale > 1))
        ):
            raise ValueError("result speeds/scale outside domain")
        if (
            not isinstance(self.limit_flags, np.ndarray)
            or self.limit_flags.shape != (m, 3)
            or self.limit_flags.dtype != np.bool_
        ):
            raise ValueError("limit_flags must be bool (m,3)")
        flags = np.array(self.limit_flags, copy=True, order="C")
        flags.flags.writeable = False
        object.__setattr__(self, "limit_flags", flags)


def _wrench(
    rotor_omega: NDArray[np.float64], rotors: RotorParameters
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    return force_and_moment_body_from_rotor_speeds(
        rotor_omega,
        rotors.rotor_positions_B,
        rotors.rotor_spin_directions,
        rotors.thrust_coefficient,
        rotors.moment_coefficient,
    )


def _motor(
    actual: NDArray[np.float64], command: NDArray[np.float64], rotors: RotorParameters, dt: float
) -> NDArray[np.float64]:
    assert rotors.minimum_rotor_omega is not None and rotors.maximum_rotor_omega is not None
    assert rotors.motor_time_constant_s is not None
    return motor_speed_first_order_step(
        actual,
        command,
        rotors.minimum_rotor_omega,
        rotors.maximum_rotor_omega,
        rotors.motor_time_constant_s,
        dt,
    )


def _plant_step(
    state: State,
    actual: NDArray[np.float64],
    command: NDArray[np.float64],
    disturbance: NDArray[np.float64],
    body: RigidBodyParameters,
    rotors: RotorParameters,
    world: WorldParameters,
    dt: float,
) -> tuple[State, NDArray[np.float64]]:
    """Projected RK4 of time-varying motor forcing under one held command."""

    def derivative(stage: State, elapsed: float) -> State:
        speeds = _motor(actual, command, rotors, elapsed)
        force, moment = _wrench(speeds, rotors)
        attitude = rotation_matrix_body_to_world(stage[2])
        force = force + quadratic_drag_force_body(
            stage[1], attitude, world.wind_velocity_W, body.quadratic_drag_coefficient_B
        )
        return rigid_body_state_derivative_from_body_wrench(
            *stage,
            force,
            moment + disturbance,
            body.mass,
            body.inertia_B,
            world.gravity_acceleration,
        )

    def advance(slope: State, h: float) -> State:
        values = [s + h * d for s, d in zip(state, slope, strict=True)]
        values[2] = normalize_quaternion_body_to_world(values[2])
        return cast(State, tuple(values))

    with np.errstate(over="raise", invalid="raise", divide="raise"):
        k1 = derivative(state, 0.0)
        k2 = derivative(advance(k1, dt / 2), dt / 2)
        k3 = derivative(advance(k2, dt / 2), dt / 2)
        k4 = derivative(advance(k3, dt), dt)
        weighted = cast(
            State,
            tuple((a + 2 * b + 2 * c + d) / 6 for a, b, c, d in zip(k1, k2, k3, k4, strict=True)),
        )
        result = advance(weighted, dt)
        if not all(np.all(np.isfinite(v)) for v in result):
            raise FloatingPointError
        return result, _motor(actual, command, rotors, dt)


def simulate_attitude_control(
    initial_state: RigidBodyInitialState,
    initial_actual_rotor_omega: NDArray[np.float64],
    truth_body: RigidBodyParameters,
    truth_rotors: RotorParameters,
    truth_world: WorldParameters,
    controller: AttitudeControllerParameters,
    schedule: AttitudeControlSchedule,
) -> AttitudeSimulationResult:
    """Run deterministic true-state attitude feedback; no sensor/estimator access.

    Truth is supplied only to this simulation; the pure controller receives q_WB,
    omega_B, the reference, collective and its own nominal assumptions. Nominal
    rotor commands must fit truth speed limits too; incompatible limits reject.
    All input arrays are copied. Any failure returns no partial result.
    """
    if not isinstance(initial_state, RigidBodyInitialState):
        raise TypeError("initial_state must be RigidBodyInitialState")
    if not isinstance(truth_body, RigidBodyParameters) or not isinstance(
        truth_world, WorldParameters
    ):
        raise TypeError("truth body/world must use their parameter dataclasses")
    if not isinstance(controller, AttitudeControllerParameters) or not isinstance(
        schedule, AttitudeControlSchedule
    ):
        raise TypeError("controller and schedule must use their dataclasses")
    controller, schedule = replace(controller), replace(schedule)
    body = RigidBodyParameters(
        _scalar("mass", truth_body.mass, positive=True),
        _inertia(truth_body.inertia_B),
        _array("quadratic_drag_coefficient_B", truth_body.quadratic_drag_coefficient_B, (3,)),
    )
    world = WorldParameters(
        _scalar("gravity_acceleration", truth_world.gravity_acceleration, positive=True),
        _array("wind_velocity_W", truth_world.wind_velocity_W, (3,)),
    )
    rotors = _rotors(truth_rotors)
    nominal = controller.nominal_rotors
    assert rotors.minimum_rotor_omega is not None and rotors.maximum_rotor_omega is not None
    assert nominal.minimum_rotor_omega is not None and nominal.maximum_rotor_omega is not None
    if (
        nominal.minimum_rotor_omega < rotors.minimum_rotor_omega
        or nominal.maximum_rotor_omega > rotors.maximum_rotor_omega
    ):
        raise ValueError("nominal rotor command limits must fit truth motor limits")
    state: State = (
        _array("position_W", initial_state.position_W, (3,)),
        _array("velocity_W", initial_state.velocity_W, (3,)),
        _quaternion("q_WB", initial_state.q_WB),
        _array("omega_B", initial_state.omega_B, (3,)),
    )
    actual = _array("initial_actual_rotor_omega", initial_actual_rotor_omega, (4,))
    _motor(actual, actual, rotors, 0.0)
    n = len(schedule.disturbance_moment_B)
    stride, dt = schedule.controller_stride, schedule.time_step_s
    m = len(schedule.collective_thrust)
    times = np.arange(n + 1, dtype=np.float64) * dt
    histories = [np.empty((n + 1, size)) for size in (3, 3, 4, 3)]
    actual_history, actual_moments = np.empty((n + 1, 4)), np.empty((n + 1, 3))
    commands, desired = np.empty((m, 4)), np.empty((m, 3))
    requested, limited, allocated = (np.empty((m, 3)) for _ in range(3))
    scales, flags = np.empty(m), np.empty((m, 3), dtype=np.bool_)
    try:
        for k in range(n + 1):
            for array, value in zip(histories, state, strict=True):
                array[k] = value
            actual_history[k] = actual
            actual_moments[k] = _wrench(actual, rotors)[1]
            if k == n:
                break
            j = k // stride
            if k % stride == 0:
                output = compute_attitude_control(
                    state[2],
                    state[3],
                    schedule.reference_q_WB[j],
                    float(schedule.collective_thrust[j]),
                    controller,
                )
                commands[j], desired[j] = (
                    output.allocation.commanded_rotor_omega,
                    output.desired_omega_B,
                )
                requested[j], limited[j] = output.moment_requested_B, output.moment_limited_B
                allocated[j], scales[j] = (
                    output.allocation.allocated_moment_B,
                    output.allocation.moment_scale,
                )
                flags[j] = [output.rate_limited, output.moment_limited, scales[j] < 1]
            state, actual = _plant_step(
                state,
                actual,
                commands[j],
                schedule.disturbance_moment_B[k],
                body,
                rotors,
                world,
                dt,
            )
    except (FloatingPointError, np.linalg.LinAlgError):
        raise ValueError("closed-loop plant arithmetic must remain finite and solvable") from None
    references = schedule.reference_q_WB[np.minimum(np.arange(n + 1) // stride, m - 1)]
    return AttitudeSimulationResult(
        times,
        histories[0],
        histories[1],
        histories[2],
        histories[3],
        actual_history,
        actual_moments,
        references,
        times[:-1:stride],
        commands,
        desired,
        requested,
        limited,
        allocated,
        scales,
        flags,
    )
