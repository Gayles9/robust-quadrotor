"""Timing, plant composition and physical recovery tests for attitude feedback."""

from dataclasses import fields, replace

import numpy as np
import pytest
from test_attitude_control import parameters, quaternion, rotors

from quadrotor_math.attitude_control import attitude_error_body
from quadrotor_math.attitude_simulation import AttitudeControlSchedule, simulate_attitude_control
from quadrotor_math.integration import rigid_body_state_rk4_step_from_rotor_speeds
from quadrotor_math.run_configuration import (
    RigidBodyInitialState,
    RigidBodyParameters,
    WorldParameters,
)


def schedule(duration=0.2, dt=0.0025, stride=4, reference=None):
    n = round(duration / dt)
    m = (n + stride - 1) // stride
    if reference is None:
        reference = np.tile([1.0, 0, 0, 0], (m, 1))
    return AttitudeControlSchedule(dt, stride, reference, np.full(m, 9.81), np.zeros((n, 3)))


def initial(q_WB=None, omega_B=None):
    return RigidBodyInitialState(
        np.zeros(3),
        np.zeros(3),
        np.array([1.0, 0, 0, 0]) if q_WB is None else q_WB,
        np.zeros(3) if omega_B is None else omega_B,
    )


def simulate(plan=None, state=None, controller=None, actual=None, body=None):
    return simulate_attitude_control(
        initial() if state is None else state,
        np.full(4, np.sqrt(9.81 / (4e-5))) if actual is None else actual,
        RigidBodyParameters(1.0, np.diag([0.02, 0.025, 0.04])) if body is None else body,
        rotors(),
        WorldParameters(9.81),
        parameters() if controller is None else controller,
        schedule() if plan is None else plan,
    )


def test_hover_equilibrium_alignment_and_independent_storage():
    result = simulate()
    assert result.time_s.shape == (81,)
    np.testing.assert_array_equal(result.control_time_s, result.time_s[:-1:4])
    np.testing.assert_allclose(result.position_W, 0, atol=2e-15)
    np.testing.assert_allclose(result.velocity_W, 0, atol=2e-14)
    np.testing.assert_allclose(result.omega_B, 0, atol=1e-14)
    np.testing.assert_allclose(result.q_WB, np.tile([1, 0, 0, 0], (81, 1)), atol=1e-14)
    for field in fields(result):
        value = getattr(result, field.name)
        assert isinstance(value, np.ndarray)
        assert not value.flags.writeable and value.flags.c_contiguous
    assert not np.shares_memory(result.actual_rotor_omega, result.commanded_rotor_omega)


def test_one_interval_matches_existing_constant_speed_rk4():
    state = initial(omega_B=np.array([0.1, -0.2, 0.05]))
    plan = schedule(0.0025)
    model = replace(
        parameters(),
        attitude_gain_B=np.full(3, 1e-20),
        rate_gain_B=np.full(3, 1e-20),
        maximum_moment_B=np.full(3, 1e-20),
    )
    # Spherical truth and nominal inertia remove the gyroscopic torque too.
    body = RigidBodyParameters(1.0, np.eye(3) * 0.02)
    model = replace(model, nominal_inertia_B=body.inertia_B)
    result = simulate(plan, state, model, body=body)
    speed = np.full(4, np.sqrt(9.81 / 4e-5))
    expected = rigid_body_state_rk4_step_from_rotor_speeds(
        state.position_W,
        state.velocity_W,
        state.q_WB,
        state.omega_B,
        speed,
        rotors().rotor_positions_B,
        rotors().rotor_spin_directions,
        1.0,
        body.inertia_B,
        9.81,
        1e-5,
        2e-7,
        0.0025,
    )
    for name, value in zip(("position_W", "velocity_W", "q_WB", "omega_B"), expected, strict=True):
        np.testing.assert_allclose(getattr(result, name)[-1], value, atol=1e-16)
    assert result.control_time_s.size == 1


def test_future_reference_cannot_change_past_and_command_hold():
    original = schedule(0.12)
    target = original.reference_q_WB.copy()
    target[5:] = quaternion([1, 0, 0], 0.2)
    modified = replace(original, reference_q_WB=target)
    first, second = simulate(original), simulate(modified)
    np.testing.assert_array_equal(first.q_WB[:21], second.q_WB[:21])
    np.testing.assert_array_equal(first.commanded_rotor_omega[:5], second.commanded_rotor_omega[:5])
    # Only one command is computed per four plant intervals; motors follow a hold.
    k = 20
    target_speed = second.commanded_rotor_omega[5]
    expected = target_speed + (second.actual_rotor_omega[k] - target_speed) * np.exp(-0.01 / 0.025)
    np.testing.assert_allclose(second.actual_rotor_omega[k + 4], expected, atol=3e-13)


@pytest.mark.parametrize("axis", np.eye(3))
@pytest.mark.parametrize("angle", [-0.5, 0.5])
def test_principal_axis_recovery(axis, angle):
    result = simulate(schedule(3.0), initial(quaternion(axis, angle)))
    assert np.linalg.norm(attitude_error_body(result.q_WB[-1], np.array([1.0, 0, 0, 0]))) < 0.003
    assert np.linalg.norm(result.omega_B[-1]) < 0.01
    assert not np.any(result.limit_flags[-100:])


def test_torque_pulse_recovery_and_sign():
    plan = schedule(3.0)
    torque = plan.disturbance_moment_B.copy()
    torque[200:280] = [0.12, -0.1, 0.06]
    result = simulate(replace(plan, disturbance_moment_B=torque))
    assert result.omega_B[201, 0] > 0
    assert result.omega_B[201, 1] < 0
    assert np.linalg.norm(attitude_error_body(result.q_WB[-1], np.array([1.0, 0, 0, 0]))) < 0.003
    assert np.linalg.norm(result.omega_B[-1]) < 0.01


@pytest.mark.parametrize(
    "field,bad",
    [
        ("time_step_s", 0),
        ("time_step_s", True),
        ("controller_stride", 0),
        ("controller_stride", 1.5),
        ("controller_stride", True),
        ("reference_q_WB", np.ones((20, 4))),
        ("collective_thrust", np.full(20, -1.0)),
        ("disturbance_moment_B", np.ones((80, 2))),
    ],
)
def test_invalid_schedule(field, bad):
    with pytest.raises((ValueError, TypeError)):
        replace(schedule(), **{field: bad})


def test_failure_does_not_modify_inputs():
    plan = schedule()
    before = plan.reference_q_WB.tobytes()
    with pytest.raises(ValueError):
        simulate(plan, initial(quaternion([1, 0, 0], 2.0)))
    assert plan.reference_q_WB.tobytes() == before


def test_time_varying_motor_forcing_matches_analytic_vertical_motion():
    # Equal motors, level attitude: integrate (command*(1-exp(-t/tau)))**2.
    duration, tau = 0.04, 0.025
    decay = np.exp(-duration / tau)
    integral = duration - 2 * tau * (1 - decay) + tau / 2 * (1 - decay**2)
    double_integral = (
        duration**2 / 2
        - 2 * (tau * duration - tau**2 * (1 - decay))
        + tau * duration / 2
        - tau**2 / 4 * (1 - decay**2)
    )
    expected_v = 9.81 * (duration - integral)
    expected_p = 9.81 * (duration**2 / 2 - double_integral)
    errors = []
    for dt in [0.01, 0.005, 0.0025]:
        result = simulate(schedule(duration, dt, round(duration / dt)), actual=np.zeros(4))
        errors.append(
            np.linalg.norm(
                [result.velocity_W[-1, 2] - expected_v, result.position_W[-1, 2] - expected_p]
            )
        )
        np.testing.assert_allclose(
            result.actual_rotor_omega[-1], np.sqrt(9.81 / 4e-5) * (1 - decay), atol=3e-13
        )
        np.testing.assert_allclose(result.omega_B, 0, atol=1e-14)
    assert errors[0] / errors[1] > 14
    assert errors[1] / errors[2] > 14
    assert errors[-1] < 3e-7


def test_partial_control_interval_and_stride_larger_than_run():
    for stride, ticks in [(4, [0.0, 0.01, 0.02]), (20, [0.0])]:
        result = simulate(schedule(0.0225, stride=stride))
        np.testing.assert_array_equal(result.control_time_s, ticks)
        assert len(result.q_WB) == 10
        assert result.time_s[-1] == 0.0225


def test_truth_nominal_boundary_and_no_implicit_parameter_copy():
    state = initial(quaternion([1, 0, 0], 0.3))
    nominal = parameters()
    first = simulate(schedule(0.02), state, nominal)
    second = simulate(
        schedule(0.02), state, nominal, body=RigidBodyParameters(1.0, np.diag([0.03, 0.04, 0.05]))
    )
    np.testing.assert_array_equal(first.commanded_rotor_omega[0], second.commanded_rotor_omega[0])
    assert not np.array_equal(first.omega_B[-1], second.omega_B[-1])
    changed = simulate(
        schedule(0.02), state, replace(nominal, nominal_inertia_B=np.diag([0.03, 0.04, 0.05]))
    )
    assert not np.array_equal(first.commanded_rotor_omega[0], changed.commanded_rotor_omega[0])


def test_nonzero_drag_evaluated_in_correct_frame():
    state = replace(initial(), velocity_W=np.array([1.0, 0, 0]))
    body = RigidBodyParameters(1.0, np.diag([0.02, 0.025, 0.04]), np.array([0.3, 0.2, 0.1]))
    result = simulate(schedule(0.0025), state, body=body)
    assert 0 < result.velocity_W[-1, 0] < 1
    np.testing.assert_allclose(result.velocity_W[-1, 1:], 0, atol=1e-14)


@pytest.mark.parametrize(
    "field,bad",
    [
        ("time_s", [0, 1]),
        ("time_s", np.arange(81, dtype=float) + 1),
        ("control_time_s", np.array([0.0, 0.001])),
        ("q_WB", np.zeros((81, 4))),
        ("omega_B", np.full((81, 3), np.nan)),
        ("moment_scale", np.full(20, 1.01)),
        ("limit_flags", np.zeros((20, 3))),
        ("actual_rotor_omega", -np.ones((81, 4))),
    ],
)
def test_result_validation(field, bad):
    with pytest.raises(ValueError):
        replace(simulate(), **{field: bad})


@pytest.mark.parametrize("actual", [np.full(4, -1.0), np.full(4, 901.0), np.full(4, np.inf)])
def test_initial_motor_bounds(actual):
    with pytest.raises(ValueError):
        simulate(actual=actual)
