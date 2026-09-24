"""Clock causality, composition, abort behavior and independent-memory contracts."""

from dataclasses import fields, replace

import numpy as np
import pytest

from experiments.attitude_control_validation import axis_quaternion
from experiments.position_control_validation import baseline_mission, make_case
from quadrotor_math.mission_simulation import MissionNumerics, simulate_mission
from quadrotor_math.missions import MissionPhase as P


def simulate(**changes):
    initial, body, world, outer, inner, plan, safety, numerics = make_case(
        {"case": "smoke", "seed": None, "refinement": 1}
    )
    kwargs = dict(
        initial_state=initial,
        initial_actual_rotor_omega=np.full(4, np.sqrt(9.81 / 4e-5)),
        truth_body=body,
        truth_rotors=inner.nominal_rotors,
        truth_world=world,
        position_controller=outer,
        attitude_controller=inner,
        plan=plan,
        safety=safety,
        numerics=numerics,
    )
    kwargs.update(changes)
    return simulate_mission(**kwargs)


def test_equilibrium_and_multirate_clocks_no_terminal_command():
    result = simulate()
    assert result.phase[-1] == P.COMPLETE and result.abort_reason is None
    assert set(result.phase) == {0, 1, 2, 3, 4}
    np.testing.assert_array_equal(result.position_W, 0)
    np.testing.assert_array_equal(result.velocity_W, 0)
    np.testing.assert_array_equal(result.q_WB, np.tile([1.0, 0, 0, 0], (len(result.time_s), 1)))
    np.testing.assert_array_equal(result.control_time_s, result.time_s[:-1:4])
    np.testing.assert_array_equal(result.position_control_time_s, result.time_s[:-1:8])
    assert result.control_time_s[-1] < result.time_s[-1]
    for field in fields(result):
        value = getattr(result, field.name)
        if isinstance(value, np.ndarray):
            assert value.flags.owndata and value.flags.c_contiguous and not value.flags.writeable
    assert not np.shares_memory(result.position_W, result.reference_position_W)


@pytest.mark.parametrize("reason", ["geofence", "tilt", "attitude_domain"])
def test_initial_guard_abort_records_no_command(reason, monkeypatch):
    initial = make_case({"case": "smoke", "seed": None, "refinement": 1})[0]
    if reason == "geofence":
        initial = replace(initial, position_W=np.array([4.0, 0, 0]))
    else:
        initial = replace(
            initial,
            q_WB=axis_quaternion(
                np.array([1.0, 0, 0]) if reason == "tilt" else np.array([0.0, 0, 1]), 2.0
            ),
        )

    def forbidden(*args, **kwargs):
        raise AssertionError("must stop before commanding or advancing")

    monkeypatch.setattr("quadrotor_math.mission_simulation._plant_step", forbidden)
    monkeypatch.setattr("quadrotor_math.mission_simulation.compute_attitude_control", forbidden)
    result = simulate(initial_state=initial)
    assert result.abort_reason == reason
    np.testing.assert_array_equal(result.phase, [P.ABORT])
    assert result.commanded_rotor_omega.shape == (0, 4)
    assert result.position_control_time_s.size == 0


def test_guard_between_control_ticks_stops_at_detected_state(monkeypatch):
    from quadrotor_math.attitude_simulation import _plant_step

    def crossing(*args):
        state, actual = _plant_step(*args)
        return (np.array([4.0, 0, 0]), *state[1:]), actual

    monkeypatch.setattr("quadrotor_math.mission_simulation._plant_step", crossing)
    result = simulate()
    assert result.abort_reason == "geofence" and result.time_s[-1] == 0.0025
    assert result.control_time_s.tolist() == [0.0]
    assert result.position_W[-1, 0] == 4


def test_future_reference_cannot_change_past_and_outer_hold():
    plan = baseline_mission("smoke")
    segments = list(plan.segments)
    from quadrotor_math.missions import ReferenceKind

    segments[2] = replace(
        segments[2], end_position_W=np.array([0.001, 0, 0]), kind=ReferenceKind.STEP
    )
    segments[3] = replace(
        segments[3], start_position_W=np.array([0.001, 0, 0]), kind=ReferenceKind.SMOOTH
    )
    first, second = simulate(), simulate(plan=replace(plan, segments=tuple(segments)))
    np.testing.assert_array_equal(first.position_W[:9], second.position_W[:9])
    np.testing.assert_array_equal(first.commanded_rotor_omega[:2], second.commanded_rotor_omega[:2])
    np.testing.assert_array_equal(second.q_reference_WB[2], second.q_reference_WB[3])
    expected = second.commanded_rotor_omega[2] + (
        second.actual_rotor_omega[8] - second.commanded_rotor_omega[2]
    ) * np.exp(-0.01 / 0.025)
    np.testing.assert_allclose(second.actual_rotor_omega[12], expected, atol=3e-13)


def test_truth_nominal_independence_and_motor_lag():
    initial, body, _, outer, _, _, _, _ = make_case(
        {"case": "smoke", "seed": None, "refinement": 1}
    )
    initial = replace(initial, position_W=np.array([0.01, 0, 0.01]))
    first = simulate(initial_state=initial)
    second = simulate(initial_state=initial, truth_body=replace(body, mass=1.01))
    np.testing.assert_array_equal(first.commanded_rotor_omega[0], second.commanded_rotor_omega[0])
    assert not np.array_equal(first.velocity_W[1], second.velocity_W[1])
    third = simulate(initial_state=initial, position_controller=replace(outer, nominal_mass=1.01))
    assert not np.array_equal(first.commanded_rotor_omega[0], third.commanded_rotor_omega[0])
    assert not np.array_equal(first.actual_rotor_omega[1], first.commanded_rotor_omega[0])


@pytest.mark.parametrize(
    "changes",
    [
        {"time_step_s": 0},
        {"time_step_s": True},
        {"attitude_stride": 0},
        {"position_stride": 3},
        {"position_stride": True},
        {"maximum_steps": 0},
        {"maximum_steps": 2.5},
    ],
)
def test_invalid_numerics(changes):
    with pytest.raises(ValueError):
        replace(MissionNumerics(0.0025, 4, 8), **changes)


def test_budget_and_infeasible_nominal_limits_fail_before_execution():
    inputs = make_case({"case": "smoke", "seed": None, "refinement": 1})
    with pytest.raises(ValueError, match="maximum_steps"):
        simulate(numerics=MissionNumerics(0.0025, 4, 8, 3))
    with pytest.raises(ValueError, match="infeasible"):
        simulate(position_controller=replace(inputs[3], maximum_collective_thrust=40.0))
    with pytest.raises(ValueError, match="bounds"):
        simulate(truth_rotors=replace(inputs[4].nominal_rotors, maximum_rotor_omega=899.0))
    with pytest.raises(ValueError, match="tilt"):
        simulate(safety=replace(inputs[6], maximum_tilt_rad=np.deg2rad(10)))


@pytest.mark.parametrize(
    "name", ["position_W", "velocity_W", "omega_B", "q_WB", "collective_thrust", "moment_scale"]
)
def test_result_rejects_nonfinite_fields(name):
    result = simulate()
    with pytest.raises(ValueError):
        replace(result, **{name: np.full_like(getattr(result, name), np.nan)})


def test_result_rejects_bad_clocks_terminal_or_flags():
    result = simulate()
    for change in (
        {"phase": np.zeros_like(result.phase)},
        {"phase": result.phase.astype(float)},
        {"abort_reason": "bad"},
        {"inner_limit_flags": result.inner_limit_flags.astype(float)},
        {"control_time_s": result.control_time_s + 0.001},
        {"moment_scale": np.full_like(result.moment_scale, 2)},
    ):
        with pytest.raises(ValueError):
            replace(result, **change)


def test_timeout_retains_failed_mission_not_false_completion():
    inputs = make_case({"case": "smoke", "seed": None, "refinement": 1})
    state = replace(inputs[0], position_W=np.array([0.2, 0, 0]))
    result = simulate(initial_state=state)
    assert result.phase[-1] == P.ABORT and result.abort_reason == "landing_timeout"
    assert result.time_s[-1] == 0.08


def test_determinism_and_failure_input_ownership():
    first, second = simulate(), simulate()
    for field in fields(first):
        np.testing.assert_array_equal(getattr(first, field.name), getattr(second, field.name))
    motors = np.full(4, 901.0)
    before = motors.copy()
    with pytest.raises(ValueError):
        simulate(initial_actual_rotor_omega=motors)
    np.testing.assert_array_equal(motors, before)


@pytest.mark.parametrize("seed", [315, 316])
def test_seeded_short_flight_regression_with_broad_physical_bounds(seed):
    from quadrotor_math.missions import MissionPlan, MissionSegment
    from quadrotor_math.missions import ReferenceKind as K

    initial = make_case({"case": "smoke", "seed": seed, "refinement": 1})[0]
    points = [np.zeros(3), np.array([0.0, 0, -0.2]), np.array([0.3, 0.2, -0.2])]
    plan = MissionPlan(
        (
            MissionSegment(P.INITIALIZE, 0.5, points[0], points[0], K.HOLD),
            MissionSegment(P.TAKEOFF, 2.0, points[0], points[1], K.SMOOTH),
            MissionSegment(P.TRACK, 3.0, points[1], points[2], K.SMOOTH),
            MissionSegment(P.TRACK, 1.0, points[2], points[2], K.HOLD),
            MissionSegment(P.LAND, 3.0, points[2], points[0], K.SMOOTH),
        ),
        0.0,
        0.08,
        0.08,
        0.5,
        4.0,
    )
    result = simulate(initial_state=initial, plan=plan)
    error = np.linalg.norm(result.position_W - result.reference_position_W, axis=1)
    rmse = np.sqrt(np.trapezoid(error**2, result.time_s) / result.time_s[-1])
    assert result.phase[-1] == P.COMPLETE and result.abort_reason is None
    assert rmse < 0.08 and error[-1] < 0.05
    assert np.linalg.norm(result.velocity_W[-1]) < 0.08
    assert not np.any(result.inner_limit_flags)


def test_outer_saturation_recovers_without_hidden_integral():
    inputs = make_case({"case": "smoke", "seed": None, "refinement": 1})
    initial = replace(inputs[0], position_W=np.array([0.2, -0.15, 0]))
    plan = replace(
        inputs[5],
        segments=tuple(replace(s, duration_s=3.0) for s in inputs[5].segments),
        completion_dwell_s=0.5,
        completion_timeout_s=3.0,
    )
    outer = replace(inputs[3], maximum_acceleration_W=np.array([0.05, 0.05, 2.0]))
    result = simulate(initial_state=initial, position_controller=outer, plan=plan)
    assert np.any(result.outer_limit_flags[:, 0])
    assert not np.any(result.outer_limit_flags[-50:])
    assert result.phase[-1] == P.COMPLETE
    assert np.linalg.norm(result.position_W[-1]) < 0.01
