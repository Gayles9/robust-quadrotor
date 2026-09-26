"""Replacement geometric reference, integration and measurement-boundary checks."""

from dataclasses import fields, replace

import numpy as np
import pytest

from experiments.estimated_feedback_validation import make_configuration
from experiments.position_control_validation import baseline_mission
from experiments.trajectory_mission_validation import build_mission
from quadrotor_math.eskf_replay import replay_eskf
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.geometric_control import (
    GeometricControllerParameters,
    GeometricDomainError,
    compute_geometric_control,
)
from quadrotor_math.geometric_filter import FeedbackDerivativeFilter
from quadrotor_math.geometric_reference import (
    build_geometric_reference,
    projected_collective_thrust,
    reference_jerk_snap,
)
from quadrotor_math.mission_simulation import simulate_mission
from quadrotor_math.missions import MissionPhase, mission_reference
from quadrotor_math.position_control import PositionReference
from quadrotor_math.rotations import rotation_matrix_body_to_world


def args(noisy=False):
    return make_configuration(
        {"case": "smoke", "seed": 31 if noisy else None, "noiseless": not noisy}
    )


def true_args(config):
    return {k: v for k, v in config.items() if k not in ("sensors", "estimator_configuration")}


def test_default_and_geometric_equilibrium_match_and_reset():
    kwargs = true_args(args())
    original = simulate_mission(**kwargs)
    first = simulate_mission(**kwargs, geometric_controller=GeometricControllerParameters())
    second = simulate_mission(**kwargs, geometric_controller=GeometricControllerParameters())
    for field in fields(first):
        a, b, c = (getattr(r, field.name) for r in (original, first, second))
        if isinstance(a, np.ndarray):
            np.testing.assert_array_equal(a, b)
            np.testing.assert_array_equal(b, c)
    assert first.phase[-1] == MissionPhase.COMPLETE


def test_geometric_outer_domain_abort_has_no_terminal_commands():
    kwargs = true_args(args())
    kwargs["initial_state"] = replace(kwargs["initial_state"], velocity_W=np.array([10.0, 0, 0]))
    result = simulate_mission(**kwargs, geometric_controller=GeometricControllerParameters())
    assert result.abort_reason == "outer_reference_domain"
    assert len(result.time_s) == 1
    assert len(result.control_time_s) == len(result.position_control_time_s) == 0


def test_inner_projection_abort_has_no_terminal_commands():
    kwargs = true_args(args())
    kwargs["position_controller"] = replace(
        kwargs["position_controller"], minimum_collective_thrust=9.8
    )
    angle = 0.2
    kwargs["initial_state"] = replace(
        kwargs["initial_state"], q_WB=np.array([np.cos(angle / 2), np.sin(angle / 2), 0, 0])
    )
    result = simulate_mission(**kwargs, geometric_controller=GeometricControllerParameters())
    assert result.abort_reason == "projected_thrust_domain"
    assert len(result.control_time_s) == len(result.position_control_time_s) == 0


def test_reject_step_and_invalid_optional_configuration():
    kwargs = true_args(args())
    kwargs["plan"] = baseline_mission("vertical_step")
    with pytest.raises(ValueError, match="STEP"):
        simulate_mission(**kwargs, geometric_controller=GeometricControllerParameters())
    with pytest.raises(TypeError, match="geometric_controller"):
        simulate_mission(**kwargs, geometric_controller=True)
    with pytest.raises(ValueError, match="allow_minimum_snap"):
        simulate_estimated_mission(**args(), allow_minimum_snap=1)


def test_planned_polynomial_jets_at_right_hand_boundaries():
    plan, _ = build_mission()
    start = 0.0
    for seg in plan.segments:
        for elapsed in (0.0, 0.3 * seg.duration_s):
            jerk, snap = reference_jerk_snap(plan, start + elapsed)
            if hasattr(seg, "trajectory"):
                np.testing.assert_allclose(jerk, seg.trajectory.evaluate(elapsed, 3), atol=1e-12)
                np.testing.assert_allclose(snap, seg.trajectory.evaluate(elapsed, 4), atol=1e-12)
            else:
                np.testing.assert_array_equal(jerk, 0)
                np.testing.assert_array_equal(snap, 0)
        start += seg.duration_s
    for value in reference_jerk_snap(plan, start):
        np.testing.assert_array_equal(value, 0)


def test_quintic_jets_independent_polynomial_oracle():
    plan = baseline_mission("hover")
    # The first motion segment is -z*(10*s^3-15*s^4+6*s^5), duration 4 s.
    coeff = np.array([0.0, 0, 0, 10, -15, 6])
    for t in (1.0, 2.3, 4.999):
        for order, actual in zip((3, 4), reference_jerk_snap(plan, t), strict=True):
            expected = (
                np.polynomial.polynomial.polyval(
                    (t - 1) / 4, np.polynomial.polynomial.polyder(coeff, order)
                )
                / 4**order
            )
            np.testing.assert_allclose(actual, [0, 0, -expected], atol=1e-14)


def test_steady_position_error_has_no_artificial_feedforward_rate():
    outer = args()["position_controller"]
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0.4)
    memory = FeedbackDerivativeFilter(0.02)
    p = np.array([0.05, -0.03, 0])
    for k in range(5):
        target, memory = build_geometric_reference(
            p, np.zeros(3), ref, np.zeros(3), np.zeros(3), outer, memory, k
        )
        np.testing.assert_array_equal(target.rotation.omega_reference_D, 0)
        np.testing.assert_array_equal(target.rotation.alpha_reference_D, 0)
        np.testing.assert_allclose(target.lift_W, [p[0], p[1], 9.81])
    R = rotation_matrix_body_to_world(target.rotation.q_reference_WB)
    assert projected_collective_thrust(
        target, target.rotation.q_reference_WB, outer
    ) == pytest.approx(np.linalg.norm(target.lift_W))
    np.testing.assert_allclose(R[:, 2], target.lift_W / np.linalg.norm(target.lift_W))


@pytest.mark.parametrize("mode", ["original", "rebase", "measured"])
def test_noisy_estimated_commands_reconstructed_without_truth_and_offline_replay(mode):
    kwargs = args(noisy=True)
    if mode != "original":
        # Accepted corrections can occur between outer ticks and must accumulate.
        sensors = kwargs["sensors"]
        kwargs["sensors"] = replace(
            sensors,
            local_position_schedule=replace(sensors.local_position_schedule, sample_period_s=0.03),
            barometric_altitude_schedule=replace(
                sensors.barometric_altitude_schedule, sample_period_s=0.015
            ),
        )
    # Give enough time for slow observations and multiple outer updates.
    plan = kwargs["plan"]
    kwargs["plan"] = replace(
        plan, segments=tuple(replace(s, duration_s=0.1) for s in plan.segments)
    )
    gains = GeometricControllerParameters(
        rebase_estimator_corrections=mode == "rebase", use_measured_acceleration=mode == "measured"
    )
    r = simulate_estimated_mission(**kwargs, geometric_controller=gains)
    replay = replay_eskf(r.measurements, kwargs["estimator_configuration"])
    np.testing.assert_array_equal(replay.covariances, r.estimates.covariances)
    for a, b in zip(replay.states, r.estimates.states, strict=True):
        for field in fields(a):
            np.testing.assert_array_equal(getattr(a, field.name), getattr(b, field.name))
    assert [e.status for e in replay.events] == [e.status for e in r.estimates.events]
    h = kwargs["numerics"].time_step_s * kwargs["numerics"].position_stride
    memory = FeedbackDerivativeFilter(h)
    outer_i = 0
    events = [e for e in r.estimates.events if e.update is not None]
    event_i = 0
    for row, time in enumerate(r.mission.control_time_s):
        k = int(np.searchsorted(r.mission.time_s, time))
        state = r.estimates.states[k]
        if time in r.mission.position_control_time_s:
            jump = np.zeros(3)
            while event_i < len(events) and events[event_i].observation.delivery_index <= k:
                delta = events[event_i].update.error_state_correction
                outer = kwargs["position_controller"]
                jump += outer.nominal_mass * (
                    outer.position_gain_W * delta[:3] + outer.velocity_gain_W * delta[3:6]
                )
                event_i += 1
            if mode == "rebase":
                memory = memory.rebase(jump)
            acceleration = None
            if mode == "measured":
                acceleration = rotation_matrix_body_to_world(state.q_WB) @ (
                    r.measurements.specific_force_measurements_B[k]
                    - state.accelerometer_bias_B
                    - r.imu_noise_mean_B[k, :3]
                ) + np.array([0.0, 0.0, kwargs["position_controller"].nominal_gravity_acceleration])
            ref, _ = mission_reference(kwargs["plan"], float(time))
            jerk, snap = reference_jerk_snap(kwargs["plan"], float(time))
            target, memory = build_geometric_reference(
                state.position_W,
                state.velocity_W,
                ref,
                jerk,
                snap,
                kwargs["position_controller"],
                memory,
                outer_i,
                estimated_acceleration_W=acceleration,
            )
            outer_i += 1
        thrust = projected_collective_thrust(target, state.q_WB, kwargs["position_controller"])
        command = compute_geometric_control(
            state.q_WB,
            r.angular_velocity_estimate_B[k],
            target.rotation,
            thrust,
            kwargs["attitude_controller"],
            gains,
        )
        np.testing.assert_array_equal(command.moment_requested_B, r.mission.moment_requested_B[row])
        np.testing.assert_array_equal(
            command.allocation.commanded_rotor_omega, r.mission.commanded_rotor_omega[row]
        )
        assert thrust == r.mission.collective_thrust[row]
    repeat = simulate_estimated_mission(**kwargs, geometric_controller=gains)
    np.testing.assert_array_equal(repeat.mission.position_W, r.mission.position_W)
    assert r.mission.control_time_s[-1] < r.mission.time_s[-1]


@pytest.mark.parametrize("value", [0, 1, "yes", None, np.bool_(True)])
def test_estimator_rebase_option_requires_explicit_boolean(value):
    with pytest.raises(ValueError, match="rebase_estimator_corrections"):
        GeometricControllerParameters(rebase_estimator_corrections=value)


def test_invalid_large_reference_leaves_filter_state_unchanged():
    config = args()
    memory = FeedbackDerivativeFilter(0.02)
    ref, _ = mission_reference(config["plan"], 0.0)
    with pytest.raises(GeometricDomainError):
        build_geometric_reference(
            np.ones(3) * 1e10,
            np.zeros(3),
            ref,
            np.zeros(3),
            np.zeros(3),
            config["position_controller"],
            memory,
            0,
        )
    assert memory.next_index == 0
