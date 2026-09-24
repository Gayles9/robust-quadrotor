"""Causal sensor/control boundaries, complete histories, and physical reconstruction."""

from dataclasses import fields, replace

import numpy as np
import pytest

from experiments.position_control_validation import make_case
from quadrotor_math.attitude_control import compute_attitude_control
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_endpoint import EskfSampledImuNoise
from quadrotor_math.eskf_innovation import EskfInnovationPolicy
from quadrotor_math.eskf_replay import EskfReplayConfiguration, replay_eskf
from quadrotor_math.estimated_mission import MissionSensors, simulate_estimated_mission
from quadrotor_math.missions import MissionPhase, mission_reference
from quadrotor_math.position_control import compute_position_control
from quadrotor_math.run_configuration import ImuParameters, PositionSensorParameters, SensorSchedule


def inputs(noisy=False, seed=31):
    initial, body, world, outer, inner, plan, safety, numerics = make_case(
        {"case": "smoke", "seed": None, "refinement": 1}
    )
    sigma_a, sigma_g = (0.04, 0.002) if noisy else (0.0, 0.0)
    imu = ImuParameters(
        np.zeros(3), np.full(3, sigma_a), np.zeros(3), np.zeros(3), np.full(3, sigma_g), np.zeros(3)
    )
    position = PositionSensorParameters(
        np.zeros(3), np.full(3, 0.02 if noisy else 0), 0.0, 0.0, 0.03 if noisy else 0
    )
    sensors = MissionSensors(imu, position, SensorSchedule(0.02, 0), SensorSchedule(0.01, 0), seed)
    prior = EskfNominalState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )
    config = EskfReplayConfiguration(
        0.0,
        prior,
        np.eye(15) * 1e-4,
        9.81,
        np.zeros((12, 12)),
        np.zeros(3),
        np.eye(3) * 0.02**2,
        0.0,
        0.0,
        0.03**2,
        innovation_policy=EskfInnovationPolicy(16.26623619623813, 10.827566170662733),
        sampled_imu_noise=EskfSampledImuNoise(
            np.diag([sigma_a**2] * 3 + [sigma_g**2] * 3), np.zeros((6, 6))
        ),
    )
    return dict(
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
        sensors=sensors,
        estimator_configuration=config,
    )


def assert_replay(result, config):
    replay = replay_eskf(result.measurements, config)
    np.testing.assert_array_equal(replay.covariances, result.estimates.covariances)
    for a, b in zip(replay.states, result.estimates.states, strict=True):
        for field in fields(a):
            np.testing.assert_array_equal(getattr(a, field.name), getattr(b, field.name))
    assert [e.status for e in replay.events] == [e.status for e in result.estimates.events]


def test_equilibrium_and_exact_measurement_control_clocks():
    kwargs = inputs()
    r = simulate_estimated_mission(**kwargs)
    assert r.mission.phase[-1] == MissionPhase.COMPLETE
    np.testing.assert_array_equal(r.mission.position_W, 0)
    np.testing.assert_array_equal(
        r.measurements.specific_force_measurements_B,
        np.tile([0.0, 0, -9.81], (len(r.mission.time_s), 1)),
    )
    np.testing.assert_array_equal(r.measurements.angular_velocity_measurements_B, 0)
    np.testing.assert_array_equal(r.mission.control_time_s, r.mission.time_s[:-1:4])
    assert r.measurements.time_s[0] == 0
    assert all(o.acquisition_index > 0 for o in r.measurements.observations)
    assert r.mission.control_time_s[-1] < r.measurements.time_s[-1]
    assert_replay(r, kwargs["estimator_configuration"])


def test_noisy_closed_loop_offline_replay_and_controller_reconstruction():
    kwargs = inputs(noisy=True)
    r = simulate_estimated_mission(**kwargs)
    assert_replay(r, kwargs["estimator_configuration"])
    for row, time in enumerate(r.mission.position_control_time_s):
        k = np.searchsorted(r.mission.time_s, time)
        estimate = r.estimates.states[k]
        ref, _ = mission_reference(kwargs["plan"], float(time))
        command = compute_position_control(
            estimate.position_W, estimate.velocity_W, ref, kwargs["position_controller"]
        )
        np.testing.assert_array_equal(
            command.requested_acceleration_W, r.mission.requested_acceleration_W[row]
        )
    for row, time in enumerate(r.mission.control_time_s):
        k = np.searchsorted(r.mission.time_s, time)
        command = compute_attitude_control(
            r.estimates.states[k].q_WB,
            r.angular_velocity_estimate_B[k],
            r.mission.q_reference_WB[row],
            r.mission.collective_thrust[row],
            kwargs["attitude_controller"],
        )
        np.testing.assert_array_equal(
            command.allocation.commanded_rotor_omega, r.mission.commanded_rotor_omega[row]
        )


@pytest.mark.parametrize("field", ["position_W", "velocity_W", "q_WB"])
def test_hidden_truth_does_not_replace_independent_prior_feedback(field):
    kwargs = inputs()
    baseline = simulate_estimated_mission(**kwargs)
    value = (
        np.array([0.2, 0.1, 0]) if field != "q_WB" else np.array([np.cos(0.1), np.sin(0.1), 0, 0])
    )
    kwargs["initial_state"] = replace(kwargs["initial_state"], **{field: value})
    changed = simulate_estimated_mission(**kwargs)
    np.testing.assert_array_equal(
        changed.mission.commanded_rotor_omega[0], baseline.mission.commanded_rotor_omega[0]
    )
    assert not np.array_equal(
        getattr(changed.mission, field)[0], getattr(baseline.mission, field)[0]
    )


def test_command_changes_when_only_explicit_prior_changes():
    kwargs = inputs()
    baseline = simulate_estimated_mission(**kwargs)
    config = kwargs["estimator_configuration"]
    kwargs["estimator_configuration"] = replace(
        config, initial_state=replace(config.initial_state, position_W=np.array([0.02, 0, 0]))
    )
    result = simulate_estimated_mission(**kwargs)
    np.testing.assert_array_equal(baseline.mission.position_W[0], result.mission.position_W[0])
    assert not np.array_equal(
        baseline.mission.commanded_rotor_omega[0], result.mission.commanded_rotor_omega[0]
    )


def test_completion_uses_estimate_not_a_hidden_truth_arrival_test():
    kwargs = inputs()
    kwargs["initial_state"] = replace(kwargs["initial_state"], position_W=np.array([0.2, 0, 0]))
    kwargs["sensors"] = replace(kwargs["sensors"], local_position_schedule=SensorSchedule(0.2, 0))
    result = simulate_estimated_mission(**kwargs)
    assert result.mission.phase[-1] == MissionPhase.COMPLETE
    assert result.mission.position_W[-1, 0] == 0.2
    assert result.estimates.states[-1].position_W[0] == 0


@pytest.mark.parametrize(
    "truth_bad,estimate_bad,reason",
    [
        (True, False, "truth_geofence"),
        (False, True, "estimate_geofence"),
        (True, True, "truth_geofence"),
    ],
)
def test_guard_sources_are_distinct_with_no_terminal_command(truth_bad, estimate_bad, reason):
    kwargs = inputs()
    if truth_bad:
        kwargs["initial_state"] = replace(kwargs["initial_state"], position_W=np.array([4.0, 0, 0]))
    if estimate_bad:
        c = kwargs["estimator_configuration"]
        kwargs["estimator_configuration"] = replace(
            c, initial_state=replace(c.initial_state, position_W=np.array([4.0, 0, 0]))
        )
    r = simulate_estimated_mission(**kwargs)
    assert r.mission.abort_reason == reason and len(r.estimates.states) == 1
    assert len(r.mission.control_time_s) == 0


def test_off_grid_delivery_stale_pending_and_no_hidden_time_shift():
    kwargs = inputs(noisy=True)
    kwargs["sensors"] = replace(
        kwargs["sensors"],
        local_position_schedule=SensorSchedule(0.01, 0.013),
        barometric_altitude_schedule=SensorSchedule(0.01, 1.0),
    )
    r = simulate_estimated_mission(**kwargs)
    assert {e.status.value for e in r.estimates.events} == {"stale", "pending"}
    first = r.estimates.events[0].observation
    assert first.acquisition_index == 4 and first.delivery_index == 10
    assert_replay(r, kwargs["estimator_configuration"])


def test_delivery_delay_does_not_shift_draws_when_fusion_disabled():
    kwargs = inputs(noisy=True)
    kwargs["estimator_configuration"] = replace(
        kwargs["estimator_configuration"], fuse_local_position=False, fuse_barometric_altitude=False
    )
    first = simulate_estimated_mission(**kwargs)
    kwargs["sensors"] = replace(
        kwargs["sensors"],
        local_position_schedule=SensorSchedule(0.02, 1),
        barometric_altitude_schedule=SensorSchedule(0.01, 0.017),
    )
    second = simulate_estimated_mission(**kwargs)
    np.testing.assert_array_equal(
        first.measurements.specific_force_measurements_B,
        second.measurements.specific_force_measurements_B,
    )
    np.testing.assert_array_equal(
        first.measurements.angular_velocity_measurements_B,
        second.measurements.angular_velocity_measurements_B,
    )
    np.testing.assert_array_equal(
        first.mission.commanded_rotor_omega, second.mission.commanded_rotor_omega
    )


def test_noisy_seed_determinism_and_independent_owned_results():
    kwargs = inputs(noisy=True)
    a, b = simulate_estimated_mission(**kwargs), simulate_estimated_mission(**kwargs)
    np.testing.assert_array_equal(a.estimates.covariances, b.estimates.covariances)
    np.testing.assert_array_equal(a.mission.commanded_rotor_omega, b.mission.commanded_rotor_omega)
    for value in (
        a.angular_velocity_estimate_B,
        a.imu_noise_mean_B,
        a.true_accelerometer_bias_B,
        a.scheduled_observation_delivery_time_s,
        a.estimates.covariances,
    ):
        assert value.flags.owndata and value.flags.c_contiguous and not value.flags.writeable
    assert not np.shares_memory(a.measurements.time_s, a.mission.time_s)
    assert not np.shares_memory(
        a.estimates.states[0].position_W, kwargs["estimator_configuration"].initial_state.position_W
    )
    other = simulate_estimated_mission(**inputs(noisy=True, seed=32))
    assert not np.array_equal(
        a.measurements.specific_force_measurements_B,
        other.measurements.specific_force_measurements_B,
    )


@pytest.mark.parametrize("bad", [True, -1, 2**128, 1.5])
def test_bad_sensor_seed(bad):
    with pytest.raises(ValueError):
        replace(inputs()["sensors"], root_seed=bad)


@pytest.mark.parametrize("change", ["period", "prior_epoch", "first_order", "datatype"])
def test_invalid_configuration_is_rejected_before_plant(change, monkeypatch):
    kwargs = inputs()
    if change == "period":
        kwargs["sensors"] = replace(
            kwargs["sensors"], local_position_schedule=SensorSchedule(0.003, 0)
        )
    elif change == "prior_epoch":
        kwargs["estimator_configuration"] = replace(
            kwargs["estimator_configuration"], initial_time_s=0.01
        )
    elif change == "first_order":
        kwargs["estimator_configuration"] = replace(
            kwargs["estimator_configuration"], sampled_imu_noise=None
        )
    else:
        kwargs["sensors"] = None

    def forbidden(*args, **kw):
        raise AssertionError("must reject before advancing plant")

    monkeypatch.setattr("quadrotor_math.mission_simulation._plant_step", forbidden)
    with pytest.raises((ValueError, TypeError)):
        simulate_estimated_mission(**kwargs)


@pytest.mark.parametrize(
    "field",
    [
        "angular_velocity_estimate_B",
        "imu_noise_mean_B",
        "true_accelerometer_bias_B",
        "scheduled_observation_delivery_time_s",
    ],
)
def test_result_corruption_is_rejected(field):
    r = simulate_estimated_mission(**inputs())
    value = getattr(r, field).copy()
    value.flat[0] = np.nan
    with pytest.raises(ValueError):
        replace(r, **{field: value})


def test_rate_and_delivery_cross_field_validation():
    r = simulate_estimated_mission(**inputs(noisy=True))
    with pytest.raises(ValueError, match="rate feedback"):
        replace(r, angular_velocity_estimate_B=r.angular_velocity_estimate_B + 0.001)
    with pytest.raises(ValueError, match="scheduled delivery"):
        replace(
            r, scheduled_observation_delivery_time_s=r.scheduled_observation_delivery_time_s + 0.01
        )


def test_imu_specific_force_includes_actual_motors_drag_and_bias():
    kwargs = inputs()
    kwargs["truth_world"] = replace(kwargs["truth_world"], wind_velocity_W=np.array([0.5, -0.3, 0]))
    kwargs["truth_body"] = replace(
        kwargs["truth_body"], quadratic_drag_coefficient_B=np.array([0.1, 0.1, 0.15])
    )
    imu = replace(
        kwargs["sensors"].imu,
        initial_accelerometer_bias_B=np.array([0.02, -0.01, 0.03]),
        initial_gyroscope_bias_B=np.array([0.001, -0.002, 0.003]),
    )
    kwargs["sensors"] = replace(kwargs["sensors"], imu=imu)
    r = simulate_estimated_mission(**kwargs)
    # At t=0: FRD rotor force=-mg e3; drag opposes v-wind componentwise.
    expected = np.array([0.1 * 0.5**2, -0.1 * 0.3**2, -9.81]) + imu.initial_accelerometer_bias_B
    np.testing.assert_allclose(
        r.measurements.specific_force_measurements_B[0], expected, atol=1e-14
    )
    np.testing.assert_array_equal(
        r.measurements.angular_velocity_measurements_B[0], imu.initial_gyroscope_bias_B
    )


def test_bias_walks_follow_independent_named_streams_and_full_grid_force_identity():
    from quadrotor_math.randomness import create_run_random_streams
    from quadrotor_math.rotations import rotation_matrix_body_to_world

    kwargs = inputs()
    imu = replace(
        kwargs["sensors"].imu,
        initial_accelerometer_bias_B=np.array([0.01, 0.02, -0.01]),
        initial_gyroscope_bias_B=np.array([0.002, -0.001, 0.003]),
        accelerometer_bias_random_walk_density_B=np.full(3, 0.002),
        gyroscope_bias_random_walk_density_B=np.full(3, 0.0002),
    )
    kwargs["sensors"] = replace(kwargs["sensors"], imu=imu)
    r = simulate_estimated_mission(**kwargs)
    rng = create_run_random_streams(kwargs["sensors"].root_seed)
    ba, bg = imu.initial_accelerometer_bias_B.copy(), imu.initial_gyroscope_bias_B.copy()
    dt = kwargs["numerics"].time_step_s
    for k in range(len(r.mission.time_s)):
        if k:
            ba += (
                imu.accelerometer_bias_random_walk_density_B
                * np.sqrt(dt)
                * rng.accelerometer_bias_random_walk.standard_normal(3)
            )
            bg += (
                imu.gyroscope_bias_random_walk_density_B
                * np.sqrt(dt)
                * rng.gyroscope_bias_random_walk.standard_normal(3)
            )
        np.testing.assert_array_equal(ba, r.true_accelerometer_bias_B[k])
        np.testing.assert_array_equal(bg, r.true_gyroscope_bias_B[k])
        R = rotation_matrix_body_to_world(r.mission.q_WB[k])
        air_B = R.T @ (r.mission.velocity_W[k] - kwargs["truth_world"].wind_velocity_W)
        drag_B = -kwargs["truth_body"].quadratic_drag_coefficient_B * np.abs(air_B) * air_B
        force_B = drag_B + np.array(
            [
                0.0,
                0,
                -kwargs["truth_rotors"].thrust_coefficient
                * np.sum(r.mission.actual_rotor_omega[k] ** 2),
            ]
        )
        np.testing.assert_allclose(
            r.measurements.specific_force_measurements_B[k],
            force_B / kwargs["truth_body"].mass + ba,
            atol=2e-14,
            rtol=1e-14,
        )
        np.testing.assert_array_equal(
            r.measurements.angular_velocity_measurements_B[k], r.mission.omega_B[k] + bg
        )


def test_future_reference_cannot_change_past_measurements_or_estimates():
    from quadrotor_math.missions import ReferenceKind

    kwargs = inputs(noisy=True)
    before = simulate_estimated_mission(**kwargs)
    segments = list(kwargs["plan"].segments)
    segments[2] = replace(
        segments[2], end_position_W=np.array([0.001, 0, 0]), kind=ReferenceKind.STEP
    )
    segments[3] = replace(
        segments[3], start_position_W=np.array([0.001, 0, 0]), kind=ReferenceKind.SMOOTH
    )
    kwargs["plan"] = replace(kwargs["plan"], segments=tuple(segments))
    after = simulate_estimated_mission(**kwargs)
    np.testing.assert_array_equal(before.estimates.covariances[:9], after.estimates.covariances[:9])
    np.testing.assert_array_equal(
        before.measurements.specific_force_measurements_B[:9],
        after.measurements.specific_force_measurements_B[:9],
    )
    np.testing.assert_array_equal(
        before.mission.commanded_rotor_omega[:2], after.mission.commanded_rotor_omega[:2]
    )
    assert not np.array_equal(
        before.mission.commanded_rotor_omega[2], after.mission.commanded_rotor_omega[2]
    )


def test_truth_guard_between_controller_ticks_does_not_issue_another_command(monkeypatch):
    from quadrotor_math.attitude_simulation import _plant_step

    def move_outside(*args):
        state, actual = _plant_step(*args)
        return (np.array([4.0, 0, 0]), *state[1:]), actual

    monkeypatch.setattr("quadrotor_math.mission_simulation._plant_step", move_outside)
    r = simulate_estimated_mission(**inputs())
    assert r.mission.abort_reason == "truth_geofence"
    assert r.mission.time_s[-1] == 0.0025 and len(r.estimates.states) == 2
    assert r.mission.control_time_s.tolist() == [0.0]


@pytest.mark.parametrize("seed", [8150, 8151])
def test_seeded_noisy_takeoff_translation_landing_regression(seed):
    from quadrotor_math.missions import MissionPlan, MissionSegment, ReferenceKind

    kwargs = inputs(noisy=True, seed=seed)
    points = [np.zeros(3), np.array([0.0, 0, -0.2]), np.array([0.3, 0.2, -0.2])]
    P, K = MissionPhase, ReferenceKind
    kwargs["plan"] = MissionPlan(
        (
            MissionSegment(P.INITIALIZE, 0.5, points[0], points[0], K.HOLD),
            MissionSegment(P.TAKEOFF, 2, points[0], points[1], K.SMOOTH),
            MissionSegment(P.TRACK, 3, points[1], points[2], K.SMOOTH),
            MissionSegment(P.TRACK, 1, points[2], points[2], K.HOLD),
            MissionSegment(P.LAND, 3, points[2], points[0], K.SMOOTH),
        ),
        0.0,
        0.08,
        0.08,
        0.5,
        4,
    )
    r = simulate_estimated_mission(**kwargs)
    error = np.linalg.norm(r.mission.position_W - r.mission.reference_position_W, axis=1)
    rmse = np.sqrt(np.trapezoid(error**2, r.mission.time_s) / r.mission.time_s[-1])
    assert r.mission.phase[-1] == MissionPhase.COMPLETE and rmse < 0.1
    assert error[-1] < 0.1 and np.linalg.norm(r.mission.velocity_W[-1]) < 0.1
    assert not np.any(r.mission.inner_limit_flags)
