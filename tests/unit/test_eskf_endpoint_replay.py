"""Endpoint selection, causal timing, correction memory and old-mode compatibility."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.eskf_validation import endpoint_case
from quadrotor_math.eskf import (
    eskf_barometric_altitude_measurement_model,
    eskf_local_position_measurement_model,
)
from quadrotor_math.eskf_endpoint import (
    initialize_eskf_endpoint,
    predict_eskf_endpoint,
    update_eskf_endpoint,
)
from quadrotor_math.eskf_innovation import EskfInnovationPolicy
from quadrotor_math.eskf_replay import (
    EskfObservationKind as Kind,
)
from quadrotor_math.eskf_replay import (
    EskfReplayInput,
    EskfReplayObservation,
    replay_eskf,
)
from quadrotor_math.eskf_synthetic import make_eskf_synthetic_case


def case():
    return endpoint_case(make_eskf_synthetic_case(20, number_of_steps=40))


def assert_same(a, b):
    np.testing.assert_array_equal(a.covariances, b.covariances)
    for x, y in zip(a.states, b.states, strict=True):
        for name in x.__dataclass_fields__:
            np.testing.assert_array_equal(getattr(x, name), getattr(y, name))


def test_requires_unambiguous_explicit_sample_noise():
    c = case().configuration
    with pytest.raises(ValueError, match="continuous_noise_covariance must be zero"):
        replace(c, continuous_noise_covariance=np.eye(12))
    with pytest.raises(TypeError, match="sampled_imu_noise"):
        replace(c, sampled_imu_noise=True)


def test_endpoint_replay_matches_explicit_joint_prediction_and_ordered_updates():
    c = case()
    config, data = c.configuration, c.measurements
    result = replay_eskf(data, config)
    joint = initialize_eskf_endpoint(
        config.initial_state, config.initial_covariance, config.sampled_imu_noise
    )
    for k, t in enumerate(data.time_s):
        if k:
            joint = predict_eskf_endpoint(
                joint,
                data.specific_force_measurements_B[k - 1],
                data.angular_velocity_measurements_B[k - 1],
                data.specific_force_measurements_B[k],
                data.angular_velocity_measurements_B[k],
                config.gravity_acceleration,
                config.sampled_imu_noise,
                float(t - data.time_s[k - 1]),
            )
        for obs in data.observations:
            if obs.delivery_index != k:
                continue
            if obs.kind is Kind.LOCAL_POSITION:
                predicted, H = eskf_local_position_measurement_model(
                    joint.nominal_state, config.local_position_bias_W
                )
                R = config.local_position_noise_covariance_W
            else:
                predicted, H = eskf_barometric_altitude_measurement_model(
                    joint.nominal_state,
                    config.barometric_reference_altitude,
                    config.barometric_altitude_bias,
                )
                R = np.array([[config.barometric_altitude_noise_variance]])
            joint, _ = update_eskf_endpoint(joint, obs.measurement, predicted, H, R)
        np.testing.assert_array_equal(joint.joint_covariance[:15, :15], result.covariances[k])
        for name in joint.nominal_state.__dataclass_fields__:
            np.testing.assert_array_equal(
                getattr(joint.nominal_state, name), getattr(result.states[k], name)
            )


def test_future_rows_cannot_affect_past_and_final_sample_is_consumed():
    c = case()
    original = replay_eskf(c.measurements, c.configuration)
    changed = c.measurements.specific_force_measurements_B.copy()
    changed[-1, 0] += 3
    result = replay_eskf(
        replace(c.measurements, specific_force_measurements_B=changed), c.configuration
    )
    np.testing.assert_array_equal(original.covariances[:-1], result.covariances[:-1])
    for a, b in zip(original.states[:-1], result.states[:-1], strict=True):
        for name in a.__dataclass_fields__:
            np.testing.assert_array_equal(getattr(a, name), getattr(b, name))
    assert not np.array_equal(original.states[-1].velocity_W, result.states[-1].velocity_W)


@pytest.mark.parametrize("status", ["stale", "pending", "disabled", "rejected"])
def test_ineligible_observations_do_not_condition_shared_sample(status):
    c = case()
    config = replace(c.configuration, fuse_barometric_altitude=False)
    obs = EskfReplayObservation(Kind.LOCAL_POSITION, 0, 0, 0, np.array([1e4, 2e4, 3e4]))
    if status == "stale":
        obs = replace(obs, delivery_index=1)
    if status == "pending":
        obs = replace(obs, delivery_index=-1)
    if status == "disabled":
        config = replace(config, fuse_local_position=False)
    if status == "rejected":
        config = replace(config, innovation_policy=EskfInnovationPolicy(1.0, 1.0))
    empty = replace(c.measurements, observations=())
    result = replay_eskf(replace(empty, observations=(obs,)), config)
    assert result.events[0].status.value == status
    assert_same(result, replay_eskf(empty, config))


def test_nonuniform_clock_and_one_epoch_boundary():
    c = case()
    initial = c.configuration.initial_state
    times = np.array([0.0, 0.03, 0.08, 0.2])
    data = EskfReplayInput(times, np.tile([0.0, 0.0, -9.81], (4, 1)), np.zeros((4, 3)))
    initial = replace(
        initial,
        position_W=np.zeros(3),
        velocity_W=np.zeros(3),
        q_WB=np.array([1.0, 0, 0, 0]),
        accelerometer_bias_B=np.zeros(3),
        gyroscope_bias_B=np.zeros(3),
    )
    config = replace(c.configuration, initial_state=initial)
    result = replay_eskf(data, config)
    np.testing.assert_allclose([s.position_W for s in result.states], 0.0, atol=1e-15)
    one = EskfReplayInput(
        times[:1], data.specific_force_measurements_B[:1], data.angular_velocity_measurements_B[:1]
    )
    output = replay_eskf(one, config)
    np.testing.assert_array_equal(output.covariances[0], config.initial_covariance)
    assert not np.shares_memory(output.covariances, config.initial_covariance)


def test_joint_state_arrays_are_owned_in_public_replay_result():
    c = case()
    result = replay_eskf(c.measurements, c.configuration)
    assert result.covariances.flags.owndata and not result.covariances.flags.writeable
    for event in result.events:
        if event.update is not None:
            assert not np.shares_memory(event.update.covariance, result.covariances)
