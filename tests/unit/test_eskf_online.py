"""Measurement-only streaming, replay parity, memory, and transactional failures."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.eskf_validation import endpoint_case
from quadrotor_math.eskf_online import EskfOnlineEstimator
from quadrotor_math.eskf_replay import (
    EskfObservationKind as Kind,
)
from quadrotor_math.eskf_replay import (
    EskfReplayObservation,
    replay_eskf,
)
from quadrotor_math.eskf_synthetic import make_eskf_synthetic_case


def case(endpoint=True):
    value = make_eskf_synthetic_case(20, number_of_steps=40)
    return endpoint_case(value) if endpoint else value


def assert_state(first, second):
    for name in first.__dataclass_fields__:
        np.testing.assert_array_equal(getattr(first, name), getattr(second, name))


def advance(stream, data, k, observations=None):
    if observations is None:
        observations = tuple(o for o in data.observations if o.delivery_index == k)
    return stream.step(
        float(data.time_s[k]),
        data.specific_force_measurements_B[k],
        data.angular_velocity_measurements_B[k],
        observations,
    )


@pytest.mark.parametrize("endpoint", [False, True])
@pytest.mark.parametrize("mode", ["normal", "disabled", "rejected", "stale"])
def test_stream_exactly_matches_replay_and_same_epoch_order(endpoint, mode):
    from quadrotor_math.eskf_innovation import EskfInnovationPolicy

    c = case(endpoint)
    config, data = c.configuration, c.measurements
    if mode == "disabled":
        config = replace(config, fuse_local_position=False)
    if mode == "rejected":
        config = replace(config, innovation_policy=EskfInnovationPolicy(1e-9, 1e-9))
    if mode == "stale":
        data = replace(
            data,
            observations=tuple(
                replace(o, delivery_index=min(o.acquisition_index + 1, len(data.time_s) - 1))
                for o in data.observations
            ),
        )
    expected = replay_eskf(data, config)
    stream = EskfOnlineEstimator(config)
    assert stream.latest is None
    events = []
    for k in range(len(data.time_s)):
        obs = tuple(reversed([o for o in data.observations if o.delivery_index == k]))
        got = advance(stream, data, k, obs)
        assert got.epoch_index == k and got.time_s == data.time_s[k]
        assert_state(got.nominal_state, expected.states[k])
        np.testing.assert_array_equal(got.covariance, expected.covariances[k])
        events.extend(got.events)
    assert [e.status for e in events] == [e.status for e in expected.events]
    for got, reference in zip(events, expected.events, strict=True):
        assert got.observation.kind == reference.observation.kind
        if reference.innovation is not None:
            np.testing.assert_array_equal(
                got.innovation.innovation, reference.innovation.innovation
            )
            assert got.innovation.normalized_innovation_squared == (
                reference.innovation.normalized_innovation_squared
            )


@pytest.mark.parametrize("endpoint", [False, True])
def test_initial_rate_is_measured_minus_explicit_prior_bias(endpoint):
    c = case(endpoint)
    got = advance(EskfOnlineEstimator(c.configuration), c.measurements, 0, ())
    assert_state(got.nominal_state, c.configuration.initial_state)
    np.testing.assert_array_equal(got.covariance, c.configuration.initial_covariance)
    np.testing.assert_array_equal(
        got.angular_velocity_estimate_B,
        c.measurements.angular_velocity_measurements_B[0]
        - c.configuration.initial_state.gyroscope_bias_B,
    )


@pytest.mark.parametrize("bad", ["time", "force_shape", "rate_nan", "future", "pending"])
def test_rejected_call_is_atomic_and_retry_matches_fresh_stream(bad):
    c = case()
    stream, control = EskfOnlineEstimator(c.configuration), EskfOnlineEstimator(c.configuration)
    first = advance(stream, c.measurements, 0)
    advance(control, c.measurements, 0)
    args = [
        float(c.measurements.time_s[1]),
        c.measurements.specific_force_measurements_B[1],
        c.measurements.angular_velocity_measurements_B[1],
        (),
    ]
    if bad == "time":
        args[0] = first.time_s
    elif bad == "force_shape":
        args[1] = np.zeros(2)
    elif bad == "rate_nan":
        args[2] = np.full(3, np.nan)
    else:
        args[3] = (
            EskfReplayObservation(
                Kind.LOCAL_POSITION, 999, 1, 2 if bad == "future" else -1, np.zeros(3)
            ),
        )
    with pytest.raises((ValueError, TypeError)):
        stream.step(*args)
    assert stream.latest.epoch_index == 0
    assert_state(stream.latest.nominal_state, first.nominal_state)
    got, expected = advance(stream, c.measurements, 1), advance(control, c.measurements, 1)
    assert_state(got.nominal_state, expected.nominal_state)
    np.testing.assert_array_equal(got.covariance, expected.covariance)


def test_failed_arithmetic_preserves_entire_endpoint_memory():
    c = case()
    stream, other = EskfOnlineEstimator(c.configuration), EskfOnlineEstimator(c.configuration)
    for k in range(21):
        advance(stream, c.measurements, k)
        advance(other, c.measurements, k)
    with pytest.raises(ValueError):
        stream.step(1e308, np.full(3, 1e308), np.full(3, 1e308))
    got, expected = advance(stream, c.measurements, 21), advance(other, c.measurements, 21)
    assert_state(got.nominal_state, expected.nominal_state)
    np.testing.assert_array_equal(got.covariance, expected.covariance)


def test_duplicate_observation_cannot_be_fused_twice():
    c = case()
    obs = EskfReplayObservation(Kind.LOCAL_POSITION, 0, 0, 0, np.zeros(3))
    stream = EskfOnlineEstimator(c.configuration)
    with pytest.raises(ValueError, match="duplicate"):
        advance(stream, c.measurements, 0, (obs, obs))
    assert stream.latest is None
    advance(stream, c.measurements, 0, (obs,))
    with pytest.raises(ValueError, match="duplicate"):
        advance(stream, c.measurements, 1, (replace(obs, delivery_index=1),))
    assert stream.latest.epoch_index == 0


def test_outputs_own_memory_and_cannot_change_continuation():
    c = case()
    stream, other = EskfOnlineEstimator(c.configuration), EskfOnlineEstimator(c.configuration)
    first = advance(stream, c.measurements, 0)
    advance(other, c.measurements, 0)
    for value in (
        first.covariance,
        first.nominal_state.position_W,
        first.angular_velocity_estimate_B,
    ):
        assert value.flags.owndata and not value.flags.writeable
        value.flags.writeable = True
        value[...] = 999
    snapshot = stream.latest
    snapshot.covariance.flags.writeable = True
    snapshot.covariance[...] = 888
    got, expected = advance(stream, c.measurements, 1), advance(other, c.measurements, 1)
    assert_state(got.nominal_state, expected.nominal_state)
    np.testing.assert_array_equal(got.covariance, expected.covariance)


@pytest.mark.parametrize("bad", [True, np.nan, np.inf, -1.0, 0.01])
def test_first_time_must_match_explicit_prior_epoch(bad):
    c = case()
    stream = EskfOnlineEstimator(c.configuration)
    with pytest.raises(ValueError):
        stream.step(bad, np.zeros(3), np.zeros(3))
    assert stream.latest is None


def test_invalid_configuration_and_observation_types():
    with pytest.raises(TypeError):
        EskfOnlineEstimator(None)
    c = case()
    stream = EskfOnlineEstimator(c.configuration)
    with pytest.raises(TypeError):
        advance(stream, c.measurements, 0, (object(),))


def test_future_measurement_change_cannot_change_any_previous_output():
    c = case()
    a, b = EskfOnlineEstimator(c.configuration), EskfOnlineEstimator(c.configuration)
    for k in range(10):
        x, y = advance(a, c.measurements, k), advance(b, c.measurements, k)
        assert_state(x.nominal_state, y.nominal_state)
    x = advance(a, c.measurements, 10)
    y = b.step(
        float(c.measurements.time_s[10]),
        c.measurements.specific_force_measurements_B[10] + 1,
        c.measurements.angular_velocity_measurements_B[10],
        tuple(o for o in c.measurements.observations if o.delivery_index == 10),
    )
    assert not np.array_equal(x.nominal_state.velocity_W, y.nominal_state.velocity_W)


def test_rate_uses_posterior_shared_sample_mean_and_bias_not_prior_values():
    from quadrotor_math.eskf import (
        eskf_barometric_altitude_measurement_model,
        eskf_local_position_measurement_model,
    )
    from quadrotor_math.eskf_endpoint import (
        initialize_eskf_endpoint,
        predict_eskf_endpoint,
        update_eskf_endpoint,
    )

    c = case()
    config = replace(c.configuration, innovation_policy=None)
    data = c.measurements
    stream = EskfOnlineEstimator(config)
    joint = initialize_eskf_endpoint(
        config.initial_state, config.initial_covariance, config.sampled_imu_noise
    )
    nonzero = False
    for k, time in enumerate(data.time_s):
        if k:
            joint = predict_eskf_endpoint(
                joint,
                data.specific_force_measurements_B[k - 1],
                data.angular_velocity_measurements_B[k - 1],
                data.specific_force_measurements_B[k],
                data.angular_velocity_measurements_B[k],
                config.gravity_acceleration,
                config.sampled_imu_noise,
                float(time - data.time_s[k - 1]),
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
        got = advance(stream, data, k)
        np.testing.assert_array_equal(got.imu_noise_mean_B, joint.imu_noise_mean_B)
        np.testing.assert_array_equal(
            got.angular_velocity_estimate_B,
            data.angular_velocity_measurements_B[k]
            - joint.nominal_state.gyroscope_bias_B
            - joint.imu_noise_mean_B[3:],
        )
        nonzero |= np.any(joint.imu_noise_mean_B[3:] != 0)
    assert nonzero


def test_second_update_failure_does_not_commit_first_update_or_consume_ids():
    c = case()
    covariance = np.zeros((15, 15))
    covariance[0, 0] = 0.01
    config = replace(
        c.configuration,
        initial_covariance=covariance,
        barometric_altitude_noise_variance=0.0,
        innovation_policy=None,
    )
    stream = EskfOnlineEstimator(config)
    position = EskfReplayObservation(Kind.LOCAL_POSITION, 0, 0, 0, np.ones(3))
    baro = EskfReplayObservation(Kind.BAROMETRIC_ALTITUDE, 0, 0, 0, np.zeros(1))
    with pytest.raises(ValueError):
        advance(stream, c.measurements, 0, (position, baro))
    assert stream.latest is None
    got = advance(stream, c.measurements, 0, (position,))
    expected = advance(EskfOnlineEstimator(config), c.measurements, 0, (position,))
    assert_state(got.nominal_state, expected.nominal_state)
    np.testing.assert_array_equal(got.covariance, expected.covariance)


@pytest.mark.parametrize("endpoint", [False, True])
def test_nonuniform_online_epochs_match_replay(endpoint):
    from quadrotor_math.eskf_replay import EskfReplayInput

    c = case(endpoint)
    data = EskfReplayInput(
        np.array([0.0, 0.01, 0.04, 0.11]),
        c.measurements.specific_force_measurements_B[:4],
        c.measurements.angular_velocity_measurements_B[:4],
    )
    expected = replay_eskf(data, c.configuration)
    stream = EskfOnlineEstimator(c.configuration)
    for k in range(4):
        got = advance(stream, data, k)
        assert_state(got.nominal_state, expected.states[k])
        np.testing.assert_array_equal(got.covariance, expected.covariances[k])
