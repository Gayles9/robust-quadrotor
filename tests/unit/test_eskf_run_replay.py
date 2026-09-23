"""Generated and persisted sensor runs through a truth-isolated ESKF adapter."""

from dataclasses import fields, replace
from pathlib import Path

import numpy as np
import pytest

from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_endpoint import EskfSampledImuNoise
from quadrotor_math.eskf_innovation import (
    EskfInnovationPolicy,
    chi_square_99_percent_eskf_innovation_policy,
)
from quadrotor_math.eskf_replay import EskfObservationKind, EskfReplayStatus, replay_eskf
from quadrotor_math.eskf_run_replay import (
    eskf_replay_configuration_from_nominal,
    eskf_replay_input_from_run_artifact,
)
from quadrotor_math.run_artifact import load_run_directory, save_run_directory
from quadrotor_math.run_configuration import (
    ConstantRotorSpeedInput,
    DeclaredMismatch,
    ImuParameters,
    IntegrationMethod,
    NominalConfiguration,
    PositionSensorParameters,
    RigidBodyInitialState,
    RigidBodyParameters,
    RotorParameters,
    RunConfiguration,
    RunNumerics,
    SensorSchedule,
    SensorSchedules,
    TruthConfiguration,
    WorldParameters,
)
from quadrotor_math.run_generation import generate_run_artifact_data
from quadrotor_math.run_manifest import RunManifest, SoftwareProvenance


def _run(
    n: int = 8, *, seed: int = 21, noise: float = 0.0, moving: bool = False
) -> RunConfiguration:
    body = RigidBodyParameters(1.0, np.diag([0.01, 0.01, 0.02]))
    rotors = RotorParameters(
        np.array([[0.1, 0.1, 0], [0.1, -0.1, 0], [-0.1, -0.1, 0], [-0.1, 0.1, 0]]),
        np.array([1.0, -1.0, 1.0, -1.0]),
        1.0e-5,
        1.0e-7,
    )
    world = WorldParameters(9.81)
    imu = ImuParameters(
        np.zeros(3),
        np.full(3, noise * 0.02),
        np.zeros(3),
        np.zeros(3),
        np.full(3, noise * 0.001),
        np.zeros(3),
    )
    sensors = PositionSensorParameters(
        np.array([0.1, -0.2, 0.3]), np.full(3, noise * 0.05), 100.0, 0.2, noise * 0.03
    )
    return RunConfiguration(
        TruthConfiguration(body, rotors, world, imu, sensors),
        NominalConfiguration(body, rotors, world, imu, sensors),
        RigidBodyInitialState(
            np.zeros(3),
            np.array([0.2, -0.1, 0.05]) if moving else np.zeros(3),
            np.array([1.0, 0, 0, 0]),
            np.zeros(3),
        ),
        RunNumerics(IntegrationMethod.PROJECTED_RK4, 0.01, n),
        SensorSchedules(
            SensorSchedule(0.01, 0.0),
            SensorSchedule(0.01, 0.0),
            SensorSchedule(0.02, 0.0),
            SensorSchedule(0.04, 0.0),
        ),
        ConstantRotorSpeedInput(np.full(4, np.sqrt(9.81 / 4.0e-5))),
        seed,
        (),
    )


def _config(run: RunConfiguration, policy=None, *, endpoint=False):
    # Known experiment prior at dt; never initialized from a truth-history row.
    state = EskfNominalState(
        np.array([0.3, -0.2, 0.5]), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )
    return eskf_replay_configuration_from_nominal(
        run.nominal,
        initial_time_s=run.numerics.truth_time_step_s,
        initial_state=state,
        initial_covariance=np.eye(15),
        continuous_noise_covariance=np.zeros((12, 12)) if endpoint else np.eye(12) * 1.0e-5,
        innovation_policy=policy,
        sampled_imu_noise=(
            EskfSampledImuNoise(np.eye(6) * 1e-4, np.eye(6) * 1e-8) if endpoint else None
        ),
    )


def test_adapter_extracts_exact_measurement_rows_and_source_identifiers() -> None:
    run = _run(noise=1.0)
    artifact = generate_run_artifact_data(run)
    data = eskf_replay_input_from_run_artifact(artifact)
    np.testing.assert_array_equal(data.time_s, artifact.truth_time_s[1:])
    np.testing.assert_array_equal(
        data.specific_force_measurements_B, artifact.accelerometer_measurements_B
    )
    assert not np.shares_memory(
        data.specific_force_measurements_B, artifact.accelerometer_measurements_B
    )
    assert [
        (o.kind, o.observation_index, o.acquisition_index, o.delivery_index)
        for o in data.observations
    ] == [
        (EskfObservationKind.LOCAL_POSITION, 0, 1, 1),
        (EskfObservationKind.LOCAL_POSITION, 1, 3, 3),
        (EskfObservationKind.BAROMETRIC_ALTITUDE, 0, 3, 3),
        (EskfObservationKind.LOCAL_POSITION, 2, 5, 5),
        (EskfObservationKind.LOCAL_POSITION, 3, 7, 7),
        (EskfObservationKind.BAROMETRIC_ALTITUDE, 1, 7, 7),
    ]


@pytest.mark.parametrize("endpoint", [False, True])
def test_generated_saved_loaded_run_has_identical_estimator_replay(
    tmp_path: Path, endpoint
) -> None:
    run = _run(30, noise=1.0)
    artifact = generate_run_artifact_data(run)
    manifest = RunManifest(run, SoftwareProvenance("0.1.0", "3.12.14", "2.5.2", "a" * 40, True))
    save_run_directory(tmp_path / "run", manifest, artifact)
    loaded_manifest, loaded = load_run_directory(tmp_path / "run")
    first = replay_eskf(
        eskf_replay_input_from_run_artifact(artifact), _config(run, endpoint=endpoint)
    )
    second = replay_eskf(
        eskf_replay_input_from_run_artifact(loaded),
        _config(loaded_manifest.run_configuration, endpoint=endpoint),
    )
    assert first.covariances.tobytes() == second.covariances.tobytes()
    for actual, expected in zip(first.states, second.states, strict=True):
        for field in fields(actual):
            assert getattr(actual, field.name).tobytes() == getattr(expected, field.name).tobytes()
    for actual, expected in zip(first.events, second.events, strict=True):
        assert actual.status is expected.status
        assert actual.update is not None and expected.update is not None
        assert actual.update.innovation.tobytes() == expected.update.innovation.tobytes()
        assert actual.update.kalman_gain.tobytes() == expected.update.kalman_gain.tobytes()


@pytest.mark.parametrize("sensor", ["accelerometer", "gyroscope"])
@pytest.mark.parametrize(
    "schedule", [SensorSchedule(0.02, 0.0), SensorSchedule(0.01, 0.01), SensorSchedule(0.01, 1.0)]
)
def test_unsupported_imu_schedules_reject(sensor: str, schedule: SensorSchedule) -> None:
    run = _run()
    run = replace(run, sensor_schedules=replace(run.sensor_schedules, **{sensor: schedule}))
    with pytest.raises(ValueError, match=sensor):
        eskf_replay_input_from_run_artifact(generate_run_artifact_data(run))


def test_delayed_and_pending_position_data_are_not_fused() -> None:
    run = _run(noise=1.0)
    run = replace(
        run,
        sensor_schedules=replace(run.sensor_schedules, local_position=SensorSchedule(0.02, 0.015)),
    )
    result = replay_eskf(
        eskf_replay_input_from_run_artifact(generate_run_artifact_data(run)), _config(run)
    )
    position = [
        e for e in result.events if e.observation.kind is EskfObservationKind.LOCAL_POSITION
    ]
    assert [e.status for e in position] == [EskfReplayStatus.STALE] * 3 + [EskfReplayStatus.PENDING]


@pytest.mark.parametrize("endpoint", [False, True])
def test_adapter_and_runner_do_not_read_truth_payloads(endpoint) -> None:
    run = _run(noise=1.0)
    original = generate_run_artifact_data(run)
    expected = replay_eskf(
        eskf_replay_input_from_run_artifact(original), _config(run, endpoint=endpoint)
    )
    # Fail loudly even on read, not merely alter values and hope a test notices.
    forbidden = {
        "truth_position_history_W",
        "truth_velocity_history_W",
        "truth_q_history_WB",
        "truth_omega_history_B",
        "accelerometer_bias_history_B",
        "gyroscope_bias_history_B",
        "commanded_rotor_omega",
    }

    class MeasurementOnly:
        def __getattr__(self, name):
            if name in forbidden:
                pytest.fail(f"truth/control access: {name}")
            return getattr(original, name)

    actual = replay_eskf(
        eskf_replay_input_from_run_artifact(MeasurementOnly()), _config(run, endpoint=endpoint)
    )
    np.testing.assert_array_equal(actual.covariances, expected.covariances)


def test_nominal_helper_squares_discrete_observation_standard_deviations() -> None:
    run = _run(noise=1.0)
    config = _config(run)
    np.testing.assert_array_equal(config.local_position_noise_covariance_W, np.eye(3) * 0.05**2)
    assert config.barometric_altitude_noise_variance == 0.03**2
    assert config.barometric_reference_altitude == 100.0
    np.testing.assert_array_equal(config.continuous_noise_covariance, np.eye(12) * 1.0e-5)


class _Override:
    def __init__(self, data, **changes):
        self.data = data
        self.changes = changes

    def __getattr__(self, name):
        return self.changes[name] if name in self.changes else getattr(self.data, name)


@pytest.mark.parametrize(
    "name",
    [
        "accelerometer_acquisition_time_s",
        "gyroscope_delivery_time_s",
        "local_position_acquisition_time_s",
        "local_position_delivery_time_s",
        "barometric_altitude_acquisition_time_s",
        "barometric_altitude_delivery_time_s",
        "accelerometer_measurements_B",
        "gyroscope_measurements_B",
        "local_position_measurements_W",
        "barometric_altitude_measurements",
    ],
)
@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_adapter_revalidates_every_floating_sensor_field(name: str, bad: float) -> None:
    original = generate_run_artifact_data(_run())
    changed = getattr(original, name).copy()
    changed.flat[-1] = bad
    with pytest.raises(ValueError, match=name):
        eskf_replay_input_from_run_artifact(_Override(original, **{name: changed}))


@pytest.mark.parametrize(
    "name",
    [
        "accelerometer_sequence_index",
        "accelerometer_observation_index",
        "gyroscope_sequence_index",
        "gyroscope_observation_index",
        "local_position_sequence_index",
        "local_position_observation_index",
        "barometric_altitude_sequence_index",
        "barometric_altitude_observation_index",
        "delivery_sensor_id",
        "delivery_sequence_index",
        "delivery_observation_index",
    ],
)
@pytest.mark.parametrize("bad", ["value", "dtype", "shape"])
def test_adapter_revalidates_source_ids_and_delivery_table(name: str, bad: str) -> None:
    original = generate_run_artifact_data(_run())
    changed = getattr(original, name).copy()
    if bad == "value":
        changed[-1] += 1
    elif bad == "dtype":
        changed = changed.astype(float)
    else:
        changed = changed[:, None]
    with pytest.raises(ValueError, match=name):
        eskf_replay_input_from_run_artifact(_Override(original, **{name: changed}))


@pytest.mark.parametrize("name", ["local_position", "barometric_altitude"])
@pytest.mark.parametrize(
    "bad",
    ["off_grid", "after_horizon", "duplicate", "negative_delay", "wrong_arrival", "false_pending"],
)
def test_adapter_rejects_noncausal_or_contradictory_observation_timing(name: str, bad: str) -> None:
    original = generate_run_artifact_data(_run())
    changes = {}
    if bad in ("off_grid", "after_horizon", "duplicate"):
        key = f"{name}_acquisition_time_s"
        value = getattr(original, key).copy()
        value[-1] = (
            value[-1] + 1e-5 if bad == "off_grid" else (1.0 if bad == "after_horizon" else value[0])
        )
    elif bad == "negative_delay":
        key = f"{name}_delivery_time_s"
        value = getattr(original, key).copy()
        value[0] -= 0.001
    else:
        key = f"{name}_delivered_at_truth_index"
        value = getattr(original, key).copy()
        value[0] = -1 if bad == "false_pending" else value[0] + 1
    changes[key] = value
    with pytest.raises(ValueError, match=name):
        eskf_replay_input_from_run_artifact(_Override(original, **changes))


@pytest.mark.parametrize(
    "clock",
    [
        np.array([]),
        np.array([0.0]),
        np.array([0.0, 0.0]),
        np.array([1.0, 2.0]),
        np.array([0.0, 0.1, 0.3]),
        np.array([0.0, np.inf]),
        np.array([0.0, 1e-16]),
        np.array([0.0, 0.1, 0.05]),
    ],
)
def test_adapter_rejects_invalid_or_unresolved_clock(clock: np.ndarray) -> None:
    with pytest.raises(ValueError, match="truth_time_s"):
        eskf_replay_input_from_run_artifact(
            _Override(generate_run_artifact_data(_run()), truth_time_s=clock)
        )


@pytest.mark.parametrize("offset", [-1e-13, 0.0, 1e-13])
def test_scheduler_boundary_arrivals_match_adapter(offset: float) -> None:
    run = _run(noise=1.0)
    run = replace(
        run,
        sensor_schedules=replace(
            run.sensor_schedules, local_position=SensorSchedule(0.02, 0.02 + offset)
        ),
    )
    artifact = generate_run_artifact_data(run)
    data = eskf_replay_input_from_run_artifact(artifact)
    first = next(
        o
        for o in data.observations
        if o.kind is EskfObservationKind.LOCAL_POSITION and o.observation_index == 0
    )
    assert first.delivery_index == (4 if offset > 0 else 3)
    assert first.delivery_index + 1 == artifact.local_position_delivered_at_truth_index[0]


def test_empty_observation_streams_and_one_row_replay_are_valid() -> None:
    run = _run(1, noise=1.0)
    data = eskf_replay_input_from_run_artifact(generate_run_artifact_data(run))
    assert data.observations == ()
    config = _config(run)
    result = replay_eskf(data, config)
    for field in fields(config.initial_state):
        np.testing.assert_array_equal(
            getattr(result.states[0], field.name), getattr(config.initial_state, field.name)
        )
    np.testing.assert_array_equal(result.covariances[0], config.initial_covariance)


@pytest.mark.parametrize("moving", [False, True])
def test_noise_free_generated_known_motion_matches_analytic_reference(moving: bool) -> None:
    run = _run(101, moving=moving)
    data = eskf_replay_input_from_run_artifact(generate_run_artifact_data(run))
    velocity = np.array([0.2, -0.1, 0.05]) if moving else np.zeros(3)
    state = EskfNominalState(
        velocity * 0.01, velocity, np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )
    config = replace(
        _config(run),
        initial_state=state,
        local_position_noise_covariance_W=np.eye(3) * 0.05**2,
        barometric_altitude_noise_variance=0.03**2,
    )
    result = replay_eskf(data, config)
    np.testing.assert_allclose(
        [s.position_W for s in result.states], data.time_s[:, None] * velocity, rtol=0, atol=3e-13
    )
    np.testing.assert_allclose(
        [s.velocity_W for s in result.states], np.tile(velocity, (101, 1)), rtol=0, atol=3e-13
    )


@pytest.mark.parametrize("moving", [False, True])
@pytest.mark.parametrize("seed", range(12))
def test_seeded_generated_runs_reduce_error_and_retain_valid_covariance(
    seed: int, moving: bool
) -> None:
    # Fixed regression ensemble, not held-out consistency evidence.
    run = _run(151, seed=seed, noise=1.0, moving=moving)
    artifact = generate_run_artifact_data(run)
    data, config = eskf_replay_input_from_run_artifact(artifact), _config(run)
    result = replay_eskf(data, config)
    dead_reckoning = replay_eskf(
        data, replace(config, fuse_local_position=False, fuse_barometric_altitude=False)
    )
    truth = artifact.truth_position_history_W[1:]
    corrected_rmse = np.sqrt(
        np.mean(np.sum((np.array([s.position_W for s in result.states]) - truth) ** 2, axis=1))
    )
    drift_rmse = np.sqrt(
        np.mean(
            np.sum((np.array([s.position_W for s in dead_reckoning.states]) - truth) ** 2, axis=1)
        )
    )
    assert corrected_rmse < 0.5 * drift_rmse
    assert np.all(np.isfinite(result.covariances))
    np.testing.assert_array_equal(result.covariances, result.covariances.transpose(0, 2, 1))
    assert np.linalg.eigvalsh(result.covariances).min() >= -1e-12
    np.testing.assert_allclose(
        [np.linalg.norm(s.q_WB) for s in result.states], 1.0, rtol=0, atol=2e-15
    )


def test_generated_vertical_bias_regression_corrects_drift_without_truth_initialization() -> None:
    run = _run(801, noise=1.0)
    imu = replace(
        run.truth.imu,
        initial_accelerometer_bias_B=np.array([0.0, 0.0, 0.15]),
        accelerometer_noise_standard_deviation_B=np.array([0.0, 0.0, 0.02]),
        gyroscope_noise_standard_deviation_B=np.zeros(3),
    )
    nominal_imu = replace(imu, initial_accelerometer_bias_B=np.zeros(3))
    run = replace(
        run,
        truth=replace(run.truth, imu=imu),
        nominal=replace(run.nominal, imu=nominal_imu),
        declared_mismatches=(
            DeclaredMismatch("imu.initial_accelerometer_bias_B", "Vertical bias regression"),
        ),
    )
    artifact = generate_run_artifact_data(run)
    data = eskf_replay_input_from_run_artifact(artifact)
    state = EskfNominalState(
        np.array([0, 0, 0.5]),
        np.array([0, 0, 0.1]),
        np.array([1.0, 0, 0, 0]),
        np.zeros(3),
        np.zeros(3),
    )
    covariance = np.zeros((15, 15))
    covariance[2, 2], covariance[5, 5], covariance[11, 11] = 1.0, 1.0, 0.1
    config = replace(
        _config(run),
        initial_state=state,
        initial_covariance=covariance,
        continuous_noise_covariance=np.zeros((12, 12)),
    )
    result = replay_eskf(data, config)
    reference = replay_eskf(
        data, replace(config, fuse_local_position=False, fuse_barometric_altitude=False)
    )
    assert abs(result.states[-1].accelerometer_bias_B[2] - 0.15) < 0.01
    assert abs(result.states[-1].position_W[2]) < 0.03
    assert abs(reference.states[-1].position_W[2]) > 6.0


@pytest.mark.parametrize("sensor", ["position", "altitude"])
@pytest.mark.parametrize("standard_deviation", [1e308, 1e-200])
def test_nominal_variance_conversion_rejects_unrepresentable_values(
    sensor: str, standard_deviation: float
) -> None:
    run = _run(noise=1.0)
    changes = (
        {"local_position_noise_standard_deviation_W": np.full(3, standard_deviation)}
        if sensor == "position"
        else {"barometric_altitude_noise_standard_deviation": standard_deviation}
    )
    nominal = replace(
        run.nominal, position_sensors=replace(run.nominal.position_sensors, **changes)
    )
    with pytest.raises(ValueError, match="variances"):
        eskf_replay_configuration_from_nominal(
            nominal,
            initial_time_s=0.01,
            initial_state=_config(run).initial_state,
            initial_covariance=np.eye(15),
            continuous_noise_covariance=np.zeros((12, 12)),
        )


@pytest.mark.parametrize("policy", [None, EskfInnovationPolicy(), EskfInnovationPolicy(7, 4)])
def test_nominal_helper_preserves_explicit_policy_without_inference(policy) -> None:
    configuration = _config(_run(noise=1), policy)
    if policy is None:
        assert configuration.innovation_policy is None
    else:
        assert configuration.innovation_policy is not policy
        assert (
            configuration.innovation_policy.local_position_nis_threshold
            == policy.local_position_nis_threshold
        )
        assert (
            configuration.innovation_policy.barometric_altitude_nis_threshold
            == policy.barometric_altitude_nis_threshold
        )


@pytest.mark.parametrize("kind", list(EskfObservationKind))
@pytest.mark.parametrize("burst_length", [1, 3])
@pytest.mark.parametrize("offset", [-1000.0, 1000.0])
def test_fixed_outlier_fixture_rejects_before_correction_and_recovers(
    kind, burst_length, offset
) -> None:
    # Thresholds and corruption are fixed before running this deterministic regression.
    # The production sensor model is unchanged; only this measurement-only fixture is edited.
    run = _run(61, seed=713, noise=1.0, moving=True)
    original = eskf_replay_input_from_run_artifact(generate_run_artifact_data(run))
    policy = chi_square_99_percent_eskf_innovation_policy()
    configuration = _config(run, policy)
    corrupt_ids = set(range(3, 3 + burst_length))
    observations = tuple(
        replace(o, measurement=o.measurement + offset)
        if o.kind is kind and o.observation_index in corrupt_ids
        else o
        for o in original.observations
    )
    changed = replace(original, observations=observations)
    result = replay_eskf(changed, configuration)
    reference_observations = []
    next_id = dict.fromkeys(EskfObservationKind, 0)
    for observation in original.observations:
        if observation.kind is kind and observation.observation_index in corrupt_ids:
            continue
        reference_observations.append(
            replace(observation, observation_index=next_id[observation.kind])
        )
        next_id[observation.kind] += 1
    reference = replay_eskf(
        replace(original, observations=tuple(reference_observations)), configuration
    )
    # Statistical rejection has exactly the same state effect as omitting these values.
    assert result.covariances.tobytes() == reference.covariances.tobytes()
    for actual, expected in zip(result.states, reference.states, strict=True):
        for field in fields(actual):
            assert getattr(actual, field.name).tobytes() == getattr(expected, field.name).tobytes()
    stream = [e for e in result.events if e.observation.kind is kind]
    assert all(stream[i].status is EskfReplayStatus.REJECTED for i in corrupt_ids)
    assert all(stream[i].update is None for i in corrupt_ids)
    assert stream[max(corrupt_ids) + 1].status is EskfReplayStatus.FUSED
    # Clean input is still owned independently; no injection modifies the artifact/adapter result.
    assert any(
        not np.array_equal(a.measurement, b.measurement)
        for a, b in zip(original.observations, changed.observations, strict=True)
    )


@pytest.mark.parametrize("diagnostics_only", [False, True])
def test_scored_saved_loaded_run_matches_all_event_diagnostics(tmp_path, diagnostics_only) -> None:
    run = _run(41, seed=281, noise=1.0)
    artifact = generate_run_artifact_data(run)
    manifest = RunManifest(run, SoftwareProvenance("0.1.0", "3.12.14", "2.5.2", "b" * 40, True))
    save_run_directory(tmp_path / "run", manifest, artifact)
    _, loaded = load_run_directory(tmp_path / "run")
    policy = (
        EskfInnovationPolicy()
        if diagnostics_only
        else chi_square_99_percent_eskf_innovation_policy()
    )
    inputs = [eskf_replay_input_from_run_artifact(a) for a in (artifact, loaded)]
    results = []
    for data in inputs:
        # Same test-only corruption for the gate. Diagnostics-only leaves correction
        # enabled and makes no guarantee of numerical viability after a gross outlier.
        observations = tuple(
            replace(o, measurement=o.measurement + 1000)
            if not diagnostics_only and o.observation_index == 4
            else o
            for o in data.observations
        )
        results.append(replay_eskf(replace(data, observations=observations), _config(run, policy)))
    first, second = results
    assert first.covariances.tobytes() == second.covariances.tobytes()
    for a, b in zip(first.states, second.states, strict=True):
        assert all(getattr(a, f.name).tobytes() == getattr(b, f.name).tobytes() for f in fields(a))
    for a, b in zip(first.events, second.events, strict=True):
        assert a.status is b.status
        assert a.nis_threshold == b.nis_threshold
        for name in ("innovation", "innovation_covariance", "whitened_innovation"):
            assert getattr(a.innovation, name).tobytes() == getattr(b.innovation, name).tobytes()
        assert (
            a.innovation.normalized_innovation_squared == b.innovation.normalized_innovation_squared
        )
    if diagnostics_only:
        assert all(e.status is EskfReplayStatus.FUSED for e in first.events)
    else:
        assert sum(e.status is EskfReplayStatus.REJECTED for e in first.events) >= 2


def test_diagnostics_only_preserves_ungated_gross_outlier_outcome() -> None:
    run = _run(41, seed=281, noise=1.0)
    data = eskf_replay_input_from_run_artifact(generate_run_artifact_data(run))
    changed = replace(
        data,
        observations=tuple(
            replace(o, measurement=o.measurement + 1000) if o.observation_index == 4 else o
            for o in data.observations
        ),
    )
    # Gross outliers can exceed the existing update/reset path's numerical domain.
    # Compare outcomes; do not require roundoff-triggered failure on every BLAS/CPU.
    outcomes = []
    for policy in (None, EskfInnovationPolicy()):
        try:
            result = replay_eskf(changed, _config(run, policy))
        except ValueError as error:
            outcomes.append(("error", str(error)))
        else:
            outcomes.append(
                (
                    "success",
                    result.covariances.tobytes(),
                    tuple(getattr(s, f.name).tobytes() for s in result.states for f in fields(s)),
                )
            )
    assert outcomes[0] == outcomes[1]


@pytest.mark.parametrize("endpoint", [False, True])
def test_scored_replay_uses_no_truth_or_rng(monkeypatch, endpoint) -> None:
    run = _run(20, noise=1.0)
    original = generate_run_artifact_data(run)
    configuration = _config(run, chi_square_99_percent_eskf_innovation_policy(), endpoint=endpoint)
    allowed = {"truth_time_s"} | {
        field.name
        for field in fields(original)
        if (
            field.name.startswith(
                (
                    "accelerometer_",
                    "gyroscope_",
                    "local_position_",
                    "barometric_altitude_",
                    "delivery_",
                )
            )
            and "bias_history" not in field.name
        )
    }

    class OnlyMeasurements:
        def __getattr__(self, name):
            if name not in allowed:
                pytest.fail(f"non-measurement access: {name}")
            return getattr(original, name)

    def forbidden(*args, **kwargs):
        pytest.fail("Replay requested an RNG")

    monkeypatch.setattr(np.random, "default_rng", forbidden)
    monkeypatch.setattr(np.random, "normal", forbidden)
    result = replay_eskf(eskf_replay_input_from_run_artifact(OnlyMeasurements()), configuration)
    assert result.events and all(e.innovation is not None for e in result.events)
