"""Public complete-run generation and persistence behavior."""

import hashlib
import json
from dataclasses import fields, replace
from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.actuation import motor_speed_first_order_step
from quadrotor_math.dynamics import rigid_body_state_derivative_from_rotor_speeds
from quadrotor_math.imu import ideal_accelerometer_specific_force_body
from quadrotor_math.integration import (
    rigid_body_state_euler_step_from_rotor_speeds,
    rigid_body_state_rk4_step_from_rotor_speeds,
)
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.run_artifact import RunArtifactData, load_run_directory, save_run_directory
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
from quadrotor_math.run_manifest import RunManifest, SoftwareProvenance, encode_run_manifest


@pytest.mark.parametrize(
    "field_name", ["gravity_acceleration", "thrust_coefficient", "moment_coefficient", "inertia_B"]
)
def test_audit_generation_preflights_truth_before_creating_random_streams(
    field_name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    configuration = _configuration()
    truth = configuration.truth
    if field_name == "gravity_acceleration":
        truth = replace(truth, world=replace(truth.world, gravity_acceleration=0.0))
        path = "world.gravity_acceleration"
    elif field_name == "inertia_B":
        inertia_B = truth.rigid_body.inertia_B.copy()
        inertia_B[0, 1] = 1.0e-9  # Structurally accepted, rejected by the numerical plant.
        truth = replace(truth, rigid_body=replace(truth.rigid_body, inertia_B=inertia_B))
        path = "rigid_body.inertia_B"
    else:
        truth = replace(truth, rotors=replace(truth.rotors, **{field_name: 0.0}))
        path = f"rotors.{field_name}"
    configuration = replace(
        configuration,
        truth=truth,
        declared_mismatches=(DeclaredMismatch(path, "Execution-domain regression"),),
    )

    def forbidden_streams(*args: object) -> None:
        pytest.fail("truth validation must precede random-stream creation")

    monkeypatch.setattr(
        "quadrotor_math.run_generation.create_run_random_streams", forbidden_streams
    )
    with pytest.raises(ValueError, match=field_name):
        generate_run_artifact_data(configuration)


@pytest.mark.parametrize("delay_offset", [1.0e-13, -1.0e-13, 0.0])
def test_audit_generated_delivery_boundary_round_trips(tmp_path: Path, delay_offset: float) -> None:
    configuration = _configuration()
    configuration = replace(
        configuration,
        sensor_schedules=replace(
            configuration.sensor_schedules, accelerometer=SensorSchedule(0.1, 0.1 + delay_offset)
        ),
    )
    data = generate_run_artifact_data(configuration)
    expected_first_delivery = 3 if delay_offset > 0.0 else 2
    assert data.accelerometer_delivered_at_truth_index[0] == expected_first_delivery
    manifest = RunManifest(
        configuration, SoftwareProvenance("0.1.0", "3.12.14", "2.5.2", "a" * 40, True)
    )
    save_run_directory(tmp_path / "run", manifest, data)
    _, loaded = load_run_directory(tmp_path / "run")
    for field in fields(data):
        np.testing.assert_array_equal(getattr(loaded, field.name), getattr(data, field.name))


def _configuration(
    *,
    truth_motor: bool = True,
    nominal_motor: bool = True,
    initial_actual: bool = True,
    integration_method: IntegrationMethod = IntegrationMethod.EULER,
    noise: bool = False,
    root_seed: int = 2026,
) -> RunConfiguration:
    positions_B = np.array([[0.2, 0.2, 0.0], [0.2, -0.2, 0.0], [-0.2, -0.2, 0.0], [-0.2, 0.2, 0.0]])
    spins = np.array([1.0, -1.0, 1.0, -1.0])
    rotor_common = dict(
        rotor_positions_B=positions_B,
        rotor_spin_directions=spins,
        thrust_coefficient=1.2e-5,
        moment_coefficient=2.0e-7,
    )
    motor = dict(
        minimum_rotor_omega=100.0,
        maximum_rotor_omega=1000.0,
        motor_time_constant_s=0.2,
    )
    truth_rotors = RotorParameters(**rotor_common, **(motor if truth_motor else {}))
    nominal_rotors = RotorParameters(**rotor_common, **(motor if nominal_motor else {}))
    rigid_body = RigidBodyParameters(1.5, np.diag([0.02, 0.025, 0.03]))
    world = WorldParameters(9.81)
    noise_scale = 1.0 if noise else 0.0
    imu = ImuParameters(
        initial_accelerometer_bias_B=np.array([0.1, -0.2, 0.3]),
        accelerometer_noise_standard_deviation_B=np.full(3, 0.02 * noise_scale),
        accelerometer_bias_random_walk_density_B=np.full(3, 0.01 * noise_scale),
        initial_gyroscope_bias_B=np.array([0.01, -0.02, 0.03]),
        gyroscope_noise_standard_deviation_B=np.full(3, 0.003 * noise_scale),
        gyroscope_bias_random_walk_density_B=np.full(3, 0.002 * noise_scale),
    )
    position_sensors = PositionSensorParameters(
        local_position_bias_W=np.array([0.2, -0.1, 0.3]),
        local_position_noise_standard_deviation_W=np.full(3, 0.04 * noise_scale),
        barometric_reference_altitude=100.0,
        barometric_altitude_bias=0.5,
        barometric_altitude_noise_standard_deviation=0.05 * noise_scale,
    )
    mismatches = (
        tuple(
            DeclaredMismatch(f"rotors.{name}", "Motor-mode fixture")
            for name in (
                "minimum_rotor_omega",
                "maximum_rotor_omega",
                "motor_time_constant_s",
            )
        )
        if truth_motor != nominal_motor
        else ()
    )
    return RunConfiguration(
        truth=TruthConfiguration(rigid_body, truth_rotors, world, imu, position_sensors),
        nominal=NominalConfiguration(rigid_body, nominal_rotors, world, imu, position_sensors),
        initial_truth_state=RigidBodyInitialState(
            position_W=np.array([1.0, 2.0, 3.0]),
            velocity_W=np.array([0.2, -0.1, 0.05]),
            q_WB=np.array([1.0, 0.0, 0.0, 0.0]),
            omega_B=np.array([0.01, 0.02, -0.03]),
        ),
        numerics=RunNumerics(integration_method, 0.1, 4),
        sensor_schedules=SensorSchedules(
            accelerometer=SensorSchedule(0.1, 0.1),
            gyroscope=SensorSchedule(0.2, 0.0),
            local_position=SensorSchedule(0.2, 0.3),
            barometric_altitude=SensorSchedule(0.1, 0.0),
        ),
        rotor_speed_input=ConstantRotorSpeedInput(np.array([900.0, 750.0, 1100.0, 650.0])),
        root_seed=root_seed,
        declared_mismatches=mismatches,
        initial_actual_rotor_omega=(
            np.array([200.0, 300.0, 400.0, 500.0]) if initial_actual else None
        ),
    )


def _quadratic_drag_configuration(
    *,
    truth_wind_velocity_W: NDArray[np.float64] | None = None,
    nominal_wind_velocity_W: NDArray[np.float64] | None = None,
    truth_quadratic_drag_coefficient_B: NDArray[np.float64] | None = None,
    nominal_quadratic_drag_coefficient_B: NDArray[np.float64] | None = None,
    noise: bool = False,
) -> RunConfiguration:
    if truth_wind_velocity_W is None:
        truth_wind_velocity_W = np.zeros(3, dtype=np.float64)
    if nominal_wind_velocity_W is None:
        nominal_wind_velocity_W = truth_wind_velocity_W
    if truth_quadratic_drag_coefficient_B is None:
        truth_quadratic_drag_coefficient_B = np.array([0.5, 0.0, 0.0])
    if nominal_quadratic_drag_coefficient_B is None:
        nominal_quadratic_drag_coefficient_B = truth_quadratic_drag_coefficient_B

    rigid_body_truth = RigidBodyParameters(
        1.0,
        np.eye(3),
        truth_quadratic_drag_coefficient_B,
    )
    rigid_body_nominal = RigidBodyParameters(
        1.0,
        np.eye(3),
        nominal_quadratic_drag_coefficient_B,
    )
    world_truth = WorldParameters(10.0, truth_wind_velocity_W)
    world_nominal = WorldParameters(10.0, nominal_wind_velocity_W)
    rotors = RotorParameters(
        rotor_positions_B=np.array(
            [
                [0.5, 0.0, 0.0],
                [0.0, 0.5, 0.0],
                [-0.5, 0.0, 0.0],
                [0.0, -0.5, 0.0],
            ]
        ),
        rotor_spin_directions=np.array([1.0, -1.0, 1.0, -1.0]),
        thrust_coefficient=1.0e-5,
        moment_coefficient=1.0e-6,
    )
    noise_scale = 1.0 if noise else 0.0
    imu = ImuParameters(
        initial_accelerometer_bias_B=np.zeros(3),
        accelerometer_noise_standard_deviation_B=np.full(3, 0.02 * noise_scale),
        accelerometer_bias_random_walk_density_B=np.full(3, 0.01 * noise_scale),
        initial_gyroscope_bias_B=np.zeros(3),
        gyroscope_noise_standard_deviation_B=np.full(3, 0.003 * noise_scale),
        gyroscope_bias_random_walk_density_B=np.full(3, 0.002 * noise_scale),
    )
    position_sensors = PositionSensorParameters(
        local_position_bias_W=np.zeros(3),
        local_position_noise_standard_deviation_W=np.full(3, 0.04 * noise_scale),
        barometric_reference_altitude=0.0,
        barometric_altitude_bias=0.0,
        barometric_altitude_noise_standard_deviation=0.05 * noise_scale,
    )
    mismatches = []
    if not np.array_equal(truth_wind_velocity_W, nominal_wind_velocity_W):
        mismatches.append(DeclaredMismatch("world.wind_velocity_W", "Wind mismatch fixture"))
    if not np.array_equal(
        truth_quadratic_drag_coefficient_B,
        nominal_quadratic_drag_coefficient_B,
    ):
        mismatches.append(
            DeclaredMismatch(
                "rigid_body.quadratic_drag_coefficient_B",
                "Quadratic-drag mismatch fixture",
            )
        )
    return RunConfiguration(
        truth=TruthConfiguration(
            rigid_body_truth,
            rotors,
            world_truth,
            imu,
            position_sensors,
        ),
        nominal=NominalConfiguration(
            rigid_body_nominal,
            rotors,
            world_nominal,
            imu,
            position_sensors,
        ),
        initial_truth_state=RigidBodyInitialState(
            position_W=np.zeros(3),
            velocity_W=np.array([2.0, 0.0, 0.0]),
            q_WB=np.array([1.0, 0.0, 0.0, 0.0]),
            omega_B=np.zeros(3),
        ),
        numerics=RunNumerics(IntegrationMethod.EULER, 0.1, 1),
        sensor_schedules=SensorSchedules(
            accelerometer=SensorSchedule(0.1, 0.0),
            gyroscope=SensorSchedule(0.1, 0.0),
            local_position=SensorSchedule(0.1, 0.0),
            barometric_altitude=SensorSchedule(0.1, 0.0),
        ),
        rotor_speed_input=ConstantRotorSpeedInput(np.full(4, 500.0)),
        root_seed=2026,
        declared_mismatches=tuple(mismatches),
    )


def _step(
    configuration: RunConfiguration, rotor_omega: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    integrator = (
        rigid_body_state_euler_step_from_rotor_speeds
        if configuration.numerics.integration_method is IntegrationMethod.EULER
        else rigid_body_state_rk4_step_from_rotor_speeds
    )
    state = configuration.initial_truth_state
    truth = configuration.truth
    return integrator(
        state.position_W,
        state.velocity_W,
        state.q_WB,
        state.omega_B,
        rotor_omega,
        truth.rotors.rotor_positions_B,
        truth.rotors.rotor_spin_directions,
        truth.rigid_body.mass,
        truth.rigid_body.inertia_B,
        truth.world.gravity_acceleration,
        truth.rotors.thrust_coefficient,
        truth.rotors.moment_coefficient,
        configuration.numerics.truth_time_step_s,
    )


def _state_row(data: RunArtifactData, index: int) -> tuple[NDArray[np.float64], ...]:
    return (
        data.truth_position_history_W[index],
        data.truth_velocity_history_W[index],
        data.truth_q_history_WB[index],
        data.truth_omega_history_B[index],
    )


def _actual_speed_history(configuration: RunConfiguration) -> NDArray[np.float64]:
    assert configuration.initial_actual_rotor_omega is not None
    minimum = configuration.truth.rotors.minimum_rotor_omega
    maximum = configuration.truth.rotors.maximum_rotor_omega
    time_constant = configuration.truth.rotors.motor_time_constant_s
    assert minimum is not None and maximum is not None and time_constant is not None
    history = [configuration.initial_actual_rotor_omega]
    for _ in range(configuration.numerics.number_of_steps):
        history.append(
            motor_speed_first_order_step(
                history[-1],
                configuration.rotor_speed_input.rotor_omega,
                minimum,
                maximum,
                time_constant,
                configuration.numerics.truth_time_step_s,
            )
        )
    return np.array(history)


@pytest.mark.parametrize("method", [IntegrationMethod.EULER, IntegrationMethod.PROJECTED_RK4])
def test_motorized_first_truth_step_uses_lagged_speed_and_selected_integrator(
    method: IntegrationMethod,
) -> None:
    configuration = _configuration(integration_method=method)
    data = generate_run_artifact_data(configuration)
    actual_at_row_one = _actual_speed_history(configuration)[1]
    expected = _step(configuration, actual_at_row_one)
    for actual_field, expected_field in zip(_state_row(data, 1), expected, strict=True):
        np.testing.assert_allclose(actual_field, expected_field, rtol=0.0, atol=1e-12)
    np.testing.assert_array_equal(
        data.commanded_rotor_omega[0], configuration.rotor_speed_input.rotor_omega
    )
    assert not np.allclose(
        data.truth_velocity_history_W[1],
        _step(configuration, configuration.rotor_speed_input.rotor_omega)[1],
    )
    assert configuration.initial_actual_rotor_omega is not None
    assert not np.allclose(
        data.truth_velocity_history_W[1],
        _step(configuration, configuration.initial_actual_rotor_omega)[1],
    )


def test_historical_ideal_actuator_uses_command_and_persists(tmp_path: Path) -> None:
    configuration = _configuration(truth_motor=False, nominal_motor=False, initial_actual=False)
    data = generate_run_artifact_data(configuration)
    for actual_field, expected_field in zip(
        _state_row(data, 1),
        _step(configuration, configuration.rotor_speed_input.rotor_omega),
        strict=True,
    ):
        np.testing.assert_allclose(actual_field, expected_field, rtol=0.0, atol=1e-12)
    manifest = RunManifest(configuration, _provenance())
    assert json.loads(encode_run_manifest(manifest))["schema"]["version"] == 1
    bound = save_run_directory(tmp_path / "historical", manifest, data)
    assert json.loads(encode_run_manifest(bound))["schema"]["version"] == 2
    loaded_manifest, loaded_data = load_run_directory(tmp_path / "historical")
    assert json.loads(encode_run_manifest(loaded_manifest))["schema"]["version"] == 2
    assert loaded_manifest.run_configuration.truth.rotors.minimum_rotor_omega is None
    assert loaded_manifest.run_configuration.nominal.rotors.minimum_rotor_omega is None
    assert loaded_manifest.run_configuration.initial_actual_rotor_omega is None
    for field in fields(RunArtifactData):
        np.testing.assert_array_equal(getattr(data, field.name), getattr(loaded_data, field.name))


@pytest.mark.parametrize(
    ("configuration", "expected_north_velocity", "expected_specific_force_body_x"),
    [
        (_quadratic_drag_configuration(), 1.8, -1.62),
        (
            _quadratic_drag_configuration(
                truth_wind_velocity_W=np.array([1.0, 0.0, 0.0]),
                nominal_wind_velocity_W=np.zeros(3),
            ),
            1.95,
            -0.45125,
        ),
        (
            _quadratic_drag_configuration(
                truth_quadratic_drag_coefficient_B=np.array([1.0, 0.0, 0.0]),
                nominal_quadratic_drag_coefficient_B=np.array([0.5, 0.0, 0.0]),
            ),
            1.6,
            -2.56,
        ),
    ],
)
def test_truth_environment_drives_propagation_and_completed_row_accelerometer(
    configuration: RunConfiguration,
    expected_north_velocity: float,
    expected_specific_force_body_x: float,
) -> None:
    data = generate_run_artifact_data(configuration)

    np.testing.assert_allclose(
        data.truth_velocity_history_W[1],
        np.array([expected_north_velocity, 0.0, 0.0]),
        rtol=0.0,
        atol=1e-15,
    )
    np.testing.assert_allclose(
        data.accelerometer_measurements_B[0],
        np.array([expected_specific_force_body_x, 0.0, -10.0]),
        rtol=0.0,
        atol=1e-15,
    )


@pytest.mark.parametrize("noise", [False, True])
@pytest.mark.parametrize(
    "altered_nominal_environment",
    [
        {
            "nominal_wind_velocity_W": np.array([1.0, 0.0, 0.0]),
        },
        {
            "nominal_quadratic_drag_coefficient_B": np.array([1.0, 0.0, 0.0]),
        },
    ],
)
def test_nominal_environment_is_exactly_isolated_from_all_artifact_arrays(
    noise: bool,
    altered_nominal_environment: dict[str, NDArray[np.float64]],
) -> None:
    baseline = generate_run_artifact_data(_quadratic_drag_configuration(noise=noise))
    altered = generate_run_artifact_data(
        _quadratic_drag_configuration(noise=noise, **altered_nominal_environment)
    )

    assert len(fields(RunArtifactData)) == 35
    for field in fields(RunArtifactData):
        np.testing.assert_array_equal(
            getattr(altered, field.name),
            getattr(baseline, field.name),
        )


@pytest.mark.parametrize(
    ("truth_motor", "nominal_motor", "initial_actual"),
    [(True, True, False), (True, False, True), (False, True, False)],
)
def test_partial_motor_configuration_rejected(
    truth_motor: bool, nominal_motor: bool, initial_actual: bool
) -> None:
    configuration = _configuration(
        truth_motor=truth_motor, nominal_motor=nominal_motor, initial_actual=initial_actual
    )
    with pytest.raises(
        ValueError,
        match="^run generation motor configuration must be either entirely omitted or complete$",
    ):
        generate_run_artifact_data(configuration)


def test_empty_acquisition_streams_preserve_shapes_and_persist(tmp_path: Path) -> None:
    configuration = _configuration()
    configuration = replace(
        configuration,
        numerics=RunNumerics(IntegrationMethod.EULER, 0.1, 1),
        sensor_schedules=SensorSchedules(
            accelerometer=SensorSchedule(0.2, 0.0),
            gyroscope=SensorSchedule(0.2, 0.0),
            local_position=SensorSchedule(0.2, 0.0),
            barometric_altitude=SensorSchedule(0.2, 0.0),
        ),
    )
    data = generate_run_artifact_data(configuration)
    for name in ("accelerometer", "gyroscope", "local_position", "barometric_altitude"):
        for suffix in (
            "sequence_index",
            "observation_index",
            "acquisition_time_s",
            "delivery_time_s",
            "delivered_at_truth_index",
        ):
            assert getattr(data, f"{name}_{suffix}").shape == (0,)
    assert data.accelerometer_measurements_B.shape == (0, 3)
    assert data.gyroscope_measurements_B.shape == (0, 3)
    assert data.local_position_measurements_W.shape == (0, 3)
    assert data.barometric_altitude_measurements.shape == (0,)
    assert data.delivery_sensor_id.shape == (0,)
    assert data.delivery_sequence_index.shape == (0,)
    assert data.delivery_observation_index.shape == (0,)
    np.testing.assert_array_equal(data.truth_time_s, [0.0, 0.1])
    assert data.truth_position_history_W.shape == (2, 3)
    assert data.accelerometer_bias_history_B.shape == (2, 3)
    assert data.gyroscope_bias_history_B.shape == (2, 3)
    np.testing.assert_array_equal(
        data.accelerometer_bias_history_B[0],
        configuration.truth.imu.initial_accelerometer_bias_B,
    )
    np.testing.assert_array_equal(
        data.gyroscope_bias_history_B[0], configuration.truth.imu.initial_gyroscope_bias_B
    )
    run_directory = tmp_path / "empty"
    save_run_directory(run_directory, RunManifest(configuration, _provenance()), data)
    _, loaded = load_run_directory(run_directory)
    for field in fields(RunArtifactData):
        np.testing.assert_array_equal(getattr(data, field.name), getattr(loaded, field.name))


def test_all_observations_pending_stay_in_streams_and_persist(tmp_path: Path) -> None:
    configuration = _configuration()
    configuration = replace(
        configuration,
        sensor_schedules=SensorSchedules(
            accelerometer=SensorSchedule(0.1, 10.0),
            gyroscope=SensorSchedule(0.2, 10.0),
            local_position=SensorSchedule(0.2, 10.0),
            barometric_altitude=SensorSchedule(0.1, 10.0),
        ),
    )
    data = generate_run_artifact_data(configuration)
    for name, count in (
        ("accelerometer", 4),
        ("gyroscope", 2),
        ("local_position", 2),
        ("barometric_altitude", 4),
    ):
        assert getattr(data, f"{name}_sequence_index").shape == (count,)
        assert np.all(getattr(data, f"{name}_delivered_at_truth_index") == -1)
    assert data.accelerometer_measurements_B.shape == (4, 3)
    assert data.gyroscope_measurements_B.shape == (2, 3)
    assert data.local_position_measurements_W.shape == (2, 3)
    assert data.barometric_altitude_measurements.shape == (4,)
    assert data.delivery_sensor_id.shape == (0,)
    assert data.delivery_sequence_index.shape == (0,)
    assert data.delivery_observation_index.shape == (0,)
    run_directory = tmp_path / "pending"
    save_run_directory(run_directory, RunManifest(configuration, _provenance()), data)
    _, loaded = load_run_directory(run_directory)
    for field in fields(RunArtifactData):
        np.testing.assert_array_equal(getattr(data, field.name), getattr(loaded, field.name))


def test_sensor_timing_delivery_order_and_zero_noise_values() -> None:
    configuration = _configuration()
    data = generate_run_artifact_data(configuration)
    np.testing.assert_array_equal(data.truth_time_s, np.arange(5) * 0.1)
    expected_streams = (
        ("accelerometer", [1, 2, 3, 4], 0.1, [2, 3, 4, -1]),
        ("gyroscope", [2, 4], 0.0, [2, 4]),
        ("local_position", [2, 4], 0.3, [-1, -1]),
        ("barometric_altitude", [1, 2, 3, 4], 0.0, [1, 2, 3, 4]),
    )
    for name, acquisition_rows, delay, delivered_at in expected_streams:
        expected_indices = np.arange(len(acquisition_rows))
        acquired = np.asarray(acquisition_rows) * 0.1
        delivered = acquired + delay
        np.testing.assert_array_equal(getattr(data, f"{name}_sequence_index"), expected_indices)
        np.testing.assert_array_equal(getattr(data, f"{name}_observation_index"), expected_indices)
        np.testing.assert_array_equal(getattr(data, f"{name}_acquisition_time_s"), acquired)
        np.testing.assert_array_equal(getattr(data, f"{name}_delivery_time_s"), delivered)
        np.testing.assert_array_equal(
            getattr(data, f"{name}_delivered_at_truth_index"), delivered_at
        )
    np.testing.assert_array_equal(data.delivery_sensor_id, [4, 1, 2, 4, 1, 4, 1, 2, 4])
    np.testing.assert_array_equal(data.delivery_sequence_index, [0, 0, 0, 1, 1, 2, 2, 1, 3])
    np.testing.assert_array_equal(data.delivery_observation_index, data.delivery_sequence_index)

    imu = configuration.truth.imu
    np.testing.assert_array_equal(
        data.accelerometer_bias_history_B, np.tile(imu.initial_accelerometer_bias_B, (5, 1))
    )
    np.testing.assert_array_equal(
        data.gyroscope_bias_history_B, np.tile(imu.initial_gyroscope_bias_B, (5, 1))
    )
    actual_speeds = _actual_speed_history(configuration)
    expected_accelerometer = []
    for index in range(1, 5):
        position_W, velocity_W, q_WB, omega_B = _state_row(data, index)
        truth = configuration.truth
        acceleration_W = rigid_body_state_derivative_from_rotor_speeds(
            position_W,
            velocity_W,
            q_WB,
            omega_B,
            actual_speeds[index],
            truth.rotors.rotor_positions_B,
            truth.rotors.rotor_spin_directions,
            truth.rigid_body.mass,
            truth.rigid_body.inertia_B,
            truth.world.gravity_acceleration,
            truth.rotors.thrust_coefficient,
            truth.rotors.moment_coefficient,
        )[1]
        ideal = ideal_accelerometer_specific_force_body(
            acceleration_W, rotation_matrix_body_to_world(q_WB), truth.world.gravity_acceleration
        )
        expected_accelerometer.append(ideal + imu.initial_accelerometer_bias_B)
    np.testing.assert_allclose(
        data.accelerometer_measurements_B, expected_accelerometer, rtol=0, atol=1e-12
    )
    np.testing.assert_allclose(
        data.gyroscope_measurements_B,
        data.truth_omega_history_B[[2, 4]] + imu.initial_gyroscope_bias_B,
        rtol=0,
        atol=1e-12,
    )
    position_sensors = configuration.truth.position_sensors
    np.testing.assert_allclose(
        data.local_position_measurements_W,
        data.truth_position_history_W[[2, 4]] + position_sensors.local_position_bias_W,
        rtol=0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        data.barometric_altitude_measurements,
        position_sensors.barometric_reference_altitude
        - data.truth_position_history_W[1:, 2]
        + position_sensors.barometric_altitude_bias,
        rtol=0,
        atol=1e-12,
    )


def test_seed_reproducibility_and_stream_separation() -> None:
    first = generate_run_artifact_data(_configuration(noise=True))
    second = generate_run_artifact_data(_configuration(noise=True))
    changed_seed = generate_run_artifact_data(_configuration(noise=True, root_seed=2027))
    assert len(fields(RunArtifactData)) == 35
    for field in fields(RunArtifactData):
        np.testing.assert_array_equal(getattr(first, field.name), getattr(second, field.name))
    for name in (
        "truth_time_s",
        "truth_position_history_W",
        "truth_velocity_history_W",
        "truth_q_history_WB",
        "truth_omega_history_B",
        "commanded_rotor_omega",
    ):
        np.testing.assert_array_equal(getattr(first, name), getattr(changed_seed, name))
    for name in (
        "accelerometer_bias_history_B",
        "gyroscope_bias_history_B",
        "accelerometer_measurements_B",
        "gyroscope_measurements_B",
        "local_position_measurements_W",
        "barometric_altitude_measurements",
    ):
        assert not np.array_equal(getattr(first, name), getattr(changed_seed, name))
    for field in fields(RunArtifactData):
        if field.name.endswith(("_index", "_time_s")) or field.name == "delivery_sensor_id":
            np.testing.assert_array_equal(
                getattr(first, field.name), getattr(changed_seed, field.name)
            )


def test_nominal_parameters_do_not_generate_truth_or_measurements() -> None:
    configuration = _configuration(noise=True)
    nominal = configuration.nominal
    changed_nominal = replace(
        nominal,
        rigid_body=replace(nominal.rigid_body, mass=2.0),
        rotors=replace(nominal.rotors, thrust_coefficient=2.0e-5),
        world=replace(nominal.world, gravity_acceleration=8.0),
        imu=replace(nominal.imu, initial_accelerometer_bias_B=np.array([1.0, 2.0, 3.0])),
        position_sensors=replace(
            nominal.position_sensors,
            local_position_bias_W=np.array([2.0, 3.0, 4.0]),
        ),
    )
    altered = replace(
        configuration,
        nominal=changed_nominal,
        declared_mismatches=tuple(
            DeclaredMismatch(path, "Nominal independence fixture")
            for path in (
                "rigid_body.mass",
                "rotors.thrust_coefficient",
                "world.gravity_acceleration",
                "imu.initial_accelerometer_bias_B",
                "position_sensors.local_position_bias_W",
            )
        ),
    )
    expected = generate_run_artifact_data(configuration)
    actual = generate_run_artifact_data(altered)
    for field in fields(RunArtifactData):
        np.testing.assert_array_equal(getattr(actual, field.name), getattr(expected, field.name))


def test_arrays_are_owned_immutable_and_configuration_unchanged() -> None:
    configuration = _configuration(noise=True)
    configuration_arrays = [
        configuration.initial_truth_state.position_W,
        configuration.initial_truth_state.velocity_W,
        configuration.initial_truth_state.q_WB,
        configuration.initial_truth_state.omega_B,
        configuration.rotor_speed_input.rotor_omega,
    ]
    assert configuration.initial_actual_rotor_omega is not None
    configuration_arrays.append(configuration.initial_actual_rotor_omega)
    for model in (configuration.truth, configuration.nominal):
        configuration_arrays.extend(
            (
                model.rigid_body.inertia_B,
                model.rigid_body.quadratic_drag_coefficient_B,
                model.rotors.rotor_positions_B,
                model.rotors.rotor_spin_directions,
                model.world.wind_velocity_W,
                model.imu.initial_accelerometer_bias_B,
                model.imu.accelerometer_noise_standard_deviation_B,
                model.imu.accelerometer_bias_random_walk_density_B,
                model.imu.initial_gyroscope_bias_B,
                model.imu.gyroscope_noise_standard_deviation_B,
                model.imu.gyroscope_bias_random_walk_density_B,
                model.position_sensors.local_position_bias_W,
                model.position_sensors.local_position_noise_standard_deviation_W,
            )
        )
    snapshots = [(array.copy(), array.flags.writeable) for array in configuration_arrays]
    data = generate_run_artifact_data(configuration)
    artifact_arrays = [getattr(data, field.name) for field in fields(RunArtifactData)]
    for field in fields(RunArtifactData):
        array = getattr(data, field.name)
        assert array.flags.owndata and array.flags.c_contiguous and not array.flags.writeable
        for config_array in configuration_arrays:
            assert not np.shares_memory(array, config_array)
    for index, array in enumerate(artifact_arrays):
        for other in artifact_arrays[index + 1 :]:
            assert not np.shares_memory(array, other)
    for array, (values, writeable) in zip(configuration_arrays, snapshots, strict=True):
        np.testing.assert_array_equal(array, values)
        assert array.flags.writeable is writeable


def _provenance() -> SoftwareProvenance:
    return SoftwareProvenance("0.1.0", "3.12.3", "2.1.0", "a" * 40, True)


def _environmental_replay_configuration(*, motorized: bool) -> RunConfiguration:
    """Build one noisy multi-step run with declared drag and wind mismatches."""
    configuration = _configuration(
        truth_motor=motorized,
        nominal_motor=motorized,
        initial_actual=motorized,
        noise=True,
        root_seed=20260922,
    )
    truth = replace(
        configuration.truth,
        rigid_body=replace(
            configuration.truth.rigid_body,
            quadratic_drag_coefficient_B=np.array([0.5, 0.25, 0.125]),
        ),
        world=replace(
            configuration.truth.world,
            wind_velocity_W=np.array([1.0, -0.5, 0.25]),
        ),
    )
    nominal = replace(
        configuration.nominal,
        rigid_body=replace(
            configuration.nominal.rigid_body,
            quadratic_drag_coefficient_B=np.array([0.4, 0.2, 0.1]),
        ),
        world=replace(
            configuration.nominal.world,
            wind_velocity_W=np.array([0.8, -0.4, 0.2]),
        ),
    )
    return replace(
        configuration,
        truth=truth,
        nominal=nominal,
        declared_mismatches=(
            DeclaredMismatch(
                "rigid_body.quadratic_drag_coefficient_B",
                "Environmental replay drag mismatch.",
            ),
            DeclaredMismatch(
                "world.wind_velocity_W",
                "Environmental replay wind mismatch.",
            ),
        ),
    )


def _run_configuration_arrays(configuration: RunConfiguration) -> list[NDArray[np.float64]]:
    """Return every array owned by one configuration for mutation checks."""
    arrays = [
        configuration.initial_truth_state.position_W,
        configuration.initial_truth_state.velocity_W,
        configuration.initial_truth_state.q_WB,
        configuration.initial_truth_state.omega_B,
        configuration.rotor_speed_input.rotor_omega,
    ]
    if configuration.initial_actual_rotor_omega is not None:
        arrays.append(configuration.initial_actual_rotor_omega)
    for model in (configuration.truth, configuration.nominal):
        arrays.extend(
            (
                model.rigid_body.inertia_B,
                model.rigid_body.quadratic_drag_coefficient_B,
                model.rotors.rotor_positions_B,
                model.rotors.rotor_spin_directions,
                model.world.wind_velocity_W,
                model.imu.initial_accelerometer_bias_B,
                model.imu.accelerometer_noise_standard_deviation_B,
                model.imu.accelerometer_bias_random_walk_density_B,
                model.imu.initial_gyroscope_bias_B,
                model.imu.gyroscope_noise_standard_deviation_B,
                model.imu.gyroscope_bias_random_walk_density_B,
                model.position_sensors.local_position_bias_W,
                model.position_sensors.local_position_noise_standard_deviation_W,
            )
        )
    return arrays


@pytest.mark.parametrize(
    (
        "truth_motor",
        "nominal_motor",
        "initial_actual",
        "unbound_version",
        "bound_version",
    ),
    [
        pytest.param(False, False, False, 1, 2, id="historical-v2"),
        pytest.param(True, True, True, 3, 4, id="motorized-v4"),
    ],
)
def test_loaded_run_configuration_replays_all_artifact_arrays_exactly(
    tmp_path: Path,
    truth_motor: bool,
    nominal_motor: bool,
    initial_actual: bool,
    unbound_version: int,
    bound_version: int,
) -> None:
    configuration = _configuration(
        truth_motor=truth_motor,
        nominal_motor=nominal_motor,
        initial_actual=initial_actual,
        noise=True,
    )
    original_data = generate_run_artifact_data(configuration)
    assert np.any(np.diff(original_data.accelerometer_bias_history_B, axis=0) != 0)
    assert np.any(np.diff(original_data.gyroscope_bias_history_B, axis=0) != 0)
    manifest = RunManifest(configuration, _provenance())
    assert json.loads(encode_run_manifest(manifest))["schema"]["version"] == unbound_version

    original_directory = tmp_path / "original"
    bound_manifest = save_run_directory(original_directory, manifest, original_data)
    assert json.loads(encode_run_manifest(bound_manifest))["schema"]["version"] == bound_version
    loaded_manifest, loaded_data = load_run_directory(original_directory)
    assert json.loads(encode_run_manifest(loaded_manifest))["schema"]["version"] == bound_version
    regenerated_data = generate_run_artifact_data(loaded_manifest.run_configuration)

    assert len(fields(RunArtifactData)) == 35
    for field in fields(RunArtifactData):
        original_array = getattr(original_data, field.name)
        np.testing.assert_array_equal(original_array, getattr(loaded_data, field.name))
        np.testing.assert_array_equal(original_array, getattr(regenerated_data, field.name))

    replayed_directory = tmp_path / "replayed"
    replayed_manifest = save_run_directory(replayed_directory, loaded_manifest, regenerated_data)
    original_manifest_bytes = (original_directory / "manifest.json").read_bytes()
    assert encode_run_manifest(replayed_manifest) == original_manifest_bytes
    assert (replayed_directory / "manifest.json").read_bytes() == original_manifest_bytes
    assert (replayed_directory / "data.npz").read_bytes() == (
        original_directory / "data.npz"
    ).read_bytes()


def test_motorized_persistence_round_trip_is_byte_identical(tmp_path: Path) -> None:
    configuration = _configuration(noise=True)
    data = generate_run_artifact_data(configuration)
    original_arrays = {
        field.name: getattr(data, field.name).copy() for field in fields(RunArtifactData)
    }
    manifest = RunManifest(configuration, _provenance())
    assert json.loads(encode_run_manifest(manifest))["schema"]["version"] == 3
    run_directory = tmp_path / "first"
    bound = save_run_directory(run_directory, manifest, data)
    assert manifest.data_npz_sha256 is None
    assert bound.data_npz_sha256 is not None
    assert json.loads(encode_run_manifest(bound))["schema"]["version"] == 4
    loaded_manifest, loaded_data = load_run_directory(run_directory)
    assert encode_run_manifest(loaded_manifest) == (run_directory / "manifest.json").read_bytes()
    for field in fields(RunArtifactData):
        np.testing.assert_array_equal(getattr(loaded_data, field.name), original_arrays[field.name])
        np.testing.assert_array_equal(getattr(data, field.name), original_arrays[field.name])
    rebound = save_run_directory(tmp_path / "second", loaded_manifest, loaded_data)
    assert encode_run_manifest(rebound) == (run_directory / "manifest.json").read_bytes()
    assert (tmp_path / "second" / "data.npz").read_bytes() == (
        run_directory / "data.npz"
    ).read_bytes()
    assert manifest.data_npz_sha256 is None


@pytest.mark.parametrize("motorized", [False, True], ids=["historical", "motorized"])
def test_environmental_run_persistence_and_replay_are_exact(
    tmp_path: Path, motorized: bool
) -> None:
    configuration = _environmental_replay_configuration(motorized=motorized)
    configuration_arrays = _run_configuration_arrays(configuration)
    configuration_snapshots = [
        (array.copy(), array.flags.writeable) for array in configuration_arrays
    ]
    original_data = generate_run_artifact_data(configuration)
    original_artifact_snapshots = {
        field.name: getattr(original_data, field.name).copy() for field in fields(RunArtifactData)
    }
    original_artifact_writeability = {
        field.name: getattr(original_data, field.name).flags.writeable
        for field in fields(RunArtifactData)
    }
    assert configuration.numerics.number_of_steps > 1
    assert np.any(configuration.truth.rigid_body.quadratic_drag_coefficient_B != 0.0)
    assert np.any(configuration.nominal.rigid_body.quadratic_drag_coefficient_B != 0.0)
    assert np.any(configuration.truth.world.wind_velocity_W != 0.0)
    assert np.any(configuration.nominal.world.wind_velocity_W != 0.0)
    assert np.any(configuration.truth.imu.accelerometer_noise_standard_deviation_B != 0.0)
    assert np.any(configuration.truth.imu.gyroscope_noise_standard_deviation_B != 0.0)
    assert np.any(configuration.truth.imu.accelerometer_bias_random_walk_density_B != 0.0)
    assert np.any(configuration.truth.imu.gyroscope_bias_random_walk_density_B != 0.0)
    assert np.any(np.diff(original_data.accelerometer_bias_history_B, axis=0) != 0.0)
    assert np.any(np.diff(original_data.gyroscope_bias_history_B, axis=0) != 0.0)

    manifest = RunManifest(configuration, _provenance())
    unbound_manifest_bytes = encode_run_manifest(manifest)
    unbound_document = json.loads(unbound_manifest_bytes)
    assert unbound_document["schema"]["version"] == 5
    assert manifest.data_npz_sha256 is None

    original_directory = tmp_path / "original"
    bound_manifest = save_run_directory(original_directory, manifest, original_data)
    manifest_bytes = (original_directory / "manifest.json").read_bytes()
    data_bytes = (original_directory / "data.npz").read_bytes()
    bound_document = json.loads(manifest_bytes)
    assert bound_manifest is not manifest
    assert bound_document["schema"]["version"] == 6
    assert set(original_directory.iterdir()) == {
        original_directory / "manifest.json",
        original_directory / "data.npz",
    }
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert bound_document["data_artifact"]["sha256"] == bound_manifest.data_npz_sha256

    loaded_manifest, loaded_data = load_run_directory(original_directory)
    loaded_configuration = loaded_manifest.run_configuration
    loaded_document = json.loads(encode_run_manifest(loaded_manifest))
    assert encode_run_manifest(loaded_manifest) == manifest_bytes
    assert loaded_document["run_configuration"] == unbound_document["run_configuration"]
    assert loaded_document["randomness"] == unbound_document["randomness"]
    assert loaded_document["software_provenance"] == unbound_document["software_provenance"]
    assert loaded_manifest.software_provenance == manifest.software_provenance
    assert loaded_configuration.root_seed == configuration.root_seed
    for field in fields(RunNumerics):
        assert getattr(loaded_configuration.numerics, field.name) == getattr(
            configuration.numerics, field.name
        )
    for sensor_name in (
        "accelerometer",
        "gyroscope",
        "local_position",
        "barometric_altitude",
    ):
        loaded_schedule = getattr(loaded_configuration.sensor_schedules, sensor_name)
        source_schedule = getattr(configuration.sensor_schedules, sensor_name)
        for field in fields(SensorSchedule):
            assert getattr(loaded_schedule, field.name) == getattr(source_schedule, field.name)
    assert tuple(
        (mismatch.parameter_path, mismatch.rationale)
        for mismatch in loaded_configuration.declared_mismatches
    ) == tuple(
        (mismatch.parameter_path, mismatch.rationale)
        for mismatch in configuration.declared_mismatches
    )
    assert loaded_configuration.truth.rigid_body.mass == configuration.truth.rigid_body.mass
    assert loaded_configuration.truth.world.gravity_acceleration == (
        configuration.truth.world.gravity_acceleration
    )
    assert loaded_configuration.nominal.rigid_body.mass == configuration.nominal.rigid_body.mass
    assert loaded_configuration.nominal.world.gravity_acceleration == (
        configuration.nominal.world.gravity_acceleration
    )
    for loaded_model, source_model in (
        (loaded_configuration.truth, configuration.truth),
        (loaded_configuration.nominal, configuration.nominal),
    ):
        np.testing.assert_array_equal(
            loaded_model.rigid_body.quadratic_drag_coefficient_B,
            source_model.rigid_body.quadratic_drag_coefficient_B,
        )
        np.testing.assert_array_equal(
            loaded_model.world.wind_velocity_W,
            source_model.world.wind_velocity_W,
        )
        for name in (
            "minimum_rotor_omega",
            "maximum_rotor_omega",
            "motor_time_constant_s",
        ):
            assert getattr(loaded_model.rotors, name) == getattr(source_model.rotors, name)
    if motorized:
        assert loaded_configuration.initial_actual_rotor_omega is not None
        assert configuration.initial_actual_rotor_omega is not None
        np.testing.assert_array_equal(
            loaded_configuration.initial_actual_rotor_omega,
            configuration.initial_actual_rotor_omega,
        )
    else:
        assert loaded_configuration.initial_actual_rotor_omega is None

    assert len(fields(RunArtifactData)) == 35
    assert all("actual_rotor" not in field.name for field in fields(RunArtifactData))
    with np.load(original_directory / "data.npz", allow_pickle=False) as archive:
        assert set(archive.files) == {field.name for field in fields(RunArtifactData)}
        assert all("actual_rotor" not in name for name in archive.files)

    regenerated_data = generate_run_artifact_data(loaded_configuration)
    for field in fields(RunArtifactData):
        original_array = getattr(original_data, field.name)
        loaded_array = getattr(loaded_data, field.name)
        regenerated_array = getattr(regenerated_data, field.name)
        np.testing.assert_array_equal(loaded_array, original_array)
        np.testing.assert_array_equal(regenerated_array, original_array)
        assert loaded_array.flags.owndata
        assert loaded_array.flags.c_contiguous
        assert not loaded_array.flags.writeable
        assert not np.shares_memory(loaded_array, original_array)
        assert not np.shares_memory(loaded_array, regenerated_array)

    loaded_environment_arrays = [
        loaded_configuration.truth.rigid_body.quadratic_drag_coefficient_B,
        loaded_configuration.nominal.rigid_body.quadratic_drag_coefficient_B,
        loaded_configuration.truth.world.wind_velocity_W,
        loaded_configuration.nominal.world.wind_velocity_W,
    ]
    source_environment_arrays = [
        configuration.truth.rigid_body.quadratic_drag_coefficient_B,
        configuration.nominal.rigid_body.quadratic_drag_coefficient_B,
        configuration.truth.world.wind_velocity_W,
        configuration.nominal.world.wind_velocity_W,
    ]
    for index, (loaded_array, source_array) in enumerate(
        zip(loaded_environment_arrays, source_environment_arrays, strict=True)
    ):
        assert loaded_array.dtype == np.dtype(np.float64)
        assert loaded_array.shape == (3,)
        assert loaded_array.flags.owndata
        assert loaded_array.flags.c_contiguous
        assert not loaded_array.flags.writeable
        assert not np.shares_memory(loaded_array, source_array)
        for other in loaded_environment_arrays[index + 1 :]:
            assert not np.shares_memory(loaded_array, other)

    replayed_directory = tmp_path / "replayed"
    replayed_manifest = save_run_directory(replayed_directory, loaded_manifest, regenerated_data)
    assert encode_run_manifest(replayed_manifest) == manifest_bytes
    assert (replayed_directory / "manifest.json").read_bytes() == manifest_bytes
    assert (replayed_directory / "data.npz").read_bytes() == data_bytes

    assert encode_run_manifest(manifest) == unbound_manifest_bytes
    assert manifest.data_npz_sha256 is None
    for array, (expected, writeable) in zip(
        configuration_arrays, configuration_snapshots, strict=True
    ):
        np.testing.assert_array_equal(array, expected)
        assert array.flags.writeable is writeable
    for field in fields(RunArtifactData):
        original_array = getattr(original_data, field.name)
        np.testing.assert_array_equal(original_array, original_artifact_snapshots[field.name])
        assert original_array.flags.writeable is original_artifact_writeability[field.name]


def test_generation_preflights_initial_rotation_before_random_streams(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    configuration = _configuration()
    initial = replace(
        configuration.initial_truth_state, q_WB=np.array([1.0 + 1.5e-12, 0.0, 0.0, 0.0])
    )
    configuration = replace(configuration, initial_truth_state=initial)

    def forbidden_streams(*args: object) -> None:
        pytest.fail("initial rotation validation must precede random-stream creation")

    monkeypatch.setattr(
        "quadrotor_math.run_generation.create_run_random_streams", forbidden_streams
    )
    with pytest.raises(ValueError, match="unit norm"):
        generate_run_artifact_data(configuration)


def test_generation_preserves_historical_zero_nominal_parameter_domain() -> None:
    configuration = _configuration()
    nominal = replace(
        configuration.nominal,
        world=replace(configuration.nominal.world, gravity_acceleration=0.0),
        rotors=replace(
            configuration.nominal.rotors, thrust_coefficient=0.0, moment_coefficient=0.0
        ),
    )
    changed = replace(
        configuration,
        nominal=nominal,
        declared_mismatches=tuple(
            DeclaredMismatch(path, "Nominal beliefs do not generate truth")
            for path in (
                "world.gravity_acceleration",
                "rotors.thrust_coefficient",
                "rotors.moment_coefficient",
            )
        ),
    )
    baseline = generate_run_artifact_data(configuration)
    actual = generate_run_artifact_data(changed)
    for field in fields(baseline):
        np.testing.assert_array_equal(getattr(actual, field.name), getattr(baseline, field.name))
