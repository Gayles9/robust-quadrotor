from dataclasses import replace

import numpy as np
import pytest

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


def test_rigid_body_parameters_take_read_only_owned_inertia_B() -> None:
    caller_inertia_B = np.diag(np.array([0.02, 0.03, 0.04], dtype=np.float64))
    expected_inertia_B = caller_inertia_B.copy()

    parameters = RigidBodyParameters(
        mass=1.5,
        inertia_B=caller_inertia_B,
    )
    caller_inertia_B[0, 0] = 99.0

    assert parameters.mass == 1.5
    np.testing.assert_array_equal(parameters.inertia_B, expected_inertia_B)
    assert parameters.inertia_B.dtype == np.float64
    assert not np.shares_memory(parameters.inertia_B, caller_inertia_B)
    assert not parameters.inertia_B.flags.writeable


@pytest.mark.parametrize(
    "non_finite_mass",
    [np.nan, np.inf, -np.inf],
)
def test_rigid_body_parameters_reject_non_finite_mass(
    non_finite_mass: float,
) -> None:
    with pytest.raises(ValueError, match="mass must be finite"):
        RigidBodyParameters(
            mass=non_finite_mass,
            inertia_B=np.eye(3, dtype=np.float64),
        )


@pytest.mark.parametrize(
    "non_positive_mass",
    [0.0, -1.0],
)
def test_rigid_body_parameters_reject_non_positive_mass(
    non_positive_mass: float,
) -> None:
    with pytest.raises(ValueError, match="mass must be positive"):
        RigidBodyParameters(
            mass=non_positive_mass,
            inertia_B=np.eye(3, dtype=np.float64),
        )


def test_rigid_body_parameters_reject_invalid_inertia_B_shape() -> None:
    with pytest.raises(
        ValueError,
        match=r"inertia_B must have shape \(3, 3\)",
    ):
        RigidBodyParameters(
            mass=1.5,
            inertia_B=np.ones((3, 1), dtype=np.float64),
        )


@pytest.mark.parametrize(
    "non_finite_value",
    [np.nan, np.inf, -np.inf],
)
def test_rigid_body_parameters_reject_non_finite_inertia_B(
    non_finite_value: float,
) -> None:
    inertia_B = np.eye(3, dtype=np.float64)
    inertia_B[1, 1] = non_finite_value

    with pytest.raises(
        ValueError,
        match="inertia_B must contain only finite values",
    ):
        RigidBodyParameters(
            mass=1.5,
            inertia_B=inertia_B,
        )


def test_rigid_body_parameters_reject_non_symmetric_inertia_B() -> None:
    inertia_B = np.array(
        [
            [0.02, 0.01, 0.0],
            [0.0, 0.03, 0.0],
            [0.0, 0.0, 0.04],
        ],
        dtype=np.float64,
    )

    with pytest.raises(ValueError, match="inertia_B must be symmetric"):
        RigidBodyParameters(
            mass=1.5,
            inertia_B=inertia_B,
        )


def test_rigid_body_parameters_reject_non_positive_definite_inertia_B() -> None:
    inertia_B = np.diag(np.array([0.02, 0.0, 0.04], dtype=np.float64))

    with pytest.raises(
        ValueError,
        match="inertia_B must be positive definite",
    ):
        RigidBodyParameters(
            mass=1.5,
            inertia_B=inertia_B,
        )


def test_rotor_parameters_take_read_only_owned_arrays() -> None:
    caller_rotor_positions_B = np.array(
        [
            [0.2, 0.2, 0.0],
            [0.2, -0.2, 0.0],
            [-0.2, -0.2, 0.0],
            [-0.2, 0.2, 0.0],
        ],
        dtype=np.float64,
    )
    caller_rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )
    expected_rotor_positions_B = caller_rotor_positions_B.copy()
    expected_rotor_spin_directions = caller_rotor_spin_directions.copy()

    parameters = RotorParameters(
        rotor_positions_B=caller_rotor_positions_B,
        rotor_spin_directions=caller_rotor_spin_directions,
        thrust_coefficient=1.2e-5,
        moment_coefficient=2.0e-7,
    )

    caller_rotor_positions_B[0, 0] = 99.0
    caller_rotor_spin_directions[0] = -99.0

    np.testing.assert_array_equal(
        parameters.rotor_positions_B,
        expected_rotor_positions_B,
    )
    np.testing.assert_array_equal(
        parameters.rotor_spin_directions,
        expected_rotor_spin_directions,
    )
    assert parameters.rotor_positions_B.dtype == np.float64
    assert parameters.rotor_spin_directions.dtype == np.float64
    assert not np.shares_memory(
        parameters.rotor_positions_B,
        caller_rotor_positions_B,
    )
    assert not np.shares_memory(
        parameters.rotor_spin_directions,
        caller_rotor_spin_directions,
    )
    assert not parameters.rotor_positions_B.flags.writeable
    assert not parameters.rotor_spin_directions.flags.writeable


def test_imu_parameters_take_read_only_owned_arrays() -> None:
    caller_initial_accelerometer_bias_B = np.array(
        [0.01, -0.02, 0.03],
        dtype=np.float64,
    )
    caller_accelerometer_noise_standard_deviation_B = np.array(
        [0.1, 0.2, 0.3],
        dtype=np.float64,
    )
    caller_accelerometer_bias_random_walk_density_B = np.array(
        [0.001, 0.002, 0.003],
        dtype=np.float64,
    )
    caller_initial_gyroscope_bias_B = np.array(
        [0.001, -0.002, 0.003],
        dtype=np.float64,
    )
    caller_gyroscope_noise_standard_deviation_B = np.array(
        [0.01, 0.02, 0.03],
        dtype=np.float64,
    )
    caller_gyroscope_bias_random_walk_density_B = np.array(
        [0.0001, 0.0002, 0.0003],
        dtype=np.float64,
    )

    caller_arrays = (
        caller_initial_accelerometer_bias_B,
        caller_accelerometer_noise_standard_deviation_B,
        caller_accelerometer_bias_random_walk_density_B,
        caller_initial_gyroscope_bias_B,
        caller_gyroscope_noise_standard_deviation_B,
        caller_gyroscope_bias_random_walk_density_B,
    )
    expected_arrays = tuple(array.copy() for array in caller_arrays)

    parameters = ImuParameters(
        initial_accelerometer_bias_B=caller_initial_accelerometer_bias_B,
        accelerometer_noise_standard_deviation_B=(caller_accelerometer_noise_standard_deviation_B),
        accelerometer_bias_random_walk_density_B=(caller_accelerometer_bias_random_walk_density_B),
        initial_gyroscope_bias_B=caller_initial_gyroscope_bias_B,
        gyroscope_noise_standard_deviation_B=(caller_gyroscope_noise_standard_deviation_B),
        gyroscope_bias_random_walk_density_B=(caller_gyroscope_bias_random_walk_density_B),
    )

    for caller_array in caller_arrays:
        caller_array[...] = 99.0

    stored_arrays = (
        parameters.initial_accelerometer_bias_B,
        parameters.accelerometer_noise_standard_deviation_B,
        parameters.accelerometer_bias_random_walk_density_B,
        parameters.initial_gyroscope_bias_B,
        parameters.gyroscope_noise_standard_deviation_B,
        parameters.gyroscope_bias_random_walk_density_B,
    )

    for stored_array, caller_array, expected_array in zip(
        stored_arrays,
        caller_arrays,
        expected_arrays,
        strict=True,
    ):
        np.testing.assert_array_equal(stored_array, expected_array)
        assert stored_array.dtype == np.float64
        assert not np.shares_memory(stored_array, caller_array)
        assert not stored_array.flags.writeable


def test_position_sensor_parameters_take_read_only_owned_arrays() -> None:
    caller_local_position_bias_W = np.array(
        [0.5, -0.25, 0.1],
        dtype=np.float64,
    )
    caller_local_position_noise_standard_deviation_W = np.array(
        [0.1, 0.2, 0.3],
        dtype=np.float64,
    )
    expected_local_position_bias_W = caller_local_position_bias_W.copy()
    expected_local_position_noise_standard_deviation_W = (
        caller_local_position_noise_standard_deviation_W.copy()
    )

    parameters = PositionSensorParameters(
        local_position_bias_W=caller_local_position_bias_W,
        local_position_noise_standard_deviation_W=(
            caller_local_position_noise_standard_deviation_W
        ),
        barometric_reference_altitude=125.0,
        barometric_altitude_bias=1.5,
        barometric_altitude_noise_standard_deviation=0.75,
    )

    caller_local_position_bias_W[...] = 99.0
    caller_local_position_noise_standard_deviation_W[...] = 99.0

    np.testing.assert_array_equal(
        parameters.local_position_bias_W,
        expected_local_position_bias_W,
    )
    np.testing.assert_array_equal(
        parameters.local_position_noise_standard_deviation_W,
        expected_local_position_noise_standard_deviation_W,
    )
    assert parameters.local_position_bias_W.dtype == np.float64
    assert parameters.local_position_noise_standard_deviation_W.dtype == np.float64
    assert not np.shares_memory(
        parameters.local_position_bias_W,
        caller_local_position_bias_W,
    )
    assert not np.shares_memory(
        parameters.local_position_noise_standard_deviation_W,
        caller_local_position_noise_standard_deviation_W,
    )
    assert not parameters.local_position_bias_W.flags.writeable
    assert not parameters.local_position_noise_standard_deviation_W.flags.writeable


def test_truth_and_nominal_configurations_remain_structurally_distinct() -> None:
    caller_truth_inertia_B = np.diag(np.array([0.02, 0.03, 0.04], dtype=np.float64))
    caller_nominal_inertia_B = caller_truth_inertia_B.copy()

    truth = TruthConfiguration(
        rigid_body=RigidBodyParameters(
            mass=1.5,
            inertia_B=caller_truth_inertia_B,
        ),
        rotors=RotorParameters(
            rotor_positions_B=np.zeros((4, 3), dtype=np.float64),
            rotor_spin_directions=np.array(
                [1.0, -1.0, 1.0, -1.0],
                dtype=np.float64,
            ),
            thrust_coefficient=1.2e-5,
            moment_coefficient=2.0e-7,
        ),
        world=WorldParameters(gravity_acceleration=9.81),
        imu=ImuParameters(
            initial_accelerometer_bias_B=np.zeros(3, dtype=np.float64),
            accelerometer_noise_standard_deviation_B=np.zeros(
                3,
                dtype=np.float64,
            ),
            accelerometer_bias_random_walk_density_B=np.zeros(
                3,
                dtype=np.float64,
            ),
            initial_gyroscope_bias_B=np.zeros(3, dtype=np.float64),
            gyroscope_noise_standard_deviation_B=np.zeros(
                3,
                dtype=np.float64,
            ),
            gyroscope_bias_random_walk_density_B=np.zeros(
                3,
                dtype=np.float64,
            ),
        ),
        position_sensors=PositionSensorParameters(
            local_position_bias_W=np.zeros(3, dtype=np.float64),
            local_position_noise_standard_deviation_W=np.zeros(
                3,
                dtype=np.float64,
            ),
            barometric_reference_altitude=125.0,
            barometric_altitude_bias=0.0,
            barometric_altitude_noise_standard_deviation=0.0,
        ),
    )
    nominal = NominalConfiguration(
        rigid_body=RigidBodyParameters(
            mass=1.4,
            inertia_B=caller_nominal_inertia_B,
        ),
        rotors=truth.rotors,
        world=truth.world,
        imu=truth.imu,
        position_sensors=truth.position_sensors,
    )

    caller_truth_inertia_B[0, 0] = 99.0
    caller_nominal_inertia_B[0, 0] = 88.0

    assert type(truth) is TruthConfiguration
    assert type(nominal) is NominalConfiguration
    assert truth.rigid_body.mass == 1.5
    assert nominal.rigid_body.mass == 1.4
    assert truth.rigid_body is not nominal.rigid_body
    assert not np.shares_memory(
        truth.rigid_body.inertia_B,
        nominal.rigid_body.inertia_B,
    )
    assert truth.rotors is nominal.rotors
    assert truth.world is nominal.world
    assert truth.imu is nominal.imu
    assert truth.position_sensors is nominal.position_sensors


def _valid_rotor_parameters_arguments() -> dict[str, object]:
    return {
        "rotor_positions_B": np.array(
            [
                [0.2, 0.2, 0.0],
                [0.2, -0.2, 0.0],
                [-0.2, -0.2, 0.0],
                [-0.2, 0.2, 0.0],
            ],
            dtype=np.float64,
        ),
        "rotor_spin_directions": np.array(
            [1.0, -1.0, 1.0, -1.0],
            dtype=np.float64,
        ),
        "thrust_coefficient": 1.2e-5,
        "moment_coefficient": 2.0e-7,
    }


def _valid_world_parameters_arguments() -> dict[str, object]:
    return {"gravity_acceleration": 9.81}


def _valid_imu_parameters_arguments() -> dict[str, object]:
    return {
        "initial_accelerometer_bias_B": np.zeros(3, dtype=np.float64),
        "accelerometer_noise_standard_deviation_B": np.zeros(
            3,
            dtype=np.float64,
        ),
        "accelerometer_bias_random_walk_density_B": np.zeros(
            3,
            dtype=np.float64,
        ),
        "initial_gyroscope_bias_B": np.zeros(3, dtype=np.float64),
        "gyroscope_noise_standard_deviation_B": np.zeros(
            3,
            dtype=np.float64,
        ),
        "gyroscope_bias_random_walk_density_B": np.zeros(
            3,
            dtype=np.float64,
        ),
    }


def _valid_position_sensor_parameters_arguments() -> dict[str, object]:
    return {
        "local_position_bias_W": np.zeros(3, dtype=np.float64),
        "local_position_noise_standard_deviation_W": np.zeros(
            3,
            dtype=np.float64,
        ),
        "barometric_reference_altitude": 125.0,
        "barometric_altitude_bias": 0.0,
        "barometric_altitude_noise_standard_deviation": 0.75,
    }


@pytest.mark.parametrize(
    ("field_name", "invalid_shape", "expected_message"),
    [
        (
            "rotor_positions_B",
            (4, 1),
            r"^rotor_positions_B must have shape \(4, 3\)$",
        ),
        (
            "rotor_spin_directions",
            (4, 1),
            r"^rotor_spin_directions must have shape \(4,\)$",
        ),
    ],
)
def test_rotor_parameters_reject_invalid_array_shape(
    field_name: str,
    invalid_shape: tuple[int, int],
    expected_message: str,
) -> None:
    arguments = _valid_rotor_parameters_arguments()
    arguments[field_name] = np.zeros(invalid_shape, dtype=np.float64)

    with pytest.raises(ValueError, match=expected_message):
        RotorParameters(**arguments)


@pytest.mark.parametrize(
    "field_name",
    ["rotor_positions_B", "rotor_spin_directions"],
)
@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_rotor_parameters_reject_non_finite_array_values(
    field_name: str,
    non_finite_value: float,
) -> None:
    arguments = _valid_rotor_parameters_arguments()
    invalid_array = np.array(arguments[field_name], dtype=np.float64, copy=True)
    invalid_array.flat[0] = non_finite_value
    arguments[field_name] = invalid_array

    with pytest.raises(
        ValueError,
        match=rf"^{field_name} must contain only finite values$",
    ):
        RotorParameters(**arguments)


@pytest.mark.parametrize("invalid_spin_direction", [0.0, 2.0])
def test_rotor_parameters_reject_invalid_spin_directions(
    invalid_spin_direction: float,
) -> None:
    arguments = _valid_rotor_parameters_arguments()
    invalid_directions = np.array(
        arguments["rotor_spin_directions"],
        dtype=np.float64,
        copy=True,
    )
    invalid_directions[0] = invalid_spin_direction
    arguments["rotor_spin_directions"] = invalid_directions

    with pytest.raises(
        ValueError,
        match=r"^rotor_spin_directions must contain only -1.0 or 1.0$",
    ):
        RotorParameters(**arguments)


@pytest.mark.parametrize(
    "field_name",
    ["thrust_coefficient", "moment_coefficient"],
)
@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_rotor_parameters_reject_non_finite_coefficients(
    field_name: str,
    non_finite_value: float,
) -> None:
    arguments = _valid_rotor_parameters_arguments()
    arguments[field_name] = non_finite_value

    with pytest.raises(ValueError, match=rf"^{field_name} must be finite$"):
        RotorParameters(**arguments)


@pytest.mark.parametrize(
    "field_name",
    ["thrust_coefficient", "moment_coefficient"],
)
def test_rotor_parameters_reject_negative_coefficients(field_name: str) -> None:
    arguments = _valid_rotor_parameters_arguments()
    arguments[field_name] = -1.0

    with pytest.raises(ValueError, match=rf"^{field_name} must be nonnegative$"):
        RotorParameters(**arguments)


@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_world_parameters_reject_non_finite_gravity_acceleration(
    non_finite_value: float,
) -> None:
    arguments = _valid_world_parameters_arguments()
    arguments["gravity_acceleration"] = non_finite_value

    with pytest.raises(
        ValueError,
        match=r"^gravity_acceleration must be finite$",
    ):
        WorldParameters(**arguments)


def test_world_parameters_reject_negative_gravity_acceleration() -> None:
    arguments = _valid_world_parameters_arguments()
    arguments["gravity_acceleration"] = -1.0

    with pytest.raises(
        ValueError,
        match=r"^gravity_acceleration must be nonnegative$",
    ):
        WorldParameters(**arguments)


@pytest.mark.parametrize(
    "field_name",
    [
        "initial_accelerometer_bias_B",
        "accelerometer_noise_standard_deviation_B",
        "accelerometer_bias_random_walk_density_B",
        "initial_gyroscope_bias_B",
        "gyroscope_noise_standard_deviation_B",
        "gyroscope_bias_random_walk_density_B",
    ],
)
def test_imu_parameters_reject_invalid_array_shape(field_name: str) -> None:
    arguments = _valid_imu_parameters_arguments()
    arguments[field_name] = np.zeros((3, 1), dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=rf"^{field_name} must have shape \(3,\)$",
    ):
        ImuParameters(**arguments)


@pytest.mark.parametrize(
    "field_name",
    [
        "initial_accelerometer_bias_B",
        "accelerometer_noise_standard_deviation_B",
        "accelerometer_bias_random_walk_density_B",
        "initial_gyroscope_bias_B",
        "gyroscope_noise_standard_deviation_B",
        "gyroscope_bias_random_walk_density_B",
    ],
)
@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_imu_parameters_reject_non_finite_array_values(
    field_name: str,
    non_finite_value: float,
) -> None:
    arguments = _valid_imu_parameters_arguments()
    invalid_array = np.array(arguments[field_name], dtype=np.float64, copy=True)
    invalid_array[0] = non_finite_value
    arguments[field_name] = invalid_array

    with pytest.raises(
        ValueError,
        match=rf"^{field_name} must contain only finite values$",
    ):
        ImuParameters(**arguments)


@pytest.mark.parametrize(
    "field_name",
    [
        "accelerometer_noise_standard_deviation_B",
        "accelerometer_bias_random_walk_density_B",
        "gyroscope_noise_standard_deviation_B",
        "gyroscope_bias_random_walk_density_B",
    ],
)
def test_imu_parameters_reject_negative_noise_or_density(field_name: str) -> None:
    arguments = _valid_imu_parameters_arguments()
    invalid_array = np.array(arguments[field_name], dtype=np.float64, copy=True)
    invalid_array[0] = -0.1
    arguments[field_name] = invalid_array

    with pytest.raises(
        ValueError,
        match=rf"^{field_name} must be nonnegative$",
    ):
        ImuParameters(**arguments)


@pytest.mark.parametrize(
    "field_name",
    ["local_position_bias_W", "local_position_noise_standard_deviation_W"],
)
def test_position_sensor_parameters_reject_invalid_array_shape(
    field_name: str,
) -> None:
    arguments = _valid_position_sensor_parameters_arguments()
    arguments[field_name] = np.zeros((3, 1), dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=rf"^{field_name} must have shape \(3,\)$",
    ):
        PositionSensorParameters(**arguments)


@pytest.mark.parametrize(
    "field_name",
    ["local_position_bias_W", "local_position_noise_standard_deviation_W"],
)
@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_position_sensor_parameters_reject_non_finite_array_values(
    field_name: str,
    non_finite_value: float,
) -> None:
    arguments = _valid_position_sensor_parameters_arguments()
    invalid_array = np.array(arguments[field_name], dtype=np.float64, copy=True)
    invalid_array[0] = non_finite_value
    arguments[field_name] = invalid_array

    with pytest.raises(
        ValueError,
        match=rf"^{field_name} must contain only finite values$",
    ):
        PositionSensorParameters(**arguments)


def test_position_sensor_parameters_reject_negative_local_position_noise() -> None:
    arguments = _valid_position_sensor_parameters_arguments()
    invalid_noise = np.array(
        arguments["local_position_noise_standard_deviation_W"],
        dtype=np.float64,
        copy=True,
    )
    invalid_noise[0] = -0.1
    arguments["local_position_noise_standard_deviation_W"] = invalid_noise

    with pytest.raises(
        ValueError,
        match=r"^local_position_noise_standard_deviation_W must be nonnegative$",
    ):
        PositionSensorParameters(**arguments)


@pytest.mark.parametrize(
    "field_name",
    [
        "barometric_reference_altitude",
        "barometric_altitude_bias",
        "barometric_altitude_noise_standard_deviation",
    ],
)
@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_position_sensor_parameters_reject_non_finite_barometric_values(
    field_name: str,
    non_finite_value: float,
) -> None:
    arguments = _valid_position_sensor_parameters_arguments()
    arguments[field_name] = non_finite_value

    with pytest.raises(ValueError, match=rf"^{field_name} must be finite$"):
        PositionSensorParameters(**arguments)


def test_position_sensor_parameters_reject_negative_barometric_noise() -> None:
    arguments = _valid_position_sensor_parameters_arguments()
    arguments["barometric_altitude_noise_standard_deviation"] = -0.1

    with pytest.raises(
        ValueError,
        match=r"^barometric_altitude_noise_standard_deviation must be nonnegative$",
    ):
        PositionSensorParameters(**arguments)


def _valid_truth_configuration_for_run() -> TruthConfiguration:
    return TruthConfiguration(
        rigid_body=RigidBodyParameters(
            mass=1.5,
            inertia_B=np.diag(np.array([0.02, 0.03, 0.04], dtype=np.float64)),
        ),
        rotors=RotorParameters(
            rotor_positions_B=np.array(
                [
                    [0.2, 0.2, 0.0],
                    [0.2, -0.2, 0.0],
                    [-0.2, -0.2, 0.0],
                    [-0.2, 0.2, 0.0],
                ],
                dtype=np.float64,
            ),
            rotor_spin_directions=np.array(
                [1.0, -1.0, 1.0, -1.0],
                dtype=np.float64,
            ),
            thrust_coefficient=1.2e-5,
            moment_coefficient=2.0e-7,
        ),
        world=WorldParameters(gravity_acceleration=9.81),
        imu=ImuParameters(
            initial_accelerometer_bias_B=np.zeros(3, dtype=np.float64),
            accelerometer_noise_standard_deviation_B=np.zeros(
                3,
                dtype=np.float64,
            ),
            accelerometer_bias_random_walk_density_B=np.zeros(
                3,
                dtype=np.float64,
            ),
            initial_gyroscope_bias_B=np.zeros(3, dtype=np.float64),
            gyroscope_noise_standard_deviation_B=np.zeros(
                3,
                dtype=np.float64,
            ),
            gyroscope_bias_random_walk_density_B=np.zeros(
                3,
                dtype=np.float64,
            ),
        ),
        position_sensors=PositionSensorParameters(
            local_position_bias_W=np.zeros(3, dtype=np.float64),
            local_position_noise_standard_deviation_W=np.zeros(
                3,
                dtype=np.float64,
            ),
            barometric_reference_altitude=125.0,
            barometric_altitude_bias=0.0,
            barometric_altitude_noise_standard_deviation=0.0,
        ),
    )


def test_integration_method_names_are_manifest_stable() -> None:
    assert {method.name: method.value for method in IntegrationMethod} == {
        "EULER": "euler",
        "PROJECTED_RK4": "projected_rk4",
    }


def test_rigid_body_initial_state_takes_read_only_owned_arrays() -> None:
    caller_position_W = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    caller_velocity_W = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    caller_q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    caller_omega_B = np.array([0.01, 0.02, 0.03], dtype=np.float64)

    caller_arrays = (
        caller_position_W,
        caller_velocity_W,
        caller_q_WB,
        caller_omega_B,
    )
    expected_arrays = tuple(array.copy() for array in caller_arrays)

    initial_state = RigidBodyInitialState(
        position_W=caller_position_W,
        velocity_W=caller_velocity_W,
        q_WB=caller_q_WB,
        omega_B=caller_omega_B,
    )

    for caller_array in caller_arrays:
        caller_array[...] = 99.0

    stored_arrays = (
        initial_state.position_W,
        initial_state.velocity_W,
        initial_state.q_WB,
        initial_state.omega_B,
    )

    for stored_array, caller_array, expected_array in zip(
        stored_arrays,
        caller_arrays,
        expected_arrays,
        strict=True,
    ):
        np.testing.assert_array_equal(stored_array, expected_array)
        assert stored_array.dtype == np.float64
        assert not np.shares_memory(stored_array, caller_array)
        assert not stored_array.flags.writeable


def test_constant_rotor_speed_input_takes_read_only_owned_rotor_omega() -> None:
    caller_rotor_omega = np.array(
        [510.0, 520.0, 530.0, 540.0],
        dtype=np.float64,
    )
    expected_rotor_omega = caller_rotor_omega.copy()

    rotor_speed_input = ConstantRotorSpeedInput(
        rotor_omega=caller_rotor_omega,
    )
    caller_rotor_omega[...] = 99.0

    np.testing.assert_array_equal(
        rotor_speed_input.rotor_omega,
        expected_rotor_omega,
    )
    assert rotor_speed_input.rotor_omega.dtype == np.float64
    assert not np.shares_memory(
        rotor_speed_input.rotor_omega,
        caller_rotor_omega,
    )
    assert not rotor_speed_input.rotor_omega.flags.writeable


def test_sensor_schedules_retain_explicit_channel_schedules() -> None:
    accelerometer_schedule = SensorSchedule(
        sample_period_s=0.01,
        delivery_delay_s=0.0,
    )
    gyroscope_schedule = SensorSchedule(
        sample_period_s=0.005,
        delivery_delay_s=0.0,
    )
    local_position_schedule = SensorSchedule(
        sample_period_s=0.1,
        delivery_delay_s=0.02,
    )
    barometric_altitude_schedule = SensorSchedule(
        sample_period_s=0.05,
        delivery_delay_s=0.01,
    )

    schedules = SensorSchedules(
        accelerometer=accelerometer_schedule,
        gyroscope=gyroscope_schedule,
        local_position=local_position_schedule,
        barometric_altitude=barometric_altitude_schedule,
    )

    assert schedules.accelerometer is accelerometer_schedule
    assert schedules.gyroscope is gyroscope_schedule
    assert schedules.local_position is local_position_schedule
    assert schedules.barometric_altitude is barometric_altitude_schedule


def test_run_configuration_groups_inputs_and_owns_mismatch_sequence() -> None:
    truth = _valid_truth_configuration_for_run()
    nominal = NominalConfiguration(
        rigid_body=RigidBodyParameters(
            mass=1.4,
            inertia_B=truth.rigid_body.inertia_B,
        ),
        rotors=truth.rotors,
        world=truth.world,
        imu=truth.imu,
        position_sensors=truth.position_sensors,
    )
    initial_truth_state = RigidBodyInitialState(
        position_W=np.zeros(3, dtype=np.float64),
        velocity_W=np.zeros(3, dtype=np.float64),
        q_WB=np.array(
            [1.0, 0.0, 0.0, 0.0],
            dtype=np.float64,
        ),
        omega_B=np.zeros(3, dtype=np.float64),
    )
    numerics = RunNumerics(
        integration_method=IntegrationMethod.PROJECTED_RK4,
        truth_time_step_s=0.005,
        number_of_steps=1000,
    )
    sensor_schedules = SensorSchedules(
        accelerometer=SensorSchedule(
            sample_period_s=0.01,
            delivery_delay_s=0.0,
        ),
        gyroscope=SensorSchedule(
            sample_period_s=0.005,
            delivery_delay_s=0.0,
        ),
        local_position=SensorSchedule(
            sample_period_s=0.1,
            delivery_delay_s=0.02,
        ),
        barometric_altitude=SensorSchedule(
            sample_period_s=0.05,
            delivery_delay_s=0.01,
        ),
    )
    rotor_speed_input = ConstantRotorSpeedInput(
        rotor_omega=np.full(4, 520.0, dtype=np.float64),
    )
    mismatch = DeclaredMismatch(
        parameter_path="rigid_body.mass",
        rationale="Exercise an intentional mass-model mismatch.",
    )
    caller_declared_mismatches = [mismatch]

    configuration = RunConfiguration(
        truth=truth,
        nominal=nominal,
        initial_truth_state=initial_truth_state,
        numerics=numerics,
        sensor_schedules=sensor_schedules,
        rotor_speed_input=rotor_speed_input,
        root_seed=0x0123456789ABCDEF0123456789ABCDEF,
        declared_mismatches=caller_declared_mismatches,
    )
    caller_declared_mismatches.append(
        DeclaredMismatch(
            parameter_path="world.gravity_acceleration",
            rationale="Caller-owned list mutation must not alter configuration.",
        )
    )

    assert configuration.truth is truth
    assert configuration.nominal is nominal
    assert configuration.initial_truth_state is initial_truth_state
    assert configuration.numerics is numerics
    assert configuration.sensor_schedules is sensor_schedules
    assert configuration.rotor_speed_input is rotor_speed_input
    assert configuration.root_seed == 0x0123456789ABCDEF0123456789ABCDEF
    assert configuration.declared_mismatches == (mismatch,)
    assert isinstance(configuration.declared_mismatches, tuple)


def _valid_run_configuration_arguments() -> dict[str, object]:
    truth = _valid_truth_configuration_for_run()
    nominal = NominalConfiguration(
        rigid_body=RigidBodyParameters(
            mass=1.4,
            inertia_B=truth.rigid_body.inertia_B,
        ),
        rotors=truth.rotors,
        world=truth.world,
        imu=truth.imu,
        position_sensors=truth.position_sensors,
    )

    return {
        "truth": truth,
        "nominal": nominal,
        "initial_truth_state": RigidBodyInitialState(
            position_W=np.zeros(3, dtype=np.float64),
            velocity_W=np.zeros(3, dtype=np.float64),
            q_WB=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            omega_B=np.zeros(3, dtype=np.float64),
        ),
        "numerics": RunNumerics(
            integration_method=IntegrationMethod.PROJECTED_RK4,
            truth_time_step_s=0.005,
            number_of_steps=1000,
        ),
        "sensor_schedules": SensorSchedules(
            accelerometer=SensorSchedule(
                sample_period_s=0.01,
                delivery_delay_s=0.0,
            ),
            gyroscope=SensorSchedule(
                sample_period_s=0.005,
                delivery_delay_s=0.0,
            ),
            local_position=SensorSchedule(
                sample_period_s=0.1,
                delivery_delay_s=0.02,
            ),
            barometric_altitude=SensorSchedule(
                sample_period_s=0.05,
                delivery_delay_s=0.01,
            ),
        ),
        "rotor_speed_input": ConstantRotorSpeedInput(
            rotor_omega=np.full(4, 520.0, dtype=np.float64),
        ),
        "root_seed": 0x0123456789ABCDEF0123456789ABCDEF,
        "declared_mismatches": [
            DeclaredMismatch(
                parameter_path="rigid_body.mass",
                rationale="Exercise an intentional mass-model mismatch.",
            )
        ],
    }


@pytest.mark.parametrize(
    ("field_name", "valid_shape", "invalid_shape", "expected_message"),
    [
        (
            "position_W",
            (3,),
            (3, 1),
            r"^position_W must have shape \(3,\)$",
        ),
        (
            "velocity_W",
            (3,),
            (3, 1),
            r"^velocity_W must have shape \(3,\)$",
        ),
        (
            "q_WB",
            (4,),
            (4, 1),
            r"^q_WB must have shape \(4,\)$",
        ),
        (
            "omega_B",
            (3,),
            (3, 1),
            r"^omega_B must have shape \(3,\)$",
        ),
    ],
)
def test_rigid_body_initial_state_rejects_invalid_array_shape(
    field_name: str,
    valid_shape: tuple[int],
    invalid_shape: tuple[int, int],
    expected_message: str,
) -> None:
    arguments: dict[str, object] = {
        "position_W": np.zeros(3, dtype=np.float64),
        "velocity_W": np.zeros(3, dtype=np.float64),
        "q_WB": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        "omega_B": np.zeros(3, dtype=np.float64),
    }
    assert np.shape(arguments[field_name]) == valid_shape
    arguments[field_name] = np.zeros(invalid_shape, dtype=np.float64)

    with pytest.raises(ValueError, match=expected_message):
        RigidBodyInitialState(**arguments)


@pytest.mark.parametrize(
    "field_name",
    ["position_W", "velocity_W", "q_WB", "omega_B"],
)
@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_rigid_body_initial_state_rejects_non_finite_array_values(
    field_name: str,
    non_finite_value: float,
) -> None:
    arguments: dict[str, object] = {
        "position_W": np.zeros(3, dtype=np.float64),
        "velocity_W": np.zeros(3, dtype=np.float64),
        "q_WB": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        "omega_B": np.zeros(3, dtype=np.float64),
    }
    invalid_array = np.array(arguments[field_name], dtype=np.float64, copy=True)
    invalid_array[0] = non_finite_value
    arguments[field_name] = invalid_array

    with pytest.raises(
        ValueError,
        match=rf"^{field_name} must contain only finite values$",
    ):
        RigidBodyInitialState(**arguments)


def test_rigid_body_initial_state_rejects_non_unit_quaternion() -> None:
    with pytest.raises(ValueError, match=r"^q_WB must have unit norm$"):
        RigidBodyInitialState(
            position_W=np.zeros(3, dtype=np.float64),
            velocity_W=np.zeros(3, dtype=np.float64),
            q_WB=np.array([2.0, 0.0, 0.0, 0.0], dtype=np.float64),
            omega_B=np.zeros(3, dtype=np.float64),
        )


def test_run_numerics_rejects_non_enum_integration_method() -> None:
    with pytest.raises(
        TypeError,
        match=r"^integration_method must be an IntegrationMethod$",
    ):
        RunNumerics(
            integration_method="projected_rk4",
            truth_time_step_s=0.005,
            number_of_steps=1000,
        )


@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_run_numerics_rejects_non_finite_truth_time_step(
    non_finite_value: float,
) -> None:
    with pytest.raises(ValueError, match=r"^truth_time_step_s must be finite$"):
        RunNumerics(
            integration_method=IntegrationMethod.PROJECTED_RK4,
            truth_time_step_s=non_finite_value,
            number_of_steps=1000,
        )


@pytest.mark.parametrize("non_positive_value", [0.0, -0.005])
def test_run_numerics_rejects_non_positive_truth_time_step(
    non_positive_value: float,
) -> None:
    with pytest.raises(ValueError, match=r"^truth_time_step_s must be positive$"):
        RunNumerics(
            integration_method=IntegrationMethod.PROJECTED_RK4,
            truth_time_step_s=non_positive_value,
            number_of_steps=1000,
        )


@pytest.mark.parametrize("invalid_value", [True, 1000.0])
def test_run_numerics_rejects_non_integer_number_of_steps(
    invalid_value: object,
) -> None:
    with pytest.raises(
        TypeError,
        match=r"^number_of_steps must be a non-Boolean integer$",
    ):
        RunNumerics(
            integration_method=IntegrationMethod.PROJECTED_RK4,
            truth_time_step_s=0.005,
            number_of_steps=invalid_value,
        )


@pytest.mark.parametrize("non_positive_value", [0, -1])
def test_run_numerics_rejects_non_positive_number_of_steps(
    non_positive_value: int,
) -> None:
    with pytest.raises(ValueError, match=r"^number_of_steps must be positive$"):
        RunNumerics(
            integration_method=IntegrationMethod.PROJECTED_RK4,
            truth_time_step_s=0.005,
            number_of_steps=non_positive_value,
        )


@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_sensor_schedule_rejects_non_finite_sample_period(
    non_finite_value: float,
) -> None:
    with pytest.raises(ValueError, match=r"^sample_period_s must be finite$"):
        SensorSchedule(
            sample_period_s=non_finite_value,
            delivery_delay_s=0.0,
        )


@pytest.mark.parametrize("non_positive_value", [0.0, -0.01])
def test_sensor_schedule_rejects_non_positive_sample_period(
    non_positive_value: float,
) -> None:
    with pytest.raises(ValueError, match=r"^sample_period_s must be positive$"):
        SensorSchedule(
            sample_period_s=non_positive_value,
            delivery_delay_s=0.0,
        )


@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_sensor_schedule_rejects_non_finite_delivery_delay(
    non_finite_value: float,
) -> None:
    with pytest.raises(ValueError, match=r"^delivery_delay_s must be finite$"):
        SensorSchedule(
            sample_period_s=0.01,
            delivery_delay_s=non_finite_value,
        )


def test_sensor_schedule_rejects_negative_delivery_delay() -> None:
    with pytest.raises(ValueError, match=r"^delivery_delay_s must be nonnegative$"):
        SensorSchedule(
            sample_period_s=0.01,
            delivery_delay_s=-0.01,
        )


def test_constant_rotor_speed_input_rejects_invalid_shape() -> None:
    with pytest.raises(ValueError, match=r"^rotor_omega must have shape \(4,\)$"):
        ConstantRotorSpeedInput(
            rotor_omega=np.zeros((4, 1), dtype=np.float64),
        )


@pytest.mark.parametrize("non_finite_value", [np.nan, np.inf, -np.inf])
def test_constant_rotor_speed_input_rejects_non_finite_values(
    non_finite_value: float,
) -> None:
    rotor_omega = np.full(4, 520.0, dtype=np.float64)
    rotor_omega[0] = non_finite_value

    with pytest.raises(
        ValueError,
        match=r"^rotor_omega must contain only finite values$",
    ):
        ConstantRotorSpeedInput(rotor_omega=rotor_omega)


def test_constant_rotor_speed_input_rejects_negative_value() -> None:
    rotor_omega = np.full(4, 520.0, dtype=np.float64)
    rotor_omega[0] = -1.0

    with pytest.raises(ValueError, match=r"^rotor_omega must be nonnegative$"):
        ConstantRotorSpeedInput(rotor_omega=rotor_omega)


@pytest.mark.parametrize("field_name", ["parameter_path", "rationale"])
@pytest.mark.parametrize("empty_value", ["", "   "])
def test_declared_mismatch_rejects_empty_string_fields(
    field_name: str,
    empty_value: str,
) -> None:
    arguments = {
        "parameter_path": "rigid_body.mass",
        "rationale": "Exercise an intentional mass-model mismatch.",
    }
    arguments[field_name] = empty_value

    with pytest.raises(ValueError, match=rf"^{field_name} must be nonempty$"):
        DeclaredMismatch(**arguments)


@pytest.mark.parametrize("invalid_value", [True, 1.5])
def test_run_configuration_rejects_non_integer_root_seed(
    invalid_value: object,
) -> None:
    arguments = _valid_run_configuration_arguments()
    arguments["root_seed"] = invalid_value

    with pytest.raises(
        TypeError,
        match=r"^root_seed must be a non-Boolean integer$",
    ):
        RunConfiguration(**arguments)


@pytest.mark.parametrize("out_of_range_value", [-1, 2**128])
def test_run_configuration_rejects_out_of_range_root_seed(
    out_of_range_value: int,
) -> None:
    arguments = _valid_run_configuration_arguments()
    arguments["root_seed"] = out_of_range_value

    with pytest.raises(
        ValueError,
        match=r"^root_seed must be in \[0, 2\*\*128\)$",
    ):
        RunConfiguration(**arguments)


def test_run_configuration_rejects_non_declared_mismatch_item() -> None:
    arguments = _valid_run_configuration_arguments()
    arguments["declared_mismatches"] = ["rigid_body.mass"]

    with pytest.raises(
        TypeError,
        match=r"^declared_mismatches must contain only DeclaredMismatch values$",
    ):
        RunConfiguration(**arguments)


def _matching_run_configuration_arguments() -> dict[str, object]:
    arguments = _valid_run_configuration_arguments()
    truth = arguments["truth"]
    assert isinstance(truth, TruthConfiguration)

    arguments["nominal"] = NominalConfiguration(
        rigid_body=RigidBodyParameters(
            mass=truth.rigid_body.mass,
            inertia_B=truth.rigid_body.inertia_B,
        ),
        rotors=RotorParameters(
            rotor_positions_B=truth.rotors.rotor_positions_B,
            rotor_spin_directions=truth.rotors.rotor_spin_directions,
            thrust_coefficient=truth.rotors.thrust_coefficient,
            moment_coefficient=truth.rotors.moment_coefficient,
        ),
        world=WorldParameters(
            gravity_acceleration=truth.world.gravity_acceleration,
        ),
        imu=ImuParameters(
            initial_accelerometer_bias_B=truth.imu.initial_accelerometer_bias_B,
            accelerometer_noise_standard_deviation_B=(
                truth.imu.accelerometer_noise_standard_deviation_B
            ),
            accelerometer_bias_random_walk_density_B=(
                truth.imu.accelerometer_bias_random_walk_density_B
            ),
            initial_gyroscope_bias_B=truth.imu.initial_gyroscope_bias_B,
            gyroscope_noise_standard_deviation_B=(truth.imu.gyroscope_noise_standard_deviation_B),
            gyroscope_bias_random_walk_density_B=(truth.imu.gyroscope_bias_random_walk_density_B),
        ),
        position_sensors=PositionSensorParameters(
            local_position_bias_W=truth.position_sensors.local_position_bias_W,
            local_position_noise_standard_deviation_W=(
                truth.position_sensors.local_position_noise_standard_deviation_W
            ),
            barometric_reference_altitude=(truth.position_sensors.barometric_reference_altitude),
            barometric_altitude_bias=truth.position_sensors.barometric_altitude_bias,
            barometric_altitude_noise_standard_deviation=(
                truth.position_sensors.barometric_altitude_noise_standard_deviation
            ),
        ),
    )
    arguments["declared_mismatches"] = []
    return arguments


def _nominal_with_one_parameter_difference(
    nominal: NominalConfiguration,
    parameter_path: str,
) -> NominalConfiguration:
    if parameter_path == "rigid_body.mass":
        return replace(
            nominal,
            rigid_body=replace(nominal.rigid_body, mass=1.6),
        )

    if parameter_path == "rigid_body.inertia_B":
        return replace(
            nominal,
            rigid_body=replace(
                nominal.rigid_body,
                inertia_B=np.diag(np.array([0.021, 0.031, 0.041], dtype=np.float64)),
            ),
        )

    if parameter_path == "rotors.rotor_positions_B":
        rotor_positions_B = np.array(
            nominal.rotors.rotor_positions_B,
            dtype=np.float64,
            copy=True,
        )
        rotor_positions_B[0, 0] = 0.25
        return replace(
            nominal,
            rotors=replace(
                nominal.rotors,
                rotor_positions_B=rotor_positions_B,
            ),
        )

    if parameter_path == "rotors.rotor_spin_directions":
        rotor_spin_directions = np.array(
            nominal.rotors.rotor_spin_directions,
            dtype=np.float64,
            copy=True,
        )
        rotor_spin_directions[0] = -1.0
        return replace(
            nominal,
            rotors=replace(
                nominal.rotors,
                rotor_spin_directions=rotor_spin_directions,
            ),
        )

    if parameter_path == "rotors.thrust_coefficient":
        return replace(
            nominal,
            rotors=replace(nominal.rotors, thrust_coefficient=1.3e-5),
        )

    if parameter_path == "rotors.moment_coefficient":
        return replace(
            nominal,
            rotors=replace(nominal.rotors, moment_coefficient=3.0e-7),
        )

    if parameter_path == "world.gravity_acceleration":
        return replace(
            nominal,
            world=replace(nominal.world, gravity_acceleration=9.7),
        )

    if parameter_path == "imu.initial_accelerometer_bias_B":
        return replace(
            nominal,
            imu=replace(
                nominal.imu,
                initial_accelerometer_bias_B=np.array(
                    [0.01, 0.0, 0.0],
                    dtype=np.float64,
                ),
            ),
        )

    if parameter_path == "imu.accelerometer_noise_standard_deviation_B":
        return replace(
            nominal,
            imu=replace(
                nominal.imu,
                accelerometer_noise_standard_deviation_B=np.array(
                    [0.01, 0.0, 0.0],
                    dtype=np.float64,
                ),
            ),
        )

    if parameter_path == "imu.accelerometer_bias_random_walk_density_B":
        return replace(
            nominal,
            imu=replace(
                nominal.imu,
                accelerometer_bias_random_walk_density_B=np.array(
                    [0.001, 0.0, 0.0],
                    dtype=np.float64,
                ),
            ),
        )

    if parameter_path == "imu.initial_gyroscope_bias_B":
        return replace(
            nominal,
            imu=replace(
                nominal.imu,
                initial_gyroscope_bias_B=np.array(
                    [0.001, 0.0, 0.0],
                    dtype=np.float64,
                ),
            ),
        )

    if parameter_path == "imu.gyroscope_noise_standard_deviation_B":
        return replace(
            nominal,
            imu=replace(
                nominal.imu,
                gyroscope_noise_standard_deviation_B=np.array(
                    [0.01, 0.0, 0.0],
                    dtype=np.float64,
                ),
            ),
        )

    if parameter_path == "imu.gyroscope_bias_random_walk_density_B":
        return replace(
            nominal,
            imu=replace(
                nominal.imu,
                gyroscope_bias_random_walk_density_B=np.array(
                    [0.0001, 0.0, 0.0],
                    dtype=np.float64,
                ),
            ),
        )

    if parameter_path == "position_sensors.local_position_bias_W":
        return replace(
            nominal,
            position_sensors=replace(
                nominal.position_sensors,
                local_position_bias_W=np.array(
                    [0.1, 0.0, 0.0],
                    dtype=np.float64,
                ),
            ),
        )

    if parameter_path == "position_sensors.local_position_noise_standard_deviation_W":
        return replace(
            nominal,
            position_sensors=replace(
                nominal.position_sensors,
                local_position_noise_standard_deviation_W=np.array(
                    [0.1, 0.0, 0.0],
                    dtype=np.float64,
                ),
            ),
        )

    if parameter_path == "position_sensors.barometric_reference_altitude":
        return replace(
            nominal,
            position_sensors=replace(
                nominal.position_sensors,
                barometric_reference_altitude=126.0,
            ),
        )

    if parameter_path == "position_sensors.barometric_altitude_bias":
        return replace(
            nominal,
            position_sensors=replace(
                nominal.position_sensors,
                barometric_altitude_bias=0.1,
            ),
        )

    if parameter_path == "position_sensors.barometric_altitude_noise_standard_deviation":
        return replace(
            nominal,
            position_sensors=replace(
                nominal.position_sensors,
                barometric_altitude_noise_standard_deviation=0.1,
            ),
        )

    raise AssertionError(f"unexpected parameter path: {parameter_path}")


def _arguments_with_exact_mismatch(parameter_path: str) -> dict[str, object]:
    arguments = _matching_run_configuration_arguments()
    nominal = arguments["nominal"]
    assert isinstance(nominal, NominalConfiguration)
    arguments["nominal"] = _nominal_with_one_parameter_difference(
        nominal,
        parameter_path,
    )
    arguments["declared_mismatches"] = [
        DeclaredMismatch(
            parameter_path=parameter_path,
            rationale="Exercise one exact intentional model mismatch.",
        )
    ]
    return arguments


@pytest.mark.parametrize(
    "channel_name",
    ["accelerometer", "gyroscope", "local_position", "barometric_altitude"],
)
def test_run_configuration_rejects_off_truth_grid_sensor_period(
    channel_name: str,
) -> None:
    arguments = _valid_run_configuration_arguments()
    sensor_schedules = arguments["sensor_schedules"]
    assert isinstance(sensor_schedules, SensorSchedules)
    channel_schedule = getattr(sensor_schedules, channel_name)
    arguments["sensor_schedules"] = replace(
        sensor_schedules,
        **{
            channel_name: replace(
                channel_schedule,
                sample_period_s=0.0125,
            )
        },
    )

    with pytest.raises(
        ValueError,
        match=r"^sample_period_s must be an integer multiple of truth_time_step_s$",
    ):
        RunConfiguration(**arguments)


def test_run_configuration_accepts_float_aligned_sensor_periods() -> None:
    arguments = _matching_run_configuration_arguments()
    numerics = arguments["numerics"]
    sensor_schedules = arguments["sensor_schedules"]
    assert isinstance(numerics, RunNumerics)
    assert isinstance(sensor_schedules, SensorSchedules)
    arguments["numerics"] = replace(numerics, truth_time_step_s=0.01)
    arguments["sensor_schedules"] = SensorSchedules(
        accelerometer=replace(
            sensor_schedules.accelerometer,
            sample_period_s=0.03,
        ),
        gyroscope=replace(
            sensor_schedules.gyroscope,
            sample_period_s=0.01,
        ),
        local_position=replace(
            sensor_schedules.local_position,
            sample_period_s=0.1,
        ),
        barometric_altitude=replace(
            sensor_schedules.barometric_altitude,
            sample_period_s=0.05,
        ),
    )

    configuration = RunConfiguration(**arguments)

    assert configuration.sensor_schedules.accelerometer.sample_period_s == 0.03


@pytest.mark.parametrize(
    "parameter_path",
    [
        "rigid_body.mass",
        "rigid_body.inertia_B",
        "rotors.rotor_positions_B",
        "rotors.rotor_spin_directions",
        "rotors.thrust_coefficient",
        "rotors.moment_coefficient",
        "world.gravity_acceleration",
        "imu.initial_accelerometer_bias_B",
        "imu.accelerometer_noise_standard_deviation_B",
        "imu.accelerometer_bias_random_walk_density_B",
        "imu.initial_gyroscope_bias_B",
        "imu.gyroscope_noise_standard_deviation_B",
        "imu.gyroscope_bias_random_walk_density_B",
        "position_sensors.local_position_bias_W",
        "position_sensors.local_position_noise_standard_deviation_W",
        "position_sensors.barometric_reference_altitude",
        "position_sensors.barometric_altitude_bias",
        "position_sensors.barometric_altitude_noise_standard_deviation",
    ],
)
def test_run_configuration_accepts_exactly_declared_supported_mismatch(
    parameter_path: str,
) -> None:
    arguments = _arguments_with_exact_mismatch(parameter_path)

    configuration = RunConfiguration(**arguments)

    assert len(configuration.declared_mismatches) == 1
    assert configuration.declared_mismatches[0].parameter_path == parameter_path


def test_run_configuration_rejects_unknown_declared_mismatch_path() -> None:
    arguments = _matching_run_configuration_arguments()
    arguments["declared_mismatches"] = [
        DeclaredMismatch(
            parameter_path="world.wind_W",
            rationale="Exercise rejection of an unsupported parameter path.",
        )
    ]

    with pytest.raises(
        ValueError,
        match=r"^declared mismatch parameter_path is not supported: world\.wind_W$",
    ):
        RunConfiguration(**arguments)


def test_run_configuration_rejects_duplicate_declared_mismatch_path() -> None:
    arguments = _arguments_with_exact_mismatch("rigid_body.mass")
    arguments["declared_mismatches"] = [
        DeclaredMismatch(
            parameter_path="rigid_body.mass",
            rationale="First declaration of the mass mismatch.",
        ),
        DeclaredMismatch(
            parameter_path="rigid_body.mass",
            rationale="Second declaration of the mass mismatch.",
        ),
    ]

    with pytest.raises(
        ValueError,
        match=r"^declared_mismatches must not contain duplicate parameter_path values$",
    ):
        RunConfiguration(**arguments)


def test_run_configuration_rejects_undeclared_scalar_difference() -> None:
    arguments = _arguments_with_exact_mismatch("rigid_body.mass")
    arguments["declared_mismatches"] = []

    with pytest.raises(
        ValueError,
        match=r"^truth and nominal differ without a declared mismatch: rigid_body\.mass$",
    ):
        RunConfiguration(**arguments)


def test_run_configuration_rejects_undeclared_array_difference() -> None:
    arguments = _arguments_with_exact_mismatch("rigid_body.inertia_B")
    arguments["declared_mismatches"] = []

    with pytest.raises(
        ValueError,
        match=(
            r"^truth and nominal differ without a declared mismatch: "
            r"rigid_body\.inertia_B$"
        ),
    ):
        RunConfiguration(**arguments)


def test_run_configuration_rejects_declared_but_equal_parameter() -> None:
    arguments = _matching_run_configuration_arguments()
    arguments["declared_mismatches"] = [
        DeclaredMismatch(
            parameter_path="rigid_body.mass",
            rationale="Exercise rejection of a declaration without a difference.",
        )
    ]

    with pytest.raises(
        ValueError,
        match=(
            r"^declared mismatch has equal truth and nominal values: "
            r"rigid_body\.mass$"
        ),
    ):
        RunConfiguration(**arguments)


def test_run_configuration_accepts_complete_mismatch_set_in_any_declaration_order() -> None:
    arguments = _matching_run_configuration_arguments()
    nominal = arguments["nominal"]
    assert isinstance(nominal, NominalConfiguration)
    nominal = _nominal_with_one_parameter_difference(nominal, "rigid_body.mass")
    nominal = _nominal_with_one_parameter_difference(
        nominal,
        "world.gravity_acceleration",
    )
    arguments["nominal"] = nominal
    arguments["declared_mismatches"] = [
        DeclaredMismatch(
            parameter_path="world.gravity_acceleration",
            rationale="Declare gravity first to exercise order independence.",
        ),
        DeclaredMismatch(
            parameter_path="rigid_body.mass",
            rationale="Declare mass second to preserve caller ordering.",
        ),
    ]

    configuration = RunConfiguration(**arguments)

    assert tuple(mismatch.parameter_path for mismatch in configuration.declared_mismatches) == (
        "world.gravity_acceleration",
        "rigid_body.mass",
    )
