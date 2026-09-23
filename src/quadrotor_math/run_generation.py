"""Assemble deterministic truth and sensor data for one validated run."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .actuation import motor_speed_first_order_step
from .dynamics import rigid_body_state_derivative_from_rotor_speeds
from .imu import (
    accelerometer_bias_random_walk_step_body,
    accelerometer_specific_force_measurement_body,
    gyroscope_angular_velocity_measurement_body,
    gyroscope_bias_random_walk_step_body,
    ideal_accelerometer_specific_force_body,
    ideal_gyroscope_angular_velocity_body,
)
from .integration import (
    rigid_body_state_euler_step_from_rotor_speeds,
    rigid_body_state_rk4_step_from_rotor_speeds,
)
from .position_sensors import (
    barometric_altitude_measurement,
    ideal_barometric_altitude_from_position_world,
    ideal_position_measurement_world,
    position_measurement_world,
)
from .randomness import create_run_random_streams
from .rotations import rotation_matrix_body_to_world
from .run_artifact import RunArtifactData
from .run_configuration import IntegrationMethod, RunConfiguration
from .sensor_scheduling import FixedRateSensorScheduler


@dataclass(slots=True)
class _Observation:
    acquisition_time_s: float
    delivery_time_s: float
    value: float | NDArray[np.float64]
    delivered_at_truth_index: int = -1


@dataclass(frozen=True, slots=True)
class _StreamArrays:
    sequence_index: NDArray[np.int64]
    observation_index: NDArray[np.int64]
    acquisition_time_s: NDArray[np.float64]
    delivery_time_s: NDArray[np.float64]
    delivered_at_truth_index: NDArray[np.int64]
    measurements: NDArray[np.float64]


def _stream_arrays(records: list[_Observation], width: int | None) -> _StreamArrays:
    """Convert one scheduler's acquisitions to exact artifact dtypes and shapes."""
    sequence_index = np.arange(len(records), dtype="<i8")
    measurements = np.asarray([record.value for record in records], dtype="<f8")
    if width is not None:
        measurements = measurements.reshape((-1, width))
    return _StreamArrays(
        sequence_index=sequence_index,
        observation_index=sequence_index.copy(),
        acquisition_time_s=np.asarray(
            [record.acquisition_time_s for record in records], dtype="<f8"
        ),
        delivery_time_s=np.asarray([record.delivery_time_s for record in records], dtype="<f8"),
        delivered_at_truth_index=np.asarray(
            [record.delivered_at_truth_index for record in records], dtype="<i8"
        ),
        measurements=measurements,
    )


def generate_run_artifact_data(configuration: RunConfiguration) -> RunArtifactData:
    """Generate one complete in-memory run without changing its validated inputs.

    Complete motor parameters yield private lagged actual speeds. Wholly omitted
    motor parameters preserve the historical ideal-actuator mode. Sensors acquire
    at completed truth rows; pending deliveries remain marked ``-1``.

    Structural configurations can represent a broader domain than this numerical
    plant. Truth gravity and rotor coefficients must be positive; inertia symmetry
    and initial attitude must satisfy the downstream dynamics/rotation contracts.
    These execution checks precede history allocation and random-stream creation.
    """
    truth = configuration.truth
    # Configurations also represent historical/nominal models. Check the narrower
    # execution domain before allocating histories or constructing random streams.
    for name, value in (
        ("gravity_acceleration", truth.world.gravity_acceleration),
        ("thrust_coefficient", truth.rotors.thrust_coefficient),
        ("moment_coefficient", truth.rotors.moment_coefficient),
    ):
        if value <= 0.0:
            raise ValueError(f"truth {name} must be positive for run generation")
    if not np.allclose(
        truth.rigid_body.inertia_B, truth.rigid_body.inertia_B.T, rtol=0.0, atol=1.0e-12
    ):
        raise ValueError("truth inertia_B must be symmetric to atol=1e-12 for run generation")
    rotation_matrix_body_to_world(configuration.initial_truth_state.q_WB)
    numerics = configuration.numerics
    command = configuration.rotor_speed_input.rotor_omega
    minimum = truth.rotors.minimum_rotor_omega
    maximum = truth.rotors.maximum_rotor_omega
    time_constant = truth.rotors.motor_time_constant_s
    initial_actual = configuration.initial_actual_rotor_omega
    motor_values = (
        minimum,
        maximum,
        time_constant,
        configuration.nominal.rotors.minimum_rotor_omega,
        configuration.nominal.rotors.maximum_rotor_omega,
        configuration.nominal.rotors.motor_time_constant_s,
        initial_actual,
    )
    motorized = not all(value is None for value in motor_values)
    if motorized and not all(value is not None for value in motor_values):
        raise ValueError(
            "run generation motor configuration must be either entirely omitted or complete"
        )

    steps = numerics.number_of_steps
    time_step_s = numerics.truth_time_step_s
    truth_time_s = np.arange(steps + 1, dtype="<f8") * time_step_s
    position_W = np.empty((steps + 1, 3), dtype="<f8")
    velocity_W = np.empty((steps + 1, 3), dtype="<f8")
    q_WB = np.empty((steps + 1, 4), dtype="<f8")
    omega_B = np.empty((steps + 1, 3), dtype="<f8")
    accelerometer_bias_B = np.empty((steps + 1, 3), dtype="<f8")
    gyroscope_bias_B = np.empty((steps + 1, 3), dtype="<f8")
    commanded_rotor_omega = np.broadcast_to(command, (steps, 4)).copy()
    initial = configuration.initial_truth_state
    position_W[0] = initial.position_W
    velocity_W[0] = initial.velocity_W
    q_WB[0] = initial.q_WB
    omega_B[0] = initial.omega_B
    accelerometer_bias_B[0] = truth.imu.initial_accelerometer_bias_B
    gyroscope_bias_B[0] = truth.imu.initial_gyroscope_bias_B
    actual_speeds = [initial_actual.copy() if initial_actual is not None else command.copy()]
    rngs = create_run_random_streams(configuration.root_seed)
    schedules = configuration.sensor_schedules
    accelerometer_scheduler: FixedRateSensorScheduler[NDArray[np.float64]] = (
        FixedRateSensorScheduler(
            sample_period_s=schedules.accelerometer.sample_period_s,
            truth_time_step_s=time_step_s,
            delivery_delay_s=schedules.accelerometer.delivery_delay_s,
        )
    )
    gyroscope_scheduler: FixedRateSensorScheduler[NDArray[np.float64]] = FixedRateSensorScheduler(
        sample_period_s=schedules.gyroscope.sample_period_s,
        truth_time_step_s=time_step_s,
        delivery_delay_s=schedules.gyroscope.delivery_delay_s,
    )
    local_position_scheduler: FixedRateSensorScheduler[NDArray[np.float64]] = (
        FixedRateSensorScheduler(
            sample_period_s=schedules.local_position.sample_period_s,
            truth_time_step_s=time_step_s,
            delivery_delay_s=schedules.local_position.delivery_delay_s,
        )
    )
    barometric_scheduler: FixedRateSensorScheduler[float] = FixedRateSensorScheduler(
        sample_period_s=schedules.barometric_altitude.sample_period_s,
        truth_time_step_s=time_step_s,
        delivery_delay_s=schedules.barometric_altitude.delivery_delay_s,
    )
    accelerometer_records: list[_Observation] = []
    gyroscope_records: list[_Observation] = []
    local_position_records: list[_Observation] = []
    barometric_records: list[_Observation] = []

    def acquire_accelerometer(index: int, acquisition_time_s: float) -> NDArray[np.float64]:
        acceleration_W = rigid_body_state_derivative_from_rotor_speeds(
            position_W[index],
            velocity_W[index],
            q_WB[index],
            omega_B[index],
            actual_speeds[index],
            truth.rotors.rotor_positions_B,
            truth.rotors.rotor_spin_directions,
            truth.rigid_body.mass,
            truth.rigid_body.inertia_B,
            truth.world.gravity_acceleration,
            truth.rotors.thrust_coefficient,
            truth.rotors.moment_coefficient,
            wind_velocity_W=truth.world.wind_velocity_W,
            quadratic_drag_coefficient_B=truth.rigid_body.quadratic_drag_coefficient_B,
        )[1]
        ideal_B = ideal_accelerometer_specific_force_body(
            acceleration_W,
            rotation_matrix_body_to_world(q_WB[index]),
            truth.world.gravity_acceleration,
        )
        measurement = accelerometer_specific_force_measurement_body(
            ideal_B,
            accelerometer_bias_B[index],
            truth.imu.accelerometer_noise_standard_deviation_B,
            rngs.accelerometer_measurement_noise,
        )
        accelerometer_records.append(
            _Observation(
                acquisition_time_s,
                acquisition_time_s + schedules.accelerometer.delivery_delay_s,
                measurement,
            )
        )
        return measurement

    def acquire_gyroscope(index: int, acquisition_time_s: float) -> NDArray[np.float64]:
        measurement = gyroscope_angular_velocity_measurement_body(
            ideal_gyroscope_angular_velocity_body(omega_B[index]),
            gyroscope_bias_B[index],
            truth.imu.gyroscope_noise_standard_deviation_B,
            rngs.gyroscope_measurement_noise,
        )
        gyroscope_records.append(
            _Observation(
                acquisition_time_s,
                acquisition_time_s + schedules.gyroscope.delivery_delay_s,
                measurement,
            )
        )
        return measurement

    def acquire_local_position(index: int, acquisition_time_s: float) -> NDArray[np.float64]:
        measurement = position_measurement_world(
            ideal_position_measurement_world(position_W[index]),
            truth.position_sensors.local_position_bias_W,
            truth.position_sensors.local_position_noise_standard_deviation_W,
            rngs.local_position_measurement_noise,
        )
        local_position_records.append(
            _Observation(
                acquisition_time_s,
                acquisition_time_s + schedules.local_position.delivery_delay_s,
                measurement,
            )
        )
        return measurement

    def acquire_barometric(index: int, acquisition_time_s: float) -> float:
        measurement = barometric_altitude_measurement(
            ideal_barometric_altitude_from_position_world(
                position_W[index],
                truth.position_sensors.barometric_reference_altitude,
            ),
            truth.position_sensors.barometric_altitude_bias,
            truth.position_sensors.barometric_altitude_noise_standard_deviation,
            rngs.barometric_altitude_measurement_noise,
        )
        barometric_records.append(
            _Observation(
                acquisition_time_s,
                acquisition_time_s + schedules.barometric_altitude.delivery_delay_s,
                measurement,
            )
        )
        return measurement

    integrator = (
        rigid_body_state_euler_step_from_rotor_speeds
        if numerics.integration_method is IntegrationMethod.EULER
        else rigid_body_state_rk4_step_from_rotor_speeds
    )
    for step in range(steps):
        if motorized:
            assert minimum is not None and maximum is not None and time_constant is not None
            actual = motor_speed_first_order_step(
                actual_speeds[-1],
                commanded_rotor_omega[step],
                minimum,
                maximum,
                time_constant,
                time_step_s,
            )
        else:
            actual = command
        actual_speeds.append(actual)
        index = step + 1
        position_W[index], velocity_W[index], q_WB[index], omega_B[index] = integrator(
            position_W[step],
            velocity_W[step],
            q_WB[step],
            omega_B[step],
            actual,
            truth.rotors.rotor_positions_B,
            truth.rotors.rotor_spin_directions,
            truth.rigid_body.mass,
            truth.rigid_body.inertia_B,
            truth.world.gravity_acceleration,
            truth.rotors.thrust_coefficient,
            truth.rotors.moment_coefficient,
            time_step_s,
            wind_velocity_W=truth.world.wind_velocity_W,
            quadratic_drag_coefficient_B=truth.rigid_body.quadratic_drag_coefficient_B,
        )
        accelerometer_bias_B[index] = accelerometer_bias_random_walk_step_body(
            accelerometer_bias_B[step],
            truth.imu.accelerometer_bias_random_walk_density_B,
            time_step_s,
            rngs.accelerometer_bias_random_walk,
        )
        gyroscope_bias_B[index] = gyroscope_bias_random_walk_step_body(
            gyroscope_bias_B[step],
            truth.imu.gyroscope_bias_random_walk_density_B,
            time_step_s,
            rngs.gyroscope_bias_random_walk,
        )
        now = float(truth_time_s[index])
        for record in accelerometer_scheduler.update(now, acquire_accelerometer):
            accelerometer_records[record.sequence_index].delivered_at_truth_index = index
        for record in gyroscope_scheduler.update(now, acquire_gyroscope):
            gyroscope_records[record.sequence_index].delivered_at_truth_index = index
        for record in local_position_scheduler.update(now, acquire_local_position):
            local_position_records[record.sequence_index].delivered_at_truth_index = index
        for barometric_delivery in barometric_scheduler.update(now, acquire_barometric):
            barometric_records[barometric_delivery.sequence_index].delivered_at_truth_index = index

    accelerometer = _stream_arrays(accelerometer_records, 3)
    gyroscope = _stream_arrays(gyroscope_records, 3)
    local_position = _stream_arrays(local_position_records, 3)
    barometric = _stream_arrays(barometric_records, None)
    deliveries = sorted(
        (record.delivered_at_truth_index, sensor_id, sequence_index)
        for sensor_id, records in enumerate(
            (accelerometer_records, gyroscope_records, local_position_records, barometric_records),
            start=1,
        )
        for sequence_index, record in enumerate(records)
        if record.delivered_at_truth_index != -1
    )
    return RunArtifactData(
        truth_time_s=truth_time_s,
        truth_position_history_W=position_W,
        truth_velocity_history_W=velocity_W,
        truth_q_history_WB=q_WB,
        truth_omega_history_B=omega_B,
        commanded_rotor_omega=commanded_rotor_omega,
        accelerometer_sequence_index=accelerometer.sequence_index,
        accelerometer_observation_index=accelerometer.observation_index,
        accelerometer_acquisition_time_s=accelerometer.acquisition_time_s,
        accelerometer_delivery_time_s=accelerometer.delivery_time_s,
        accelerometer_delivered_at_truth_index=accelerometer.delivered_at_truth_index,
        accelerometer_measurements_B=accelerometer.measurements,
        gyroscope_sequence_index=gyroscope.sequence_index,
        gyroscope_observation_index=gyroscope.observation_index,
        gyroscope_acquisition_time_s=gyroscope.acquisition_time_s,
        gyroscope_delivery_time_s=gyroscope.delivery_time_s,
        gyroscope_delivered_at_truth_index=gyroscope.delivered_at_truth_index,
        gyroscope_measurements_B=gyroscope.measurements,
        local_position_sequence_index=local_position.sequence_index,
        local_position_observation_index=local_position.observation_index,
        local_position_acquisition_time_s=local_position.acquisition_time_s,
        local_position_delivery_time_s=local_position.delivery_time_s,
        local_position_delivered_at_truth_index=local_position.delivered_at_truth_index,
        local_position_measurements_W=local_position.measurements,
        barometric_altitude_sequence_index=barometric.sequence_index,
        barometric_altitude_observation_index=barometric.observation_index,
        barometric_altitude_acquisition_time_s=barometric.acquisition_time_s,
        barometric_altitude_delivery_time_s=barometric.delivery_time_s,
        barometric_altitude_delivered_at_truth_index=barometric.delivered_at_truth_index,
        barometric_altitude_measurements=barometric.measurements,
        accelerometer_bias_history_B=accelerometer_bias_B,
        gyroscope_bias_history_B=gyroscope_bias_B,
        delivery_sensor_id=np.asarray([row[1] for row in deliveries], dtype="<i8"),
        delivery_sequence_index=np.asarray([row[2] for row in deliveries], dtype="<i8"),
        delivery_observation_index=np.asarray([row[2] for row in deliveries], dtype="<i8"),
    )
