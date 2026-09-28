"""Causal sensor -> endpoint ESKF -> cascade/geometric control, with truth isolation."""

from dataclasses import dataclass, replace
from typing import Any

import numpy as np
from numpy.typing import NDArray

from .attitude_control import AttitudeControllerParameters
from .attitude_simulation import State, _wrench
from .dynamics import quadratic_drag_force_body, translational_acceleration_world_from_body_force
from .eskf_live_faults import EskfLiveObservationFaults
from .eskf_online import EskfOnlineEstimator
from .eskf_replay import (
    EskfObservationKind,
    EskfReplayConfiguration,
    EskfReplayEvent,
    EskfReplayInput,
    EskfReplayObservation,
    EskfReplayResult,
    EskfReplayStatus,
    _owned_array,
    _scalar,
)
from .eskf_run_replay import _first_delivery_index
from .geometric_control import GeometricControllerParameters
from .imu import (
    accelerometer_bias_random_walk_step_body,
    accelerometer_specific_force_measurement_body,
    gyroscope_angular_velocity_measurement_body,
    gyroscope_bias_random_walk_step_body,
    ideal_accelerometer_specific_force_body,
)
from .mission_simulation import MissionNumerics, MissionResult, _simulate_mission
from .missions import MissionPlan, MissionSafetyLimits
from .observation_health import ObservationHealthMonitor
from .observation_supervision import ObservationSupervisor
from .position_control import PositionControllerParameters
from .position_sensors import (
    barometric_altitude_measurement,
    ideal_barometric_altitude_from_position_world,
    position_measurement_world,
)
from .randomness import create_run_random_streams
from .rotations import rotation_matrix_body_to_world
from .run_configuration import (
    ImuParameters,
    PositionSensorParameters,
    RigidBodyInitialState,
    RigidBodyParameters,
    RotorParameters,
    SensorSchedule,
    WorldParameters,
)
from .sensor_scheduling import FixedRateSensorScheduler, _time_comparison_tolerance_s


@dataclass(frozen=True, slots=True, eq=False)
class MissionSensors:
    """Truth measurement distributions and slow schedules, never estimator assumptions.

    IMU is paired at every plant epoch including zero, without delivery delay.
    Slow schedules start at their first positive period, using the existing
    scheduler. Bias walks advance once per completed plant interval. All six
    independent named streams derive from root_seed under randomness version 1.
    """

    imu: ImuParameters
    position: PositionSensorParameters
    local_position_schedule: SensorSchedule
    barometric_altitude_schedule: SensorSchedule
    root_seed: int

    def __post_init__(self) -> None:
        for name, kind in (
            ("imu", ImuParameters),
            ("position", PositionSensorParameters),
            ("local_position_schedule", SensorSchedule),
            ("barometric_altitude_schedule", SensorSchedule),
        ):
            value = getattr(self, name)
            if not isinstance(value, kind):
                raise TypeError(f"{name} must be {kind.__name__}")
            object.__setattr__(self, name, replace(value))
        for name in ("local_position_schedule", "barometric_altitude_schedule"):
            value = getattr(self, name)
            _scalar("sample_period_s", value.sample_period_s, nonnegative=True)
            _scalar("delivery_delay_s", value.delivery_delay_s, nonnegative=True)
        if type(self.root_seed) is not int or not 0 <= self.root_seed < 2**128:
            raise ValueError("root_seed must be a non-Boolean integer in [0, 2**128)")


@dataclass(frozen=True, slots=True, eq=False)
class EstimatedMissionResult:
    """Owned full-grid truth, sensor and online estimate evidence (ADR 0013).

    mission retains true state, but commands and completion used estimates.
    Slow timestamps are aligned with measurements.observations (canonical
    delivery order, then pending). True biases are evaluation-only FRD SI values.
    The 6-column noise mean is endpoint accelerometer/gyro conditional sample
    noise; rate feedback subtracts its final three columns and the estimated bias.
    No physical ground contact or deployable safety guarantee is represented.
    """

    mission: MissionResult
    measurements: EskfReplayInput
    estimates: EskfReplayResult
    angular_velocity_estimate_B: NDArray[np.float64]
    imu_noise_mean_B: NDArray[np.float64]
    true_accelerometer_bias_B: NDArray[np.float64]
    true_gyroscope_bias_B: NDArray[np.float64]
    scheduled_observation_delivery_time_s: NDArray[np.float64]

    def __post_init__(self) -> None:
        for name, kind in (
            ("mission", MissionResult),
            ("measurements", EskfReplayInput),
            ("estimates", EskfReplayResult),
        ):
            value = getattr(self, name)
            if not isinstance(value, kind):
                raise TypeError(f"{name} must be {kind.__name__}")
            object.__setattr__(self, name, replace(value))
        times = self.mission.time_s
        if not (
            np.array_equal(times, self.measurements.time_s)
            and np.array_equal(times, self.estimates.time_s)
        ):
            raise ValueError("truth, measurement and estimate clocks must exactly agree")
        n = len(times)
        for name, width in (
            ("angular_velocity_estimate_B", 3),
            ("imu_noise_mean_B", 6),
            ("true_accelerometer_bias_B", 3),
            ("true_gyroscope_bias_B", 3),
        ):
            object.__setattr__(self, name, _owned_array(name, getattr(self, name), (n, width)))
        obs = self.measurements.observations
        scheduled = _owned_array(
            "scheduled_observation_delivery_time_s",
            self.scheduled_observation_delivery_time_s,
            (len(obs),),
        )
        if len(self.estimates.events) != len(obs):
            raise ValueError("every acquired slow observation requires exactly one disposition")
        for o, event, delivery_time in zip(obs, self.estimates.events, scheduled, strict=True):
            e = event.observation
            if (o.kind, o.observation_index, o.acquisition_index, o.delivery_index) != (
                e.kind,
                e.observation_index,
                e.acquisition_index,
                e.delivery_index,
            ) or not np.array_equal(o.measurement, e.measurement):
                raise ValueError("event ledger must match the complete acquisition ledger")
            if delivery_time < times[o.acquisition_index] or (
                _first_delivery_index(times, float(delivery_time)) != o.delivery_index
            ):
                raise ValueError("scheduled delivery must match the actual causal arrival")
        object.__setattr__(self, "scheduled_observation_delivery_time_s", scheduled)
        with np.errstate(over="ignore", invalid="ignore"):
            rate = (
                self.measurements.angular_velocity_measurements_B
                - np.array([s.gyroscope_bias_B for s in self.estimates.states])
                - self.imu_noise_mean_B[:, 3:]
            )
        if not np.array_equal(rate, self.angular_velocity_estimate_B):
            raise ValueError("rate feedback must equal measured gyro minus posterior bias/noise")


@dataclass(slots=True)
class _SlowRecord:
    kind: EskfObservationKind
    source_index: int
    acquisition_index: int
    scheduled_time_s: float
    value: NDArray[np.float64]
    delivery_index: int = -1

    def observation(self) -> EskfReplayObservation:
        return EskfReplayObservation(
            self.kind, self.source_index, self.acquisition_index, self.delivery_index, self.value
        )


def simulate_estimated_mission(
    initial_state: RigidBodyInitialState,
    initial_actual_rotor_omega: NDArray[np.float64],
    truth_body: RigidBodyParameters,
    truth_rotors: RotorParameters,
    truth_world: WorldParameters,
    position_controller: PositionControllerParameters,
    attitude_controller: AttitudeControllerParameters,
    plan: MissionPlan,
    safety: MissionSafetyLimits,
    numerics: MissionNumerics,
    sensors: MissionSensors,
    estimator_configuration: EskfReplayConfiguration,
    *,
    geometric_controller: GeometricControllerParameters | None = None,
    allow_minimum_snap: bool = False,
    observation_health: ObservationHealthMonitor | None = None,
    observation_supervision: ObservationSupervisor | None = None,
    observation_faults: EskfLiveObservationFaults | None = None,
) -> EstimatedMissionResult:
    """Execute measurements and ESKF before same-epoch feedback, without lookahead.

    Geometric control is opt-in. For cascade minimum-snap comparison, explicitly
    set allow_minimum_snap=True. Legacy calls and defaults remain unchanged.
    The explicit prior must be at t=0 and endpoint noise must be configured.
    Inputs are snapshotted. The truth plant, safety oracle and sensor producer
    are separate from the measurement-only estimator and pure controllers.
    Every epoch including an immediate abort is sampled and recorded. Sensor
    or estimator arithmetic failures raise without returning a partial mission;
    sampled safety/domain/timeout aborts retain their entire terminal history.
    A fresh optional observation_health monitor records passive diagnostics in
    its own history (ADR 0021). If execution raises, it retains only the observed
    prefix; no partial mission result is returned. Reset it before reuse.
    An optional fresh observation_supervision bound to that same monitor adds
    the explicit numerical-abort policy in ADR 0022. Reset both before reuse.
    Optional fresh observation_faults alters nominal arrivals before the ESKF.
    Its separate exhaustive source/fault ledger is sealed after execution;
    result.measurements contains only surviving inputs (including pending).
    """
    if not isinstance(sensors, MissionSensors) or not isinstance(
        estimator_configuration, EskfReplayConfiguration
    ):
        raise TypeError("sensors and estimator configuration must use their dataclasses")
    if not isinstance(numerics, MissionNumerics) or not isinstance(plan, MissionPlan):
        raise TypeError("numerics and plan must use their dataclasses")
    sensors, config, numerics, plan = (
        replace(sensors),
        replace(estimator_configuration),
        replace(numerics),
        replace(plan),
    )
    if config.initial_time_s != 0 or config.sampled_imu_noise is None:
        raise ValueError("estimated mission requires an endpoint ESKF prior at zero")
    for value, kind in (
        (truth_body, RigidBodyParameters),
        (truth_rotors, RotorParameters),
        (truth_world, WorldParameters),
    ):
        if not isinstance(value, kind):
            raise TypeError("truth models must use their dataclasses")
    body, rotors, world = replace(truth_body), replace(truth_rotors), replace(truth_world)
    dt = numerics.time_step_s
    horizon = plan.reference_duration_s + plan.completion_timeout_s + dt
    if dt <= 2 * _time_comparison_tolerance_s(horizon, horizon):
        raise ValueError("plant clock must resolve sensor scheduler tolerance")
    schedules = (sensors.local_position_schedule, sensors.barometric_altitude_schedule)
    if observation_faults is not None:
        if not isinstance(observation_faults, EskfLiveObservationFaults):
            raise TypeError("observation_faults must be EskfLiveObservationFaults or None")
        if (
            observation_faults.epoch_count
            or observation_faults.initial_time_s != config.initial_time_s
        ):
            raise ValueError("observation faults must be fresh at the estimator prior epoch")
        try:
            if any(
                not np.isfinite(f.delay_steps * dt + horizon) for f in observation_faults.faults
            ):
                raise ValueError("fault delivery times must remain finite")
        except OverflowError:
            raise ValueError("fault delivery times must remain finite") from None
    if observation_health is not None:
        if not isinstance(observation_health, ObservationHealthMonitor):
            raise TypeError("observation_health must be ObservationHealthMonitor or None")
        if (
            observation_health.latest is not None
            or observation_health.initial_time_s != config.initial_time_s
        ):
            raise ValueError(
                "observation health monitor must be fresh at the estimator prior epoch"
            )
        health_config = observation_health.configuration
        for policy, schedule, enabled in zip(
            (health_config.local_position, health_config.barometric_altitude),
            schedules,
            (config.fuse_local_position, config.fuse_barometric_altitude),
            strict=True,
        ):
            if (
                policy.sample_period_s != schedule.sample_period_s
                or policy.delivery_delay_s != schedule.delivery_delay_s
                or policy.check_interval_s != dt
                or policy.enabled != enabled
            ):
                raise ValueError(
                    "observation health policy must match sensor schedule, clock and fusion flag"
                )
    if observation_supervision is not None:
        if not isinstance(observation_supervision, ObservationSupervisor):
            raise TypeError("observation_supervision must be ObservationSupervisor or None")
        if observation_supervision.monitor is not observation_health:
            raise ValueError("observation supervisor must be bound to the supplied health monitor")
        if observation_supervision.latest is not None:
            raise ValueError("observation supervisor must be fresh")
    schedulers = tuple(
        FixedRateSensorScheduler[NDArray[np.float64]](
            sample_period_s=s.sample_period_s,
            truth_time_step_s=dt,
            delivery_delay_s=s.delivery_delay_s,
        )
        for s in schedules
    )
    records: tuple[list[_SlowRecord], list[_SlowRecord]] = ([], [])
    stream = EskfOnlineEstimator(config)
    rng = create_run_random_streams(sensors.root_seed)
    imu, position = sensors.imu, sensors.position
    bias_a, bias_g = imu.initial_accelerometer_bias_B, imu.initial_gyroscope_bias_B
    force_rows: list[NDArray[np.float64]] = []
    gyro_rows: list[NDArray[np.float64]] = []
    bias_a_rows: list[NDArray[np.float64]] = []
    bias_g_rows: list[NDArray[np.float64]] = []
    states = []
    covariance_rows: list[NDArray[np.float64]] = []
    rate_rows: list[NDArray[np.float64]] = []
    noise_rows: list[NDArray[np.float64]] = []
    events: list[EskfReplayEvent] = []
    rebase_corrections = (
        isinstance(geometric_controller, GeometricControllerParameters)
        and geometric_controller.rebase_estimator_corrections
    )
    pending_force_jump = np.zeros(3)
    measured_derivatives = (
        isinstance(geometric_controller, GeometricControllerParameters)
        and geometric_controller.use_measured_acceleration
    )
    feedback_acceleration = np.zeros(3)

    def estimated_acceleration() -> NDArray[np.float64]:
        return feedback_acceleration

    def consume_estimator_force_jump() -> NDArray[np.float64]:
        nonlocal pending_force_jump
        jump, pending_force_jump = pending_force_jump, np.zeros(3)
        return jump

    def observe(k: int, time: float, truth: State, actual: NDArray[np.float64]) -> State:
        nonlocal bias_a, bias_g, pending_force_jump, feedback_acceleration
        if k:
            bias_a = accelerometer_bias_random_walk_step_body(
                bias_a,
                imu.accelerometer_bias_random_walk_density_B,
                dt,
                rng.accelerometer_bias_random_walk,
            )
            bias_g = gyroscope_bias_random_walk_step_body(
                bias_g,
                imu.gyroscope_bias_random_walk_density_B,
                dt,
                rng.gyroscope_bias_random_walk,
            )
        R_WB = rotation_matrix_body_to_world(truth[2])
        force_B = _wrench(actual, rotors)[0] + quadratic_drag_force_body(
            truth[1], R_WB, world.wind_velocity_W, body.quadratic_drag_coefficient_B
        )
        acceleration_W = translational_acceleration_world_from_body_force(
            force_B, R_WB, body.mass, world.gravity_acceleration
        )
        specific_force = ideal_accelerometer_specific_force_body(
            acceleration_W, R_WB, world.gravity_acceleration
        )
        measured_force = accelerometer_specific_force_measurement_body(
            specific_force,
            bias_a,
            imu.accelerometer_noise_standard_deviation_B,
            rng.accelerometer_measurement_noise,
        )
        measured_rate = gyroscope_angular_velocity_measurement_body(
            truth[3],
            bias_g,
            imu.gyroscope_noise_standard_deviation_B,
            rng.gyroscope_measurement_noise,
        )

        def acquire_local(index: int, acquired: float) -> NDArray[np.float64]:
            if index != k:
                raise ValueError("slow acquisition must use only the current truth epoch")
            value = position_measurement_world(
                truth[0],
                position.local_position_bias_W,
                position.local_position_noise_standard_deviation_W,
                rng.local_position_measurement_noise,
            )
            records[0].append(
                _SlowRecord(
                    EskfObservationKind.LOCAL_POSITION,
                    len(records[0]),
                    k,
                    acquired + schedules[0].delivery_delay_s,
                    value,
                )
            )
            return value

        def acquire_altitude(index: int, acquired: float) -> NDArray[np.float64]:
            if index != k:
                raise ValueError("slow acquisition must use only the current truth epoch")
            value = np.array(
                [
                    barometric_altitude_measurement(
                        ideal_barometric_altitude_from_position_world(
                            truth[0], position.barometric_reference_altitude
                        ),
                        position.barometric_altitude_bias,
                        position.barometric_altitude_noise_standard_deviation,
                        rng.barometric_altitude_measurement_noise,
                    )
                ]
            )
            records[1].append(
                _SlowRecord(
                    EskfObservationKind.BAROMETRIC_ALTITUDE,
                    len(records[1]),
                    k,
                    acquired + schedules[1].delivery_delay_s,
                    value,
                )
            )
            return value

        delivered: list[EskfReplayObservation] = []
        for scheduler, source, acquire in zip(
            schedulers, records, (acquire_local, acquire_altitude), strict=True
        ):
            for delivery in scheduler.update(time, acquire):
                record = source[delivery.sequence_index]
                record.delivery_index = k
                delivered.append(record.observation())
        arrivals = tuple(delivered)
        if observation_faults is not None:
            arrivals = observation_faults.step(time, arrivals)
        estimate = stream.step(time, measured_force, measured_rate, arrivals)
        if observation_health is not None:
            observation_health.step(time, estimate.events)
        if observation_supervision is not None:
            observation_supervision.step()
        if measured_derivatives:
            posterior = estimate.nominal_state
            with np.errstate(over="raise", invalid="raise"):
                feedback_acceleration = rotation_matrix_body_to_world(posterior.q_WB) @ (
                    measured_force - posterior.accelerometer_bias_B - estimate.imu_noise_mean_B[:3]
                ) + np.array([0.0, 0.0, position_controller.nominal_gravity_acceleration])
        if rebase_corrections:
            try:
                with np.errstate(over="raise", invalid="raise"):
                    for event in estimate.events:
                        if event.update is not None:
                            correction = event.update.error_state_correction
                            pending_force_jump += position_controller.nominal_mass * (
                                position_controller.position_gain_W * correction[:3]
                                + position_controller.velocity_gain_W * correction[3:6]
                            )
            except FloatingPointError:
                raise ValueError("estimator force correction must remain finite") from None
        force_rows.append(measured_force)
        gyro_rows.append(measured_rate)
        bias_a_rows.append(bias_a)
        bias_g_rows.append(bias_g)
        states.append(estimate.nominal_state)
        covariance_rows.append(estimate.covariance)
        rate_rows.append(estimate.angular_velocity_estimate_B)
        noise_rows.append(estimate.imu_noise_mean_B)
        events.extend(estimate.events)
        state = estimate.nominal_state
        return state.position_W, state.velocity_W, state.q_WB, estimate.angular_velocity_estimate_B

    # Preserve the positional legacy adapter call when no new option is used.
    options: dict[str, Any] = {}
    if geometric_controller is not None:
        options["geometric_controller"] = geometric_controller
    if rebase_corrections:
        options["consume_estimator_force_jump"] = consume_estimator_force_jump
    if measured_derivatives:
        options["estimated_acceleration"] = estimated_acceleration
    if observation_supervision is not None:
        options["observation_guard"] = lambda: observation_supervision.abort_reason
    if allow_minimum_snap:
        options["allow_minimum_snap"] = allow_minimum_snap
    if type(allow_minimum_snap) is not bool:
        raise ValueError("allow_minimum_snap must be bool")
    mission = _simulate_mission(
        initial_state,
        initial_actual_rotor_omega,
        body,
        rotors,
        world,
        position_controller,
        attitude_controller,
        plan,
        safety,
        numerics,
        observe,
        **options,
    )
    all_records = [r for source in records for r in source]
    measurements = EskfReplayInput(
        mission.time_s,
        np.asarray(force_rows),
        np.asarray(gyro_rows),
        tuple(r.observation() for r in all_records),
    )
    delivery_times = {(r.kind, r.source_index): r.scheduled_time_s for r in all_records}
    if observation_faults is not None:
        injection = observation_faults.finish(measurements)
        measurements = injection.measurements
        delivery_times = {
            (r.output.kind, r.output.observation_index): (
                delivery_times[r.source.kind, r.source.observation_index] + r.fault.delay_steps * dt
            )
            for r in injection.records
            if r.output is not None
        }
    events.extend(
        EskfReplayEvent(o, EskfReplayStatus.PENDING)
        for o in measurements.observations
        if o.delivery_index == -1
    )
    estimates = EskfReplayResult(
        mission.time_s, tuple(states), np.asarray(covariance_rows), tuple(events)
    )
    return EstimatedMissionResult(
        mission,
        measurements,
        estimates,
        np.asarray(rate_rows),
        np.asarray(noise_rows),
        np.asarray(bias_a_rows),
        np.asarray(bias_g_rows),
        np.array([delivery_times[o.kind, o.observation_index] for o in measurements.observations]),
    )
