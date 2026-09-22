from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True, eq=False)
class RigidBodyParameters:
    """Store mass, body inertia, and body-axis quadratic drag coefficients."""

    mass: float
    inertia_B: NDArray[np.float64]
    quadratic_drag_coefficient_B: NDArray[np.float64] = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )

    def __post_init__(self) -> None:
        """Validate and own read-only float64 inertia and drag arrays."""
        if not np.isfinite(self.mass):
            raise ValueError("mass must be finite")

        if self.mass <= 0.0:
            raise ValueError("mass must be positive")

        if self.inertia_B.shape != (3, 3):
            raise ValueError("inertia_B must have shape (3, 3)")

        owned_inertia_B = np.array(
            self.inertia_B,
            dtype=np.float64,
            copy=True,
        )

        if not np.all(np.isfinite(owned_inertia_B)):
            raise ValueError("inertia_B must contain only finite values")

        if not np.allclose(owned_inertia_B, owned_inertia_B.T):
            raise ValueError("inertia_B must be symmetric")

        try:
            np.linalg.cholesky(owned_inertia_B)
        except np.linalg.LinAlgError:
            raise ValueError("inertia_B must be positive definite") from None

        if self.quadratic_drag_coefficient_B.shape != (3,):
            raise ValueError("quadratic_drag_coefficient_B must have shape (3,)")

        owned_quadratic_drag_coefficient_B = np.array(
            self.quadratic_drag_coefficient_B,
            dtype=np.float64,
            order="C",
            copy=True,
        )
        if not np.all(np.isfinite(owned_quadratic_drag_coefficient_B)):
            raise ValueError("quadratic_drag_coefficient_B must contain only finite values")

        if np.any(owned_quadratic_drag_coefficient_B < 0.0):
            raise ValueError("quadratic_drag_coefficient_B must be nonnegative")

        owned_inertia_B.flags.writeable = False
        object.__setattr__(self, "inertia_B", owned_inertia_B)
        owned_quadratic_drag_coefficient_B.flags.writeable = False
        object.__setattr__(
            self,
            "quadratic_drag_coefficient_B",
            owned_quadratic_drag_coefficient_B,
        )


def _take_read_only_float64_array(
    values: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return an independently owned read-only float64 array."""
    owned_values = np.array(
        values,
        dtype=np.float64,
        copy=True,
    )
    owned_values.flags.writeable = False
    return owned_values


@dataclass(frozen=True, slots=True, eq=False)
class RotorParameters:
    """Store rotor geometry, aerodynamic coefficients, and optional motor parameters."""

    rotor_positions_B: NDArray[np.float64]
    rotor_spin_directions: NDArray[np.float64]
    thrust_coefficient: float
    moment_coefficient: float
    minimum_rotor_omega: float | None = None
    maximum_rotor_omega: float | None = None
    motor_time_constant_s: float | None = None

    def __post_init__(self) -> None:
        """Validate and take read-only float64 copies of rotor arrays."""
        if self.rotor_positions_B.shape != (4, 3):
            raise ValueError("rotor_positions_B must have shape (4, 3)")

        if self.rotor_spin_directions.shape != (4,):
            raise ValueError("rotor_spin_directions must have shape (4,)")

        if not np.all(np.isfinite(self.rotor_positions_B)):
            raise ValueError("rotor_positions_B must contain only finite values")

        if not np.all(np.isfinite(self.rotor_spin_directions)):
            raise ValueError("rotor_spin_directions must contain only finite values")

        valid_spin_directions = (self.rotor_spin_directions == -1.0) | (
            self.rotor_spin_directions == 1.0
        )
        if not np.all(valid_spin_directions):
            raise ValueError("rotor_spin_directions must contain only -1.0 or 1.0")

        if not np.isfinite(self.thrust_coefficient):
            raise ValueError("thrust_coefficient must be finite")

        if not np.isfinite(self.moment_coefficient):
            raise ValueError("moment_coefficient must be finite")

        if self.thrust_coefficient < 0.0:
            raise ValueError("thrust_coefficient must be nonnegative")

        if self.moment_coefficient < 0.0:
            raise ValueError("moment_coefficient must be nonnegative")

        minimum_rotor_omega = self.minimum_rotor_omega
        maximum_rotor_omega = self.maximum_rotor_omega
        motor_time_constant_s = self.motor_time_constant_s

        if not (
            minimum_rotor_omega is None
            and maximum_rotor_omega is None
            and motor_time_constant_s is None
        ):
            if (
                minimum_rotor_omega is None
                or maximum_rotor_omega is None
                or motor_time_constant_s is None
            ):
                raise ValueError("motor parameters must either all be provided or all be omitted")

            if not np.isfinite(minimum_rotor_omega):
                raise ValueError("minimum_rotor_omega must be finite")

            if not np.isfinite(maximum_rotor_omega):
                raise ValueError("maximum_rotor_omega must be finite")

            if minimum_rotor_omega < 0.0:
                raise ValueError("minimum_rotor_omega must be nonnegative")

            if maximum_rotor_omega <= minimum_rotor_omega:
                raise ValueError("maximum_rotor_omega must be greater than minimum_rotor_omega")

            maximum_safe_rotor_omega = np.sqrt(np.finfo(np.float64).max)
            if maximum_rotor_omega > maximum_safe_rotor_omega:
                raise ValueError("maximum_rotor_omega must be small enough to square safely")

            if not np.isfinite(motor_time_constant_s):
                raise ValueError("motor_time_constant_s must be finite")

            if motor_time_constant_s <= 0.0:
                raise ValueError("motor_time_constant_s must be positive")

        object.__setattr__(
            self,
            "rotor_positions_B",
            _take_read_only_float64_array(self.rotor_positions_B),
        )
        object.__setattr__(
            self,
            "rotor_spin_directions",
            _take_read_only_float64_array(self.rotor_spin_directions),
        )


@dataclass(frozen=True, slots=True, eq=False)
class WorldParameters:
    """Store world-model parameters."""

    gravity_acceleration: float
    wind_velocity_W: NDArray[np.float64] = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )

    def __post_init__(self) -> None:
        """Validate gravity and own a read-only float64 wind vector."""
        if not np.isfinite(self.gravity_acceleration):
            raise ValueError("gravity_acceleration must be finite")

        if self.gravity_acceleration < 0.0:
            raise ValueError("gravity_acceleration must be nonnegative")

        if self.wind_velocity_W.shape != (3,):
            raise ValueError("wind_velocity_W must have shape (3,)")

        owned_wind_velocity_W = np.array(
            self.wind_velocity_W,
            dtype=np.float64,
            order="C",
            copy=True,
        )
        if not np.all(np.isfinite(owned_wind_velocity_W)):
            raise ValueError("wind_velocity_W must contain only finite values")

        owned_wind_velocity_W.flags.writeable = False
        object.__setattr__(self, "wind_velocity_W", owned_wind_velocity_W)


@dataclass(frozen=True, slots=True, eq=False)
class ImuParameters:
    """Store immutable IMU bias and noise parameters."""

    initial_accelerometer_bias_B: NDArray[np.float64]
    accelerometer_noise_standard_deviation_B: NDArray[np.float64]
    accelerometer_bias_random_walk_density_B: NDArray[np.float64]
    initial_gyroscope_bias_B: NDArray[np.float64]
    gyroscope_noise_standard_deviation_B: NDArray[np.float64]
    gyroscope_bias_random_walk_density_B: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Validate and take read-only float64 copies of IMU arrays."""
        array_fields = (
            (
                "initial_accelerometer_bias_B",
                self.initial_accelerometer_bias_B,
            ),
            (
                "accelerometer_noise_standard_deviation_B",
                self.accelerometer_noise_standard_deviation_B,
            ),
            (
                "accelerometer_bias_random_walk_density_B",
                self.accelerometer_bias_random_walk_density_B,
            ),
            (
                "initial_gyroscope_bias_B",
                self.initial_gyroscope_bias_B,
            ),
            (
                "gyroscope_noise_standard_deviation_B",
                self.gyroscope_noise_standard_deviation_B,
            ),
            (
                "gyroscope_bias_random_walk_density_B",
                self.gyroscope_bias_random_walk_density_B,
            ),
        )

        for field_name, values in array_fields:
            if values.shape != (3,):
                raise ValueError(f"{field_name} must have shape (3,)")

        for field_name, values in array_fields:
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{field_name} must contain only finite values")

        nonnegative_array_fields = (
            (
                "accelerometer_noise_standard_deviation_B",
                self.accelerometer_noise_standard_deviation_B,
            ),
            (
                "accelerometer_bias_random_walk_density_B",
                self.accelerometer_bias_random_walk_density_B,
            ),
            (
                "gyroscope_noise_standard_deviation_B",
                self.gyroscope_noise_standard_deviation_B,
            ),
            (
                "gyroscope_bias_random_walk_density_B",
                self.gyroscope_bias_random_walk_density_B,
            ),
        )

        for field_name, values in nonnegative_array_fields:
            if np.any(values < 0.0):
                raise ValueError(f"{field_name} must be nonnegative")

        object.__setattr__(
            self,
            "initial_accelerometer_bias_B",
            _take_read_only_float64_array(self.initial_accelerometer_bias_B),
        )
        object.__setattr__(
            self,
            "accelerometer_noise_standard_deviation_B",
            _take_read_only_float64_array(self.accelerometer_noise_standard_deviation_B),
        )
        object.__setattr__(
            self,
            "accelerometer_bias_random_walk_density_B",
            _take_read_only_float64_array(self.accelerometer_bias_random_walk_density_B),
        )
        object.__setattr__(
            self,
            "initial_gyroscope_bias_B",
            _take_read_only_float64_array(self.initial_gyroscope_bias_B),
        )
        object.__setattr__(
            self,
            "gyroscope_noise_standard_deviation_B",
            _take_read_only_float64_array(self.gyroscope_noise_standard_deviation_B),
        )
        object.__setattr__(
            self,
            "gyroscope_bias_random_walk_density_B",
            _take_read_only_float64_array(self.gyroscope_bias_random_walk_density_B),
        )


@dataclass(frozen=True, slots=True, eq=False)
class PositionSensorParameters:
    """Store immutable local-position and barometric sensor parameters."""

    local_position_bias_W: NDArray[np.float64]
    local_position_noise_standard_deviation_W: NDArray[np.float64]
    barometric_reference_altitude: float
    barometric_altitude_bias: float
    barometric_altitude_noise_standard_deviation: float

    def __post_init__(self) -> None:
        """Validate and take read-only float64 copies of position arrays."""
        if self.local_position_bias_W.shape != (3,):
            raise ValueError("local_position_bias_W must have shape (3,)")

        if self.local_position_noise_standard_deviation_W.shape != (3,):
            raise ValueError("local_position_noise_standard_deviation_W must have shape (3,)")

        if not np.all(np.isfinite(self.local_position_bias_W)):
            raise ValueError("local_position_bias_W must contain only finite values")

        if not np.all(np.isfinite(self.local_position_noise_standard_deviation_W)):
            raise ValueError(
                "local_position_noise_standard_deviation_W must contain only finite values"
            )

        if np.any(self.local_position_noise_standard_deviation_W < 0.0):
            raise ValueError("local_position_noise_standard_deviation_W must be nonnegative")

        if not np.isfinite(self.barometric_reference_altitude):
            raise ValueError("barometric_reference_altitude must be finite")

        if not np.isfinite(self.barometric_altitude_bias):
            raise ValueError("barometric_altitude_bias must be finite")

        if not np.isfinite(self.barometric_altitude_noise_standard_deviation):
            raise ValueError("barometric_altitude_noise_standard_deviation must be finite")

        if self.barometric_altitude_noise_standard_deviation < 0.0:
            raise ValueError("barometric_altitude_noise_standard_deviation must be nonnegative")

        object.__setattr__(
            self,
            "local_position_bias_W",
            _take_read_only_float64_array(self.local_position_bias_W),
        )
        object.__setattr__(
            self,
            "local_position_noise_standard_deviation_W",
            _take_read_only_float64_array(self.local_position_noise_standard_deviation_W),
        )


@dataclass(frozen=True, slots=True, eq=False)
class TruthConfiguration:
    """Group actual plant, world, and sensor parameters used by truth."""

    rigid_body: RigidBodyParameters
    rotors: RotorParameters
    world: WorldParameters
    imu: ImuParameters
    position_sensors: PositionSensorParameters


@dataclass(frozen=True, slots=True, eq=False)
class NominalConfiguration:
    """Group independently assumed model and sensor parameters."""

    rigid_body: RigidBodyParameters
    rotors: RotorParameters
    world: WorldParameters
    imu: ImuParameters
    position_sensors: PositionSensorParameters


def _truth_nominal_parameter_equalities(
    truth: TruthConfiguration,
    nominal: NominalConfiguration,
) -> dict[str, bool]:
    """Return exact equality results for every supported mismatch parameter."""
    return {
        "rigid_body.mass": bool(truth.rigid_body.mass == nominal.rigid_body.mass),
        "rigid_body.inertia_B": bool(
            np.array_equal(truth.rigid_body.inertia_B, nominal.rigid_body.inertia_B)
        ),
        "rigid_body.quadratic_drag_coefficient_B": bool(
            np.array_equal(
                truth.rigid_body.quadratic_drag_coefficient_B,
                nominal.rigid_body.quadratic_drag_coefficient_B,
            )
        ),
        "rotors.rotor_positions_B": bool(
            np.array_equal(
                truth.rotors.rotor_positions_B,
                nominal.rotors.rotor_positions_B,
            )
        ),
        "rotors.rotor_spin_directions": bool(
            np.array_equal(
                truth.rotors.rotor_spin_directions,
                nominal.rotors.rotor_spin_directions,
            )
        ),
        "rotors.thrust_coefficient": bool(
            truth.rotors.thrust_coefficient == nominal.rotors.thrust_coefficient
        ),
        "rotors.moment_coefficient": bool(
            truth.rotors.moment_coefficient == nominal.rotors.moment_coefficient
        ),
        "rotors.minimum_rotor_omega": bool(
            truth.rotors.minimum_rotor_omega == nominal.rotors.minimum_rotor_omega
        ),
        "rotors.maximum_rotor_omega": bool(
            truth.rotors.maximum_rotor_omega == nominal.rotors.maximum_rotor_omega
        ),
        "rotors.motor_time_constant_s": bool(
            truth.rotors.motor_time_constant_s == nominal.rotors.motor_time_constant_s
        ),
        "world.gravity_acceleration": bool(
            truth.world.gravity_acceleration == nominal.world.gravity_acceleration
        ),
        "world.wind_velocity_W": bool(
            np.array_equal(truth.world.wind_velocity_W, nominal.world.wind_velocity_W)
        ),
        "imu.initial_accelerometer_bias_B": bool(
            np.array_equal(
                truth.imu.initial_accelerometer_bias_B,
                nominal.imu.initial_accelerometer_bias_B,
            )
        ),
        "imu.accelerometer_noise_standard_deviation_B": bool(
            np.array_equal(
                truth.imu.accelerometer_noise_standard_deviation_B,
                nominal.imu.accelerometer_noise_standard_deviation_B,
            )
        ),
        "imu.accelerometer_bias_random_walk_density_B": bool(
            np.array_equal(
                truth.imu.accelerometer_bias_random_walk_density_B,
                nominal.imu.accelerometer_bias_random_walk_density_B,
            )
        ),
        "imu.initial_gyroscope_bias_B": bool(
            np.array_equal(
                truth.imu.initial_gyroscope_bias_B,
                nominal.imu.initial_gyroscope_bias_B,
            )
        ),
        "imu.gyroscope_noise_standard_deviation_B": bool(
            np.array_equal(
                truth.imu.gyroscope_noise_standard_deviation_B,
                nominal.imu.gyroscope_noise_standard_deviation_B,
            )
        ),
        "imu.gyroscope_bias_random_walk_density_B": bool(
            np.array_equal(
                truth.imu.gyroscope_bias_random_walk_density_B,
                nominal.imu.gyroscope_bias_random_walk_density_B,
            )
        ),
        "position_sensors.local_position_bias_W": bool(
            np.array_equal(
                truth.position_sensors.local_position_bias_W,
                nominal.position_sensors.local_position_bias_W,
            )
        ),
        "position_sensors.local_position_noise_standard_deviation_W": bool(
            np.array_equal(
                truth.position_sensors.local_position_noise_standard_deviation_W,
                nominal.position_sensors.local_position_noise_standard_deviation_W,
            )
        ),
        "position_sensors.barometric_reference_altitude": bool(
            truth.position_sensors.barometric_reference_altitude
            == nominal.position_sensors.barometric_reference_altitude
        ),
        "position_sensors.barometric_altitude_bias": bool(
            truth.position_sensors.barometric_altitude_bias
            == nominal.position_sensors.barometric_altitude_bias
        ),
        "position_sensors.barometric_altitude_noise_standard_deviation": bool(
            truth.position_sensors.barometric_altitude_noise_standard_deviation
            == nominal.position_sensors.barometric_altitude_noise_standard_deviation
        ),
    }


class IntegrationMethod(StrEnum):
    """Stable manifest names for supported truth integration methods."""

    EULER = "euler"
    PROJECTED_RK4 = "projected_rk4"


@dataclass(frozen=True, slots=True, eq=False)
class RigidBodyInitialState:
    """Store an immutable independently owned rigid-body initial state."""

    position_W: NDArray[np.float64]
    velocity_W: NDArray[np.float64]
    q_WB: NDArray[np.float64]
    omega_B: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Validate and take read-only float64 copies of state arrays."""
        array_fields = (
            ("position_W", self.position_W, (3,)),
            ("velocity_W", self.velocity_W, (3,)),
            ("q_WB", self.q_WB, (4,)),
            ("omega_B", self.omega_B, (3,)),
        )

        for field_name, values, expected_shape in array_fields:
            if values.shape != expected_shape:
                raise ValueError(f"{field_name} must have shape {expected_shape}")

        for field_name, values, _ in array_fields:
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{field_name} must contain only finite values")

        if not np.isclose(
            np.linalg.norm(self.q_WB),
            1.0,
            rtol=1e-12,
            atol=1e-12,
        ):
            raise ValueError("q_WB must have unit norm")

        object.__setattr__(
            self,
            "position_W",
            _take_read_only_float64_array(self.position_W),
        )
        object.__setattr__(
            self,
            "velocity_W",
            _take_read_only_float64_array(self.velocity_W),
        )
        object.__setattr__(
            self,
            "q_WB",
            _take_read_only_float64_array(self.q_WB),
        )
        object.__setattr__(
            self,
            "omega_B",
            _take_read_only_float64_array(self.omega_B),
        )


@dataclass(frozen=True, slots=True, eq=False)
class RunNumerics:
    """Store truth-integration method, time step, and authoritative horizon."""

    integration_method: IntegrationMethod
    truth_time_step_s: float
    number_of_steps: int

    def __post_init__(self) -> None:
        """Validate truth-integration method, time step, and horizon."""
        if not isinstance(self.integration_method, IntegrationMethod):
            raise TypeError("integration_method must be an IntegrationMethod")

        if not np.isfinite(self.truth_time_step_s):
            raise ValueError("truth_time_step_s must be finite")

        if self.truth_time_step_s <= 0.0:
            raise ValueError("truth_time_step_s must be positive")

        if isinstance(self.number_of_steps, bool) or not isinstance(
            self.number_of_steps,
            int,
        ):
            raise TypeError("number_of_steps must be a non-Boolean integer")

        if self.number_of_steps <= 0:
            raise ValueError("number_of_steps must be positive")


@dataclass(frozen=True, slots=True, eq=False)
class SensorSchedule:
    """Store one sensor's requested sample period and delivery delay."""

    sample_period_s: float
    delivery_delay_s: float

    def __post_init__(self) -> None:
        """Validate the requested sample period and delivery delay."""
        if not np.isfinite(self.sample_period_s):
            raise ValueError("sample_period_s must be finite")

        if self.sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be positive")

        if not np.isfinite(self.delivery_delay_s):
            raise ValueError("delivery_delay_s must be finite")

        if self.delivery_delay_s < 0.0:
            raise ValueError("delivery_delay_s must be nonnegative")


@dataclass(frozen=True, slots=True, eq=False)
class SensorSchedules:
    """Group the explicit schedules for every supported sensor channel."""

    accelerometer: SensorSchedule
    gyroscope: SensorSchedule
    local_position: SensorSchedule
    barometric_altitude: SensorSchedule


@dataclass(frozen=True, slots=True, eq=False)
class ConstantRotorSpeedInput:
    """Store an immutable independently owned constant rotor-speed input."""

    rotor_omega: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Validate and take a read-only float64 copy of rotor speeds."""
        if self.rotor_omega.shape != (4,):
            raise ValueError("rotor_omega must have shape (4,)")

        if not np.all(np.isfinite(self.rotor_omega)):
            raise ValueError("rotor_omega must contain only finite values")

        if np.any(self.rotor_omega < 0.0):
            raise ValueError("rotor_omega must be nonnegative")

        object.__setattr__(
            self,
            "rotor_omega",
            _take_read_only_float64_array(self.rotor_omega),
        )


@dataclass(frozen=True, slots=True, eq=False)
class DeclaredMismatch:
    """Describe one intentional difference between truth and nominal models."""

    parameter_path: str
    rationale: str

    def __post_init__(self) -> None:
        """Validate that the mismatch description is nonempty."""
        if not self.parameter_path.strip():
            raise ValueError("parameter_path must be nonempty")

        if not self.rationale.strip():
            raise ValueError("rationale must be nonempty")


@dataclass(frozen=True, slots=True, eq=False)
class RunConfiguration:
    """Group the complete immutable inputs required to reproduce one run.

    initial_actual_rotor_omega is the actual rotor angular-speed state at the
    initial truth time, in rad/s. None preserves historical configurations.
    """

    truth: TruthConfiguration
    nominal: NominalConfiguration
    initial_truth_state: RigidBodyInitialState
    numerics: RunNumerics
    sensor_schedules: SensorSchedules
    rotor_speed_input: ConstantRotorSpeedInput
    root_seed: int
    declared_mismatches: tuple[DeclaredMismatch, ...]
    initial_actual_rotor_omega: NDArray[np.float64] | None = None

    def __post_init__(self) -> None:
        """Validate run-level values and own the mismatch sequence."""
        if isinstance(self.root_seed, bool) or not isinstance(
            self.root_seed,
            int,
        ):
            raise TypeError("root_seed must be a non-Boolean integer")

        if self.root_seed < 0 or self.root_seed >= 2**128:
            raise ValueError("root_seed must be in [0, 2**128)")

        if not all(isinstance(mismatch, DeclaredMismatch) for mismatch in self.declared_mismatches):
            raise TypeError("declared_mismatches must contain only DeclaredMismatch values")

        schedules = (
            self.sensor_schedules.accelerometer,
            self.sensor_schedules.gyroscope,
            self.sensor_schedules.local_position,
            self.sensor_schedules.barometric_altitude,
        )
        for schedule in schedules:
            sample_stride_ratio = schedule.sample_period_s / self.numerics.truth_time_step_s
            if not np.isfinite(sample_stride_ratio):
                raise ValueError("sample_period_s must be an integer multiple of truth_time_step_s")

            sample_stride = int(round(sample_stride_ratio))
            effective_sample_period_s = sample_stride * self.numerics.truth_time_step_s
            if sample_stride <= 0 or not np.isclose(
                schedule.sample_period_s,
                effective_sample_period_s,
                rtol=1e-12,
                atol=0.0,
            ):
                raise ValueError("sample_period_s must be an integer multiple of truth_time_step_s")

        parameter_equalities = _truth_nominal_parameter_equalities(
            self.truth,
            self.nominal,
        )
        owned_declared_mismatches = tuple(self.declared_mismatches)
        declared_parameter_paths = tuple(
            mismatch.parameter_path for mismatch in owned_declared_mismatches
        )

        for parameter_path in declared_parameter_paths:
            if parameter_path not in parameter_equalities:
                raise ValueError(
                    f"declared mismatch parameter_path is not supported: {parameter_path}"
                )

        if len(set(declared_parameter_paths)) != len(declared_parameter_paths):
            raise ValueError("declared_mismatches must not contain duplicate parameter_path values")

        declared_parameter_path_set = set(declared_parameter_paths)
        for parameter_path, parameters_are_equal in parameter_equalities.items():
            if not parameters_are_equal and parameter_path not in declared_parameter_path_set:
                raise ValueError(
                    f"truth and nominal differ without a declared mismatch: {parameter_path}"
                )

        for parameter_path, parameters_are_equal in parameter_equalities.items():
            if parameters_are_equal and parameter_path in declared_parameter_path_set:
                raise ValueError(
                    f"declared mismatch has equal truth and nominal values: {parameter_path}"
                )

        object.__setattr__(
            self,
            "declared_mismatches",
            owned_declared_mismatches,
        )
        if self.initial_actual_rotor_omega is not None:
            minimum_rotor_omega = self.truth.rotors.minimum_rotor_omega
            maximum_rotor_omega = self.truth.rotors.maximum_rotor_omega
            if minimum_rotor_omega is None or maximum_rotor_omega is None:
                raise ValueError("initial_actual_rotor_omega requires truth motor parameters")

            owned_initial_actual_rotor_omega = np.array(
                self.initial_actual_rotor_omega,
                dtype=np.float64,
                order="C",
                copy=True,
            )
            if owned_initial_actual_rotor_omega.shape != (4,):
                raise ValueError("initial_actual_rotor_omega must have shape (4,)")

            if not np.all(np.isfinite(owned_initial_actual_rotor_omega)):
                raise ValueError("initial_actual_rotor_omega must contain only finite values")

            if np.any(owned_initial_actual_rotor_omega < minimum_rotor_omega) or np.any(
                owned_initial_actual_rotor_omega > maximum_rotor_omega
            ):
                raise ValueError(
                    "initial_actual_rotor_omega must be within truth rotor speed limits"
                )

            owned_initial_actual_rotor_omega.flags.writeable = False
            object.__setattr__(
                self,
                "initial_actual_rotor_omega",
                owned_initial_actual_rotor_omega,
            )
