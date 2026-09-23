"""Pure canonical JSON representation of one reproducible run."""

import json
import platform
import subprocess
from dataclasses import dataclass
from importlib import metadata
from pathlib import Path
from typing import Final, Never, cast

import numpy as np
from numpy.typing import NDArray

from .randomness import (
    _RUN_RANDOM_STREAM_DERIVATION_VERSION,
    _RUN_RANDOM_STREAM_IDS,
    RunRandomStream,
)
from .run_configuration import (
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

RUN_MANIFEST_SCHEMA_NAME: Final[str] = "robust_quadrotor.run_manifest"
RUN_MANIFEST_SCHEMA_VERSION: Final[int] = 1
_ARTIFACT_BOUND_SCHEMA_VERSION: Final[int] = 2
_MOTORIZED_UNBOUND_SCHEMA_VERSION: Final[int] = 3
_MOTORIZED_BOUND_SCHEMA_VERSION: Final[int] = 4
_ENVIRONMENTAL_UNBOUND_SCHEMA_VERSION: Final[int] = 5
_ENVIRONMENTAL_BOUND_SCHEMA_VERSION: Final[int] = 6
_BOUND_SCHEMA_VERSIONS: Final[frozenset[int]] = frozenset(
    {
        _ARTIFACT_BOUND_SCHEMA_VERSION,
        _MOTORIZED_BOUND_SCHEMA_VERSION,
        _ENVIRONMENTAL_BOUND_SCHEMA_VERSION,
    }
)
_MOTORIZED_SCHEMA_VERSIONS: Final[frozenset[int]] = frozenset(
    {_MOTORIZED_UNBOUND_SCHEMA_VERSION, _MOTORIZED_BOUND_SCHEMA_VERSION}
)
_MOTOR_FIELD_SCHEMA_VERSIONS: Final[frozenset[int]] = frozenset(
    {
        _MOTORIZED_UNBOUND_SCHEMA_VERSION,
        _MOTORIZED_BOUND_SCHEMA_VERSION,
        _ENVIRONMENTAL_UNBOUND_SCHEMA_VERSION,
        _ENVIRONMENTAL_BOUND_SCHEMA_VERSION,
    }
)
_ENVIRONMENTAL_SCHEMA_VERSIONS: Final[frozenset[int]] = frozenset(
    {_ENVIRONMENTAL_UNBOUND_SCHEMA_VERSION, _ENVIRONMENTAL_BOUND_SCHEMA_VERSION}
)

_MANIFEST_V1_KEYS: Final[frozenset[str]] = frozenset(
    {"randomness", "run_configuration", "schema", "software_provenance"}
)
_MANIFEST_V2_KEYS: Final[frozenset[str]] = frozenset(
    {"data_artifact", "randomness", "run_configuration", "schema", "software_provenance"}
)
_DATA_ARTIFACT_KEYS: Final[frozenset[str]] = frozenset({"sha256"})
_SCHEMA_KEYS: Final[frozenset[str]] = frozenset({"name", "version"})
_RANDOMNESS_KEYS: Final[frozenset[str]] = frozenset({"derivation_version", "streams"})
_RANDOM_STREAM_KEYS: Final[frozenset[str]] = frozenset({"id", "name"})
_SOFTWARE_PROVENANCE_KEYS: Final[frozenset[str]] = frozenset(
    {
        "git_commit_sha",
        "git_worktree_clean",
        "numpy_version",
        "package_version",
        "python_version",
    }
)
_RUN_CONFIGURATION_KEYS: Final[frozenset[str]] = frozenset(
    {
        "declared_mismatches",
        "initial_truth_state",
        "nominal",
        "numerics",
        "root_seed",
        "rotor_speed_input",
        "sensor_schedules",
        "truth",
    }
)
_RUN_CONFIGURATION_V3_KEYS: Final[frozenset[str]] = _RUN_CONFIGURATION_KEYS | frozenset(
    {"initial_actual_rotor_omega"}
)
_CONFIGURATION_GROUP_KEYS: Final[frozenset[str]] = frozenset(
    {"imu", "position_sensors", "rigid_body", "rotors", "world"}
)
_INITIAL_TRUTH_STATE_KEYS: Final[frozenset[str]] = frozenset(
    {"omega_B", "position_W", "q_WB", "velocity_W"}
)
_NUMERICS_KEYS: Final[frozenset[str]] = frozenset(
    {"duration_s", "integration_method", "number_of_steps", "truth_time_step_s"}
)
_SENSOR_SCHEDULES_KEYS: Final[frozenset[str]] = frozenset(
    {"accelerometer", "barometric_altitude", "gyroscope", "local_position"}
)
_ROTOR_SPEED_INPUT_KEYS: Final[frozenset[str]] = frozenset({"rotor_omega"})
_RIGID_BODY_KEYS: Final[frozenset[str]] = frozenset({"inertia_B", "mass"})
_ENVIRONMENTAL_RIGID_BODY_KEYS: Final[frozenset[str]] = _RIGID_BODY_KEYS | frozenset(
    {"quadratic_drag_coefficient_B"}
)
_ROTOR_KEYS: Final[frozenset[str]] = frozenset(
    {"moment_coefficient", "rotor_positions_B", "rotor_spin_directions", "thrust_coefficient"}
)
_ROTOR_V3_KEYS: Final[frozenset[str]] = _ROTOR_KEYS | frozenset(
    {"minimum_rotor_omega", "maximum_rotor_omega", "motor_time_constant_s"}
)
_WORLD_KEYS: Final[frozenset[str]] = frozenset({"gravity_acceleration"})
_ENVIRONMENTAL_WORLD_KEYS: Final[frozenset[str]] = _WORLD_KEYS | frozenset({"wind_velocity_W"})
_IMU_KEYS: Final[frozenset[str]] = frozenset(
    {
        "accelerometer_bias_random_walk_density_B",
        "accelerometer_noise_standard_deviation_B",
        "gyroscope_bias_random_walk_density_B",
        "gyroscope_noise_standard_deviation_B",
        "initial_accelerometer_bias_B",
        "initial_gyroscope_bias_B",
    }
)
_POSITION_SENSOR_KEYS: Final[frozenset[str]] = frozenset(
    {
        "barometric_altitude_bias",
        "barometric_altitude_noise_standard_deviation",
        "barometric_reference_altitude",
        "local_position_bias_W",
        "local_position_noise_standard_deviation_W",
    }
)
_SENSOR_SCHEDULE_KEYS: Final[frozenset[str]] = frozenset(
    {
        "delivery_delay_s",
        "effective_sample_period_s",
        "requested_sample_period_s",
        "sample_stride",
    }
)
_DECLARED_MISMATCH_KEYS: Final[frozenset[str]] = frozenset({"parameter_path", "rationale"})


@dataclass(frozen=True, slots=True)
class SoftwareProvenance:
    """Represent caller-supplied software provenance without capturing an environment."""

    package_version: str
    python_version: str
    numpy_version: str
    git_commit_sha: str
    git_worktree_clean: bool


def capture_software_provenance(repository_root: Path) -> SoftwareProvenance:
    """Capture installed versions and Git state for an explicit repository root."""
    package_version = metadata.version("robust-quadrotor")
    python_version = platform.python_version()
    numpy_version = np.__version__

    commit_result = subprocess.run(
        ["git", "-C", str(repository_root), "rev-parse", "--verify", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    )
    status_result = subprocess.run(
        [
            "git",
            "-C",
            str(repository_root),
            "status",
            "--porcelain=v1",
            "--untracked-files=normal",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return SoftwareProvenance(
        package_version=package_version,
        python_version=python_version,
        numpy_version=numpy_version,
        git_commit_sha=commit_result.stdout.strip(),
        git_worktree_clean=status_result.stdout == "",
    )


@dataclass(frozen=True, slots=True, eq=False)
class RunManifest:
    """Hold a complete run configuration and its supplied software provenance."""

    run_configuration: RunConfiguration
    software_provenance: SoftwareProvenance
    data_npz_sha256: str | None = None


def _encode_rigid_body(
    parameters: RigidBodyParameters, *, include_environmental_parameters: bool = False
) -> dict[str, object]:
    encoded_rigid_body: dict[str, object] = {
        "mass": parameters.mass,
        "inertia_B": parameters.inertia_B.tolist(),
    }
    if include_environmental_parameters:
        encoded_rigid_body["quadratic_drag_coefficient_B"] = (
            parameters.quadratic_drag_coefficient_B.tolist()
        )
    return encoded_rigid_body


def _encode_rotors(
    parameters: RotorParameters,
    *,
    include_motor_parameters: bool = False,
    allow_omitted_motor_parameters: bool = False,
) -> dict[str, object]:
    encoded_rotors: dict[str, object] = {
        "rotor_positions_B": parameters.rotor_positions_B.tolist(),
        "rotor_spin_directions": parameters.rotor_spin_directions.tolist(),
        "thrust_coefficient": parameters.thrust_coefficient,
        "moment_coefficient": parameters.moment_coefficient,
    }
    if include_motor_parameters:
        minimum_rotor_omega = parameters.minimum_rotor_omega
        maximum_rotor_omega = parameters.maximum_rotor_omega
        motor_time_constant_s = parameters.motor_time_constant_s
        if (
            minimum_rotor_omega is None
            or maximum_rotor_omega is None
            or motor_time_constant_s is None
        ) and not (
            allow_omitted_motor_parameters
            and minimum_rotor_omega is None
            and maximum_rotor_omega is None
            and motor_time_constant_s is None
        ):
            raise ValueError("version-3 rotor parameters must be complete")
        encoded_rotors.update(
            {
                "minimum_rotor_omega": minimum_rotor_omega,
                "maximum_rotor_omega": maximum_rotor_omega,
                "motor_time_constant_s": motor_time_constant_s,
            }
        )
    return encoded_rotors


def _encode_world(
    parameters: WorldParameters, *, include_environmental_parameters: bool = False
) -> dict[str, object]:
    encoded_world: dict[str, object] = {"gravity_acceleration": parameters.gravity_acceleration}
    if include_environmental_parameters:
        encoded_world["wind_velocity_W"] = parameters.wind_velocity_W.tolist()
    return encoded_world


def _encode_imu(parameters: ImuParameters) -> dict[str, object]:
    return {
        "initial_accelerometer_bias_B": parameters.initial_accelerometer_bias_B.tolist(),
        "accelerometer_noise_standard_deviation_B": (
            parameters.accelerometer_noise_standard_deviation_B.tolist()
        ),
        "accelerometer_bias_random_walk_density_B": (
            parameters.accelerometer_bias_random_walk_density_B.tolist()
        ),
        "initial_gyroscope_bias_B": parameters.initial_gyroscope_bias_B.tolist(),
        "gyroscope_noise_standard_deviation_B": (
            parameters.gyroscope_noise_standard_deviation_B.tolist()
        ),
        "gyroscope_bias_random_walk_density_B": (
            parameters.gyroscope_bias_random_walk_density_B.tolist()
        ),
    }


def _encode_position_sensors(parameters: PositionSensorParameters) -> dict[str, object]:
    return {
        "local_position_bias_W": parameters.local_position_bias_W.tolist(),
        "local_position_noise_standard_deviation_W": (
            parameters.local_position_noise_standard_deviation_W.tolist()
        ),
        "barometric_reference_altitude": parameters.barometric_reference_altitude,
        "barometric_altitude_bias": parameters.barometric_altitude_bias,
        "barometric_altitude_noise_standard_deviation": (
            parameters.barometric_altitude_noise_standard_deviation
        ),
    }


def _encode_configuration_group(
    configuration: TruthConfiguration | NominalConfiguration,
    *,
    include_motor_parameters: bool = False,
    allow_omitted_motor_parameters: bool = False,
    include_environmental_parameters: bool = False,
) -> dict[str, object]:
    return {
        "rigid_body": _encode_rigid_body(
            configuration.rigid_body,
            include_environmental_parameters=include_environmental_parameters,
        ),
        "rotors": _encode_rotors(
            configuration.rotors,
            include_motor_parameters=include_motor_parameters,
            allow_omitted_motor_parameters=allow_omitted_motor_parameters,
        ),
        "world": _encode_world(
            configuration.world,
            include_environmental_parameters=include_environmental_parameters,
        ),
        "imu": _encode_imu(configuration.imu),
        "position_sensors": _encode_position_sensors(configuration.position_sensors),
    }


def _encode_initial_truth_state(state: RigidBodyInitialState) -> dict[str, object]:
    return {
        "position_W": state.position_W.tolist(),
        "velocity_W": state.velocity_W.tolist(),
        "q_WB": state.q_WB.tolist(),
        "omega_B": state.omega_B.tolist(),
    }


def _encode_numerics(numerics: RunNumerics) -> dict[str, object]:
    return {
        "integration_method": numerics.integration_method.value,
        "truth_time_step_s": numerics.truth_time_step_s,
        "number_of_steps": numerics.number_of_steps,
        "duration_s": numerics.number_of_steps * numerics.truth_time_step_s,
    }


def _encode_sensor_schedule(
    schedule: SensorSchedule,
    truth_time_step_s: float,
) -> dict[str, object]:
    sample_stride = int(round(schedule.sample_period_s / truth_time_step_s))
    return {
        "requested_sample_period_s": schedule.sample_period_s,
        "sample_stride": sample_stride,
        "effective_sample_period_s": sample_stride * truth_time_step_s,
        "delivery_delay_s": schedule.delivery_delay_s,
    }


def _encode_sensor_schedules(
    schedules: SensorSchedules,
    truth_time_step_s: float,
) -> dict[str, object]:
    return {
        "accelerometer": _encode_sensor_schedule(schedules.accelerometer, truth_time_step_s),
        "gyroscope": _encode_sensor_schedule(schedules.gyroscope, truth_time_step_s),
        "local_position": _encode_sensor_schedule(schedules.local_position, truth_time_step_s),
        "barometric_altitude": _encode_sensor_schedule(
            schedules.barometric_altitude, truth_time_step_s
        ),
    }


def _encode_rotor_speed_input(rotor_input: ConstantRotorSpeedInput) -> dict[str, object]:
    return {"rotor_omega": rotor_input.rotor_omega.tolist()}


def _encode_declared_mismatch(mismatch: DeclaredMismatch) -> dict[str, object]:
    return {
        "parameter_path": mismatch.parameter_path,
        "rationale": mismatch.rationale,
    }


def _encode_run_configuration(
    configuration: RunConfiguration,
    *,
    include_motor_parameters: bool = False,
    allow_omitted_motor_parameters: bool = False,
    include_environmental_parameters: bool = False,
) -> dict[str, object]:
    encoded_configuration: dict[str, object] = {
        "truth": _encode_configuration_group(
            configuration.truth,
            include_motor_parameters=include_motor_parameters,
            allow_omitted_motor_parameters=allow_omitted_motor_parameters,
            include_environmental_parameters=include_environmental_parameters,
        ),
        "nominal": _encode_configuration_group(
            configuration.nominal,
            include_motor_parameters=include_motor_parameters,
            allow_omitted_motor_parameters=allow_omitted_motor_parameters,
            include_environmental_parameters=include_environmental_parameters,
        ),
        "initial_truth_state": _encode_initial_truth_state(configuration.initial_truth_state),
        "numerics": _encode_numerics(configuration.numerics),
        "sensor_schedules": _encode_sensor_schedules(
            configuration.sensor_schedules,
            configuration.numerics.truth_time_step_s,
        ),
        "rotor_speed_input": _encode_rotor_speed_input(configuration.rotor_speed_input),
        "root_seed": str(configuration.root_seed),
        "declared_mismatches": [
            _encode_declared_mismatch(mismatch) for mismatch in configuration.declared_mismatches
        ],
    }
    if include_motor_parameters:
        initial_actual_rotor_omega = configuration.initial_actual_rotor_omega
        if initial_actual_rotor_omega is None and not allow_omitted_motor_parameters:
            raise ValueError("version-3 initial actual rotor speed must be present")
        encoded_configuration["initial_actual_rotor_omega"] = (
            initial_actual_rotor_omega.tolist() if initial_actual_rotor_omega is not None else None
        )
    return encoded_configuration


def _has_complete_motor_configuration(configuration: RunConfiguration) -> bool:
    """Distinguish wholly historical, complete modern, and partial motor state."""
    motor_values = (
        configuration.truth.rotors.minimum_rotor_omega,
        configuration.truth.rotors.maximum_rotor_omega,
        configuration.truth.rotors.motor_time_constant_s,
        configuration.nominal.rotors.minimum_rotor_omega,
        configuration.nominal.rotors.maximum_rotor_omega,
        configuration.nominal.rotors.motor_time_constant_s,
        configuration.initial_actual_rotor_omega,
    )
    if all(value is None for value in motor_values):
        return False
    if all(value is not None for value in motor_values):
        return True
    raise ValueError("manifest motor configuration must be either entirely omitted or complete")


def _has_nonzero_environmental_configuration(configuration: RunConfiguration) -> bool:
    """Return whether any validated truth or nominal environmental element is nonzero."""
    environmental_arrays = (
        configuration.truth.rigid_body.quadratic_drag_coefficient_B,
        configuration.nominal.rigid_body.quadratic_drag_coefficient_B,
        configuration.truth.world.wind_velocity_W,
        configuration.nominal.world.wind_velocity_W,
    )
    return any(np.any(values != 0.0) for values in environmental_arrays)


def _ordered_run_random_streams() -> list[tuple[RunRandomStream, int]]:
    """Return the authoritative run streams in stable numeric-ID order."""
    return sorted(_RUN_RANDOM_STREAM_IDS.items(), key=lambda item: item[1])


def _encode_randomness() -> dict[str, object]:
    streams_by_id = _ordered_run_random_streams()
    return {
        "derivation_version": _RUN_RANDOM_STREAM_DERIVATION_VERSION,
        "streams": [{"id": stream_id, "name": stream.value} for stream, stream_id in streams_by_id],
    }


def _encode_software_provenance(provenance: SoftwareProvenance) -> dict[str, object]:
    return {
        "package_version": provenance.package_version,
        "python_version": provenance.python_version,
        "numpy_version": provenance.numpy_version,
        "git_commit_sha": provenance.git_commit_sha,
        "git_worktree_clean": provenance.git_worktree_clean,
    }


def _validate_data_npz_sha256(value: object) -> str:
    """Require one exact lowercase SHA-256 digest without rewriting it."""
    error_message = "data_artifact.sha256 must be 64 lowercase hexadecimal characters"
    if type(value) is not str:
        raise ValueError(error_message)
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ValueError(error_message)
    return value


def encode_run_manifest(manifest: RunManifest) -> bytes:
    """Encode a supplied manifest as compact, sorted, finite UTF-8 JSON bytes."""
    schema_version = RUN_MANIFEST_SCHEMA_VERSION
    data_npz_sha256 = manifest.data_npz_sha256
    has_complete_motor_configuration = _has_complete_motor_configuration(manifest.run_configuration)
    has_nonzero_environmental_configuration = _has_nonzero_environmental_configuration(
        manifest.run_configuration
    )
    include_motor_parameters = (
        has_complete_motor_configuration or has_nonzero_environmental_configuration
    )
    if has_nonzero_environmental_configuration:
        schema_version = _ENVIRONMENTAL_UNBOUND_SCHEMA_VERSION
    if data_npz_sha256 is not None:
        data_npz_sha256 = _validate_data_npz_sha256(data_npz_sha256)
        if has_nonzero_environmental_configuration:
            schema_version = _ENVIRONMENTAL_BOUND_SCHEMA_VERSION
        else:
            schema_version = (
                _MOTORIZED_BOUND_SCHEMA_VERSION
                if has_complete_motor_configuration
                else _ARTIFACT_BOUND_SCHEMA_VERSION
            )
    elif has_complete_motor_configuration and not has_nonzero_environmental_configuration:
        schema_version = _MOTORIZED_UNBOUND_SCHEMA_VERSION

    manifest_mapping: dict[str, object] = {
        "schema": {
            "name": RUN_MANIFEST_SCHEMA_NAME,
            "version": schema_version,
        },
        "run_configuration": _encode_run_configuration(
            manifest.run_configuration,
            include_motor_parameters=include_motor_parameters,
            allow_omitted_motor_parameters=has_nonzero_environmental_configuration,
            include_environmental_parameters=has_nonzero_environmental_configuration,
        ),
        "randomness": _encode_randomness(),
        "software_provenance": _encode_software_provenance(manifest.software_provenance),
    }
    if data_npz_sha256 is not None:
        manifest_mapping["data_artifact"] = {"sha256": data_npz_sha256}
    return json.dumps(
        manifest_mapping,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def _decoded_object(value: object) -> dict[str, object]:
    """Type one valid parsed JSON object for explicit field access."""
    return cast(dict[str, object], value)


def _require_json_object(value: object, *, error_message: str) -> dict[str, object]:
    """Require one concrete manifest-envelope JSON object."""
    if not isinstance(value, dict):
        raise ValueError(error_message)
    return cast(dict[str, object], value)


def _require_json_array(value: object, *, error_message: str) -> list[object]:
    """Require one concrete parsed JSON array."""
    if not isinstance(value, list):
        raise ValueError(error_message)
    return cast(list[object], value)


def _require_exact_keys(
    mapping: dict[str, object],
    *,
    expected_keys: frozenset[str],
    error_message: str,
) -> None:
    """Require one concrete manifest-envelope key set."""
    if frozenset(mapping) != expected_keys:
        raise ValueError(error_message)


def _validate_randomness(value: object) -> None:
    """Require exact versioned metadata for the ordered run random streams."""
    randomness_mapping = _require_json_object(
        value,
        error_message="randomness must be a JSON object",
    )
    _require_exact_keys(
        randomness_mapping,
        expected_keys=_RANDOMNESS_KEYS,
        error_message="randomness keys must be exactly: derivation_version, streams",
    )
    derivation_version = randomness_mapping["derivation_version"]
    if (
        type(derivation_version) is not int
        or derivation_version != _RUN_RANDOM_STREAM_DERIVATION_VERSION
    ):
        raise ValueError(
            "randomness.derivation_version must be the non-Boolean integer "
            f"{_RUN_RANDOM_STREAM_DERIVATION_VERSION}"
        )

    encoded_streams = _require_json_array(
        randomness_mapping["streams"],
        error_message="randomness.streams must be a JSON array",
    )
    ordered_streams = _ordered_run_random_streams()
    if len(encoded_streams) != len(ordered_streams):
        raise ValueError(f"randomness.streams must contain exactly {len(ordered_streams)} entries")

    for index, (expected_stream, expected_id) in enumerate(ordered_streams):
        entry_path = f"randomness.streams[{index}]"
        stream_mapping = _require_json_object(
            encoded_streams[index],
            error_message=f"{entry_path} must be a JSON object",
        )
        _require_exact_keys(
            stream_mapping,
            expected_keys=_RANDOM_STREAM_KEYS,
            error_message=f"{entry_path} keys must be exactly: id, name",
        )
        encoded_id = stream_mapping["id"]
        if type(encoded_id) is not int or encoded_id != expected_id:
            raise ValueError(f"{entry_path}.id must be the non-Boolean integer {expected_id}")
        encoded_name = stream_mapping["name"]
        if type(encoded_name) is not str or encoded_name != expected_stream.value:
            raise ValueError(f"{entry_path}.name must equal '{expected_stream.value}'")


def _require_nonempty_string(value: object, *, error_message: str) -> str:
    """Require one exact, nonempty string without rewriting it."""
    if type(value) is not str or value == "":
        raise ValueError(error_message)
    return value


def _validate_software_provenance(value: object) -> dict[str, object]:
    """Require exact supplied software-provenance fields before reconstruction."""
    provenance_mapping = _require_json_object(
        value,
        error_message="software_provenance must be a JSON object",
    )
    _require_exact_keys(
        provenance_mapping,
        expected_keys=_SOFTWARE_PROVENANCE_KEYS,
        error_message=(
            "software_provenance keys must be exactly: "
            "git_commit_sha, git_worktree_clean, numpy_version, package_version, python_version"
        ),
    )
    _require_nonempty_string(
        provenance_mapping["package_version"],
        error_message="software_provenance.package_version must be a nonempty string",
    )
    _require_nonempty_string(
        provenance_mapping["python_version"],
        error_message="software_provenance.python_version must be a nonempty string",
    )
    _require_nonempty_string(
        provenance_mapping["numpy_version"],
        error_message="software_provenance.numpy_version must be a nonempty string",
    )
    _require_nonempty_string(
        provenance_mapping["git_commit_sha"],
        error_message="software_provenance.git_commit_sha must be a nonempty string",
    )
    if type(provenance_mapping["git_worktree_clean"]) is not bool:
        raise ValueError("software_provenance.git_worktree_clean must be a Boolean")
    return provenance_mapping


def _validate_configuration_group(value: object, *, path: str) -> dict[str, object]:
    """Require only the five named fields of one configuration group."""
    group_mapping = _require_json_object(
        value,
        error_message=f"{path} must be a JSON object",
    )
    _require_exact_keys(
        group_mapping,
        expected_keys=_CONFIGURATION_GROUP_KEYS,
        error_message=(
            f"{path} keys must be exactly: imu, position_sensors, rigid_body, rotors, world"
        ),
    )
    return group_mapping


def _validate_parameter_object(
    group_mapping: dict[str, object],
    *,
    group_name: str,
    parameter_name: str,
    expected_keys: frozenset[str],
    exact_keys_text: str,
) -> dict[str, object]:
    """Require one selected child parameter object and its explicit key set."""
    path = f"run_configuration.{group_name}.{parameter_name}"
    parameter_mapping = _require_json_object(
        group_mapping[parameter_name],
        error_message=f"{path} must be a JSON object",
    )
    _require_exact_keys(
        parameter_mapping,
        expected_keys=expected_keys,
        error_message=f"{path} keys must be exactly: {exact_keys_text}",
    )
    return parameter_mapping


def _validate_parameter_group_objects(
    group_mapping: dict[str, object],
    *,
    group_name: str,
    schema_version: int,
) -> None:
    """Require the five child parameter schemas in fixed protocol order."""
    environmental = schema_version in _ENVIRONMENTAL_SCHEMA_VERSIONS
    _validate_parameter_object(
        group_mapping,
        group_name=group_name,
        parameter_name="rigid_body",
        expected_keys=(_ENVIRONMENTAL_RIGID_BODY_KEYS if environmental else _RIGID_BODY_KEYS),
        exact_keys_text=(
            "inertia_B, mass, quadratic_drag_coefficient_B" if environmental else "inertia_B, mass"
        ),
    )
    _validate_parameter_object(
        group_mapping,
        group_name=group_name,
        parameter_name="rotors",
        expected_keys=(
            _ROTOR_V3_KEYS if schema_version in _MOTOR_FIELD_SCHEMA_VERSIONS else _ROTOR_KEYS
        ),
        exact_keys_text=(
            "maximum_rotor_omega, minimum_rotor_omega, moment_coefficient, "
            "motor_time_constant_s, rotor_positions_B, rotor_spin_directions, thrust_coefficient"
            if schema_version in _MOTOR_FIELD_SCHEMA_VERSIONS
            else "moment_coefficient, rotor_positions_B, rotor_spin_directions, thrust_coefficient"
        ),
    )
    _validate_parameter_object(
        group_mapping,
        group_name=group_name,
        parameter_name="world",
        expected_keys=_ENVIRONMENTAL_WORLD_KEYS if environmental else _WORLD_KEYS,
        exact_keys_text=(
            "gravity_acceleration, wind_velocity_W" if environmental else "gravity_acceleration"
        ),
    )
    _validate_parameter_object(
        group_mapping,
        group_name=group_name,
        parameter_name="imu",
        expected_keys=_IMU_KEYS,
        exact_keys_text=(
            "accelerometer_bias_random_walk_density_B, "
            "accelerometer_noise_standard_deviation_B, "
            "gyroscope_bias_random_walk_density_B, "
            "gyroscope_noise_standard_deviation_B, "
            "initial_accelerometer_bias_B, initial_gyroscope_bias_B"
        ),
    )
    _validate_parameter_object(
        group_mapping,
        group_name=group_name,
        parameter_name="position_sensors",
        expected_keys=_POSITION_SENSOR_KEYS,
        exact_keys_text=(
            "barometric_altitude_bias, barometric_altitude_noise_standard_deviation, "
            "barometric_reference_altitude, local_position_bias_W, "
            "local_position_noise_standard_deviation_W"
        ),
    )


def _validate_sensor_schedule_object(
    schedules_mapping: dict[str, object],
    *,
    channel: str,
) -> None:
    """Require one named schedule object and its explicit key set."""
    path = f"run_configuration.sensor_schedules.{channel}"
    schedule_mapping = _require_json_object(
        schedules_mapping[channel],
        error_message=f"{path} must be a JSON object",
    )
    _require_exact_keys(
        schedule_mapping,
        expected_keys=_SENSOR_SCHEDULE_KEYS,
        error_message=(
            f"{path} keys must be exactly: "
            "delivery_delay_s, effective_sample_period_s, "
            "requested_sample_period_s, sample_stride"
        ),
    )


def _validate_sensor_schedule_objects(schedules_mapping: dict[str, object]) -> None:
    """Require the four known schedule schemas in fixed channel order."""
    _validate_sensor_schedule_object(schedules_mapping, channel="accelerometer")
    _validate_sensor_schedule_object(schedules_mapping, channel="gyroscope")
    _validate_sensor_schedule_object(schedules_mapping, channel="local_position")
    _validate_sensor_schedule_object(schedules_mapping, channel="barometric_altitude")


def _validate_declared_mismatch_entries(encoded_mismatches: list[object]) -> None:
    """Require every declared mismatch's object and explicit key set in list order."""
    for index, entry in enumerate(encoded_mismatches):
        path = f"run_configuration.declared_mismatches[{index}]"
        mismatch_mapping = _require_json_object(
            entry,
            error_message=f"{path} must be a JSON object",
        )
        _require_exact_keys(
            mismatch_mapping,
            expected_keys=_DECLARED_MISMATCH_KEYS,
            error_message=f"{path} keys must be exactly: parameter_path, rationale",
        )


def _validate_run_configuration_structure(
    value: object, *, schema_version: int
) -> dict[str, object]:
    """Require the encoded run configuration's outer object and key boundaries."""
    configuration_mapping = _require_json_object(
        value,
        error_message="run_configuration must be a JSON object",
    )
    _require_exact_keys(
        configuration_mapping,
        expected_keys=(
            _RUN_CONFIGURATION_V3_KEYS
            if schema_version in _MOTOR_FIELD_SCHEMA_VERSIONS
            else _RUN_CONFIGURATION_KEYS
        ),
        error_message=(
            "run_configuration keys must be exactly: "
            + (
                "declared_mismatches, initial_actual_rotor_omega, initial_truth_state, "
                if schema_version in _MOTOR_FIELD_SCHEMA_VERSIONS
                else "declared_mismatches, initial_truth_state, "
            )
            + "nominal, numerics, root_seed, rotor_speed_input, sensor_schedules, truth"
        ),
    )

    truth_mapping = _validate_configuration_group(
        configuration_mapping["truth"],
        path="run_configuration.truth",
    )
    nominal_mapping = _validate_configuration_group(
        configuration_mapping["nominal"],
        path="run_configuration.nominal",
    )
    _validate_parameter_group_objects(
        truth_mapping, group_name="truth", schema_version=schema_version
    )
    _validate_parameter_group_objects(
        nominal_mapping, group_name="nominal", schema_version=schema_version
    )

    initial_truth_state = _require_json_object(
        configuration_mapping["initial_truth_state"],
        error_message="run_configuration.initial_truth_state must be a JSON object",
    )
    _require_exact_keys(
        initial_truth_state,
        expected_keys=_INITIAL_TRUTH_STATE_KEYS,
        error_message=(
            "run_configuration.initial_truth_state keys must be exactly: "
            "omega_B, position_W, q_WB, velocity_W"
        ),
    )

    numerics = _require_json_object(
        configuration_mapping["numerics"],
        error_message="run_configuration.numerics must be a JSON object",
    )
    _require_exact_keys(
        numerics,
        expected_keys=_NUMERICS_KEYS,
        error_message=(
            "run_configuration.numerics keys must be exactly: "
            "duration_s, integration_method, number_of_steps, truth_time_step_s"
        ),
    )

    sensor_schedules = _require_json_object(
        configuration_mapping["sensor_schedules"],
        error_message="run_configuration.sensor_schedules must be a JSON object",
    )
    _require_exact_keys(
        sensor_schedules,
        expected_keys=_SENSOR_SCHEDULES_KEYS,
        error_message=(
            "run_configuration.sensor_schedules keys must be exactly: "
            "accelerometer, barometric_altitude, gyroscope, local_position"
        ),
    )
    _validate_sensor_schedule_objects(sensor_schedules)

    rotor_speed_input = _require_json_object(
        configuration_mapping["rotor_speed_input"],
        error_message="run_configuration.rotor_speed_input must be a JSON object",
    )
    _require_exact_keys(
        rotor_speed_input,
        expected_keys=_ROTOR_SPEED_INPUT_KEYS,
        error_message="run_configuration.rotor_speed_input keys must be exactly: rotor_omega",
    )

    encoded_mismatches = _require_json_array(
        configuration_mapping["declared_mismatches"],
        error_message="run_configuration.declared_mismatches must be a JSON array",
    )
    _validate_declared_mismatch_entries(encoded_mismatches)
    return configuration_mapping


def _decoded_array(value: object) -> NDArray[np.float64]:
    """Convert one explicitly selected JSON array to float64."""
    return np.array(value, dtype=np.float64)


def _decode_rigid_body(mapping: dict[str, object], *, schema_version: int) -> RigidBodyParameters:
    return RigidBodyParameters(
        mass=cast(float, mapping["mass"]),
        inertia_B=_decoded_array(mapping["inertia_B"]),
        quadratic_drag_coefficient_B=(
            _decoded_array(mapping["quadratic_drag_coefficient_B"])
            if schema_version in _ENVIRONMENTAL_SCHEMA_VERSIONS
            else np.zeros(3, dtype=np.float64)
        ),
    )


def _decode_rotors(mapping: dict[str, object], *, motorized: bool) -> RotorParameters:
    return RotorParameters(
        rotor_positions_B=_decoded_array(mapping["rotor_positions_B"]),
        rotor_spin_directions=_decoded_array(mapping["rotor_spin_directions"]),
        thrust_coefficient=cast(float, mapping["thrust_coefficient"]),
        moment_coefficient=cast(float, mapping["moment_coefficient"]),
        minimum_rotor_omega=cast(float, mapping["minimum_rotor_omega"]) if motorized else None,
        maximum_rotor_omega=cast(float, mapping["maximum_rotor_omega"]) if motorized else None,
        motor_time_constant_s=cast(float, mapping["motor_time_constant_s"]) if motorized else None,
    )


def _decode_world(mapping: dict[str, object], *, schema_version: int) -> WorldParameters:
    return WorldParameters(
        gravity_acceleration=cast(float, mapping["gravity_acceleration"]),
        wind_velocity_W=(
            _decoded_array(mapping["wind_velocity_W"])
            if schema_version in _ENVIRONMENTAL_SCHEMA_VERSIONS
            else np.zeros(3, dtype=np.float64)
        ),
    )


def _decode_imu(mapping: dict[str, object]) -> ImuParameters:
    return ImuParameters(
        initial_accelerometer_bias_B=_decoded_array(mapping["initial_accelerometer_bias_B"]),
        accelerometer_noise_standard_deviation_B=_decoded_array(
            mapping["accelerometer_noise_standard_deviation_B"]
        ),
        accelerometer_bias_random_walk_density_B=_decoded_array(
            mapping["accelerometer_bias_random_walk_density_B"]
        ),
        initial_gyroscope_bias_B=_decoded_array(mapping["initial_gyroscope_bias_B"]),
        gyroscope_noise_standard_deviation_B=_decoded_array(
            mapping["gyroscope_noise_standard_deviation_B"]
        ),
        gyroscope_bias_random_walk_density_B=_decoded_array(
            mapping["gyroscope_bias_random_walk_density_B"]
        ),
    )


def _decode_position_sensors(mapping: dict[str, object]) -> PositionSensorParameters:
    return PositionSensorParameters(
        local_position_bias_W=_decoded_array(mapping["local_position_bias_W"]),
        local_position_noise_standard_deviation_W=_decoded_array(
            mapping["local_position_noise_standard_deviation_W"]
        ),
        barometric_reference_altitude=cast(float, mapping["barometric_reference_altitude"]),
        barometric_altitude_bias=cast(float, mapping["barometric_altitude_bias"]),
        barometric_altitude_noise_standard_deviation=cast(
            float, mapping["barometric_altitude_noise_standard_deviation"]
        ),
    )


def _decode_truth_configuration(
    mapping: dict[str, object], *, schema_version: int, motorized: bool
) -> TruthConfiguration:
    return TruthConfiguration(
        rigid_body=_decode_rigid_body(
            _decoded_object(mapping["rigid_body"]), schema_version=schema_version
        ),
        rotors=_decode_rotors(_decoded_object(mapping["rotors"]), motorized=motorized),
        world=_decode_world(_decoded_object(mapping["world"]), schema_version=schema_version),
        imu=_decode_imu(_decoded_object(mapping["imu"])),
        position_sensors=_decode_position_sensors(_decoded_object(mapping["position_sensors"])),
    )


def _decode_nominal_configuration(
    mapping: dict[str, object], *, schema_version: int, motorized: bool
) -> NominalConfiguration:
    return NominalConfiguration(
        rigid_body=_decode_rigid_body(
            _decoded_object(mapping["rigid_body"]), schema_version=schema_version
        ),
        rotors=_decode_rotors(_decoded_object(mapping["rotors"]), motorized=motorized),
        world=_decode_world(_decoded_object(mapping["world"]), schema_version=schema_version),
        imu=_decode_imu(_decoded_object(mapping["imu"])),
        position_sensors=_decode_position_sensors(_decoded_object(mapping["position_sensors"])),
    )


def _decode_initial_truth_state(mapping: dict[str, object]) -> RigidBodyInitialState:
    return RigidBodyInitialState(
        position_W=_decoded_array(mapping["position_W"]),
        velocity_W=_decoded_array(mapping["velocity_W"]),
        q_WB=_decoded_array(mapping["q_WB"]),
        omega_B=_decoded_array(mapping["omega_B"]),
    )


def _decode_numerics(mapping: dict[str, object]) -> RunNumerics:
    truth_time_step_s = cast(float, mapping["truth_time_step_s"])
    number_of_steps = cast(int, mapping["number_of_steps"])
    duration_s = cast(float, mapping["duration_s"])
    numerics = RunNumerics(
        integration_method=IntegrationMethod(cast(str, mapping["integration_method"])),
        truth_time_step_s=truth_time_step_s,
        number_of_steps=number_of_steps,
    )
    expected_duration_s = truth_time_step_s * number_of_steps
    if duration_s != expected_duration_s:
        raise ValueError(
            "manifest run_configuration.numerics.duration_s must equal "
            "truth_time_step_s * number_of_steps"
        )
    return numerics


def _decode_sensor_schedule(mapping: dict[str, object]) -> SensorSchedule:
    return SensorSchedule(
        sample_period_s=cast(float, mapping["requested_sample_period_s"]),
        delivery_delay_s=cast(float, mapping["delivery_delay_s"]),
    )


def _decode_sensor_schedules(mapping: dict[str, object]) -> SensorSchedules:
    return SensorSchedules(
        accelerometer=_decode_sensor_schedule(_decoded_object(mapping["accelerometer"])),
        gyroscope=_decode_sensor_schedule(_decoded_object(mapping["gyroscope"])),
        local_position=_decode_sensor_schedule(_decoded_object(mapping["local_position"])),
        barometric_altitude=_decode_sensor_schedule(
            _decoded_object(mapping["barometric_altitude"])
        ),
    )


def _decode_rotor_speed_input(mapping: dict[str, object]) -> ConstantRotorSpeedInput:
    return ConstantRotorSpeedInput(rotor_omega=_decoded_array(mapping["rotor_omega"]))


def _decode_declared_mismatch(mapping: dict[str, object]) -> DeclaredMismatch:
    return DeclaredMismatch(
        parameter_path=cast(str, mapping["parameter_path"]),
        rationale=cast(str, mapping["rationale"]),
    )


def _environmental_manifest_has_complete_motor_configuration(
    mapping: dict[str, object],
) -> bool:
    """Require the v5/v6 motor representation to be wholly null or complete."""
    truth_rotors = _decoded_object(_decoded_object(mapping["truth"])["rotors"])
    nominal_rotors = _decoded_object(_decoded_object(mapping["nominal"])["rotors"])
    motor_values = (
        truth_rotors["minimum_rotor_omega"],
        truth_rotors["maximum_rotor_omega"],
        truth_rotors["motor_time_constant_s"],
        nominal_rotors["minimum_rotor_omega"],
        nominal_rotors["maximum_rotor_omega"],
        nominal_rotors["motor_time_constant_s"],
        mapping["initial_actual_rotor_omega"],
    )
    if all(value is None for value in motor_values):
        return False
    if all(value is not None for value in motor_values):
        return True
    raise ValueError("manifest motor configuration must be either entirely omitted or complete")


def _decode_run_configuration(
    mapping: dict[str, object], *, schema_version: int
) -> RunConfiguration:
    encoded_mismatches = cast(list[object], mapping["declared_mismatches"])
    motorized = schema_version in _MOTORIZED_SCHEMA_VERSIONS
    if schema_version in _ENVIRONMENTAL_SCHEMA_VERSIONS:
        motorized = _environmental_manifest_has_complete_motor_configuration(mapping)
    configuration = RunConfiguration(
        truth=_decode_truth_configuration(
            _decoded_object(mapping["truth"]),
            schema_version=schema_version,
            motorized=motorized,
        ),
        nominal=_decode_nominal_configuration(
            _decoded_object(mapping["nominal"]),
            schema_version=schema_version,
            motorized=motorized,
        ),
        initial_truth_state=_decode_initial_truth_state(
            _decoded_object(mapping["initial_truth_state"])
        ),
        numerics=_decode_numerics(_decoded_object(mapping["numerics"])),
        sensor_schedules=_decode_sensor_schedules(_decoded_object(mapping["sensor_schedules"])),
        rotor_speed_input=_decode_rotor_speed_input(_decoded_object(mapping["rotor_speed_input"])),
        root_seed=int(cast(str, mapping["root_seed"])),
        declared_mismatches=tuple(
            _decode_declared_mismatch(_decoded_object(mismatch)) for mismatch in encoded_mismatches
        ),
        initial_actual_rotor_omega=(
            _decoded_array(mapping["initial_actual_rotor_omega"]) if motorized else None
        ),
    )
    if (
        schema_version in _ENVIRONMENTAL_SCHEMA_VERSIONS
        and not _has_nonzero_environmental_configuration(configuration)
    ):
        raise ValueError(
            "manifest environmental configuration must contain at least one nonzero value"
        )
    schedules = _decoded_object(mapping["sensor_schedules"])
    time_step_s = configuration.numerics.truth_time_step_s
    for channel in ("accelerometer", "gyroscope", "local_position", "barometric_altitude"):
        encoded_schedule = _decoded_object(schedules[channel])
        schedule = cast(SensorSchedule, getattr(configuration.sensor_schedules, channel))
        expected_stride = int(round(schedule.sample_period_s / time_step_s))
        stride = encoded_schedule["sample_stride"]
        if type(stride) is not int or stride != expected_stride:
            raise ValueError(f"manifest {channel}.sample_stride must equal the configured stride")
        period = encoded_schedule["effective_sample_period_s"]
        if type(period) not in (int, float) or period != expected_stride * time_step_s:
            raise ValueError(
                f"manifest {channel}.effective_sample_period_s must equal sample_stride "
                "* truth_time_step_s"
            )
    return configuration


def _decode_software_provenance(mapping: dict[str, object]) -> SoftwareProvenance:
    return SoftwareProvenance(
        package_version=cast(str, mapping["package_version"]),
        python_version=cast(str, mapping["python_version"]),
        numpy_version=cast(str, mapping["numpy_version"]),
        git_commit_sha=cast(str, mapping["git_commit_sha"]),
        git_worktree_clean=cast(bool, mapping["git_worktree_clean"]),
    )


def _reject_duplicate_json_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    """Build one JSON object while rejecting duplicate member names."""
    mapping: dict[str, object] = {}
    for key, value in pairs:
        if key in mapping:
            raise ValueError(f"manifest JSON contains duplicate key: {key}")
        mapping[key] = value
    return mapping


def _reject_nonstandard_json_constant(constant: str) -> Never:
    """Reject a nonstandard numeric constant during JSON parsing."""
    raise ValueError(f"manifest JSON contains nonstandard constant: {constant}")


def decode_run_manifest(manifest_bytes: bytes) -> RunManifest:
    """Reconstruct a run manifest from valid canonical JSON bytes."""
    parsed_manifest: object = json.loads(
        manifest_bytes,
        object_pairs_hook=_reject_duplicate_json_keys,
        parse_constant=_reject_nonstandard_json_constant,
    )
    manifest_mapping = _require_json_object(
        parsed_manifest,
        error_message="manifest must be a JSON object",
    )
    if "data_artifact" in manifest_mapping:
        _require_exact_keys(
            manifest_mapping,
            expected_keys=_MANIFEST_V2_KEYS,
            error_message=(
                "manifest keys must be exactly: "
                "data_artifact, randomness, run_configuration, schema, software_provenance"
            ),
        )
    else:
        _require_exact_keys(
            manifest_mapping,
            expected_keys=_MANIFEST_V1_KEYS,
            error_message=(
                "manifest keys must be exactly: "
                "randomness, run_configuration, schema, software_provenance"
            ),
        )
    schema_mapping = _require_json_object(
        manifest_mapping["schema"],
        error_message="schema must be a JSON object",
    )
    _require_exact_keys(
        schema_mapping,
        expected_keys=_SCHEMA_KEYS,
        error_message="schema keys must be exactly: name, version",
    )
    schema_name = schema_mapping["name"]
    if type(schema_name) is not str or schema_name != RUN_MANIFEST_SCHEMA_NAME:
        raise ValueError("schema.name must equal 'robust_quadrotor.run_manifest'")
    schema_version = schema_mapping["version"]
    if type(schema_version) is not int or schema_version not in (
        RUN_MANIFEST_SCHEMA_VERSION,
        _ARTIFACT_BOUND_SCHEMA_VERSION,
        _MOTORIZED_UNBOUND_SCHEMA_VERSION,
        _MOTORIZED_BOUND_SCHEMA_VERSION,
        _ENVIRONMENTAL_UNBOUND_SCHEMA_VERSION,
        _ENVIRONMENTAL_BOUND_SCHEMA_VERSION,
    ):
        raise ValueError("schema.version must be 1, 2, 3, 4, 5, or 6")

    data_npz_sha256: str | None = None
    if schema_version in _BOUND_SCHEMA_VERSIONS:
        _require_exact_keys(
            manifest_mapping,
            expected_keys=_MANIFEST_V2_KEYS,
            error_message=(
                "run manifest keys must be exactly: "
                "data_artifact, randomness, run_configuration, schema, software_provenance"
            ),
        )
        data_artifact_mapping = _require_json_object(
            manifest_mapping["data_artifact"],
            error_message="data_artifact must be a JSON object",
        )
        _require_exact_keys(
            data_artifact_mapping,
            expected_keys=_DATA_ARTIFACT_KEYS,
            error_message="data_artifact keys must be exactly: sha256",
        )
        data_npz_sha256 = _validate_data_npz_sha256(data_artifact_mapping["sha256"])
    else:
        _require_exact_keys(
            manifest_mapping,
            expected_keys=_MANIFEST_V1_KEYS,
            error_message=(
                "run manifest keys must be exactly: "
                "randomness, run_configuration, schema, software_provenance"
            ),
        )

    _validate_randomness(manifest_mapping["randomness"])
    software_provenance_mapping = _validate_software_provenance(
        manifest_mapping["software_provenance"]
    )
    run_configuration_mapping = _validate_run_configuration_structure(
        manifest_mapping["run_configuration"], schema_version=schema_version
    )

    return RunManifest(
        run_configuration=_decode_run_configuration(
            run_configuration_mapping, schema_version=schema_version
        ),
        software_provenance=_decode_software_provenance(software_provenance_mapping),
        data_npz_sha256=data_npz_sha256,
    )
