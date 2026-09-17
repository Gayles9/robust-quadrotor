import json
import math
from pathlib import Path
from subprocess import CompletedProcess
from typing import cast

import numpy as np
import pytest
from numpy.typing import NDArray

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
from quadrotor_math.run_manifest import (
    RUN_MANIFEST_SCHEMA_NAME,
    RUN_MANIFEST_SCHEMA_VERSION,
    RunManifest,
    SoftwareProvenance,
    capture_software_provenance,
    decode_run_manifest,
    encode_run_manifest,
)

_DATA_NPZ_SHA256 = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"


def _run_configuration() -> RunConfiguration:
    truth = TruthConfiguration(
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
            rotor_spin_directions=np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64),
            thrust_coefficient=1.2e-5,
            moment_coefficient=2.0e-7,
        ),
        world=WorldParameters(gravity_acceleration=9.81),
        imu=ImuParameters(
            initial_accelerometer_bias_B=np.array([0.1, -0.2, 0.0], dtype=np.float64),
            accelerometer_noise_standard_deviation_B=np.array([0.01, 0.02, 0.03], dtype=np.float64),
            accelerometer_bias_random_walk_density_B=np.array(
                [0.001, 0.002, 0.003], dtype=np.float64
            ),
            initial_gyroscope_bias_B=np.array([0.01, -0.02, 0.0], dtype=np.float64),
            gyroscope_noise_standard_deviation_B=np.array([0.001, 0.002, 0.003], dtype=np.float64),
            gyroscope_bias_random_walk_density_B=np.array(
                [0.0001, 0.0002, 0.0003], dtype=np.float64
            ),
        ),
        position_sensors=PositionSensorParameters(
            local_position_bias_W=np.array([0.5, -0.25, 0.0], dtype=np.float64),
            local_position_noise_standard_deviation_W=np.array([0.1, 0.2, 0.3], dtype=np.float64),
            barometric_reference_altitude=100.0,
            barometric_altitude_bias=1.0,
            barometric_altitude_noise_standard_deviation=0.5,
        ),
    )
    nominal = NominalConfiguration(
        rigid_body=RigidBodyParameters(mass=1.4, inertia_B=truth.rigid_body.inertia_B),
        rotors=truth.rotors,
        world=truth.world,
        imu=truth.imu,
        position_sensors=truth.position_sensors,
    )
    return RunConfiguration(
        truth=truth,
        nominal=nominal,
        initial_truth_state=RigidBodyInitialState(
            position_W=np.array([1.0, 2.0, -3.0], dtype=np.float64),
            velocity_W=np.array([0.5, 0.0, -0.5], dtype=np.float64),
            q_WB=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            omega_B=np.array([0.1, -0.2, 0.3], dtype=np.float64),
        ),
        numerics=RunNumerics(
            integration_method=IntegrationMethod.PROJECTED_RK4,
            truth_time_step_s=0.02,
            number_of_steps=25,
        ),
        sensor_schedules=SensorSchedules(
            accelerometer=SensorSchedule(sample_period_s=0.04, delivery_delay_s=0.0),
            gyroscope=SensorSchedule(sample_period_s=0.02, delivery_delay_s=0.01),
            local_position=SensorSchedule(sample_period_s=0.1, delivery_delay_s=0.03),
            barometric_altitude=SensorSchedule(sample_period_s=0.2, delivery_delay_s=0.0),
        ),
        rotor_speed_input=ConstantRotorSpeedInput(
            rotor_omega=np.array([400.0, 410.0, 420.0, 430.0], dtype=np.float64),
        ),
        root_seed=1208925819614629174706299,
        declared_mismatches=(
            DeclaredMismatch(
                parameter_path="rigid_body.mass",
                rationale="Exercise an intentional mass-model mismatch.",
            ),
        ),
    )


def _software_provenance() -> SoftwareProvenance:
    return SoftwareProvenance(
        package_version="0.1.0",
        python_version="3.12.3",
        numpy_version="2.5.2",
        git_commit_sha="99988cde60be738af974a3762dd68512258b0e1f",
        git_worktree_clean=True,
    )


def _artifact_bound_manifest() -> RunManifest:
    return RunManifest(
        run_configuration=_run_configuration(),
        software_provenance=_software_provenance(),
        data_npz_sha256=_DATA_NPZ_SHA256,
    )


@pytest.mark.parametrize(
    ("status_stdout", "expected_clean"),
    [
        ("", True),
        ("?? untracked-result.txt\n", False),
    ],
)
def test_capture_software_provenance_uses_explicit_root_and_complete_git_status(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    status_stdout: str,
    expected_clean: bool,
) -> None:
    expected_sha = "0123456789abcdef0123456789abcdef01234567"
    metadata_calls: list[str] = []
    git_calls: list[tuple[list[str], dict[str, object]]] = []
    commit_command = ["git", "-C", str(tmp_path), "rev-parse", "--verify", "HEAD"]
    status_command = [
        "git",
        "-C",
        str(tmp_path),
        "status",
        "--porcelain=v1",
        "--untracked-files=normal",
    ]

    def fake_package_version(package_name: str) -> str:
        metadata_calls.append(package_name)
        return "9.8.7"

    def fake_git_run(args: list[str], **options: object) -> CompletedProcess[str]:
        git_calls.append((args, options))
        if args == commit_command:
            return CompletedProcess(args, 0, stdout=f"{expected_sha}\n", stderr="")
        if args == status_command:
            return CompletedProcess(args, 0, stdout=status_stdout, stderr="")
        pytest.fail(f"unexpected Git command: {args!r}")

    monkeypatch.setattr("quadrotor_math.run_manifest.metadata.version", fake_package_version)
    monkeypatch.setattr("quadrotor_math.run_manifest.platform.python_version", lambda: "3.12.9")
    monkeypatch.setattr("quadrotor_math.run_manifest.np.__version__", "2.99.0")
    monkeypatch.setattr("quadrotor_math.run_manifest.subprocess.run", fake_git_run)

    provenance = capture_software_provenance(tmp_path)

    assert type(provenance) is SoftwareProvenance
    assert provenance.package_version == "9.8.7"
    assert provenance.python_version == "3.12.9"
    assert provenance.numpy_version == "2.99.0"
    assert provenance.git_commit_sha == expected_sha
    assert provenance.git_worktree_clean is expected_clean
    assert metadata_calls == ["robust-quadrotor"]
    assert [args for args, _ in git_calls] == [commit_command, status_command]
    assert all(
        options == {"check": True, "capture_output": True, "text": True} for _, options in git_calls
    )


def test_run_manifest_schema_constants_are_fixed() -> None:
    assert RUN_MANIFEST_SCHEMA_NAME == "robust_quadrotor.run_manifest"
    assert RUN_MANIFEST_SCHEMA_VERSION == 1


def test_run_manifest_representations_retain_explicit_inputs() -> None:
    configuration = _run_configuration()
    provenance = _software_provenance()

    manifest = RunManifest(
        run_configuration=configuration,
        software_provenance=provenance,
    )

    assert manifest.run_configuration is configuration
    assert manifest.software_provenance is provenance
    assert provenance.package_version == "0.1.0"
    assert provenance.python_version == "3.12.3"
    assert provenance.numpy_version == "2.5.2"
    assert provenance.git_commit_sha == "99988cde60be738af974a3762dd68512258b0e1f"
    assert provenance.git_worktree_clean is True


def test_encode_run_manifest_produces_complete_canonical_utf8_bytes() -> None:
    manifest = RunManifest(
        run_configuration=_run_configuration(),
        software_provenance=_software_provenance(),
    )
    expected_mapping = {
        "schema": {"name": "robust_quadrotor.run_manifest", "version": 1},
        "run_configuration": {
            "truth": {
                "rigid_body": {
                    "mass": 1.5,
                    "inertia_B": [[0.02, 0.0, 0.0], [0.0, 0.03, 0.0], [0.0, 0.0, 0.04]],
                },
                "rotors": {
                    "rotor_positions_B": [
                        [0.2, 0.2, 0.0],
                        [0.2, -0.2, 0.0],
                        [-0.2, -0.2, 0.0],
                        [-0.2, 0.2, 0.0],
                    ],
                    "rotor_spin_directions": [1.0, -1.0, 1.0, -1.0],
                    "thrust_coefficient": 1.2e-5,
                    "moment_coefficient": 2.0e-7,
                },
                "world": {"gravity_acceleration": 9.81},
                "imu": {
                    "initial_accelerometer_bias_B": [0.1, -0.2, 0.0],
                    "accelerometer_noise_standard_deviation_B": [0.01, 0.02, 0.03],
                    "accelerometer_bias_random_walk_density_B": [0.001, 0.002, 0.003],
                    "initial_gyroscope_bias_B": [0.01, -0.02, 0.0],
                    "gyroscope_noise_standard_deviation_B": [0.001, 0.002, 0.003],
                    "gyroscope_bias_random_walk_density_B": [0.0001, 0.0002, 0.0003],
                },
                "position_sensors": {
                    "local_position_bias_W": [0.5, -0.25, 0.0],
                    "local_position_noise_standard_deviation_W": [0.1, 0.2, 0.3],
                    "barometric_reference_altitude": 100.0,
                    "barometric_altitude_bias": 1.0,
                    "barometric_altitude_noise_standard_deviation": 0.5,
                },
            },
            "nominal": {
                "rigid_body": {
                    "mass": 1.4,
                    "inertia_B": [[0.02, 0.0, 0.0], [0.0, 0.03, 0.0], [0.0, 0.0, 0.04]],
                },
                "rotors": {
                    "rotor_positions_B": [
                        [0.2, 0.2, 0.0],
                        [0.2, -0.2, 0.0],
                        [-0.2, -0.2, 0.0],
                        [-0.2, 0.2, 0.0],
                    ],
                    "rotor_spin_directions": [1.0, -1.0, 1.0, -1.0],
                    "thrust_coefficient": 1.2e-5,
                    "moment_coefficient": 2.0e-7,
                },
                "world": {"gravity_acceleration": 9.81},
                "imu": {
                    "initial_accelerometer_bias_B": [0.1, -0.2, 0.0],
                    "accelerometer_noise_standard_deviation_B": [0.01, 0.02, 0.03],
                    "accelerometer_bias_random_walk_density_B": [0.001, 0.002, 0.003],
                    "initial_gyroscope_bias_B": [0.01, -0.02, 0.0],
                    "gyroscope_noise_standard_deviation_B": [0.001, 0.002, 0.003],
                    "gyroscope_bias_random_walk_density_B": [0.0001, 0.0002, 0.0003],
                },
                "position_sensors": {
                    "local_position_bias_W": [0.5, -0.25, 0.0],
                    "local_position_noise_standard_deviation_W": [0.1, 0.2, 0.3],
                    "barometric_reference_altitude": 100.0,
                    "barometric_altitude_bias": 1.0,
                    "barometric_altitude_noise_standard_deviation": 0.5,
                },
            },
            "initial_truth_state": {
                "position_W": [1.0, 2.0, -3.0],
                "velocity_W": [0.5, 0.0, -0.5],
                "q_WB": [1.0, 0.0, 0.0, 0.0],
                "omega_B": [0.1, -0.2, 0.3],
            },
            "numerics": {
                "integration_method": "projected_rk4",
                "truth_time_step_s": 0.02,
                "number_of_steps": 25,
                "duration_s": 0.5,
            },
            "sensor_schedules": {
                "accelerometer": {
                    "requested_sample_period_s": 0.04,
                    "sample_stride": 2,
                    "effective_sample_period_s": 0.04,
                    "delivery_delay_s": 0.0,
                },
                "gyroscope": {
                    "requested_sample_period_s": 0.02,
                    "sample_stride": 1,
                    "effective_sample_period_s": 0.02,
                    "delivery_delay_s": 0.01,
                },
                "local_position": {
                    "requested_sample_period_s": 0.1,
                    "sample_stride": 5,
                    "effective_sample_period_s": 0.1,
                    "delivery_delay_s": 0.03,
                },
                "barometric_altitude": {
                    "requested_sample_period_s": 0.2,
                    "sample_stride": 10,
                    "effective_sample_period_s": 0.2,
                    "delivery_delay_s": 0.0,
                },
            },
            "rotor_speed_input": {"rotor_omega": [400.0, 410.0, 420.0, 430.0]},
            "root_seed": "1208925819614629174706299",
            "declared_mismatches": [
                {
                    "parameter_path": "rigid_body.mass",
                    "rationale": "Exercise an intentional mass-model mismatch.",
                }
            ],
        },
        "randomness": {
            "derivation_version": 1,
            "streams": [
                {"id": 1, "name": "accelerometer.measurement_noise"},
                {"id": 2, "name": "accelerometer.bias_random_walk"},
                {"id": 3, "name": "gyroscope.measurement_noise"},
                {"id": 4, "name": "gyroscope.bias_random_walk"},
                {"id": 5, "name": "local_position.measurement_noise"},
                {"id": 6, "name": "barometric_altitude.measurement_noise"},
            ],
        },
        "software_provenance": {
            "package_version": "0.1.0",
            "python_version": "3.12.3",
            "numpy_version": "2.5.2",
            "git_commit_sha": "99988cde60be738af974a3762dd68512258b0e1f",
            "git_worktree_clean": True,
        },
    }
    expected_bytes = json.dumps(
        expected_mapping,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")

    encoded_bytes = encode_run_manifest(manifest)

    assert type(encoded_bytes) is bytes
    assert encoded_bytes == expected_bytes
    assert json.loads(encoded_bytes) == expected_mapping


def test_encode_run_manifest_binds_data_npz_digest_with_schema_version_two() -> None:
    expected_mapping = _valid_artifact_bound_manifest_mapping()
    expected_bytes = _canonical_manifest_bytes(expected_mapping)

    encoded_bytes = encode_run_manifest(_artifact_bound_manifest())
    encoded_mapping = cast(dict[str, object], json.loads(encoded_bytes))

    assert type(encoded_bytes) is bytes
    assert encoded_bytes == expected_bytes
    assert encoded_mapping == expected_mapping
    assert encoded_mapping["schema"] == {
        "name": "robust_quadrotor.run_manifest",
        "version": 2,
    }
    assert set(encoded_mapping) == {
        "data_artifact",
        "randomness",
        "run_configuration",
        "schema",
        "software_provenance",
    }
    assert encoded_mapping["data_artifact"] == {"sha256": _DATA_NPZ_SHA256}


def _configuration_owned_arrays(
    configuration: RunConfiguration,
) -> tuple[NDArray[np.float64], ...]:
    return (
        configuration.truth.rigid_body.inertia_B,
        configuration.truth.rotors.rotor_positions_B,
        configuration.truth.rotors.rotor_spin_directions,
        configuration.truth.imu.initial_accelerometer_bias_B,
        configuration.truth.imu.accelerometer_noise_standard_deviation_B,
        configuration.truth.imu.accelerometer_bias_random_walk_density_B,
        configuration.truth.imu.initial_gyroscope_bias_B,
        configuration.truth.imu.gyroscope_noise_standard_deviation_B,
        configuration.truth.imu.gyroscope_bias_random_walk_density_B,
        configuration.truth.position_sensors.local_position_bias_W,
        configuration.truth.position_sensors.local_position_noise_standard_deviation_W,
        configuration.nominal.rigid_body.inertia_B,
        configuration.nominal.rotors.rotor_positions_B,
        configuration.nominal.rotors.rotor_spin_directions,
        configuration.nominal.imu.initial_accelerometer_bias_B,
        configuration.nominal.imu.accelerometer_noise_standard_deviation_B,
        configuration.nominal.imu.accelerometer_bias_random_walk_density_B,
        configuration.nominal.imu.initial_gyroscope_bias_B,
        configuration.nominal.imu.gyroscope_noise_standard_deviation_B,
        configuration.nominal.imu.gyroscope_bias_random_walk_density_B,
        configuration.nominal.position_sensors.local_position_bias_W,
        configuration.nominal.position_sensors.local_position_noise_standard_deviation_W,
        configuration.initial_truth_state.position_W,
        configuration.initial_truth_state.velocity_W,
        configuration.initial_truth_state.q_WB,
        configuration.initial_truth_state.omega_B,
        configuration.rotor_speed_input.rotor_omega,
    )


def test_decode_run_manifest_round_trips_complete_manifest() -> None:
    original_manifest = RunManifest(
        run_configuration=_run_configuration(),
        software_provenance=_software_provenance(),
    )
    encoded_bytes = encode_run_manifest(original_manifest)

    decoded_manifest = decode_run_manifest(encoded_bytes)

    assert type(decoded_manifest) is RunManifest
    assert decoded_manifest is not original_manifest
    assert type(decoded_manifest.run_configuration) is RunConfiguration
    assert decoded_manifest.run_configuration is not original_manifest.run_configuration
    assert decoded_manifest.software_provenance is not original_manifest.software_provenance
    assert decoded_manifest.software_provenance == original_manifest.software_provenance
    assert decoded_manifest.run_configuration.root_seed == 1208925819614629174706299
    assert type(decoded_manifest.run_configuration.root_seed) is int
    assert (
        decoded_manifest.run_configuration.numerics.integration_method
        is IntegrationMethod.PROJECTED_RK4
    )
    mismatches = decoded_manifest.run_configuration.declared_mismatches
    assert type(mismatches) is tuple
    assert len(mismatches) == 1
    assert type(mismatches[0]) is DeclaredMismatch
    assert mismatches[0].parameter_path == "rigid_body.mass"
    assert mismatches[0].rationale == "Exercise an intentional mass-model mismatch."
    assert encode_run_manifest(decoded_manifest) == encoded_bytes


def test_decode_run_manifest_round_trips_data_npz_digest() -> None:
    artifact_bytes = _canonical_manifest_bytes(_valid_artifact_bound_manifest_mapping())

    decoded_manifest = decode_run_manifest(artifact_bytes)

    assert type(decoded_manifest) is RunManifest
    assert decoded_manifest.data_npz_sha256 == _DATA_NPZ_SHA256
    assert encode_run_manifest(decoded_manifest) == artifact_bytes

    standalone_bytes = _canonical_manifest_bytes(_valid_manifest_mapping())
    standalone_manifest = decode_run_manifest(standalone_bytes)
    assert standalone_manifest.data_npz_sha256 is None
    assert encode_run_manifest(standalone_manifest) == standalone_bytes


def test_decode_run_manifest_rejects_duplicate_json_key() -> None:
    canonical_bytes = encode_run_manifest(_artifact_bound_manifest())
    canonical_mapping = json.loads(canonical_bytes)
    assert canonical_mapping["schema"] == {
        "name": "robust_quadrotor.run_manifest",
        "version": 2,
    }
    schema_member = b'"schema":' + json.dumps(
        canonical_mapping["schema"], sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    assert canonical_bytes.count(b"," + schema_member) == 1

    duplicated_bytes = canonical_bytes.replace(
        b"," + schema_member, b"," + schema_member + b"," + schema_member, 1
    )

    assert duplicated_bytes.count(b'"schema":') == 2
    top_level_pairs = json.loads(duplicated_bytes, object_pairs_hook=list)
    schema_values = [value for key, value in top_level_pairs if key == "schema"]
    assert len(schema_values) == 2
    assert schema_values[0] == schema_values[1]
    assert json.loads(duplicated_bytes) == canonical_mapping

    with pytest.raises(ValueError) as error:
        decode_run_manifest(duplicated_bytes)

    assert str(error.value) == "manifest JSON contains duplicate key: schema"


def test_decode_run_manifest_rejects_nested_duplicate_json_key() -> None:
    canonical_bytes = encode_run_manifest(_artifact_bound_manifest())
    canonical_mapping = json.loads(canonical_bytes)
    assert canonical_mapping["schema"] == {
        "name": "robust_quadrotor.run_manifest",
        "version": 2,
    }
    schema_member = b'"schema":' + json.dumps(
        canonical_mapping["schema"], sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    version_member = b'"version":2'
    assert canonical_bytes.count(schema_member) == 1
    assert schema_member.count(version_member) == 1

    duplicated_schema_member = schema_member.replace(
        version_member, version_member + b"," + version_member, 1
    )
    duplicated_bytes = canonical_bytes.replace(schema_member, duplicated_schema_member, 1)

    top_level_pairs = json.loads(duplicated_bytes, object_pairs_hook=list)
    top_level_keys = [key for key, _ in top_level_pairs]
    assert len(top_level_keys) == len(set(top_level_keys))
    schema_pairs = [value for key, value in top_level_pairs if key == "schema"]
    assert len(schema_pairs) == 1
    version_values = [value for key, value in schema_pairs[0] if key == "version"]
    assert version_values == [2, 2]
    assert json.loads(duplicated_bytes) == canonical_mapping

    with pytest.raises(ValueError) as error:
        decode_run_manifest(duplicated_bytes)

    assert str(error.value) == "manifest JSON contains duplicate key: version"


def test_decode_run_manifest_rejects_nonstandard_nan_constant() -> None:
    canonical_bytes = encode_run_manifest(_artifact_bound_manifest())
    canonical_numerics = json.loads(canonical_bytes)["run_configuration"]["numerics"]
    assert canonical_numerics["duration_s"] == 0.5
    assert math.isfinite(canonical_numerics["duration_s"])

    duration_member = b'"duration_s":0.5'
    nan_member = b'"duration_s":NaN'
    assert canonical_bytes.count(duration_member) == 1
    mutated_bytes = canonical_bytes.replace(duration_member, nan_member, 1)
    assert mutated_bytes.count(nan_member) == 1
    assert b'"duration_s":"NaN"' not in mutated_bytes
    assert mutated_bytes.count(b'"duration_s":') == 1
    assert mutated_bytes.replace(nan_member, duration_member, 1) == canonical_bytes

    mutated_numerics = json.loads(mutated_bytes)["run_configuration"]["numerics"]
    assert math.isnan(mutated_numerics["duration_s"])

    with pytest.raises(ValueError) as error:
        decode_run_manifest(mutated_bytes)

    assert str(error.value) == "manifest JSON contains nonstandard constant: NaN"


@pytest.mark.parametrize(
    ("constant", "expected_message", "expected_sign"),
    [
        pytest.param(
            "Infinity",
            "manifest JSON contains nonstandard constant: Infinity",
            1.0,
            id="positive-infinity",
        ),
        pytest.param(
            "-Infinity",
            "manifest JSON contains nonstandard constant: -Infinity",
            -1.0,
            id="negative-infinity",
        ),
    ],
)
def test_decode_run_manifest_rejects_nonstandard_infinity_constant(
    constant: str, expected_message: str, expected_sign: float
) -> None:
    canonical_bytes = encode_run_manifest(_artifact_bound_manifest())
    canonical_numerics = json.loads(canonical_bytes)["run_configuration"]["numerics"]
    assert canonical_numerics["duration_s"] == 0.5
    assert math.isfinite(canonical_numerics["duration_s"])

    duration_member = b'"duration_s":0.5'
    infinity_member = b'"duration_s":' + constant.encode("ascii")
    assert canonical_bytes.count(duration_member) == 1
    mutated_bytes = canonical_bytes.replace(duration_member, infinity_member, 1)
    assert mutated_bytes.count(infinity_member) == 1
    assert b'"duration_s":"' + constant.encode("ascii") + b'"' not in mutated_bytes
    assert mutated_bytes.count(b'"duration_s":') == 1
    assert mutated_bytes.replace(infinity_member, duration_member, 1) == canonical_bytes

    mutated_numerics = json.loads(mutated_bytes)["run_configuration"]["numerics"]
    assert math.isinf(mutated_numerics["duration_s"])
    assert math.copysign(1.0, mutated_numerics["duration_s"]) == expected_sign

    with pytest.raises(ValueError) as error:
        decode_run_manifest(mutated_bytes)

    assert str(error.value) == expected_message


def test_decode_run_manifest_rejects_inconsistent_duration() -> None:
    canonical_bytes = encode_run_manifest(_artifact_bound_manifest())
    canonical_numerics = json.loads(canonical_bytes)["run_configuration"]["numerics"]
    assert canonical_numerics["truth_time_step_s"] == 0.02
    assert canonical_numerics["number_of_steps"] == 25
    assert canonical_numerics["duration_s"] == 0.5

    duration_member = b'"duration_s":0.5'
    inconsistent_member = b'"duration_s":0.75'
    assert canonical_bytes.count(duration_member) == 1
    mutated_bytes = canonical_bytes.replace(duration_member, inconsistent_member, 1)
    assert mutated_bytes.count(inconsistent_member) == 1
    assert mutated_bytes.count(b'"duration_s":') == 1
    assert mutated_bytes.replace(inconsistent_member, duration_member, 1) == canonical_bytes

    mutated_numerics = json.loads(mutated_bytes)["run_configuration"]["numerics"]
    assert mutated_numerics["truth_time_step_s"] == 0.02
    assert mutated_numerics["number_of_steps"] == 25
    assert mutated_numerics["duration_s"] == 0.75
    expected_duration = mutated_numerics["truth_time_step_s"] * mutated_numerics["number_of_steps"]
    assert expected_duration == 0.5
    assert mutated_numerics["duration_s"] != expected_duration

    with pytest.raises(
        ValueError,
        match=(
            r"^manifest run_configuration\.numerics\.duration_s must equal "
            r"truth_time_step_s \* number_of_steps$"
        ),
    ):
        decode_run_manifest(mutated_bytes)


def test_decode_run_manifest_reconstructs_fresh_read_only_arrays() -> None:
    original_manifest = RunManifest(
        run_configuration=_run_configuration(),
        software_provenance=_software_provenance(),
    )
    decoded_manifest = decode_run_manifest(encode_run_manifest(original_manifest))
    original_configuration = original_manifest.run_configuration
    decoded_configuration = decoded_manifest.run_configuration

    assert decoded_configuration.truth is not decoded_configuration.nominal
    assert decoded_configuration.truth.rigid_body is not decoded_configuration.nominal.rigid_body
    assert decoded_configuration.truth.rotors is not decoded_configuration.nominal.rotors
    assert decoded_configuration.truth.world is not decoded_configuration.nominal.world
    assert decoded_configuration.truth.imu is not decoded_configuration.nominal.imu
    assert (
        decoded_configuration.truth.position_sensors
        is not decoded_configuration.nominal.position_sensors
    )

    original_arrays = _configuration_owned_arrays(original_configuration)
    decoded_arrays = _configuration_owned_arrays(decoded_configuration)
    assert len(original_arrays) == len(decoded_arrays) == 27
    for original_array, decoded_array in zip(original_arrays, decoded_arrays, strict=True):
        np.testing.assert_array_equal(decoded_array, original_array)
        assert decoded_array.dtype == np.float64
        assert decoded_array.flags.owndata
        assert not decoded_array.flags.writeable
        assert not np.shares_memory(decoded_array, original_array)

    for truth_array, nominal_array in zip(decoded_arrays[:11], decoded_arrays[11:22], strict=True):
        assert not np.shares_memory(truth_array, nominal_array)


def _valid_manifest_mapping() -> dict[str, object]:
    manifest = RunManifest(
        run_configuration=_run_configuration(),
        software_provenance=_software_provenance(),
    )
    encoded_bytes = encode_run_manifest(manifest)
    return cast(dict[str, object], json.loads(encoded_bytes))


def _valid_artifact_bound_manifest_mapping() -> dict[str, object]:
    mapping = _valid_manifest_mapping()
    schema = cast(dict[str, object], mapping["schema"])
    schema["version"] = 2
    mapping["data_artifact"] = {"sha256": _DATA_NPZ_SHA256}
    return mapping


def _canonical_manifest_bytes(mapping: dict[str, object]) -> bytes:
    return json.dumps(
        mapping,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def _assert_manifest_decode_error(manifest_bytes: bytes, expected_message: str) -> None:
    with pytest.raises(ValueError) as error:
        decode_run_manifest(manifest_bytes)

    assert str(error.value) == expected_message


def test_decode_run_manifest_rejects_non_object_root() -> None:
    _assert_manifest_decode_error(b"[]", "manifest must be a JSON object")


@pytest.mark.parametrize("mutation", ["missing", "unknown"])
def test_decode_run_manifest_requires_exact_top_level_keys(mutation: str) -> None:
    mapping = _valid_manifest_mapping()
    if mutation == "missing":
        mapping.pop("schema")
    else:
        mapping["unexpected"] = None

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "manifest keys must be exactly: randomness, run_configuration, schema, software_provenance",
    )


def test_decode_run_manifest_requires_schema_object() -> None:
    mapping = _valid_manifest_mapping()
    mapping["schema"] = []

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "schema must be a JSON object",
    )


@pytest.mark.parametrize("mutation", ["missing", "unknown"])
def test_decode_run_manifest_requires_exact_schema_keys(mutation: str) -> None:
    mapping = _valid_manifest_mapping()
    schema = cast(dict[str, object], mapping["schema"])
    if mutation == "missing":
        schema.pop("name")
    else:
        schema["unexpected"] = None

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "schema keys must be exactly: name, version",
    )


@pytest.mark.parametrize("invalid_name", [None, 1, "robust_quadrotor.run_manifest.v2"])
def test_decode_run_manifest_requires_fixed_schema_name(invalid_name: object) -> None:
    mapping = _valid_manifest_mapping()
    schema = cast(dict[str, object], mapping["schema"])
    schema["name"] = invalid_name

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "schema.name must equal 'robust_quadrotor.run_manifest'",
    )


@pytest.mark.parametrize("invalid_version", [True, 3])
def test_decode_run_manifest_requires_fixed_integer_schema_version(
    invalid_version: object,
) -> None:
    mapping = _valid_manifest_mapping()
    schema = cast(dict[str, object], mapping["schema"])
    schema["version"] = invalid_version

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "schema.version must be 1 or 2",
    )


@pytest.mark.parametrize("invalid_version", [1.0, "1"])
def test_decode_run_manifest_rejects_non_integer_schema_version(invalid_version: object) -> None:
    mapping = _valid_manifest_mapping()
    schema = cast(dict[str, object], mapping["schema"])
    schema["version"] = invalid_version

    with pytest.raises(ValueError):
        decode_run_manifest(_canonical_manifest_bytes(mapping))


def test_decode_run_manifest_requires_data_artifact_for_schema_version_two() -> None:
    mapping = _valid_manifest_mapping()
    schema = cast(dict[str, object], mapping["schema"])
    schema["version"] = 2

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "run manifest keys must be exactly: "
        "data_artifact, randomness, run_configuration, schema, software_provenance",
    )


def test_decode_run_manifest_forbids_data_artifact_for_schema_version_one() -> None:
    mapping = _valid_manifest_mapping()
    mapping["data_artifact"] = {"sha256": _DATA_NPZ_SHA256}

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "run manifest keys must be exactly: "
        "randomness, run_configuration, schema, software_provenance",
    )


def test_decode_run_manifest_requires_data_artifact_object() -> None:
    mapping = _valid_artifact_bound_manifest_mapping()
    mapping["data_artifact"] = []

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "data_artifact must be a JSON object",
    )


@pytest.mark.parametrize("mutation", ["missing", "unknown"])
def test_decode_run_manifest_requires_exact_data_artifact_keys(mutation: str) -> None:
    mapping = _valid_artifact_bound_manifest_mapping()
    artifact = cast(dict[str, object], mapping["data_artifact"])
    if mutation == "missing":
        artifact.pop("sha256")
    else:
        artifact["unexpected"] = None

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "data_artifact keys must be exactly: sha256",
    )


@pytest.mark.parametrize(
    ("invalid_digest", "expected_length"),
    [
        pytest.param(None, None, id="null"),
        pytest.param("", 0, id="empty"),
        pytest.param(_DATA_NPZ_SHA256[:-1], 63, id="short"),
        pytest.param(_DATA_NPZ_SHA256 + "0", 65, id="long"),
        pytest.param("A" + _DATA_NPZ_SHA256[1:], 64, id="uppercase"),
        pytest.param("g" + _DATA_NPZ_SHA256[1:], 64, id="non_hex"),
    ],
)
def test_decode_run_manifest_requires_lowercase_sha256_digest(
    invalid_digest: object,
    expected_length: int | None,
) -> None:
    assert len(_DATA_NPZ_SHA256) == 64
    if isinstance(invalid_digest, str):
        assert len(invalid_digest) == expected_length

    mapping = _valid_artifact_bound_manifest_mapping()
    artifact = cast(dict[str, object], mapping["data_artifact"])
    artifact["sha256"] = invalid_digest

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "data_artifact.sha256 must be 64 lowercase hexadecimal characters",
    )


def _randomness_mapping(mapping: dict[str, object]) -> dict[str, object]:
    return cast(dict[str, object], mapping["randomness"])


def _stream_entries(randomness: dict[str, object]) -> list[object]:
    return cast(list[object], randomness["streams"])


def test_decode_run_manifest_requires_randomness_object() -> None:
    mapping = _valid_manifest_mapping()
    mapping["randomness"] = []

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness must be a JSON object",
    )


@pytest.mark.parametrize("mutation", ["missing", "unknown"])
def test_decode_run_manifest_requires_exact_randomness_keys(mutation: str) -> None:
    mapping = _valid_manifest_mapping()
    randomness = _randomness_mapping(mapping)
    if mutation == "missing":
        randomness.pop("derivation_version")
    else:
        randomness["unexpected"] = None

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness keys must be exactly: derivation_version, streams",
    )


@pytest.mark.parametrize("invalid_version", [True, 2])
def test_decode_run_manifest_requires_fixed_random_derivation_version(
    invalid_version: object,
) -> None:
    mapping = _valid_manifest_mapping()
    randomness = _randomness_mapping(mapping)
    randomness["derivation_version"] = invalid_version

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness.derivation_version must be the non-Boolean integer 1",
    )


def test_decode_run_manifest_requires_streams_array() -> None:
    mapping = _valid_manifest_mapping()
    randomness = _randomness_mapping(mapping)
    randomness["streams"] = {}

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness.streams must be a JSON array",
    )


@pytest.mark.parametrize("mutation", ["remove", "append"])
def test_decode_run_manifest_requires_six_stream_entries(mutation: str) -> None:
    mapping = _valid_manifest_mapping()
    streams = _stream_entries(_randomness_mapping(mapping))
    if mutation == "remove":
        streams.pop()
    else:
        streams.append({"id": 7, "name": "unexpected"})

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness.streams must contain exactly 6 entries",
    )


def test_decode_run_manifest_requires_stream_entry_object() -> None:
    mapping = _valid_manifest_mapping()
    streams = _stream_entries(_randomness_mapping(mapping))
    streams[0] = None

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness.streams[0] must be a JSON object",
    )


@pytest.mark.parametrize("mutation", ["missing", "unknown"])
def test_decode_run_manifest_requires_exact_stream_entry_keys(mutation: str) -> None:
    mapping = _valid_manifest_mapping()
    streams = _stream_entries(_randomness_mapping(mapping))
    first_stream = cast(dict[str, object], streams[0])
    if mutation == "missing":
        first_stream.pop("id")
    else:
        first_stream["unexpected"] = None

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness.streams[0] keys must be exactly: id, name",
    )


@pytest.mark.parametrize("invalid_id", [True, 99])
def test_decode_run_manifest_requires_first_stable_stream_id(invalid_id: object) -> None:
    mapping = _valid_manifest_mapping()
    streams = _stream_entries(_randomness_mapping(mapping))
    first_stream = cast(dict[str, object], streams[0])
    first_stream["id"] = invalid_id

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness.streams[0].id must be the non-Boolean integer 1",
    )


@pytest.mark.parametrize("invalid_name", [None, "unexpected"])
def test_decode_run_manifest_requires_first_stable_stream_name(invalid_name: object) -> None:
    mapping = _valid_manifest_mapping()
    streams = _stream_entries(_randomness_mapping(mapping))
    first_stream = cast(dict[str, object], streams[0])
    first_stream["name"] = invalid_name

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness.streams[0].name must equal 'accelerometer.measurement_noise'",
    )


def test_decode_run_manifest_requires_stable_stream_order() -> None:
    mapping = _valid_manifest_mapping()
    streams = _stream_entries(_randomness_mapping(mapping))
    streams[0], streams[1] = streams[1], streams[0]

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness.streams[0].id must be the non-Boolean integer 1",
    )


def test_decode_run_manifest_requires_final_stable_stream_name() -> None:
    mapping = _valid_manifest_mapping()
    streams = _stream_entries(_randomness_mapping(mapping))
    final_stream = cast(dict[str, object], streams[5])
    final_stream["name"] = "unexpected"

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "randomness.streams[5].name must equal 'barometric_altitude.measurement_noise'",
    )


def _software_provenance_mapping(mapping: dict[str, object]) -> dict[str, object]:
    return cast(dict[str, object], mapping["software_provenance"])


def test_decode_run_manifest_requires_software_provenance_object() -> None:
    mapping = _valid_manifest_mapping()
    mapping["software_provenance"] = []

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "software_provenance must be a JSON object",
    )


@pytest.mark.parametrize("mutation", ["missing", "unknown"])
def test_decode_run_manifest_requires_exact_software_provenance_keys(mutation: str) -> None:
    mapping = _valid_manifest_mapping()
    provenance = _software_provenance_mapping(mapping)
    if mutation == "missing":
        provenance.pop("package_version")
    else:
        provenance["unexpected"] = None

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "software_provenance keys must be exactly: "
        "git_commit_sha, git_worktree_clean, numpy_version, package_version, python_version",
    )


@pytest.mark.parametrize("invalid_value", [None, ""])
def test_decode_run_manifest_requires_nonempty_package_version(invalid_value: object) -> None:
    mapping = _valid_manifest_mapping()
    provenance = _software_provenance_mapping(mapping)
    provenance["package_version"] = invalid_value

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "software_provenance.package_version must be a nonempty string",
    )


@pytest.mark.parametrize("invalid_value", [1, ""])
def test_decode_run_manifest_requires_nonempty_python_version(invalid_value: object) -> None:
    mapping = _valid_manifest_mapping()
    provenance = _software_provenance_mapping(mapping)
    provenance["python_version"] = invalid_value

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "software_provenance.python_version must be a nonempty string",
    )


@pytest.mark.parametrize("invalid_value", [[], ""])
def test_decode_run_manifest_requires_nonempty_numpy_version(invalid_value: object) -> None:
    mapping = _valid_manifest_mapping()
    provenance = _software_provenance_mapping(mapping)
    provenance["numpy_version"] = invalid_value

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "software_provenance.numpy_version must be a nonempty string",
    )


@pytest.mark.parametrize("invalid_value", [None, ""])
def test_decode_run_manifest_requires_nonempty_git_commit_sha(invalid_value: object) -> None:
    mapping = _valid_manifest_mapping()
    provenance = _software_provenance_mapping(mapping)
    provenance["git_commit_sha"] = invalid_value

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "software_provenance.git_commit_sha must be a nonempty string",
    )


@pytest.mark.parametrize("invalid_value", [1, "true", None])
def test_decode_run_manifest_requires_boolean_git_worktree_clean(
    invalid_value: object,
) -> None:
    mapping = _valid_manifest_mapping()
    provenance = _software_provenance_mapping(mapping)
    provenance["git_worktree_clean"] = invalid_value

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "software_provenance.git_worktree_clean must be a Boolean",
    )


def _run_configuration_mapping(mapping: dict[str, object]) -> dict[str, object]:
    return cast(dict[str, object], mapping["run_configuration"])


@pytest.mark.parametrize("mutation", ["object", "missing", "unknown"])
@pytest.mark.parametrize(
    ("section", "missing_key", "exact_keys"),
    [
        pytest.param(
            None,
            "truth",
            "declared_mismatches, initial_truth_state, nominal, numerics, root_seed, "
            "rotor_speed_input, sensor_schedules, truth",
            id="run_configuration",
        ),
        pytest.param(
            "truth",
            "rigid_body",
            "imu, position_sensors, rigid_body, rotors, world",
            id="truth",
        ),
        pytest.param(
            "nominal",
            "world",
            "imu, position_sensors, rigid_body, rotors, world",
            id="nominal",
        ),
        pytest.param(
            "initial_truth_state",
            "q_WB",
            "omega_B, position_W, q_WB, velocity_W",
            id="initial_truth_state",
        ),
        pytest.param(
            "numerics",
            "duration_s",
            "duration_s, integration_method, number_of_steps, truth_time_step_s",
            id="numerics",
        ),
        pytest.param(
            "sensor_schedules",
            "gyroscope",
            "accelerometer, barometric_altitude, gyroscope, local_position",
            id="sensor_schedules",
        ),
        pytest.param(
            "rotor_speed_input",
            "rotor_omega",
            "rotor_omega",
            id="rotor_speed_input",
        ),
    ],
)
def test_decode_run_manifest_requires_configuration_shell_object_and_keys(
    section: str | None,
    missing_key: str,
    exact_keys: str,
    mutation: str,
) -> None:
    mapping = _valid_manifest_mapping()
    if section is None:
        parent = mapping
        key = "run_configuration"
        boundary = key
    else:
        parent = _run_configuration_mapping(mapping)
        key = section
        boundary = f"run_configuration.{section}"

    if mutation == "object":
        parent[key] = []
        expected_message = f"{boundary} must be a JSON object"
    else:
        target = cast(dict[str, object], parent[key])
        if mutation == "missing":
            target.pop(missing_key)
        else:
            target["unexpected"] = None
        expected_message = f"{boundary} keys must be exactly: {exact_keys}"

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        expected_message,
    )


def test_decode_run_manifest_requires_declared_mismatches_array() -> None:
    mapping = _valid_manifest_mapping()
    configuration = _run_configuration_mapping(mapping)
    configuration["declared_mismatches"] = {}

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        "run_configuration.declared_mismatches must be a JSON array",
    )


@pytest.mark.parametrize("mutation", ["object", "missing", "unknown"])
@pytest.mark.parametrize(
    ("group", "parameter_object", "missing_key", "exact_keys"),
    [
        pytest.param("truth", "rigid_body", "mass", "inertia_B, mass", id="truth-rigid_body"),
        pytest.param(
            "truth",
            "rotors",
            "rotor_positions_B",
            "moment_coefficient, rotor_positions_B, rotor_spin_directions, thrust_coefficient",
            id="truth-rotors",
        ),
        pytest.param(
            "truth",
            "world",
            "gravity_acceleration",
            "gravity_acceleration",
            id="truth-world",
        ),
        pytest.param(
            "truth",
            "imu",
            "initial_accelerometer_bias_B",
            "accelerometer_bias_random_walk_density_B, "
            "accelerometer_noise_standard_deviation_B, "
            "gyroscope_bias_random_walk_density_B, "
            "gyroscope_noise_standard_deviation_B, "
            "initial_accelerometer_bias_B, initial_gyroscope_bias_B",
            id="truth-imu",
        ),
        pytest.param(
            "truth",
            "position_sensors",
            "local_position_bias_W",
            "barometric_altitude_bias, barometric_altitude_noise_standard_deviation, "
            "barometric_reference_altitude, local_position_bias_W, "
            "local_position_noise_standard_deviation_W",
            id="truth-position_sensors",
        ),
        pytest.param("nominal", "rigid_body", "mass", "inertia_B, mass", id="nominal-rigid_body"),
        pytest.param(
            "nominal",
            "rotors",
            "rotor_positions_B",
            "moment_coefficient, rotor_positions_B, rotor_spin_directions, thrust_coefficient",
            id="nominal-rotors",
        ),
        pytest.param(
            "nominal",
            "world",
            "gravity_acceleration",
            "gravity_acceleration",
            id="nominal-world",
        ),
        pytest.param(
            "nominal",
            "imu",
            "initial_accelerometer_bias_B",
            "accelerometer_bias_random_walk_density_B, "
            "accelerometer_noise_standard_deviation_B, "
            "gyroscope_bias_random_walk_density_B, "
            "gyroscope_noise_standard_deviation_B, "
            "initial_accelerometer_bias_B, initial_gyroscope_bias_B",
            id="nominal-imu",
        ),
        pytest.param(
            "nominal",
            "position_sensors",
            "local_position_bias_W",
            "barometric_altitude_bias, barometric_altitude_noise_standard_deviation, "
            "barometric_reference_altitude, local_position_bias_W, "
            "local_position_noise_standard_deviation_W",
            id="nominal-position_sensors",
        ),
    ],
)
def test_decode_run_manifest_requires_nested_parameter_object_and_exact_keys(
    group: str,
    parameter_object: str,
    missing_key: str,
    exact_keys: str,
    mutation: str,
) -> None:
    mapping = _valid_manifest_mapping()
    configuration = _run_configuration_mapping(mapping)
    group_mapping = cast(dict[str, object], configuration[group])
    path = f"run_configuration.{group}.{parameter_object}"

    if mutation == "object":
        group_mapping[parameter_object] = []
        expected_message = f"{path} must be a JSON object"
    else:
        parameter_mapping = cast(dict[str, object], group_mapping[parameter_object])
        if mutation == "missing":
            parameter_mapping.pop(missing_key)
        else:
            parameter_mapping["unexpected"] = None
        expected_message = f"{path} keys must be exactly: {exact_keys}"

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        expected_message,
    )


@pytest.mark.parametrize("mutation", ["object", "missing", "unknown"])
@pytest.mark.parametrize(
    ("channel", "missing_key"),
    [
        pytest.param("accelerometer", "requested_sample_period_s", id="accelerometer"),
        pytest.param("gyroscope", "sample_stride", id="gyroscope"),
        pytest.param("local_position", "effective_sample_period_s", id="local_position"),
        pytest.param("barometric_altitude", "delivery_delay_s", id="barometric_altitude"),
    ],
)
def test_decode_run_manifest_requires_sensor_schedule_object_and_exact_keys(
    channel: str,
    missing_key: str,
    mutation: str,
) -> None:
    mapping = _valid_manifest_mapping()
    configuration = _run_configuration_mapping(mapping)
    schedules = cast(dict[str, object], configuration["sensor_schedules"])
    path = f"run_configuration.sensor_schedules.{channel}"

    if mutation == "object":
        schedules[channel] = []
        expected_message = f"{path} must be a JSON object"
    else:
        schedule = cast(dict[str, object], schedules[channel])
        if mutation == "missing":
            schedule.pop(missing_key)
        else:
            schedule["unexpected"] = None
        exact_keys = (
            "delivery_delay_s, effective_sample_period_s, requested_sample_period_s, sample_stride"
        )
        expected_message = f"{path} keys must be exactly: {exact_keys}"

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        expected_message,
    )


@pytest.mark.parametrize(
    ("mutation", "expected_message"),
    [
        pytest.param(
            "object_zero",
            "run_configuration.declared_mismatches[0] must be a JSON object",
            id="object_zero",
        ),
        pytest.param(
            "missing_zero",
            "run_configuration.declared_mismatches[0] keys must be exactly: "
            "parameter_path, rationale",
            id="missing_zero",
        ),
        pytest.param(
            "unknown_zero",
            "run_configuration.declared_mismatches[0] keys must be exactly: "
            "parameter_path, rationale",
            id="unknown_zero",
        ),
        pytest.param(
            "object_one",
            "run_configuration.declared_mismatches[1] must be a JSON object",
            id="object_one",
        ),
    ],
)
def test_decode_run_manifest_requires_declared_mismatch_entry_object_and_exact_keys(
    mutation: str,
    expected_message: str,
) -> None:
    mapping = _valid_manifest_mapping()
    configuration = _run_configuration_mapping(mapping)
    entries = cast(list[object], configuration["declared_mismatches"])
    if mutation == "object_zero":
        entries[0] = None
    elif mutation == "object_one":
        entries.append(None)
    else:
        first_entry = cast(dict[str, object], entries[0])
        if mutation == "missing_zero":
            first_entry.pop("parameter_path")
        else:
            first_entry["unexpected"] = None

    _assert_manifest_decode_error(
        _canonical_manifest_bytes(mapping),
        expected_message,
    )
