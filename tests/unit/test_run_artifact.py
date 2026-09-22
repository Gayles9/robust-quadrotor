"""Contract for the future in-memory run artifact array model."""

import hashlib
import io
import json
import os
import stat
from dataclasses import FrozenInstanceError, fields, replace
from pathlib import Path
from typing import cast
from unittest.mock import Mock

import numpy as np
import pytest

import quadrotor_math.run_artifact as run_artifact
from quadrotor_math.run_artifact import RunArtifactData
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
    RunManifest,
    SoftwareProvenance,
    decode_run_manifest,
    encode_run_manifest,
)
from quadrotor_math.validation import validate_rigid_body_state_history

_N = 2
_T = _N + 1
_M_A = 2
_M_G = 1
_M_L = 2
_M_B = 1
_D = 5
_FLOAT_DTYPE = np.dtype("<f8")
_INTEGER_DTYPE = np.dtype("<i8")

_ARRAY_FIELDS = (
    "truth_time_s",
    "truth_position_history_W",
    "truth_velocity_history_W",
    "truth_q_history_WB",
    "truth_omega_history_B",
    "commanded_rotor_omega",
    "accelerometer_sequence_index",
    "accelerometer_observation_index",
    "accelerometer_acquisition_time_s",
    "accelerometer_delivery_time_s",
    "accelerometer_delivered_at_truth_index",
    "accelerometer_measurements_B",
    "gyroscope_sequence_index",
    "gyroscope_observation_index",
    "gyroscope_acquisition_time_s",
    "gyroscope_delivery_time_s",
    "gyroscope_delivered_at_truth_index",
    "gyroscope_measurements_B",
    "local_position_sequence_index",
    "local_position_observation_index",
    "local_position_acquisition_time_s",
    "local_position_delivery_time_s",
    "local_position_delivered_at_truth_index",
    "local_position_measurements_W",
    "barometric_altitude_sequence_index",
    "barometric_altitude_observation_index",
    "barometric_altitude_acquisition_time_s",
    "barometric_altitude_delivery_time_s",
    "barometric_altitude_delivered_at_truth_index",
    "barometric_altitude_measurements",
    "accelerometer_bias_history_B",
    "gyroscope_bias_history_B",
    "delivery_sensor_id",
    "delivery_sequence_index",
    "delivery_observation_index",
)

_FLOAT_FIELDS = (
    "truth_time_s",
    "truth_position_history_W",
    "truth_velocity_history_W",
    "truth_q_history_WB",
    "truth_omega_history_B",
    "commanded_rotor_omega",
    "accelerometer_acquisition_time_s",
    "accelerometer_delivery_time_s",
    "accelerometer_measurements_B",
    "gyroscope_acquisition_time_s",
    "gyroscope_delivery_time_s",
    "gyroscope_measurements_B",
    "local_position_acquisition_time_s",
    "local_position_delivery_time_s",
    "local_position_measurements_W",
    "barometric_altitude_acquisition_time_s",
    "barometric_altitude_delivery_time_s",
    "barometric_altitude_measurements",
    "accelerometer_bias_history_B",
    "gyroscope_bias_history_B",
)

_INTEGER_FIELDS = (
    "accelerometer_sequence_index",
    "accelerometer_observation_index",
    "accelerometer_delivered_at_truth_index",
    "gyroscope_sequence_index",
    "gyroscope_observation_index",
    "gyroscope_delivered_at_truth_index",
    "local_position_sequence_index",
    "local_position_observation_index",
    "local_position_delivered_at_truth_index",
    "barometric_altitude_sequence_index",
    "barometric_altitude_observation_index",
    "barometric_altitude_delivered_at_truth_index",
    "delivery_sensor_id",
    "delivery_sequence_index",
    "delivery_observation_index",
)

_ONE_DIMENSION_FIELDS = (
    "truth_time_s",
    "accelerometer_sequence_index",
    "accelerometer_observation_index",
    "accelerometer_acquisition_time_s",
    "accelerometer_delivery_time_s",
    "accelerometer_delivered_at_truth_index",
    "gyroscope_sequence_index",
    "gyroscope_observation_index",
    "gyroscope_acquisition_time_s",
    "gyroscope_delivery_time_s",
    "gyroscope_delivered_at_truth_index",
    "local_position_sequence_index",
    "local_position_observation_index",
    "local_position_acquisition_time_s",
    "local_position_delivery_time_s",
    "local_position_delivered_at_truth_index",
    "barometric_altitude_sequence_index",
    "barometric_altitude_observation_index",
    "barometric_altitude_acquisition_time_s",
    "barometric_altitude_delivery_time_s",
    "barometric_altitude_delivered_at_truth_index",
    "barometric_altitude_measurements",
    "delivery_sensor_id",
    "delivery_sequence_index",
    "delivery_observation_index",
)

_WIDTH_THREE_FIELDS = (
    "truth_position_history_W",
    "truth_velocity_history_W",
    "truth_omega_history_B",
    "accelerometer_measurements_B",
    "gyroscope_measurements_B",
    "local_position_measurements_W",
    "accelerometer_bias_history_B",
    "gyroscope_bias_history_B",
)

_WIDTH_FOUR_FIELDS = (
    "truth_q_history_WB",
    "commanded_rotor_omega",
)

_TRUTH_HISTORY_FIELDS = (
    "truth_time_s",
    "truth_position_history_W",
    "truth_velocity_history_W",
    "truth_q_history_WB",
    "truth_omega_history_B",
)

_BIAS_HISTORY_FIELDS = (
    "accelerometer_bias_history_B",
    "gyroscope_bias_history_B",
)

_SENSOR_STREAM_FIELDS: dict[str, tuple[str, ...]] = {
    "accelerometer": (
        "accelerometer_sequence_index",
        "accelerometer_observation_index",
        "accelerometer_acquisition_time_s",
        "accelerometer_delivery_time_s",
        "accelerometer_delivered_at_truth_index",
        "accelerometer_measurements_B",
    ),
    "gyroscope": (
        "gyroscope_sequence_index",
        "gyroscope_observation_index",
        "gyroscope_acquisition_time_s",
        "gyroscope_delivery_time_s",
        "gyroscope_delivered_at_truth_index",
        "gyroscope_measurements_B",
    ),
    "local_position": (
        "local_position_sequence_index",
        "local_position_observation_index",
        "local_position_acquisition_time_s",
        "local_position_delivery_time_s",
        "local_position_delivered_at_truth_index",
        "local_position_measurements_W",
    ),
    "barometric_altitude": (
        "barometric_altitude_sequence_index",
        "barometric_altitude_observation_index",
        "barometric_altitude_acquisition_time_s",
        "barometric_altitude_delivery_time_s",
        "barometric_altitude_delivered_at_truth_index",
        "barometric_altitude_measurements",
    ),
}

_DELIVERY_TABLE_FIELDS = (
    "delivery_sensor_id",
    "delivery_sequence_index",
    "delivery_observation_index",
)

_EXPECTED_SHAPES: dict[str, tuple[int, ...]] = {
    "truth_time_s": (_T,),
    "truth_position_history_W": (_T, 3),
    "truth_velocity_history_W": (_T, 3),
    "truth_q_history_WB": (_T, 4),
    "truth_omega_history_B": (_T, 3),
    "commanded_rotor_omega": (_N, 4),
    "accelerometer_sequence_index": (_M_A,),
    "accelerometer_observation_index": (_M_A,),
    "accelerometer_acquisition_time_s": (_M_A,),
    "accelerometer_delivery_time_s": (_M_A,),
    "accelerometer_delivered_at_truth_index": (_M_A,),
    "accelerometer_measurements_B": (_M_A, 3),
    "gyroscope_sequence_index": (_M_G,),
    "gyroscope_observation_index": (_M_G,),
    "gyroscope_acquisition_time_s": (_M_G,),
    "gyroscope_delivery_time_s": (_M_G,),
    "gyroscope_delivered_at_truth_index": (_M_G,),
    "gyroscope_measurements_B": (_M_G, 3),
    "local_position_sequence_index": (_M_L,),
    "local_position_observation_index": (_M_L,),
    "local_position_acquisition_time_s": (_M_L,),
    "local_position_delivery_time_s": (_M_L,),
    "local_position_delivered_at_truth_index": (_M_L,),
    "local_position_measurements_W": (_M_L, 3),
    "barometric_altitude_sequence_index": (_M_B,),
    "barometric_altitude_observation_index": (_M_B,),
    "barometric_altitude_acquisition_time_s": (_M_B,),
    "barometric_altitude_delivery_time_s": (_M_B,),
    "barometric_altitude_delivered_at_truth_index": (_M_B,),
    "barometric_altitude_measurements": (_M_B,),
    "accelerometer_bias_history_B": (_T, 3),
    "gyroscope_bias_history_B": (_T, 3),
    "delivery_sensor_id": (_D,),
    "delivery_sequence_index": (_D,),
    "delivery_observation_index": (_D,),
}


def _writable_source_arrays() -> dict[str, np.ndarray]:
    """Build fresh source arrays, including a non-C-contiguous position view."""
    position_base = np.array(
        [[0.0, 0.0, 0.0], [0.1, 0.0, -0.1], [0.2, 0.0, -0.2]],
        dtype=_FLOAT_DTYPE,
        order="F",
    )
    return {
        "truth_time_s": np.array([0.0, 0.1, 0.2], dtype=_FLOAT_DTYPE),
        "truth_position_history_W": position_base.view(),
        "truth_velocity_history_W": np.array(
            [[1.0, 0.0, -1.0], [1.0, 0.0, -1.0], [1.0, 0.0, -1.0]],
            dtype=_FLOAT_DTYPE,
        ),
        "truth_q_history_WB": np.array(
            [[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            dtype=_FLOAT_DTYPE,
        ),
        "truth_omega_history_B": np.zeros((_T, 3), dtype=_FLOAT_DTYPE),
        "commanded_rotor_omega": np.array(
            [[400.0, 410.0, 420.0, 430.0], [400.0, 410.0, 420.0, 430.0]],
            dtype=_FLOAT_DTYPE,
        ),
        "accelerometer_sequence_index": np.array([0, 1], dtype=_INTEGER_DTYPE),
        "accelerometer_observation_index": np.array([0, 1], dtype=_INTEGER_DTYPE),
        "accelerometer_acquisition_time_s": np.array([0.1, 0.2], dtype=_FLOAT_DTYPE),
        "accelerometer_delivery_time_s": np.array([0.1, 0.2], dtype=_FLOAT_DTYPE),
        "accelerometer_delivered_at_truth_index": np.array([1, 2], dtype=_INTEGER_DTYPE),
        "accelerometer_measurements_B": np.array(
            [[0.0, 0.0, -9.71], [0.0, 0.0, -9.72]], dtype=_FLOAT_DTYPE
        ),
        "gyroscope_sequence_index": np.array([0], dtype=_INTEGER_DTYPE),
        "gyroscope_observation_index": np.array([0], dtype=_INTEGER_DTYPE),
        "gyroscope_acquisition_time_s": np.array([0.2], dtype=_FLOAT_DTYPE),
        "gyroscope_delivery_time_s": np.array([0.2], dtype=_FLOAT_DTYPE),
        "gyroscope_delivered_at_truth_index": np.array([2], dtype=_INTEGER_DTYPE),
        "gyroscope_measurements_B": np.array([[0.01, -0.02, 0.03]], dtype=_FLOAT_DTYPE),
        "local_position_sequence_index": np.array([0, 1], dtype=_INTEGER_DTYPE),
        "local_position_observation_index": np.array([0, 1], dtype=_INTEGER_DTYPE),
        "local_position_acquisition_time_s": np.array([0.1, 0.2], dtype=_FLOAT_DTYPE),
        "local_position_delivery_time_s": np.array([0.2, 0.3], dtype=_FLOAT_DTYPE),
        "local_position_delivered_at_truth_index": np.array([2, -1], dtype=_INTEGER_DTYPE),
        "local_position_measurements_W": np.array(
            [[0.1, 0.2, -0.1], [0.2, 0.3, -0.2]], dtype=_FLOAT_DTYPE
        ),
        "barometric_altitude_sequence_index": np.array([0], dtype=_INTEGER_DTYPE),
        "barometric_altitude_observation_index": np.array([0], dtype=_INTEGER_DTYPE),
        "barometric_altitude_acquisition_time_s": np.array([0.2], dtype=_FLOAT_DTYPE),
        "barometric_altitude_delivery_time_s": np.array([0.2], dtype=_FLOAT_DTYPE),
        "barometric_altitude_delivered_at_truth_index": np.array([2], dtype=_INTEGER_DTYPE),
        "barometric_altitude_measurements": np.array([123.4], dtype=_FLOAT_DTYPE),
        "accelerometer_bias_history_B": np.array(
            [[0.10, 0.0, 0.0], [0.11, 0.0, 0.0], [0.12, 0.0, 0.0]],
            dtype=_FLOAT_DTYPE,
        ),
        "gyroscope_bias_history_B": np.array(
            [[0.0, 0.01, 0.0], [0.0, 0.02, 0.0], [0.0, 0.03, 0.0]],
            dtype=_FLOAT_DTYPE,
        ),
        "delivery_sensor_id": np.array([1, 1, 2, 3, 4], dtype=_INTEGER_DTYPE),
        "delivery_sequence_index": np.array([0, 1, 0, 0, 0], dtype=_INTEGER_DTYPE),
        "delivery_observation_index": np.array([0, 1, 0, 0, 0], dtype=_INTEGER_DTYPE),
    }


def _artifact_data(sources: dict[str, np.ndarray]) -> RunArtifactData:
    """Pass every future public field explicitly to the artifact constructor."""
    return RunArtifactData(
        truth_time_s=sources["truth_time_s"],
        truth_position_history_W=sources["truth_position_history_W"],
        truth_velocity_history_W=sources["truth_velocity_history_W"],
        truth_q_history_WB=sources["truth_q_history_WB"],
        truth_omega_history_B=sources["truth_omega_history_B"],
        commanded_rotor_omega=sources["commanded_rotor_omega"],
        accelerometer_sequence_index=sources["accelerometer_sequence_index"],
        accelerometer_observation_index=sources["accelerometer_observation_index"],
        accelerometer_acquisition_time_s=sources["accelerometer_acquisition_time_s"],
        accelerometer_delivery_time_s=sources["accelerometer_delivery_time_s"],
        accelerometer_delivered_at_truth_index=sources["accelerometer_delivered_at_truth_index"],
        accelerometer_measurements_B=sources["accelerometer_measurements_B"],
        gyroscope_sequence_index=sources["gyroscope_sequence_index"],
        gyroscope_observation_index=sources["gyroscope_observation_index"],
        gyroscope_acquisition_time_s=sources["gyroscope_acquisition_time_s"],
        gyroscope_delivery_time_s=sources["gyroscope_delivery_time_s"],
        gyroscope_delivered_at_truth_index=sources["gyroscope_delivered_at_truth_index"],
        gyroscope_measurements_B=sources["gyroscope_measurements_B"],
        local_position_sequence_index=sources["local_position_sequence_index"],
        local_position_observation_index=sources["local_position_observation_index"],
        local_position_acquisition_time_s=sources["local_position_acquisition_time_s"],
        local_position_delivery_time_s=sources["local_position_delivery_time_s"],
        local_position_delivered_at_truth_index=sources["local_position_delivered_at_truth_index"],
        local_position_measurements_W=sources["local_position_measurements_W"],
        barometric_altitude_sequence_index=sources["barometric_altitude_sequence_index"],
        barometric_altitude_observation_index=sources["barometric_altitude_observation_index"],
        barometric_altitude_acquisition_time_s=sources["barometric_altitude_acquisition_time_s"],
        barometric_altitude_delivery_time_s=sources["barometric_altitude_delivery_time_s"],
        barometric_altitude_delivered_at_truth_index=sources[
            "barometric_altitude_delivered_at_truth_index"
        ],
        barometric_altitude_measurements=sources["barometric_altitude_measurements"],
        accelerometer_bias_history_B=sources["accelerometer_bias_history_B"],
        gyroscope_bias_history_B=sources["gyroscope_bias_history_B"],
        delivery_sensor_id=sources["delivery_sensor_id"],
        delivery_sequence_index=sources["delivery_sequence_index"],
        delivery_observation_index=sources["delivery_observation_index"],
    )


def _artifact_npz_members(data: RunArtifactData) -> dict[str, np.ndarray]:
    """Map the explicit test schema to one valid artifact's arrays."""
    return {name: getattr(data, name) for name in _ARRAY_FIELDS}


def _encode_npz_members(members: dict[str, np.ndarray]) -> bytes:
    """Encode supplied members as an uncompressed in-memory NPZ payload."""
    buffer = io.BytesIO()
    np.savez(buffer, **members)
    return buffer.getvalue()


def _assert_npz_member_schema_error(payload: bytes) -> None:
    with pytest.raises(ValueError) as error:
        run_artifact._decode_data_npz(payload)

    assert str(error.value) == "data.npz member names must match the run artifact schema exactly"


def _unbound_save_manifest() -> RunManifest:
    """Build supplied v1 metadata for the two-transition artifact fixture."""
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
            initial_accelerometer_bias_B=np.array([0.1, 0.0, 0.0], dtype=np.float64),
            accelerometer_noise_standard_deviation_B=np.array([0.01, 0.02, 0.03], dtype=np.float64),
            accelerometer_bias_random_walk_density_B=np.array(
                [0.001, 0.002, 0.003], dtype=np.float64
            ),
            initial_gyroscope_bias_B=np.array([0.0, 0.01, 0.0], dtype=np.float64),
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
    configuration = RunConfiguration(
        truth=truth,
        nominal=NominalConfiguration(
            rigid_body=RigidBodyParameters(mass=1.4, inertia_B=truth.rigid_body.inertia_B),
            rotors=truth.rotors,
            world=truth.world,
            imu=truth.imu,
            position_sensors=truth.position_sensors,
        ),
        initial_truth_state=RigidBodyInitialState(
            position_W=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            velocity_W=np.array([1.0, 0.0, -1.0], dtype=np.float64),
            q_WB=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            omega_B=np.array([0.0, 0.0, 0.0], dtype=np.float64),
        ),
        numerics=RunNumerics(
            integration_method=IntegrationMethod.PROJECTED_RK4,
            truth_time_step_s=0.1,
            number_of_steps=2,
        ),
        sensor_schedules=SensorSchedules(
            accelerometer=SensorSchedule(sample_period_s=0.1, delivery_delay_s=0.0),
            gyroscope=SensorSchedule(sample_period_s=0.2, delivery_delay_s=0.0),
            local_position=SensorSchedule(sample_period_s=0.1, delivery_delay_s=0.1),
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
    return RunManifest(
        run_configuration=configuration,
        software_provenance=SoftwareProvenance(
            package_version="0.1.0",
            python_version="3.12.3",
            numpy_version="2.5.2",
            git_commit_sha="99988cde60be738af974a3762dd68512258b0e1f",
            git_worktree_clean=True,
        ),
    )


def test_run_artifact_data_preserves_exact_array_schema() -> None:
    sources = _writable_source_arrays()
    artifact = _artifact_data(sources)

    assert len(_ARRAY_FIELDS) == 35
    assert len(_FLOAT_FIELDS) == 20
    assert len(_INTEGER_FIELDS) == 15
    assert set(_FLOAT_FIELDS).isdisjoint(_INTEGER_FIELDS)
    assert set(_FLOAT_FIELDS) | set(_INTEGER_FIELDS) == set(_ARRAY_FIELDS)
    assert tuple(sources) == _ARRAY_FIELDS
    assert tuple(field.name for field in fields(artifact)) == _ARRAY_FIELDS

    for name in _ARRAY_FIELDS:
        stored = getattr(artifact, name)
        expected = sources[name]
        assert type(stored) is np.ndarray
        assert expected.shape == _EXPECTED_SHAPES[name]
        assert stored.shape == _EXPECTED_SHAPES[name]
        assert stored.dtype == (_FLOAT_DTYPE if name in _FLOAT_FIELDS else _INTEGER_DTYPE)
        np.testing.assert_array_equal(stored, expected)

    assert artifact.barometric_altitude_measurements.ndim == 1
    assert artifact.barometric_altitude_measurements.shape == (1,)
    assert artifact.commanded_rotor_omega.shape == (_N, 4)
    assert artifact.accelerometer_bias_history_B.shape == (_T, 3)
    assert artifact.gyroscope_bias_history_B.shape == (_T, 3)


def test_run_artifact_data_owns_read_only_c_contiguous_copies() -> None:
    sources = _writable_source_arrays()
    expected = {name: np.array(source, copy=True) for name, source in sources.items()}
    position_source = sources["truth_position_history_W"]
    assert position_source.base is not None
    assert position_source.flags.writeable
    assert not position_source.flags.c_contiguous

    artifact = _artifact_data(sources)

    for name in _ARRAY_FIELDS:
        stored = getattr(artifact, name)
        source = sources[name]
        assert not np.shares_memory(stored, source)
        assert stored.flags.owndata
        assert stored.flags.c_contiguous
        assert not stored.flags.writeable
        assert source.flags.writeable
        np.testing.assert_array_equal(stored, expected[name])

    for index, name in enumerate(_ARRAY_FIELDS):
        sources[name][...] = 999 + index

    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(artifact, name), expected[name])

    for name in (
        "truth_time_s",
        "truth_position_history_W",
        "barometric_altitude_measurements",
        "delivery_sensor_id",
    ):
        with pytest.raises(ValueError):
            getattr(artifact, name).flat[0] = 999


def test_run_artifact_data_is_frozen_slotted_and_identity_compared() -> None:
    first = _artifact_data(_writable_source_arrays())
    second = _artifact_data(_writable_source_arrays())

    with pytest.raises(FrozenInstanceError):
        first.truth_time_s = np.array([0.0, 0.1, 0.2], dtype=_FLOAT_DTYPE)

    assert not hasattr(first, "__dict__")
    assert first is not second
    assert first != second
    assert first == first


def _assert_artifact_data_error(sources: dict[str, np.ndarray], expected_message: str) -> None:
    with pytest.raises(ValueError) as error:
        _artifact_data(sources)

    assert str(error.value) == expected_message


@pytest.mark.parametrize("field_name", ["truth_time_s", "delivery_sensor_id"])
def test_run_artifact_data_requires_numpy_array(field_name: str) -> None:
    sources = _writable_source_arrays()
    sources[field_name] = cast(np.ndarray, sources[field_name].tolist())

    _assert_artifact_data_error(sources, f"{field_name} must be a NumPy array")


@pytest.mark.parametrize(
    ("field_name", "invalid_dtype", "expected_dtype_name"),
    [
        *[
            pytest.param(
                name,
                np.dtype("<f4"),
                "little-endian float64",
                id=f"{name}_float32",
            )
            for name in _FLOAT_FIELDS
        ],
        *[
            pytest.param(
                name,
                np.dtype("<i4"),
                "little-endian int64",
                id=f"{name}_int32",
            )
            for name in _INTEGER_FIELDS
        ],
        pytest.param(
            "truth_time_s",
            np.dtype(">f8"),
            "little-endian float64",
            id="truth_time_s_big_endian",
        ),
        pytest.param(
            "delivery_sensor_id",
            np.dtype(">i8"),
            "little-endian int64",
            id="delivery_sensor_id_big_endian",
        ),
    ],
)
def test_run_artifact_data_requires_exact_field_dtype(
    field_name: str, invalid_dtype: np.dtype, expected_dtype_name: str
) -> None:
    sources = _writable_source_arrays()
    invalid_array = sources[field_name].astype(invalid_dtype)
    if invalid_dtype.byteorder == ">":
        invalid_array = invalid_array.reshape(-1, 1)
    sources[field_name] = invalid_array

    _assert_artifact_data_error(
        sources,
        f"{field_name} must have dtype {expected_dtype_name}",
    )


@pytest.mark.parametrize(
    ("field_name", "mutation", "expected_shape"),
    [
        *[
            pytest.param(name, "wrong_rank", "(n,)", id=f"{name}_wrong_rank")
            for name in _ONE_DIMENSION_FIELDS
        ],
        *[
            pytest.param(name, "wrong_rank", "(n, 3)", id=f"{name}_wrong_rank")
            for name in _WIDTH_THREE_FIELDS
        ],
        *[
            pytest.param(name, "wrong_rank", "(n, 4)", id=f"{name}_wrong_rank")
            for name in _WIDTH_FOUR_FIELDS
        ],
        *[
            pytest.param(name, "wrong_width", "(n, 3)", id=f"{name}_wrong_width")
            for name in _WIDTH_THREE_FIELDS
        ],
        *[
            pytest.param(name, "wrong_width", "(n, 4)", id=f"{name}_wrong_width")
            for name in _WIDTH_FOUR_FIELDS
        ],
    ],
)
def test_run_artifact_data_requires_static_field_shape(
    field_name: str, mutation: str, expected_shape: str
) -> None:
    sources = _writable_source_arrays()
    source = sources[field_name]
    if field_name in _ONE_DIMENSION_FIELDS:
        invalid_array = source.reshape(-1, 1)
    elif mutation == "wrong_rank":
        invalid_array = source.ravel()
    else:
        invalid_array = np.zeros((source.shape[0], source.shape[1] + 1), dtype=source.dtype)
    sources[field_name] = invalid_array

    _assert_artifact_data_error(sources, f"{field_name} must have shape {expected_shape}")


def _remove_final_leading_entry(sources: dict[str, np.ndarray], field_name: str) -> None:
    source = sources[field_name]
    shortened = source[:-1]
    assert type(shortened) is np.ndarray
    assert shortened.dtype == source.dtype
    assert shortened.ndim == source.ndim
    assert shortened.shape[1:] == source.shape[1:]
    assert shortened.shape[0] == source.shape[0] - 1
    sources[field_name] = shortened


@pytest.mark.parametrize("field_name", _TRUTH_HISTORY_FIELDS)
def test_run_artifact_data_requires_equal_truth_history_lengths(field_name: str) -> None:
    sources = _writable_source_arrays()
    _remove_final_leading_entry(sources, field_name)

    _assert_artifact_data_error(
        sources,
        "truth history fields must have equal leading dimensions",
    )


@pytest.mark.parametrize("field_name", _BIAS_HISTORY_FIELDS)
def test_run_artifact_data_requires_bias_histories_to_match_truth_length(field_name: str) -> None:
    sources = _writable_source_arrays()
    _remove_final_leading_entry(sources, field_name)

    _assert_artifact_data_error(
        sources,
        f"{field_name} must have {_T} rows to match truth history",
    )


@pytest.mark.parametrize("command_rows", [1, 3])
def test_run_artifact_data_requires_command_rows_to_match_truth_transitions(
    command_rows: int,
) -> None:
    sources = _writable_source_arrays()
    command_source = sources["commanded_rotor_omega"]
    if command_rows == 1:
        changed_command = command_source[:1]
    else:
        changed_command = np.concatenate(
            (command_source, command_source[:1].copy(order="C")),
            axis=0,
            dtype=command_source.dtype,
        )
    assert type(changed_command) is np.ndarray
    assert changed_command.dtype == command_source.dtype
    assert changed_command.ndim == command_source.ndim
    assert changed_command.shape[1:] == command_source.shape[1:]
    assert changed_command.shape[0] == command_rows
    sources["commanded_rotor_omega"] = changed_command

    _assert_artifact_data_error(
        sources,
        f"commanded_rotor_omega must have {_N} rows to match truth transitions",
    )


@pytest.mark.parametrize(
    ("channel", "field_name"),
    [
        pytest.param(channel, name, id=name)
        for channel, channel_fields in _SENSOR_STREAM_FIELDS.items()
        for name in channel_fields
    ],
)
def test_run_artifact_data_requires_equal_sensor_stream_lengths(
    channel: str, field_name: str
) -> None:
    sources = _writable_source_arrays()
    _remove_final_leading_entry(sources, field_name)

    _assert_artifact_data_error(
        sources,
        f"{channel} fields must have equal leading dimensions",
    )


@pytest.mark.parametrize(
    "field_name",
    [
        pytest.param("accelerometer_sequence_index", id="accelerometer_sequence_index"),
        pytest.param("gyroscope_sequence_index", id="gyroscope_sequence_index"),
        pytest.param("local_position_sequence_index", id="local_position_sequence_index"),
        pytest.param("barometric_altitude_sequence_index", id="barometric_altitude_sequence_index"),
    ],
)
def test_run_artifact_data_requires_consecutive_stream_sequence_indices(field_name: str) -> None:
    sources = _writable_source_arrays()
    source = sources[field_name]
    assert source[0] == 0
    changed_index = source.copy()
    changed_index[0] = 1
    assert type(changed_index) is np.ndarray
    assert changed_index.dtype == _INTEGER_DTYPE
    assert changed_index.ndim == source.ndim
    assert changed_index.shape == source.shape
    assert changed_index.tolist() == ([1, 1] if source.shape[0] == 2 else [1])
    sources[field_name] = changed_index

    _assert_artifact_data_error(
        sources,
        f"{field_name} must equal consecutive zero-based indices",
    )


@pytest.mark.parametrize(
    "field_name",
    [
        pytest.param("accelerometer_observation_index", id="accelerometer_observation_index"),
        pytest.param("gyroscope_observation_index", id="gyroscope_observation_index"),
        pytest.param("local_position_observation_index", id="local_position_observation_index"),
        pytest.param(
            "barometric_altitude_observation_index", id="barometric_altitude_observation_index"
        ),
    ],
)
def test_run_artifact_data_requires_consecutive_stream_observation_indices(
    field_name: str,
) -> None:
    sources = _writable_source_arrays()
    source = sources[field_name]
    assert source[0] == 0
    changed_index = source.copy()
    changed_index[0] = 1
    assert type(changed_index) is np.ndarray
    assert changed_index.dtype == _INTEGER_DTYPE
    assert changed_index.ndim == source.ndim
    assert changed_index.shape == source.shape
    assert changed_index.tolist() == ([1, 1] if source.shape[0] == 2 else [1])
    sources[field_name] = changed_index

    _assert_artifact_data_error(
        sources,
        f"{field_name} must equal consecutive zero-based indices",
    )


@pytest.mark.parametrize(
    ("field_name", "invalid_index"),
    [
        pytest.param(
            "accelerometer_delivered_at_truth_index", -2, id="accelerometer-less-than-pending"
        ),
        pytest.param("accelerometer_delivered_at_truth_index", 0, id="accelerometer-initial-row"),
        pytest.param(
            "accelerometer_delivered_at_truth_index", 3, id="accelerometer-after-final-row"
        ),
        pytest.param("gyroscope_delivered_at_truth_index", -2, id="gyroscope-less-than-pending"),
        pytest.param("gyroscope_delivered_at_truth_index", 0, id="gyroscope-initial-row"),
        pytest.param("gyroscope_delivered_at_truth_index", 3, id="gyroscope-after-final-row"),
        pytest.param(
            "local_position_delivered_at_truth_index", -2, id="local-position-less-than-pending"
        ),
        pytest.param("local_position_delivered_at_truth_index", 0, id="local-position-initial-row"),
        pytest.param(
            "local_position_delivered_at_truth_index", 3, id="local-position-after-final-row"
        ),
        pytest.param(
            "barometric_altitude_delivered_at_truth_index",
            -2,
            id="barometric-altitude-less-than-pending",
        ),
        pytest.param(
            "barometric_altitude_delivered_at_truth_index",
            0,
            id="barometric-altitude-initial-row",
        ),
        pytest.param(
            "barometric_altitude_delivered_at_truth_index",
            3,
            id="barometric-altitude-after-final-row",
        ),
    ],
)
def test_run_artifact_data_requires_delivered_truth_indices_in_range(
    field_name: str, invalid_index: int
) -> None:
    sources = _writable_source_arrays()
    assert sources["truth_time_s"].shape == (3,)
    source = sources[field_name]
    changed_index = source.copy()
    changed_index[0] = invalid_index
    assert type(changed_index) is np.ndarray
    assert changed_index.dtype == _INTEGER_DTYPE
    assert changed_index.ndim == source.ndim
    assert changed_index.shape == source.shape
    np.testing.assert_array_equal(changed_index[1:], source[1:])
    if field_name == "local_position_delivered_at_truth_index":
        assert source[1] == -1
        assert changed_index[1] == -1
    sources[field_name] = changed_index

    _assert_artifact_data_error(
        sources,
        f"{field_name} entries must be -1 or truth indices from 1 through 2",
    )


@pytest.mark.parametrize(
    ("field_name", "nonfinite_kind", "nonfinite_value"),
    [
        pytest.param(field_name, kind, value, id=f"{field_name}-{kind}")
        for field_name in (
            "accelerometer_acquisition_time_s",
            "accelerometer_delivery_time_s",
            "gyroscope_acquisition_time_s",
            "gyroscope_delivery_time_s",
            "local_position_acquisition_time_s",
            "local_position_delivery_time_s",
            "barometric_altitude_acquisition_time_s",
            "barometric_altitude_delivery_time_s",
        )
        for kind, value in (
            ("nan", float("nan")),
            ("positive_infinity", float("inf")),
            ("negative_infinity", float("-inf")),
        )
    ],
)
def test_run_artifact_data_requires_finite_sensor_timestamps(
    field_name: str, nonfinite_kind: str, nonfinite_value: float
) -> None:
    sources = _writable_source_arrays()
    source = sources[field_name]
    assert np.isfinite(source[0])
    changed_timestamp = source.copy()
    changed_timestamp[0] = nonfinite_value
    assert type(changed_timestamp) is np.ndarray
    assert changed_timestamp.dtype == _FLOAT_DTYPE
    assert changed_timestamp.ndim == source.ndim
    assert changed_timestamp.shape == source.shape
    np.testing.assert_array_equal(changed_timestamp[1:], source[1:])
    if nonfinite_kind == "nan":
        assert np.isnan(changed_timestamp[0])
    elif nonfinite_kind == "positive_infinity":
        assert np.isposinf(changed_timestamp[0])
    else:
        assert nonfinite_kind == "negative_infinity"
        assert np.isneginf(changed_timestamp[0])
    sources[field_name] = changed_timestamp

    _assert_artifact_data_error(sources, f"{field_name} must contain only finite values")


@pytest.mark.parametrize(
    "invalid_sensor_id",
    [
        pytest.param(-1, id="below-domain"),
        pytest.param(0, id="zero"),
        pytest.param(5, id="above-domain"),
    ],
)
def test_run_artifact_data_requires_stable_delivery_sensor_ids(invalid_sensor_id: int) -> None:
    sources = _writable_source_arrays()
    source = sources["delivery_sensor_id"]
    assert source[0] == 1
    changed_sensor_ids = source.copy()
    changed_sensor_ids[0] = invalid_sensor_id
    assert type(changed_sensor_ids) is np.ndarray
    assert changed_sensor_ids.dtype == _INTEGER_DTYPE
    assert changed_sensor_ids.ndim == source.ndim == 1
    assert changed_sensor_ids.shape == source.shape
    np.testing.assert_array_equal(changed_sensor_ids[1:], source[1:])
    sources["delivery_sensor_id"] = changed_sensor_ids

    _assert_artifact_data_error(
        sources,
        "delivery_sensor_id entries must be one of: 1, 2, 3, 4",
    )


def _assert_global_delivery_table_mutation(
    sources: dict[str, np.ndarray],
    original_sources: dict[str, np.ndarray],
    expected_records: list[tuple[int, int, int]],
) -> None:
    for name in _ARRAY_FIELDS:
        if name not in _DELIVERY_TABLE_FIELDS:
            assert sources[name] is original_sources[name]
    for name in _DELIVERY_TABLE_FIELDS:
        value = sources[name]
        assert type(value) is np.ndarray
        assert value.dtype == _INTEGER_DTYPE
        assert value.ndim == 1
        assert value.shape == (len(expected_records),)
    actual_records = list(
        zip(
            sources["delivery_sensor_id"].tolist(),
            sources["delivery_sequence_index"].tolist(),
            sources["delivery_observation_index"].tolist(),
            strict=True,
        )
    )
    assert actual_records == expected_records


@pytest.mark.parametrize(
    "mutation",
    [
        pytest.param("sequence_out_of_range", id="sequence_out_of_range"),
        pytest.param("observation_out_of_range", id="observation_out_of_range"),
        pytest.param("mismatched_pair", id="mismatched_pair"),
        pytest.param("pending_observation", id="pending_observation"),
        pytest.param("missing_entry", id="missing_entry"),
        pytest.param("duplicate_entry", id="duplicate_entry"),
    ],
)
def test_run_artifact_data_requires_exact_delivery_table_membership(mutation: str) -> None:
    sources = _writable_source_arrays()
    original_sources = sources.copy()
    valid_records = [(1, 0, 0), (1, 1, 1), (2, 0, 0), (3, 0, 0), (4, 0, 0)]
    _assert_global_delivery_table_mutation(sources, original_sources, valid_records)
    expected_records = valid_records.copy()

    if mutation == "sequence_out_of_range":
        changed_sequences = sources["delivery_sequence_index"].copy()
        assert changed_sequences[0] == 0
        changed_sequences[0] = 2
        sources["delivery_sequence_index"] = changed_sequences
        expected_records[0] = (1, 2, 0)
    elif mutation == "observation_out_of_range":
        changed_observations = sources["delivery_observation_index"].copy()
        assert changed_observations[0] == 0
        changed_observations[0] = 2
        sources["delivery_observation_index"] = changed_observations
        expected_records[0] = (1, 0, 2)
    elif mutation == "mismatched_pair":
        assert sources["accelerometer_sequence_index"].tolist() == [0, 1]
        assert sources["accelerometer_observation_index"].tolist() == [0, 1]
        changed_observations = sources["delivery_observation_index"].copy()
        changed_observations[0] = 1
        sources["delivery_observation_index"] = changed_observations
        expected_records[0] = (1, 0, 1)
    elif mutation == "pending_observation":
        assert sources["local_position_delivered_at_truth_index"].tolist() == [2, -1]
        matching_rows = [row for row, record in enumerate(valid_records) if record == (3, 0, 0)]
        assert matching_rows == [3]
        row = matching_rows[0]
        for name in ("delivery_sequence_index", "delivery_observation_index"):
            changed_indices = sources[name].copy()
            changed_indices[row] = 1
            sources[name] = changed_indices
        expected_records[row] = (3, 1, 1)
    elif mutation == "missing_entry":
        for name in _DELIVERY_TABLE_FIELDS:
            sources[name] = sources[name][:-1].copy()
        expected_records.pop()
    else:
        assert mutation == "duplicate_entry"
        for name in _DELIVERY_TABLE_FIELDS:
            source = sources[name]
            sources[name] = np.concatenate((source, source[:1]))
        expected_records.append(valid_records[0])

    _assert_global_delivery_table_mutation(sources, original_sources, expected_records)
    _assert_artifact_data_error(
        sources,
        "delivery table must contain each delivered stream observation exactly once",
    )


@pytest.mark.parametrize(
    ("first_row", "second_row"),
    [
        pytest.param(0, 1, id="truth_index_order"),
        pytest.param(2, 3, id="sensor_id_tie_order"),
    ],
)
def test_run_artifact_data_requires_canonical_delivery_table_order(
    first_row: int, second_row: int
) -> None:
    sources = _writable_source_arrays()
    original_sources = sources.copy()
    valid_records = [(1, 0, 0), (1, 1, 1), (2, 0, 0), (3, 0, 0), (4, 0, 0)]
    _assert_global_delivery_table_mutation(sources, original_sources, valid_records)
    row_order = list(range(_D))
    row_order[first_row], row_order[second_row] = row_order[second_row], row_order[first_row]
    for name in _DELIVERY_TABLE_FIELDS:
        sources[name] = sources[name][row_order]
    expected_records = [valid_records[row] for row in row_order]
    _assert_global_delivery_table_mutation(sources, original_sources, expected_records)
    assert sorted(expected_records) == sorted(valid_records)
    if first_row == 0:
        assert sources["accelerometer_delivered_at_truth_index"].tolist() == [1, 2]
        assert expected_records[:2] == [(1, 1, 1), (1, 0, 0)]
    else:
        assert sources["gyroscope_delivered_at_truth_index"].tolist() == [2]
        assert sources["local_position_delivered_at_truth_index"].tolist() == [2, -1]
        assert expected_records[2:4] == [(3, 0, 0), (2, 0, 0)]

    _assert_artifact_data_error(
        sources,
        "delivery table must be ordered by delivered truth index, sensor ID, and sequence index",
    )


@pytest.mark.parametrize("field_name", _DELIVERY_TABLE_FIELDS)
def test_run_artifact_data_requires_equal_delivery_table_lengths(field_name: str) -> None:
    sources = _writable_source_arrays()
    _remove_final_leading_entry(sources, field_name)

    _assert_artifact_data_error(
        sources,
        "delivery table fields must have equal leading dimensions",
    )


def test_encode_data_npz_preserves_exact_member_schema_and_values() -> None:
    data = _artifact_data(_writable_source_arrays())

    payload = run_artifact._encode_data_npz(data)

    assert type(payload) is bytes
    assert payload
    with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
        assert archive.files == list(_ARRAY_FIELDS)
        assert len(archive.files) == 35
        assert set(archive.files) == set(_ARRAY_FIELDS)
        for name in _ARRAY_FIELDS:
            loaded = archive[name]
            stored = getattr(data, name)
            assert type(loaded) is np.ndarray
            assert loaded.dtype == stored.dtype
            assert loaded.ndim == stored.ndim
            assert loaded.shape == stored.shape
            assert not loaded.dtype.hasobject
            np.testing.assert_array_equal(loaded, stored)

        assert archive["barometric_altitude_measurements"].shape == (1,)
        assert archive["commanded_rotor_omega"].shape == (2, 4)
        assert archive["local_position_delivered_at_truth_index"][1] == -1
        np.testing.assert_array_equal(
            archive["delivery_sensor_id"],
            np.array([1, 1, 2, 3, 4], dtype=_INTEGER_DTYPE),
        )


def test_encode_data_npz_is_byte_deterministic() -> None:
    data = _artifact_data(_writable_source_arrays())

    first_payload = run_artifact._encode_data_npz(data)
    second_payload = run_artifact._encode_data_npz(data)

    assert first_payload == second_payload
    first_digest = hashlib.sha256(first_payload).hexdigest()
    second_digest = hashlib.sha256(second_payload).hexdigest()
    assert first_digest == second_digest
    for digest in (first_digest, second_digest):
        assert len(digest) == 64
        assert all(character in "0123456789abcdef" for character in digest)


def test_save_run_directory_publishes_exact_payload_and_digest(tmp_path: Path) -> None:
    run_directory = tmp_path / "run-0001"
    data = _artifact_data(_writable_source_arrays())
    manifest = _unbound_save_manifest()
    input_manifest_bytes = encode_run_manifest(manifest)
    input_manifest_mapping = json.loads(input_manifest_bytes)

    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    assert manifest.data_npz_sha256 is None
    assert input_manifest_mapping["schema"]["version"] == 1
    assert "data_artifact" not in input_manifest_mapping

    saved_manifest = run_artifact.save_run_directory(run_directory, manifest, data)

    assert run_directory.is_dir()
    assert {entry.name for entry in run_directory.iterdir()} == {"data.npz", "manifest.json"}
    assert {entry.name for entry in tmp_path.iterdir()} == {"run-0001"}
    assert set(tmp_path.iterdir()) == {run_directory}

    assert saved_manifest is not manifest
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == input_manifest_bytes
    assert json.loads(encode_run_manifest(manifest))["schema"]["version"] == 1

    data_npz_bytes = (run_directory / "data.npz").read_bytes()
    assert data_npz_bytes
    assert data_npz_bytes == run_artifact._encode_data_npz(data)
    with np.load(io.BytesIO(data_npz_bytes), allow_pickle=False) as archive:
        assert archive.files == list(_ARRAY_FIELDS)
        assert len(archive.files) == 35

    expected_sha256 = hashlib.sha256(data_npz_bytes).hexdigest()
    assert len(expected_sha256) == 64
    assert all(character in "0123456789abcdef" for character in expected_sha256)
    assert saved_manifest.data_npz_sha256 == expected_sha256

    manifest_bytes = (run_directory / "manifest.json").read_bytes()
    assert manifest_bytes == encode_run_manifest(saved_manifest)
    saved_mapping = json.loads(manifest_bytes)
    assert set(saved_mapping) == {
        "data_artifact",
        "randomness",
        "run_configuration",
        "schema",
        "software_provenance",
    }
    assert saved_mapping["schema"]["version"] == 2
    assert saved_mapping["data_artifact"] == {"sha256": expected_sha256}

    decoded_manifest = decode_run_manifest(manifest_bytes)
    assert decoded_manifest.data_npz_sha256 == expected_sha256
    assert encode_run_manifest(decoded_manifest) == manifest_bytes


@pytest.mark.parametrize(
    "destination_kind",
    [
        pytest.param("empty_directory", id="empty_directory"),
        pytest.param("populated_directory", id="populated_directory"),
        pytest.param("regular_file", id="regular_file"),
    ],
)
def test_save_run_directory_preserves_existing_destination(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    destination_kind: str,
) -> None:
    run_directory = tmp_path / "run-0001"
    manifest = _unbound_save_manifest()
    data = _artifact_data(_writable_source_arrays())

    if destination_kind == "empty_directory":
        run_directory.mkdir()
    elif destination_kind == "populated_directory":
        run_directory.mkdir()
        (run_directory / "sentinel.bin").write_bytes(b"preserve-existing-directory")
    else:
        run_directory.write_bytes(b"preserve-existing-file")

    originally_directory = run_directory.is_dir()
    original_names = (
        {entry.name for entry in run_directory.iterdir()} if originally_directory else None
    )
    original_bytes = (
        (run_directory / "sentinel.bin").read_bytes()
        if destination_kind == "populated_directory"
        else run_directory.read_bytes()
        if destination_kind == "regular_file"
        else None
    )
    encode_mock = Mock(return_value=b"must-not-encode")
    monkeypatch.setattr(run_artifact, "_encode_data_npz", encode_mock)

    with pytest.raises(FileExistsError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert str(error.value) == f"run directory already exists: {run_directory}"
    encode_mock.assert_not_called()
    assert manifest.data_npz_sha256 is None
    assert run_directory.is_dir() is originally_directory
    if originally_directory:
        assert {entry.name for entry in run_directory.iterdir()} == original_names
        if original_bytes is not None:
            assert (run_directory / "sentinel.bin").read_bytes() == original_bytes
    else:
        assert run_directory.is_file()
        assert run_directory.read_bytes() == original_bytes
    assert {entry.name for entry in tmp_path.iterdir()} == {"run-0001"}
    assert set(tmp_path.iterdir()) == {run_directory}


@pytest.mark.parametrize(
    "failure_point",
    [
        pytest.param("manifest_write", id="manifest_write"),
        pytest.param("publication_rename", id="publication_rename"),
    ],
)
def test_save_run_directory_cleans_staging_after_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_point: str,
) -> None:
    run_directory = tmp_path / "run-0001"
    manifest = _unbound_save_manifest()
    data = _artifact_data(_writable_source_arrays())
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())

    if failure_point == "manifest_write":
        original_write_bytes = Path.write_bytes

        def fail_manifest_write(path: Path, payload: bytes) -> int:
            if (
                path.name == "manifest.json"
                and path.parent.parent == tmp_path
                and path.parent.name.startswith(".run-0001.staging-")
            ):
                raise OSError("injected manifest write failure")
            return original_write_bytes(path, payload)

        monkeypatch.setattr(Path, "write_bytes", fail_manifest_write)
        expected_message = "injected manifest write failure"
    else:

        def fail_publication_rename(staging_directory: Path, destination: Path) -> None:
            assert staging_directory.parent == tmp_path
            assert staging_directory.name.startswith(".run-0001.staging-")
            assert destination == run_directory
            raise OSError("injected publication failure")

        monkeypatch.setattr(
            run_artifact,
            "_publish_staging_directory",
            fail_publication_rename,
            raising=False,
        )
        expected_message = "injected publication failure"

    with pytest.raises(OSError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert str(error.value) == expected_message
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    assert manifest.data_npz_sha256 is None


def test_save_run_directory_does_not_replace_destination_created_during_save(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    run_directory = tmp_path / "run-0001"
    manifest = _unbound_save_manifest()
    data = _artifact_data(_writable_source_arrays())
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())

    real_encode_data_npz = run_artifact._encode_data_npz
    encoded_payloads: list[bytes] = []

    def create_destination_during_encode(artifact: RunArtifactData) -> bytes:
        payload = real_encode_data_npz(artifact)
        encoded_payloads.append(payload)
        run_directory.mkdir()
        return payload

    monkeypatch.setattr(run_artifact, "_encode_data_npz", create_destination_during_encode)
    real_path_exists = Path.exists

    def miss_destination_in_later_check(path: Path) -> bool:
        if path == run_directory and run_directory.is_dir():
            return False
        return real_path_exists(path)

    monkeypatch.setattr(Path, "exists", miss_destination_in_later_check)

    with pytest.raises(FileExistsError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert str(error.value) == f"run directory already exists: {run_directory}"
    assert len(encoded_payloads) == 1
    assert encoded_payloads[0]
    assert real_path_exists(run_directory)
    assert run_directory.is_dir()
    assert not list(run_directory.iterdir())
    assert not (run_directory / "manifest.json").exists()
    assert not (run_directory / "data.npz").exists()
    assert {entry.name for entry in tmp_path.iterdir()} == {"run-0001"}
    assert set(tmp_path.iterdir()) == {run_directory}
    assert manifest.data_npz_sha256 is None


def test_decode_data_npz_round_trips_with_pickle_disabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    data = _artifact_data(_writable_source_arrays())
    payload = run_artifact._encode_data_npz(data)
    assert type(payload) is bytes
    assert payload

    real_np_load = run_artifact.np.load
    load_calls: list[tuple[tuple[object, ...], dict[str, object]]] = []

    def observe_np_load(*args: object, **kwargs: object) -> object:
        load_calls.append((args, dict(kwargs)))
        return real_np_load(*args, **kwargs)

    monkeypatch.setattr(run_artifact.np, "load", observe_np_load)

    decoded = run_artifact._decode_data_npz(payload)

    assert len(load_calls) == 1
    positional, keyword = load_calls[0]
    assert len(positional) == 1
    assert isinstance(positional[0], io.BytesIO)
    assert positional[0].getvalue() == payload
    assert keyword.get("allow_pickle") is False

    assert type(decoded) is RunArtifactData
    assert decoded is not data
    assert tuple(field.name for field in fields(decoded)) == _ARRAY_FIELDS
    for name in _ARRAY_FIELDS:
        decoded_array = getattr(decoded, name)
        original_array = getattr(data, name)
        assert type(decoded_array) is np.ndarray
        assert decoded_array.dtype == original_array.dtype
        assert decoded_array.ndim == original_array.ndim
        assert decoded_array.shape == original_array.shape
        assert not decoded_array.dtype.hasobject
        np.testing.assert_array_equal(decoded_array, original_array)
        assert decoded_array.flags.owndata
        assert decoded_array.flags.c_contiguous
        assert not decoded_array.flags.writeable
        assert not np.shares_memory(decoded_array, original_array)

    assert decoded.barometric_altitude_measurements.shape == (1,)
    assert decoded.commanded_rotor_omega.shape == (2, 4)
    assert decoded.local_position_delivered_at_truth_index[-1] == -1
    np.testing.assert_array_equal(
        decoded.delivery_sensor_id,
        np.array([1, 1, 2, 3, 4], dtype=_INTEGER_DTYPE),
    )
    assert run_artifact._encode_data_npz(decoded) == payload


def test_load_run_directory_round_trips_saved_manifest_and_data(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    run_directory = tmp_path / "run-0001"
    manifest = _unbound_save_manifest()
    data = _artifact_data(_writable_source_arrays())
    saved_manifest = run_artifact.save_run_directory(run_directory, manifest, data)
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    manifest_bytes = manifest_path.read_bytes()
    data_bytes = data_path.read_bytes()
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}

    real_read_bytes = Path.read_bytes
    read_calls: list[Path] = []

    def observe_read_bytes(path: Path) -> bytes:
        read_calls.append(path)
        return real_read_bytes(path)

    monkeypatch.setattr(Path, "read_bytes", observe_read_bytes)

    loaded_manifest, loaded_data = run_artifact.load_run_directory(run_directory)

    assert read_calls == [manifest_path, data_path]
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}
    assert manifest_path.read_bytes() == manifest_bytes
    assert data_path.read_bytes() == data_bytes

    assert type(loaded_manifest) is RunManifest
    assert loaded_manifest is not saved_manifest
    assert loaded_manifest.data_npz_sha256 == saved_manifest.data_npz_sha256
    assert loaded_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert encode_run_manifest(loaded_manifest) == manifest_bytes
    assert json.loads(encode_run_manifest(loaded_manifest))["schema"]["version"] == 2
    assert encode_run_manifest(decode_run_manifest(manifest_bytes)) == manifest_bytes

    assert type(loaded_data) is RunArtifactData
    assert loaded_data is not data
    assert len(_ARRAY_FIELDS) == 35
    for name in _ARRAY_FIELDS:
        loaded_array = getattr(loaded_data, name)
        original_array = getattr(data, name)
        assert type(loaded_array) is np.ndarray
        assert loaded_array.dtype == original_array.dtype
        assert loaded_array.ndim == original_array.ndim
        assert loaded_array.shape == original_array.shape
        np.testing.assert_array_equal(loaded_array, original_array)
        assert loaded_array.flags.owndata
        assert loaded_array.flags.c_contiguous
        assert not loaded_array.flags.writeable
        assert not np.shares_memory(loaded_array, original_array)

    assert loaded_data.barometric_altitude_measurements.shape == (1,)
    assert loaded_data.commanded_rotor_omega.shape == (2, 4)
    assert loaded_data.local_position_delivered_at_truth_index[-1] == -1
    np.testing.assert_array_equal(
        loaded_data.delivery_sensor_id,
        np.array([1, 1, 2, 3, 4], dtype=_INTEGER_DTYPE),
    )
    assert run_artifact._encode_data_npz(loaded_data) == data_bytes


def test_save_and_load_motorized_run_directory_round_trip(tmp_path: Path) -> None:
    historical_manifest = _unbound_save_manifest()
    historical_configuration = historical_manifest.run_configuration
    motor_parameters = {
        "minimum_rotor_omega": 100.0,
        "maximum_rotor_omega": 1000.0,
        "motor_time_constant_s": 0.2,
    }
    caller_initial_actual_rotor_omega = np.array([100.0, 300.0, 700.0, 1000.0], dtype=np.float64)
    configuration = replace(
        historical_configuration,
        truth=replace(
            historical_configuration.truth,
            rotors=replace(historical_configuration.truth.rotors, **motor_parameters),
        ),
        nominal=replace(
            historical_configuration.nominal,
            rotors=replace(historical_configuration.nominal.rotors, **motor_parameters),
        ),
        initial_actual_rotor_omega=caller_initial_actual_rotor_omega,
    )
    input_manifest = replace(historical_manifest, run_configuration=configuration)
    input_manifest_bytes = encode_run_manifest(input_manifest)
    input_mapping = json.loads(input_manifest_bytes)
    assert input_manifest.data_npz_sha256 is None
    assert input_mapping["schema"]["version"] == 3

    source_arrays = _writable_source_arrays()
    artifact = _artifact_data(source_arrays)
    artifact_arrays = _artifact_npz_members(artifact)
    artifact_snapshots = {name: array.copy() for name, array in artifact_arrays.items()}
    source_snapshots = {name: array.copy() for name, array in source_arrays.items()}
    artifact_flags = {
        name: (array.flags.owndata, array.flags.c_contiguous, array.flags.writeable)
        for name, array in artifact_arrays.items()
    }
    source_flags = {
        name: (array.flags.owndata, array.flags.c_contiguous, array.flags.writeable)
        for name, array in source_arrays.items()
    }
    initial_vector_flags = (
        caller_initial_actual_rotor_omega.flags.owndata,
        caller_initial_actual_rotor_omega.flags.c_contiguous,
        caller_initial_actual_rotor_omega.flags.writeable,
    )
    configuration_vector_flags = (
        configuration.initial_actual_rotor_omega.flags.owndata,
        configuration.initial_actual_rotor_omega.flags.c_contiguous,
        configuration.initial_actual_rotor_omega.flags.writeable,
    )
    run_directory = tmp_path / "motorized-run"
    assert not run_directory.exists()

    saved_manifest = run_artifact.save_run_directory(run_directory, input_manifest, artifact)

    assert type(saved_manifest) is RunManifest
    assert saved_manifest is not input_manifest
    assert saved_manifest.data_npz_sha256 is not None
    assert input_manifest.data_npz_sha256 is None
    assert encode_run_manifest(input_manifest) == input_manifest_bytes
    assert set(run_directory.iterdir()) == {
        run_directory / "data.npz",
        run_directory / "manifest.json",
    }
    assert set(tmp_path.iterdir()) == {run_directory}
    data_bytes = (run_directory / "data.npz").read_bytes()
    manifest_bytes = (run_directory / "manifest.json").read_bytes()
    data_digest = hashlib.sha256(data_bytes).hexdigest()
    saved_mapping = json.loads(manifest_bytes)
    assert saved_manifest.data_npz_sha256 == data_digest
    assert saved_mapping["schema"]["version"] == 4
    assert set(saved_mapping) == {
        "data_artifact",
        "randomness",
        "run_configuration",
        "schema",
        "software_provenance",
    }
    assert saved_mapping["data_artifact"] == {"sha256": data_digest}
    assert saved_mapping["run_configuration"] == input_mapping["run_configuration"]
    assert saved_mapping["randomness"] == input_mapping["randomness"]
    assert saved_mapping["software_provenance"] == input_mapping["software_provenance"]
    assert saved_mapping["run_configuration"]["initial_actual_rotor_omega"] == [
        100.0,
        300.0,
        700.0,
        1000.0,
    ]
    for group_name in ("truth", "nominal"):
        encoded_rotors = saved_mapping["run_configuration"][group_name]["rotors"]
        for parameter_name, expected_value in motor_parameters.items():
            assert encoded_rotors[parameter_name] == expected_value
    assert manifest_bytes == encode_run_manifest(saved_manifest)

    for rotors in (configuration.truth.rotors, configuration.nominal.rotors):
        assert (
            rotors.minimum_rotor_omega,
            rotors.maximum_rotor_omega,
            rotors.motor_time_constant_s,
        ) == (100.0, 1000.0, 0.2)
    np.testing.assert_array_equal(
        configuration.initial_actual_rotor_omega,
        np.array([100.0, 300.0, 700.0, 1000.0], dtype=np.float64),
    )
    np.testing.assert_array_equal(
        caller_initial_actual_rotor_omega,
        np.array([100.0, 300.0, 700.0, 1000.0], dtype=np.float64),
    )
    assert (
        caller_initial_actual_rotor_omega.flags.owndata,
        caller_initial_actual_rotor_omega.flags.c_contiguous,
        caller_initial_actual_rotor_omega.flags.writeable,
    ) == initial_vector_flags
    assert (
        configuration.initial_actual_rotor_omega.flags.owndata,
        configuration.initial_actual_rotor_omega.flags.c_contiguous,
        configuration.initial_actual_rotor_omega.flags.writeable,
    ) == configuration_vector_flags
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(artifact_arrays[name], artifact_snapshots[name])
        np.testing.assert_array_equal(source_arrays[name], source_snapshots[name])
        assert (
            artifact_arrays[name].flags.owndata,
            artifact_arrays[name].flags.c_contiguous,
            artifact_arrays[name].flags.writeable,
        ) == artifact_flags[name]
        assert (
            source_arrays[name].flags.owndata,
            source_arrays[name].flags.c_contiguous,
            source_arrays[name].flags.writeable,
        ) == source_flags[name]

    caller_initial_actual_rotor_omega[...] = 200.0
    loaded_manifest, loaded_artifact = run_artifact.load_run_directory(run_directory)

    assert type(loaded_manifest) is RunManifest
    assert json.loads(encode_run_manifest(loaded_manifest))["schema"]["version"] == 4
    assert loaded_manifest.data_npz_sha256 == data_digest
    loaded_configuration = loaded_manifest.run_configuration
    for rotors in (loaded_configuration.truth.rotors, loaded_configuration.nominal.rotors):
        assert (
            rotors.minimum_rotor_omega,
            rotors.maximum_rotor_omega,
            rotors.motor_time_constant_s,
        ) == (100.0, 1000.0, 0.2)
    loaded_initial_actual_rotor_omega = loaded_configuration.initial_actual_rotor_omega
    np.testing.assert_array_equal(
        loaded_initial_actual_rotor_omega,
        np.array([100.0, 300.0, 700.0, 1000.0], dtype=np.float64),
    )
    assert loaded_initial_actual_rotor_omega.dtype == np.float64
    assert loaded_initial_actual_rotor_omega.shape == (4,)
    assert loaded_initial_actual_rotor_omega.flags.owndata
    assert loaded_initial_actual_rotor_omega.flags.c_contiguous
    assert not loaded_initial_actual_rotor_omega.flags.writeable
    assert not np.shares_memory(
        loaded_initial_actual_rotor_omega,
        configuration.initial_actual_rotor_omega,
    )
    assert not np.shares_memory(
        loaded_initial_actual_rotor_omega,
        caller_initial_actual_rotor_omega,
    )
    with pytest.raises(ValueError, match="assignment destination is read-only"):
        loaded_initial_actual_rotor_omega[0] = 200.0
    assert tuple(
        (mismatch.parameter_path, mismatch.rationale)
        for mismatch in loaded_configuration.declared_mismatches
    ) == tuple(
        (mismatch.parameter_path, mismatch.rationale)
        for mismatch in configuration.declared_mismatches
    )
    assert loaded_manifest.software_provenance == input_manifest.software_provenance
    assert encode_run_manifest(loaded_manifest) == manifest_bytes

    assert type(loaded_artifact) is RunArtifactData
    assert tuple(field.name for field in fields(loaded_artifact)) == _ARRAY_FIELDS
    for name in _ARRAY_FIELDS:
        loaded_array = getattr(loaded_artifact, name)
        original_array = artifact_arrays[name]
        np.testing.assert_array_equal(loaded_array, original_array)
        assert loaded_array.dtype == original_array.dtype
        assert loaded_array.shape == original_array.shape
        assert loaded_array.flags.owndata
        assert loaded_array.flags.c_contiguous
        assert not loaded_array.flags.writeable
        assert not np.shares_memory(loaded_array, original_array)
    assert run_artifact._encode_data_npz(loaded_artifact) == data_bytes
    assert input_manifest.data_npz_sha256 is None
    assert encode_run_manifest(input_manifest) == input_manifest_bytes


def _environmental_save_manifest(*, motorized: bool) -> RunManifest:
    """Extend the valid save fixture with declared nonzero environmental models."""
    manifest = _unbound_save_manifest()
    configuration = manifest.run_configuration
    motor_parameters = (
        {
            "minimum_rotor_omega": 100.0,
            "maximum_rotor_omega": 1000.0,
            "motor_time_constant_s": 0.2,
        }
        if motorized
        else {}
    )
    truth = replace(
        configuration.truth,
        rigid_body=replace(
            configuration.truth.rigid_body,
            quadratic_drag_coefficient_B=np.array([0.3, 0.2, 0.1]),
        ),
        rotors=replace(configuration.truth.rotors, **motor_parameters),
        world=replace(
            configuration.truth.world,
            wind_velocity_W=np.array([1.0, -0.5, 0.25]),
        ),
    )
    nominal = replace(
        configuration.nominal,
        rigid_body=replace(
            configuration.nominal.rigid_body,
            quadratic_drag_coefficient_B=np.array([0.25, 0.15, 0.05]),
        ),
        rotors=replace(configuration.nominal.rotors, **motor_parameters),
        world=replace(
            configuration.nominal.world,
            wind_velocity_W=np.array([0.8, -0.4, 0.2]),
        ),
    )
    environmental_configuration = replace(
        configuration,
        truth=truth,
        nominal=nominal,
        initial_actual_rotor_omega=(np.array([100.0, 300.0, 700.0, 1000.0]) if motorized else None),
        declared_mismatches=(
            *configuration.declared_mismatches,
            DeclaredMismatch(
                "rigid_body.quadratic_drag_coefficient_B",
                "Environmental persistence drag mismatch.",
            ),
            DeclaredMismatch(
                "world.wind_velocity_W",
                "Environmental persistence wind mismatch.",
            ),
        ),
    )
    return replace(manifest, run_configuration=environmental_configuration)


@pytest.mark.parametrize("motorized", [False, True], ids=["historical", "motorized"])
def test_environmental_save_load_preserves_authenticated_artifact_schema(
    tmp_path: Path, motorized: bool
) -> None:
    manifest = _environmental_save_manifest(motorized=motorized)
    unbound_manifest_bytes = encode_run_manifest(manifest)
    assert json.loads(unbound_manifest_bytes)["schema"]["version"] == 5
    sources = _writable_source_arrays()
    source_snapshots = {name: array.copy() for name, array in sources.items()}
    data = _artifact_data(sources)
    data_snapshots = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}

    run_directory = tmp_path / "environmental-run"
    bound_manifest = run_artifact.save_run_directory(run_directory, manifest, data)
    manifest_bytes = (run_directory / "manifest.json").read_bytes()
    data_bytes = (run_directory / "data.npz").read_bytes()
    assert bound_manifest is not manifest
    assert json.loads(manifest_bytes)["schema"]["version"] == 6
    assert set(run_directory.iterdir()) == {
        run_directory / "manifest.json",
        run_directory / "data.npz",
    }
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()

    loaded_manifest, loaded_data = run_artifact.load_run_directory(run_directory)
    assert encode_run_manifest(loaded_manifest) == manifest_bytes
    loaded_configuration = loaded_manifest.run_configuration
    for loaded_model, source_model in (
        (loaded_configuration.truth, manifest.run_configuration.truth),
        (loaded_configuration.nominal, manifest.run_configuration.nominal),
    ):
        for loaded_array, source_array in (
            (
                loaded_model.rigid_body.quadratic_drag_coefficient_B,
                source_model.rigid_body.quadratic_drag_coefficient_B,
            ),
            (loaded_model.world.wind_velocity_W, source_model.world.wind_velocity_W),
        ):
            np.testing.assert_array_equal(loaded_array, source_array)
            assert loaded_array.flags.owndata
            assert loaded_array.flags.c_contiguous
            assert not loaded_array.flags.writeable
            assert not np.shares_memory(loaded_array, source_array)

    assert tuple(field.name for field in fields(RunArtifactData)) == _ARRAY_FIELDS
    assert len(_ARRAY_FIELDS) == 35
    assert all("actual_rotor" not in name for name in _ARRAY_FIELDS)
    with np.load(io.BytesIO(data_bytes), allow_pickle=False) as archive:
        assert archive.files == list(_ARRAY_FIELDS)
        assert all("actual_rotor" not in name for name in archive.files)
    for name in _ARRAY_FIELDS:
        loaded_array = getattr(loaded_data, name)
        original_array = getattr(data, name)
        np.testing.assert_array_equal(loaded_array, original_array)
        assert loaded_array.flags.owndata
        assert loaded_array.flags.c_contiguous
        assert not loaded_array.flags.writeable
        assert not np.shares_memory(loaded_array, original_array)

    replay_directory = tmp_path / "environmental-replay"
    replay_manifest = run_artifact.save_run_directory(
        replay_directory, loaded_manifest, loaded_data
    )
    assert encode_run_manifest(replay_manifest) == manifest_bytes
    assert (replay_directory / "manifest.json").read_bytes() == manifest_bytes
    assert (replay_directory / "data.npz").read_bytes() == data_bytes
    assert encode_run_manifest(manifest) == unbound_manifest_bytes
    assert manifest.data_npz_sha256 is None
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(sources[name], source_snapshots[name])
        np.testing.assert_array_equal(getattr(data, name), data_snapshots[name])


def test_load_run_directory_rejects_data_digest_mismatch_before_npz_decode(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    run_directory = tmp_path / "run-0001"
    manifest = _unbound_save_manifest()
    input_manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_data(_writable_source_arrays())
    saved_manifest = run_artifact.save_run_directory(run_directory, manifest, data)
    saved_manifest_bytes = encode_run_manifest(saved_manifest)
    saved_digest = saved_manifest.data_npz_sha256
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    manifest_bytes = manifest_path.read_bytes()
    assert manifest_bytes == saved_manifest_bytes

    tampered_bytes = b"tampered-data-npz"
    data_path.write_bytes(tampered_bytes)
    decode_mock = Mock(side_effect=AssertionError("NPZ decoding must not start"))
    monkeypatch.setattr(run_artifact, "_decode_data_npz", decode_mock)

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == "data.npz SHA-256 does not match manifest"
    decode_mock.assert_not_called()
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}
    assert manifest_path.read_bytes() == manifest_bytes
    assert data_path.read_bytes() == tampered_bytes
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == input_manifest_bytes
    assert saved_manifest.data_npz_sha256 == saved_digest
    assert encode_run_manifest(saved_manifest) == saved_manifest_bytes


@pytest.mark.parametrize(
    "directory_kind",
    [
        pytest.param("missing_path", id="missing_path"),
        pytest.param("regular_file", id="regular_file"),
    ],
)
def test_load_run_directory_requires_directory(tmp_path: Path, directory_kind: str) -> None:
    if directory_kind == "missing_path":
        run_directory = tmp_path / "missing-run"
        original_bytes = None
    else:
        run_directory = tmp_path / "run-0001"
        original_bytes = b"regular-file-sentinel"
        run_directory.write_bytes(original_bytes)
    original_entries = set(tmp_path.iterdir())

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == "run directory must be a directory"
    assert set(tmp_path.iterdir()) == original_entries
    if original_bytes is None:
        assert not run_directory.exists()
    else:
        assert run_directory.is_file()
        assert run_directory.read_bytes() == original_bytes


@pytest.mark.parametrize(
    "directory_change",
    [
        pytest.param("missing_manifest", id="missing_manifest"),
        pytest.param("missing_data", id="missing_data"),
        pytest.param("unexpected_entry", id="unexpected_entry"),
    ],
)
def test_load_run_directory_requires_exact_entries(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    directory_change: str,
) -> None:
    run_directory = tmp_path / "run-0001"
    manifest = _unbound_save_manifest()
    data = _artifact_data(_writable_source_arrays())
    run_artifact.save_run_directory(run_directory, manifest, data)
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    assert set(run_directory.iterdir()) == {manifest_path, data_path}

    if directory_change == "missing_manifest":
        manifest_path.unlink()
    elif directory_change == "missing_data":
        data_path.unlink()
    else:
        (run_directory / "unexpected.bin").write_bytes(b"unexpected-entry-sentinel")
    original_contents = {entry.name: entry.read_bytes() for entry in run_directory.iterdir()}
    assert set(tmp_path.iterdir()) == {run_directory}
    real_read_bytes = Path.read_bytes
    read_names: list[str] = []

    def observe_read_bytes(path: Path) -> bytes:
        read_names.append(path.name)
        return real_read_bytes(path)

    monkeypatch.setattr(Path, "read_bytes", observe_read_bytes)

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == "run directory entries must be exactly: data.npz, manifest.json"
    assert read_names == []
    assert {
        entry.name: real_read_bytes(entry) for entry in run_directory.iterdir()
    } == original_contents
    assert set(tmp_path.iterdir()) == {run_directory}


@pytest.mark.parametrize("manifest_version", [pytest.param(1, id="unbound")])
def test_load_run_directory_requires_bound_manifest_before_reading_data(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    manifest_version: int,
) -> None:
    run_directory = tmp_path / "run-0001"
    original_unbound_manifest = _unbound_save_manifest()
    unbound_manifest_bytes = encode_run_manifest(original_unbound_manifest)
    assert json.loads(unbound_manifest_bytes)["schema"]["version"] == manifest_version
    data = _artifact_data(_writable_source_arrays())
    run_artifact.save_run_directory(run_directory, original_unbound_manifest, data)
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    data_bytes = data_path.read_bytes()
    manifest_path.write_bytes(unbound_manifest_bytes)
    assert set(run_directory.iterdir()) == {manifest_path, data_path}

    decode_mock = Mock(side_effect=AssertionError("NPZ decoding must not start"))
    monkeypatch.setattr(run_artifact, "_decode_data_npz", decode_mock)
    real_read_bytes = Path.read_bytes
    read_names: list[str] = []

    def observe_read_bytes(path: Path) -> bytes:
        read_names.append(path.name)
        return real_read_bytes(path)

    monkeypatch.setattr(Path, "read_bytes", observe_read_bytes)

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == "manifest.json must bind data.npz"
    assert read_names == ["manifest.json"]
    decode_mock.assert_not_called()
    assert real_read_bytes(manifest_path) == unbound_manifest_bytes
    assert real_read_bytes(data_path) == data_bytes
    assert original_unbound_manifest.data_npz_sha256 is None
    assert encode_run_manifest(original_unbound_manifest) == unbound_manifest_bytes
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}


@pytest.mark.parametrize("missing_name", _ARRAY_FIELDS)
def test_decode_data_npz_requires_every_exact_member(missing_name: str) -> None:
    data = _artifact_data(_writable_source_arrays())
    members = _artifact_npz_members(data)
    assert tuple(members) == _ARRAY_FIELDS
    members.pop(missing_name)
    payload = _encode_npz_members(members)

    _assert_npz_member_schema_error(payload)


def test_decode_data_npz_rejects_unknown_member() -> None:
    data = _artifact_data(_writable_source_arrays())
    members = _artifact_npz_members(data)
    assert tuple(members) == _ARRAY_FIELDS
    members["unexpected"] = np.array([1], dtype="<i8")
    payload = _encode_npz_members(members)

    _assert_npz_member_schema_error(payload)


def _longer_artifact_data() -> RunArtifactData:
    """Extend only truth, bias, and command rows to an intrinsic three-step run."""
    sources = _writable_source_arrays()
    changed_names = (*_TRUTH_HISTORY_FIELDS, *_BIAS_HISTORY_FIELDS, "commanded_rotor_omega")
    unchanged_sources = {name: sources[name] for name in _ARRAY_FIELDS if name not in changed_names}
    sources["truth_time_s"] = np.concatenate(
        (sources["truth_time_s"], np.array([0.3], dtype=_FLOAT_DTYPE))
    )
    for name in (*_TRUTH_HISTORY_FIELDS[1:], *_BIAS_HISTORY_FIELDS, "commanded_rotor_omega"):
        source = sources[name]
        sources[name] = np.concatenate((source, source[-1:]), axis=0)
        assert type(sources[name]) is np.ndarray
        assert sources[name].dtype == _FLOAT_DTYPE
        assert sources[name].ndim == source.ndim
        assert sources[name].shape[0] == source.shape[0] + 1
        assert sources[name].shape[1:] == source.shape[1:]
    for name, original in unchanged_sources.items():
        assert sources[name] is original
    assert type(sources["truth_time_s"]) is np.ndarray
    assert sources["truth_time_s"].dtype == _FLOAT_DTYPE
    assert sources["truth_time_s"].shape == (4,)
    np.testing.assert_array_equal(
        sources["truth_time_s"], np.array([0.0, 0.1, 0.2, 0.3], dtype=_FLOAT_DTYPE)
    )

    data = _artifact_data(sources)
    assert data.truth_time_s.shape == (4,)
    assert data.commanded_rotor_omega.shape == (3, 4)
    for name in (*_TRUTH_HISTORY_FIELDS[1:], *_BIAS_HISTORY_FIELDS):
        assert getattr(data, name).shape[0] == 4
    for name, original in unchanged_sources.items():
        np.testing.assert_array_equal(getattr(data, name), original)
    return data


def _off_grid_artifact_data() -> RunArtifactData:
    """Move only the middle truth timestamp off the configured grid."""
    sources = _writable_source_arrays()
    source = sources["truth_time_s"]
    changed_time = source.copy()
    assert source[1] == 0.1
    changed_time[1] = 0.11
    assert type(changed_time) is np.ndarray
    assert changed_time.dtype == _FLOAT_DTYPE
    assert changed_time.ndim == source.ndim == 1
    assert changed_time.shape == source.shape
    np.testing.assert_array_equal(changed_time[[0, 2]], source[[0, 2]])
    sources["truth_time_s"] = changed_time
    data = _artifact_data(sources)
    assert data.truth_time_s[1] == 0.11
    return data


def _write_directly_bound_run_directory(
    run_directory: Path, manifest: RunManifest, data: RunArtifactData
) -> tuple[RunManifest, bytes, bytes]:
    """Write authenticated fixture bytes without passing through save validation."""
    data_bytes = run_artifact._encode_data_npz(data)
    digest = hashlib.sha256(data_bytes).hexdigest()
    bound_manifest = replace(manifest, data_npz_sha256=digest)
    assert bound_manifest is not manifest
    manifest_bytes = encode_run_manifest(bound_manifest)
    assert json.loads(manifest_bytes)["schema"]["version"] == 2
    run_directory.mkdir()
    (run_directory / "manifest.json").write_bytes(manifest_bytes)
    (run_directory / "data.npz").write_bytes(data_bytes)
    return bound_manifest, manifest_bytes, data_bytes


@pytest.mark.parametrize(
    ("mutation", "expected_message"),
    [
        pytest.param(
            "truth_row_count",
            "truth_time_s length must equal configuration.numerics.number_of_steps + 1",
            id="truth_row_count",
        ),
        pytest.param(
            "truth_time_grid",
            "truth_time_s must equal the configured truth-time grid",
            id="truth_time_grid",
        ),
        pytest.param(
            "rotor_command",
            "commanded_rotor_omega rows must equal configuration.rotor_speed_input.rotor_omega",
            id="rotor_command",
        ),
    ],
)
def test_save_run_directory_rejects_configuration_incompatible_data_before_encoding(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mutation: str,
    expected_message: str,
) -> None:
    manifest = _unbound_save_manifest()
    manifest_bytes = encode_run_manifest(manifest)
    assert manifest.run_configuration.numerics.number_of_steps == 2
    if mutation == "truth_row_count":
        data = _longer_artifact_data()
    elif mutation == "truth_time_grid":
        data = _off_grid_artifact_data()
    else:
        assert mutation == "rotor_command"
        sources = _writable_source_arrays()
        source = sources["commanded_rotor_omega"]
        changed_command = source.copy()
        assert source[0, 0] == 400.0
        changed_command[0, 0] = 401.0
        assert type(changed_command) is np.ndarray
        assert changed_command.dtype == _FLOAT_DTYPE
        assert changed_command.ndim == source.ndim == 2
        assert changed_command.shape == source.shape
        np.testing.assert_array_equal(changed_command[0, 1:], source[0, 1:])
        np.testing.assert_array_equal(changed_command[1:], source[1:])
        sources["commanded_rotor_omega"] = changed_command
        data = _artifact_data(sources)

    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    run_directory = tmp_path / "incompatible-run"
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    encode_mock = Mock(side_effect=AssertionError("NPZ encoding must not start"))
    monkeypatch.setattr(run_artifact, "_encode_data_npz", encode_mock)

    with pytest.raises(ValueError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert str(error.value) == expected_message
    encode_mock.assert_not_called()
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def test_load_run_directory_rejects_configuration_incompatible_data(tmp_path: Path) -> None:
    manifest = _unbound_save_manifest()
    unbound_manifest_bytes = encode_run_manifest(manifest)
    data = _off_grid_artifact_data()
    run_directory = tmp_path / "incompatible-run"
    bound_manifest, manifest_bytes, data_bytes = _write_directly_bound_run_directory(
        run_directory, manifest, data
    )
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}
    decoded_data = run_artifact._decode_data_npz(data_bytes)
    assert decoded_data.truth_time_s[1] == 0.11

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == "truth_time_s must equal the configured truth-time grid"
    assert manifest_path.read_bytes() == manifest_bytes
    assert data_path.read_bytes() == data_bytes
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == unbound_manifest_bytes
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert encode_run_manifest(bound_manifest) == manifest_bytes
    assert run_artifact._encode_data_npz(data) == data_bytes


def _assert_initial_values_match_manifest(
    data: RunArtifactData, manifest: RunManifest, *, mismatched_field: str | None = None
) -> None:
    """Audit the six initial boundaries, allowing one selected mismatch."""
    configuration = manifest.run_configuration
    boundaries = (
        ("truth_position_history_W", configuration.initial_truth_state.position_W),
        ("truth_velocity_history_W", configuration.initial_truth_state.velocity_W),
        ("truth_q_history_WB", configuration.initial_truth_state.q_WB),
        ("truth_omega_history_B", configuration.initial_truth_state.omega_B),
        (
            "accelerometer_bias_history_B",
            configuration.truth.imu.initial_accelerometer_bias_B,
        ),
        ("gyroscope_bias_history_B", configuration.truth.imu.initial_gyroscope_bias_B),
    )
    assert mismatched_field is None or mismatched_field in {name for name, _ in boundaries}
    for name, configured_value in boundaries:
        actual_value = getattr(data, name)[0]
        if name == mismatched_field:
            assert not np.array_equal(actual_value, configured_value)
        else:
            np.testing.assert_array_equal(actual_value, configured_value)


def _artifact_with_initial_value_mutation(
    manifest: RunManifest,
    field_name: str,
    replacement: float | tuple[float, float, float, float],
) -> RunArtifactData:
    """Change one history at row zero while retaining intrinsic validity."""
    sources = _writable_source_arrays()
    _assert_initial_values_match_manifest(_artifact_data(sources), manifest)
    unchanged_sources = {name: sources[name] for name in _ARRAY_FIELDS if name != field_name}
    source = sources[field_name]
    changed_history = source.copy()
    if field_name == "truth_q_history_WB":
        assert replacement == (0.0, 1.0, 0.0, 0.0)
        np.testing.assert_array_equal(source[0], np.array([1.0, 0.0, 0.0, 0.0], dtype=_FLOAT_DTYPE))
        changed_history[0] = np.array(replacement, dtype=_FLOAT_DTYPE)
        assert np.linalg.norm(changed_history[0]) == 1.0
        np.testing.assert_array_equal(changed_history[0, 2:], source[0, 2:])
    else:
        assert isinstance(replacement, float)
        assert np.isfinite(replacement)
        assert source[0, 0] != replacement
        changed_history[0, 0] = replacement
        np.testing.assert_array_equal(changed_history[0, 1:], source[0, 1:])
    assert type(changed_history) is np.ndarray
    assert changed_history.dtype == source.dtype == _FLOAT_DTYPE
    assert changed_history.ndim == source.ndim
    assert changed_history.shape == source.shape
    np.testing.assert_array_equal(changed_history[1:], source[1:])
    sources[field_name] = changed_history
    for name, original in unchanged_sources.items():
        assert sources[name] is original

    data = _artifact_data(sources)
    assert type(data) is RunArtifactData
    np.testing.assert_array_equal(getattr(data, field_name), changed_history)
    for name, original in unchanged_sources.items():
        np.testing.assert_array_equal(getattr(data, name), original)
    _assert_initial_values_match_manifest(data, manifest, mismatched_field=field_name)
    return data


@pytest.mark.parametrize(
    ("field_name", "replacement", "expected_message"),
    [
        pytest.param(
            "truth_position_history_W",
            1.0,
            "truth_position_history_W row 0 must equal "
            "configuration.initial_truth_state.position_W",
            id="position",
        ),
        pytest.param(
            "truth_velocity_history_W",
            1.5,
            "truth_velocity_history_W row 0 must equal "
            "configuration.initial_truth_state.velocity_W",
            id="velocity",
        ),
        pytest.param(
            "truth_q_history_WB",
            (0.0, 1.0, 0.0, 0.0),
            "truth_q_history_WB row 0 must equal configuration.initial_truth_state.q_WB",
            id="attitude",
        ),
        pytest.param(
            "truth_omega_history_B",
            0.2,
            "truth_omega_history_B row 0 must equal configuration.initial_truth_state.omega_B",
            id="angular_velocity",
        ),
        pytest.param(
            "accelerometer_bias_history_B",
            0.2,
            "accelerometer_bias_history_B row 0 must equal "
            "configuration.truth.imu.initial_accelerometer_bias_B",
            id="accelerometer_bias",
        ),
        pytest.param(
            "gyroscope_bias_history_B",
            0.02,
            "gyroscope_bias_history_B row 0 must equal "
            "configuration.truth.imu.initial_gyroscope_bias_B",
            id="gyroscope_bias",
        ),
    ],
)
def test_save_run_directory_rejects_initial_state_or_bias_mismatch_before_encoding(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    field_name: str,
    replacement: float | tuple[float, float, float, float],
    expected_message: str,
) -> None:
    manifest = _unbound_save_manifest()
    data = _artifact_with_initial_value_mutation(manifest, field_name, replacement)
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    manifest_bytes = encode_run_manifest(manifest)
    run_directory = tmp_path / "incompatible-initial-values"
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    encode_mock = Mock(side_effect=AssertionError("NPZ encoding must not start"))
    monkeypatch.setattr(run_artifact, "_encode_data_npz", encode_mock)

    with pytest.raises(ValueError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert str(error.value) == expected_message
    encode_mock.assert_not_called()
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def test_load_run_directory_rejects_initial_state_mismatch(tmp_path: Path) -> None:
    manifest = _unbound_save_manifest()
    data = _artifact_with_initial_value_mutation(manifest, "truth_position_history_W", 1.0)
    unbound_manifest_bytes = encode_run_manifest(manifest)
    run_directory = tmp_path / "incompatible-initial-values"
    bound_manifest, manifest_bytes, data_bytes = _write_directly_bound_run_directory(
        run_directory, manifest, data
    )
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    decoded = run_artifact._decode_data_npz(data_bytes)
    np.testing.assert_array_equal(
        decoded.truth_position_history_W[0], data.truth_position_history_W[0]
    )
    assert not np.array_equal(
        decoded.truth_position_history_W[0],
        manifest.run_configuration.initial_truth_state.position_W,
    )

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == (
        "truth_position_history_W row 0 must equal configuration.initial_truth_state.position_W"
    )
    assert manifest_path.read_bytes() == manifest_bytes
    assert data_path.read_bytes() == data_bytes
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == unbound_manifest_bytes
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert encode_run_manifest(bound_manifest) == manifest_bytes
    assert run_artifact._encode_data_npz(data) == data_bytes


def _established_truth_history_error(data: RunArtifactData) -> str:
    """Return the existing rigid-body history validator's exact error."""
    with pytest.raises(ValueError) as error:
        validate_rigid_body_state_history(
            data.truth_time_s,
            data.truth_position_history_W,
            data.truth_velocity_history_W,
            data.truth_q_history_WB,
            data.truth_omega_history_B,
        )
    return str(error.value)


def _artifact_with_truth_history_mutation(mutation: str, manifest: RunManifest) -> RunArtifactData:
    """Change only one noninitial truth-history row of the valid artifact."""
    sources = _writable_source_arrays()
    baseline = _artifact_data(sources)
    assert run_artifact._validate_run_artifact_against_manifest(manifest, baseline) is None
    assert (
        validate_rigid_body_state_history(
            baseline.truth_time_s,
            baseline.truth_position_history_W,
            baseline.truth_velocity_history_W,
            baseline.truth_q_history_WB,
            baseline.truth_omega_history_B,
        )
        is None
    )
    field_name = {
        "position_nonfinite": "truth_position_history_W",
        "velocity_nonfinite": "truth_velocity_history_W",
        "attitude_nonfinite": "truth_q_history_WB",
        "angular_velocity_nonfinite": "truth_omega_history_B",
        "attitude_nonunit": "truth_q_history_WB",
    }[mutation]
    unchanged_sources = {name: sources[name] for name in _ARRAY_FIELDS if name != field_name}
    source = sources[field_name]
    changed_history = source.copy()
    if mutation == "attitude_nonunit":
        changed_history[1] = np.array([2.0, 0.0, 0.0, 0.0], dtype=_FLOAT_DTYPE)
        assert np.all(np.isfinite(changed_history[1]))
        assert np.linalg.norm(changed_history[1]) == 2.0
    else:
        changed_history[1, 0] = np.nan
        assert np.isnan(changed_history[1, 0])
    assert type(changed_history) is np.ndarray
    assert changed_history.dtype == source.dtype == _FLOAT_DTYPE
    assert changed_history.ndim == source.ndim == 2
    assert changed_history.shape == source.shape
    np.testing.assert_array_equal(changed_history[0], source[0])
    np.testing.assert_array_equal(changed_history[1, 1:], source[1, 1:])
    np.testing.assert_array_equal(changed_history[2:], source[2:])
    sources[field_name] = changed_history
    for name, original in unchanged_sources.items():
        assert sources[name] is original

    data = _artifact_data(sources)
    assert type(data) is RunArtifactData
    np.testing.assert_array_equal(getattr(data, field_name), changed_history)
    for name, original in unchanged_sources.items():
        np.testing.assert_array_equal(getattr(data, name), original)
    _assert_initial_values_match_manifest(data, manifest)
    assert data.truth_time_s.shape == baseline.truth_time_s.shape
    assert data.commanded_rotor_omega.shape == baseline.commanded_rotor_omega.shape
    np.testing.assert_array_equal(data.truth_time_s, baseline.truth_time_s)
    np.testing.assert_array_equal(data.commanded_rotor_omega, baseline.commanded_rotor_omega)
    return data


@pytest.mark.parametrize(
    "mutation",
    [
        pytest.param("position_nonfinite", id="position_nonfinite"),
        pytest.param("velocity_nonfinite", id="velocity_nonfinite"),
        pytest.param("attitude_nonfinite", id="attitude_nonfinite"),
        pytest.param("angular_velocity_nonfinite", id="angular_velocity_nonfinite"),
        pytest.param("attitude_nonunit", id="attitude_nonunit"),
    ],
)
def test_save_run_directory_applies_established_truth_history_validation_before_encoding(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mutation: str
) -> None:
    manifest = _unbound_save_manifest()
    manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_with_truth_history_mutation(mutation, manifest)
    expected_message = _established_truth_history_error(data)
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    run_directory = tmp_path / "invalid-truth-history"
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    encode_mock = Mock(side_effect=AssertionError("NPZ encoding must not start"))
    monkeypatch.setattr(run_artifact, "_encode_data_npz", encode_mock)

    with pytest.raises(ValueError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert str(error.value) == expected_message
    encode_mock.assert_not_called()
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def test_load_run_directory_applies_established_truth_history_validation(tmp_path: Path) -> None:
    manifest = _unbound_save_manifest()
    unbound_manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_with_truth_history_mutation("position_nonfinite", manifest)
    expected_message = _established_truth_history_error(data)
    run_directory = tmp_path / "invalid-truth-history"
    bound_manifest, manifest_bytes, data_bytes = _write_directly_bound_run_directory(
        run_directory, manifest, data
    )
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    decoded = run_artifact._decode_data_npz(data_bytes)
    assert np.isnan(decoded.truth_position_history_W[1, 0])
    assert _established_truth_history_error(decoded) == expected_message

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == expected_message
    assert manifest_path.read_bytes() == manifest_bytes
    assert data_path.read_bytes() == data_bytes
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == unbound_manifest_bytes
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert encode_run_manifest(bound_manifest) == manifest_bytes
    assert run_artifact._encode_data_npz(data) == data_bytes


def _artifact_with_extra_pending_sensor_observation(
    manifest: RunManifest, channel_name: str
) -> RunArtifactData:
    """Append one internally valid pending observation beyond the run horizon."""
    sources = _writable_source_arrays()
    baseline = _artifact_data(sources)
    assert run_artifact._validate_run_artifact_against_manifest(manifest, baseline) is None
    configuration = manifest.run_configuration
    schedule = getattr(configuration.sensor_schedules, channel_name)
    sample_stride = round(schedule.sample_period_s / configuration.numerics.truth_time_step_s)
    current_length = sources[f"{channel_name}_sequence_index"].size
    assert sample_stride > 0
    assert current_length == configuration.numerics.number_of_steps // sample_stride

    acquisition_time_s, delivery_time_s, measurement_name, measurement_row = {
        "accelerometer": (
            0.3,
            0.3,
            "accelerometer_measurements_B",
            np.array([[0.0, 0.0, 0.0]], dtype=_FLOAT_DTYPE),
        ),
        "gyroscope": (
            0.4,
            0.4,
            "gyroscope_measurements_B",
            np.array([[0.0, 0.0, 0.0]], dtype=_FLOAT_DTYPE),
        ),
        "local_position": (
            0.3,
            0.4,
            "local_position_measurements_W",
            np.array([[0.0, 0.0, 0.0]], dtype=_FLOAT_DTYPE),
        ),
        "barometric_altitude": (
            0.4,
            0.4,
            "barometric_altitude_measurements",
            np.array([123.4], dtype=_FLOAT_DTYPE),
        ),
    }[channel_name]
    assert np.isfinite(acquisition_time_s)
    assert np.isfinite(delivery_time_s)
    assert acquisition_time_s > (
        configuration.numerics.number_of_steps * configuration.numerics.truth_time_step_s
    )
    assert delivery_time_s >= acquisition_time_s
    assert type(measurement_row) is np.ndarray
    assert measurement_row.dtype == _FLOAT_DTYPE
    assert measurement_row.shape == ((1,) if channel_name == "barometric_altitude" else (1, 3))
    assert np.all(np.isfinite(measurement_row))

    stream_rows = {
        f"{channel_name}_sequence_index": np.array([current_length], dtype=_INTEGER_DTYPE),
        f"{channel_name}_observation_index": np.array([current_length], dtype=_INTEGER_DTYPE),
        f"{channel_name}_acquisition_time_s": np.array([acquisition_time_s], dtype=_FLOAT_DTYPE),
        f"{channel_name}_delivery_time_s": np.array([delivery_time_s], dtype=_FLOAT_DTYPE),
        f"{channel_name}_delivered_at_truth_index": np.array([-1], dtype=_INTEGER_DTYPE),
        measurement_name: measurement_row,
    }
    unchanged_sources = {name: sources[name] for name in _ARRAY_FIELDS if name not in stream_rows}
    for name, appended_row in stream_rows.items():
        source = sources[name]
        changed_stream = np.concatenate((source, appended_row), axis=0)
        assert type(changed_stream) is np.ndarray
        assert changed_stream.dtype == source.dtype
        assert changed_stream.ndim == source.ndim
        assert changed_stream.shape == (current_length + 1, *source.shape[1:])
        np.testing.assert_array_equal(changed_stream[:-1], source)
        sources[name] = changed_stream
    for name, original in unchanged_sources.items():
        assert sources[name] is original

    data = _artifact_data(sources)
    assert type(data) is RunArtifactData
    for name in stream_rows:
        assert getattr(data, name).shape[0] == current_length + 1
        np.testing.assert_array_equal(getattr(data, name)[:-1], getattr(baseline, name))
    np.testing.assert_array_equal(
        getattr(data, f"{channel_name}_sequence_index"),
        np.arange(current_length + 1, dtype=_INTEGER_DTYPE),
    )
    np.testing.assert_array_equal(
        getattr(data, f"{channel_name}_observation_index"),
        np.arange(current_length + 1, dtype=_INTEGER_DTYPE),
    )
    assert np.all(np.isfinite(getattr(data, f"{channel_name}_acquisition_time_s")))
    assert np.all(np.isfinite(getattr(data, f"{channel_name}_delivery_time_s")))
    assert getattr(data, f"{channel_name}_delivered_at_truth_index")[-1] == -1
    for name, original in unchanged_sources.items():
        np.testing.assert_array_equal(getattr(data, name), original)
    _assert_initial_values_match_manifest(data, manifest)
    # Successful construction checks exact delivery-table membership and order.
    return data


@pytest.mark.parametrize(
    "channel_name",
    [
        pytest.param("accelerometer", id="accelerometer"),
        pytest.param("gyroscope", id="gyroscope"),
        pytest.param("local_position", id="local_position"),
        pytest.param("barometric_altitude", id="barometric_altitude"),
    ],
)
def test_save_run_directory_requires_configured_sensor_acquisition_count_before_encoding(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, channel_name: str
) -> None:
    manifest = _unbound_save_manifest()
    manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_with_extra_pending_sensor_observation(manifest, channel_name)
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    run_directory = tmp_path / "extra-sensor-observation"
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    encode_mock = Mock(side_effect=AssertionError("NPZ encoding must not start"))
    monkeypatch.setattr(run_artifact, "_encode_data_npz", encode_mock)

    with pytest.raises(ValueError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert str(error.value) == (
        f"{channel_name} stream length must equal the configured acquisition count"
    )
    encode_mock.assert_not_called()
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def test_load_run_directory_requires_configured_sensor_acquisition_count(tmp_path: Path) -> None:
    manifest = _unbound_save_manifest()
    unbound_manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_with_extra_pending_sensor_observation(manifest, "accelerometer")
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    run_directory = tmp_path / "extra-sensor-observation"
    bound_manifest, manifest_bytes, data_bytes = _write_directly_bound_run_directory(
        run_directory, manifest, data
    )
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    decoded = run_artifact._decode_data_npz(data_bytes)
    np.testing.assert_array_equal(decoded.accelerometer_sequence_index, np.array([0, 1, 2]))
    assert decoded.accelerometer_delivered_at_truth_index[-1] == -1

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == (
        "accelerometer stream length must equal the configured acquisition count"
    )
    assert manifest_path.read_bytes() == manifest_bytes
    assert data_path.read_bytes() == data_bytes
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == unbound_manifest_bytes
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert encode_run_manifest(bound_manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def _assert_configured_sensor_timestamps(
    manifest: RunManifest,
    channel_name: str,
    acquisition_time_s: np.ndarray,
    delivery_time_s: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Audit one valid stream against its configured acquisition and delivery times."""
    configuration = manifest.run_configuration
    number_of_steps = configuration.numerics.number_of_steps
    truth_time_step_s = configuration.numerics.truth_time_step_s
    schedule = getattr(configuration.sensor_schedules, channel_name)
    sample_stride = int(round(schedule.sample_period_s / truth_time_step_s))
    expected_count = number_of_steps // sample_stride
    acquisition_rows = np.arange(1, expected_count + 1, dtype=_INTEGER_DTYPE) * sample_stride
    expected_acquisition_time_s = acquisition_rows.astype(_FLOAT_DTYPE) * truth_time_step_s
    expected_delivery_time_s = expected_acquisition_time_s + schedule.delivery_delay_s

    assert expected_acquisition_time_s.dtype == _FLOAT_DTYPE
    assert expected_delivery_time_s.dtype == _FLOAT_DTYPE
    assert acquisition_time_s.shape == expected_acquisition_time_s.shape == (expected_count,)
    assert delivery_time_s.shape == expected_delivery_time_s.shape == (expected_count,)
    assert np.allclose(acquisition_time_s, expected_acquisition_time_s, rtol=0.0, atol=1.0e-12)
    assert np.allclose(delivery_time_s, expected_delivery_time_s, rtol=0.0, atol=1.0e-12)
    return expected_acquisition_time_s, expected_delivery_time_s


@pytest.mark.parametrize(
    ("channel_name", "timestamp_kind"),
    [
        pytest.param(channel_name, timestamp_kind, id=f"{channel_name}-{timestamp_kind}")
        for channel_name in (
            "accelerometer",
            "gyroscope",
            "local_position",
            "barometric_altitude",
        )
        for timestamp_kind in ("acquisition", "delivery")
    ],
)
def test_save_run_directory_requires_configured_sensor_timestamps_before_encoding(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, channel_name: str, timestamp_kind: str
) -> None:
    manifest = _unbound_save_manifest()
    manifest_bytes = encode_run_manifest(manifest)
    sources = _writable_source_arrays()
    baseline = _artifact_data(sources)
    expected_acquisition_time_s, expected_delivery_time_s = _assert_configured_sensor_timestamps(
        manifest,
        channel_name,
        getattr(baseline, f"{channel_name}_acquisition_time_s"),
        getattr(baseline, f"{channel_name}_delivery_time_s"),
    )
    field_name = f"{channel_name}_{timestamp_kind}_time_s"
    source = sources[field_name]
    changed_time_s = source.copy()
    changed_time_s[0] += 0.01
    assert type(changed_time_s) is np.ndarray
    assert changed_time_s.dtype == source.dtype == _FLOAT_DTYPE
    assert changed_time_s.ndim == source.ndim == 1
    assert changed_time_s.shape == source.shape
    np.testing.assert_array_equal(changed_time_s[1:], source[1:])
    expected_time_s = (
        expected_acquisition_time_s if timestamp_kind == "acquisition" else expected_delivery_time_s
    )
    assert abs(changed_time_s[0] - expected_time_s[0]) > 1.0e-12
    assert not np.allclose(changed_time_s, expected_time_s, rtol=0.0, atol=1.0e-12)
    sources[field_name] = changed_time_s
    data = _artifact_data(sources)
    assert type(data) is RunArtifactData
    np.testing.assert_array_equal(getattr(data, field_name), changed_time_s)
    for name in _ARRAY_FIELDS:
        if name != field_name:
            np.testing.assert_array_equal(getattr(data, name), getattr(baseline, name))
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    run_directory = tmp_path / "off-schedule-sensor-timestamp"
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    encode_mock = Mock(side_effect=AssertionError("NPZ encoding must not start"))
    monkeypatch.setattr(run_artifact, "_encode_data_npz", encode_mock)

    with pytest.raises(ValueError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    if timestamp_kind == "acquisition":
        expected_message = f"{field_name} must equal the configured acquisition schedule"
    else:
        expected_message = (
            f"{field_name} must equal configured acquisition times plus delivery delay"
        )
    assert str(error.value) == expected_message
    encode_mock.assert_not_called()
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def test_load_run_directory_requires_configured_sensor_timestamps(tmp_path: Path) -> None:
    manifest = _unbound_save_manifest()
    unbound_manifest_bytes = encode_run_manifest(manifest)
    sources = _writable_source_arrays()
    baseline = _artifact_data(sources)
    expected_acquisition_time_s, _ = _assert_configured_sensor_timestamps(
        manifest,
        "accelerometer",
        baseline.accelerometer_acquisition_time_s,
        baseline.accelerometer_delivery_time_s,
    )
    changed_acquisition_time_s = sources["accelerometer_acquisition_time_s"].copy()
    changed_acquisition_time_s[0] += 0.01
    assert type(changed_acquisition_time_s) is np.ndarray
    assert changed_acquisition_time_s.dtype == _FLOAT_DTYPE
    assert changed_acquisition_time_s.ndim == 1
    assert changed_acquisition_time_s.shape == baseline.accelerometer_acquisition_time_s.shape
    np.testing.assert_array_equal(
        changed_acquisition_time_s[1:], baseline.accelerometer_acquisition_time_s[1:]
    )
    assert abs(changed_acquisition_time_s[0] - expected_acquisition_time_s[0]) > 1.0e-12
    sources["accelerometer_acquisition_time_s"] = changed_acquisition_time_s
    data = _artifact_data(sources)
    assert type(data) is RunArtifactData
    for name in _ARRAY_FIELDS:
        if name != "accelerometer_acquisition_time_s":
            np.testing.assert_array_equal(getattr(data, name), getattr(baseline, name))
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    run_directory = tmp_path / "off-schedule-sensor-timestamp"
    bound_manifest, manifest_bytes, data_bytes = _write_directly_bound_run_directory(
        run_directory, manifest, data
    )
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    decoded = run_artifact._decode_data_npz(data_bytes)
    np.testing.assert_array_equal(
        decoded.accelerometer_acquisition_time_s, changed_acquisition_time_s
    )

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == (
        "accelerometer_acquisition_time_s must equal the configured acquisition schedule"
    )
    assert manifest_path.read_bytes() == manifest_bytes
    assert data_path.read_bytes() == data_bytes
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == unbound_manifest_bytes
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert encode_run_manifest(bound_manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def _expected_delivery_truth_rows(
    truth_time_s: np.ndarray, delivery_time_s: np.ndarray
) -> np.ndarray:
    """Find the first truth update that reaches each scheduled delivery time."""
    expected_rows = np.full(delivery_time_s.shape, -1, dtype=_INTEGER_DTYPE)
    for observation_index, scheduled_delivery_time_s in enumerate(delivery_time_s):
        for truth_index in range(1, truth_time_s.shape[0]):
            if truth_time_s[truth_index] >= scheduled_delivery_time_s - 1.0e-12:
                expected_rows[observation_index] = truth_index
                break
    assert type(expected_rows) is np.ndarray
    assert expected_rows.dtype == _INTEGER_DTYPE
    assert expected_rows.ndim == 1
    assert expected_rows.shape == delivery_time_s.shape
    return expected_rows


def _artifact_with_delivery_truth_row_mutation(
    manifest: RunManifest, channel_name: str, replacement: tuple[int, ...]
) -> RunArtifactData:
    """Change one delivered-row array and rebuild only the global delivery table."""
    sources = _writable_source_arrays()
    baseline = _artifact_data(sources)
    valid_rows = {
        "accelerometer": (1, 2),
        "gyroscope": (2,),
        "local_position": (2, -1),
        "barometric_altitude": (2,),
    }
    channels = tuple(valid_rows)
    for channel in channels:
        _assert_configured_sensor_timestamps(
            manifest,
            channel,
            getattr(baseline, f"{channel}_acquisition_time_s"),
            getattr(baseline, f"{channel}_delivery_time_s"),
        )
        expected_rows = _expected_delivery_truth_rows(
            baseline.truth_time_s, getattr(baseline, f"{channel}_delivery_time_s")
        )
        np.testing.assert_array_equal(
            expected_rows, np.array(valid_rows[channel], dtype=_INTEGER_DTYPE)
        )
        np.testing.assert_array_equal(
            getattr(baseline, f"{channel}_delivered_at_truth_index"), expected_rows
        )

    field_name = f"{channel_name}_delivered_at_truth_index"
    source = sources[field_name]
    changed_rows = source.copy()
    assert changed_rows.shape == (len(replacement),)
    changed_rows[:] = replacement
    assert type(changed_rows) is np.ndarray
    assert changed_rows.dtype == source.dtype == _INTEGER_DTYPE
    assert changed_rows.ndim == source.ndim == 1
    assert changed_rows.shape == source.shape
    assert not np.array_equal(changed_rows, source)
    sources[field_name] = changed_rows

    delivered_records: list[tuple[int, int, int, int]] = []
    for sensor_id, channel in enumerate(channels, start=1):
        for truth_index, sequence_index, observation_index in zip(
            sources[f"{channel}_delivered_at_truth_index"],
            sources[f"{channel}_sequence_index"],
            sources[f"{channel}_observation_index"],
            strict=True,
        ):
            if truth_index != -1:
                delivered_records.append(
                    (int(truth_index), sensor_id, int(sequence_index), int(observation_index))
                )
    delivered_records.sort(key=lambda record: record[:3])
    delivery_table_rows = {
        "delivery_sensor_id": [record[1] for record in delivered_records],
        "delivery_sequence_index": [record[2] for record in delivered_records],
        "delivery_observation_index": [record[3] for record in delivered_records],
    }
    assert set(delivery_table_rows) == set(_DELIVERY_TABLE_FIELDS)
    for name, values in delivery_table_rows.items():
        sources[name] = np.array(values, dtype=_INTEGER_DTYPE)
        assert type(sources[name]) is np.ndarray
        assert sources[name].dtype == _INTEGER_DTYPE
        assert sources[name].shape == (len(delivered_records),)

    data = _artifact_data(sources)
    assert type(data) is RunArtifactData
    np.testing.assert_array_equal(getattr(data, field_name), changed_rows)
    for name in _ARRAY_FIELDS:
        if name != field_name and name not in _DELIVERY_TABLE_FIELDS:
            np.testing.assert_array_equal(getattr(data, name), getattr(baseline, name))
    for name in _DELIVERY_TABLE_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), sources[name])
    actual_global_records = list(
        zip(
            data.delivery_sensor_id.tolist(),
            data.delivery_sequence_index.tolist(),
            data.delivery_observation_index.tolist(),
            strict=True,
        )
    )
    assert actual_global_records == [record[1:] for record in delivered_records]
    for channel in channels:
        _assert_configured_sensor_timestamps(
            manifest,
            channel,
            getattr(data, f"{channel}_acquisition_time_s"),
            getattr(data, f"{channel}_delivery_time_s"),
        )
    expected_selected_rows = _expected_delivery_truth_rows(
        data.truth_time_s, getattr(data, f"{channel_name}_delivery_time_s")
    )
    assert not np.array_equal(getattr(data, field_name), expected_selected_rows)
    _assert_initial_values_match_manifest(data, manifest)
    return data


@pytest.mark.parametrize(
    ("channel_name", "replacement"),
    [
        pytest.param("accelerometer", (2, 2), id="accelerometer-wrong-row"),
        pytest.param("gyroscope", (1,), id="gyroscope-wrong-row"),
        pytest.param("local_position", (1, -1), id="local_position-wrong-row"),
        pytest.param("barometric_altitude", (1,), id="barometric_altitude-wrong-row"),
        pytest.param("local_position", (2, 2), id="local_position-pending-marked-delivered"),
        pytest.param(
            "barometric_altitude", (-1,), id="barometric_altitude-delivered-marked-pending"
        ),
    ],
)
def test_save_run_directory_requires_configured_delivery_truth_rows_before_encoding(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, channel_name: str, replacement: tuple[int, ...]
) -> None:
    manifest = _unbound_save_manifest()
    manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_with_delivery_truth_row_mutation(manifest, channel_name, replacement)
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    run_directory = tmp_path / "off-schedule-delivery-truth-row"
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    encode_mock = Mock(side_effect=AssertionError("NPZ encoding must not start"))
    monkeypatch.setattr(run_artifact, "_encode_data_npz", encode_mock)

    with pytest.raises(ValueError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert str(error.value) == (
        f"{channel_name}_delivered_at_truth_index must match configured delivery times "
        "and truth updates"
    )
    encode_mock.assert_not_called()
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def test_load_run_directory_requires_configured_delivery_truth_rows(tmp_path: Path) -> None:
    manifest = _unbound_save_manifest()
    unbound_manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_with_delivery_truth_row_mutation(manifest, "local_position", (2, 2))
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    run_directory = tmp_path / "off-schedule-delivery-truth-row"
    bound_manifest, manifest_bytes, data_bytes = _write_directly_bound_run_directory(
        run_directory, manifest, data
    )
    manifest_path = run_directory / "manifest.json"
    data_path = run_directory / "data.npz"
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    decoded = run_artifact._decode_data_npz(data_bytes)
    np.testing.assert_array_equal(
        decoded.local_position_delivered_at_truth_index, np.array([2, 2], dtype=_INTEGER_DTYPE)
    )
    np.testing.assert_array_equal(
        decoded.delivery_sensor_id, np.array([1, 1, 2, 3, 3, 4], dtype=_INTEGER_DTYPE)
    )
    np.testing.assert_array_equal(
        decoded.delivery_sequence_index, np.array([0, 1, 0, 0, 1, 0], dtype=_INTEGER_DTYPE)
    )
    np.testing.assert_array_equal(
        decoded.delivery_observation_index, np.array([0, 1, 0, 0, 1, 0], dtype=_INTEGER_DTYPE)
    )

    with pytest.raises(ValueError) as error:
        run_artifact.load_run_directory(run_directory)

    assert str(error.value) == (
        "local_position_delivered_at_truth_index must match "
        "configured delivery times and truth updates"
    )
    assert manifest_path.read_bytes() == manifest_bytes
    assert data_path.read_bytes() == data_bytes
    assert set(run_directory.iterdir()) == {manifest_path, data_path}
    assert set(tmp_path.iterdir()) == {run_directory}
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == unbound_manifest_bytes
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert encode_run_manifest(bound_manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def test_write_bytes_and_fsync_writes_exact_payload_before_syncing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "durable-payload.bin"
    payload = b"durable-run-artifact-payload"
    real_write_bytes = Path.write_bytes
    real_open = os.open
    real_fsync = os.fsync
    real_close = os.close
    events: list[str] = []
    write_completed = False
    synchronized = False
    opened_descriptors: list[int] = []

    def observe_write_bytes(write_path: Path, written_payload: bytes) -> int:
        nonlocal write_completed
        events.append("write_bytes")
        assert write_path == path
        assert written_payload == payload
        written_count = real_write_bytes(write_path, written_payload)
        assert written_count == len(payload)
        write_completed = True
        return written_count

    def observe_open(open_path: Path, flags: int) -> int:
        events.append("os.open")
        assert write_completed
        assert open_path == path
        assert flags & os.O_ACCMODE == os.O_RDONLY
        assert path.read_bytes() == payload
        descriptor = real_open(open_path, flags)
        opened_descriptors.append(descriptor)
        return descriptor

    def observe_fsync(descriptor: int) -> None:
        nonlocal synchronized
        events.append("os.fsync")
        assert opened_descriptors == [descriptor]
        assert stat.S_ISREG(os.fstat(descriptor).st_mode)
        assert os.path.samefile(f"/proc/self/fd/{descriptor}", path)
        assert path.read_bytes() == payload
        real_fsync(descriptor)
        synchronized = True

    def observe_close(descriptor: int) -> None:
        events.append("os.close")
        assert synchronized
        assert opened_descriptors == [descriptor]
        real_close(descriptor)

    with monkeypatch.context() as patches:
        patches.setattr(Path, "write_bytes", observe_write_bytes)
        patches.setattr(os, "open", observe_open)
        patches.setattr(os, "fsync", observe_fsync)
        patches.setattr(os, "close", observe_close)
        run_artifact._write_bytes_and_fsync(path, payload)

    assert events == ["write_bytes", "os.open", "os.fsync", "os.close"]
    assert write_completed
    assert synchronized
    assert len(opened_descriptors) == 1
    assert path.read_bytes() == payload


def test_fsync_directory_opens_syncs_and_closes_exact_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    directory = tmp_path / "durable-directory"
    directory.mkdir()
    real_open = os.open
    real_fsync = os.fsync
    real_close = os.close
    events: list[str] = []
    opened_descriptors: list[int] = []
    synchronized = False

    def observe_open(open_path: Path, flags: int) -> int:
        events.append("os.open")
        assert open_path == directory
        assert flags & os.O_ACCMODE == os.O_RDONLY
        assert flags & os.O_DIRECTORY
        descriptor = real_open(open_path, flags)
        opened_descriptors.append(descriptor)
        return descriptor

    def observe_fsync(descriptor: int) -> None:
        nonlocal synchronized
        events.append("os.fsync")
        assert opened_descriptors == [descriptor]
        assert stat.S_ISDIR(os.fstat(descriptor).st_mode)
        real_fsync(descriptor)
        synchronized = True

    def observe_close(descriptor: int) -> None:
        events.append("os.close")
        assert synchronized
        assert opened_descriptors == [descriptor]
        real_close(descriptor)

    with monkeypatch.context() as patches:
        patches.setattr(os, "open", observe_open)
        patches.setattr(os, "fsync", observe_fsync)
        patches.setattr(os, "close", observe_close)
        run_artifact._fsync_directory(directory)

    assert events == ["os.open", "os.fsync", "os.close"]
    assert len(opened_descriptors) == 1
    assert synchronized


def test_save_run_directory_durably_publishes_in_required_order(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_directory = tmp_path / "run-0001"
    manifest = _unbound_save_manifest()
    original_manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_data(_writable_source_arrays())
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    real_publish = run_artifact._publish_staging_directory
    events: list[str] = []
    writes: list[tuple[Path, bytes]] = []
    staging_sync_states: list[tuple[Path, set[Path], bool]] = []
    publication_states: list[tuple[Path, Path, set[Path], bool]] = []
    parent_sync_states: list[tuple[Path, bool, bool, set[Path]]] = []

    def observe_write(path: Path, payload: bytes) -> None:
        events.append("write_data" if path.name == "data.npz" else "write_manifest")
        writes.append((path, payload))
        path.write_bytes(payload)

    def observe_directory_sync(directory: Path) -> None:
        if directory == run_directory.parent:
            events.append("sync_parent")
            staged_path = publication_states[-1][0] if publication_states else None
            parent_sync_states.append(
                (
                    directory,
                    run_directory.is_dir(),
                    staged_path is not None and not staged_path.exists(),
                    set(run_directory.iterdir()) if run_directory.is_dir() else set(),
                )
            )
        else:
            events.append("sync_staging")
            staging_sync_states.append(
                (directory, set(directory.iterdir()), not run_directory.exists())
            )

    def observe_publish(staging_directory: Path, destination: Path) -> None:
        events.append("publish")
        publication_states.append(
            (
                staging_directory,
                destination,
                set(staging_directory.iterdir()),
                not destination.exists(),
            )
        )
        real_publish(staging_directory, destination)

    monkeypatch.setattr(run_artifact, "_write_bytes_and_fsync", observe_write, raising=False)
    monkeypatch.setattr(run_artifact, "_fsync_directory", observe_directory_sync, raising=False)
    monkeypatch.setattr(run_artifact, "_publish_staging_directory", observe_publish)

    bound_manifest = run_artifact.save_run_directory(run_directory, manifest, data)
    events.append("return")

    assert len(publication_states) == 1
    staging_directory = publication_states[0][0]
    data_path = run_directory / "data.npz"
    manifest_path = run_directory / "manifest.json"
    staged_entries = {staging_directory / "data.npz", staging_directory / "manifest.json"}
    assert events == [
        "write_data",
        "write_manifest",
        "sync_staging",
        "publish",
        "sync_parent",
        "return",
    ]
    assert [path for path, _ in writes] == [
        staging_directory / "data.npz",
        staging_directory / "manifest.json",
    ]
    assert staging_sync_states == [(staging_directory, staged_entries, True)]
    assert publication_states == [(staging_directory, run_directory, staged_entries, True)]
    assert parent_sync_states == [(tmp_path, True, True, {data_path, manifest_path})]
    assert set(tmp_path.iterdir()) == {run_directory}
    assert set(run_directory.iterdir()) == {data_path, manifest_path}
    published_data_bytes = data_path.read_bytes()
    published_manifest_bytes = manifest_path.read_bytes()
    assert writes[0][1] == published_data_bytes
    assert writes[1][1] == published_manifest_bytes
    assert bound_manifest.data_npz_sha256 == hashlib.sha256(published_data_bytes).hexdigest()
    assert encode_run_manifest(bound_manifest) == published_manifest_bytes
    assert (
        encode_run_manifest(decode_run_manifest(published_manifest_bytes))
        == published_manifest_bytes
    )
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == original_manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


@pytest.mark.parametrize(
    "failure_stage",
    [
        pytest.param("data_file", id="data_file"),
        pytest.param("manifest_file", id="manifest_file"),
        pytest.param("staging_directory", id="staging_directory"),
    ],
)
def test_save_run_directory_cleans_staging_after_prepublication_durability_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure_stage: str
) -> None:
    run_directory = tmp_path / "run-0001"
    manifest = _unbound_save_manifest()
    original_manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_data(_writable_source_arrays())
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    injected_error = OSError(f"injected {failure_stage} synchronization failure")
    written_paths: list[Path] = []
    synced_directories: list[Path] = []
    publish_mock = Mock(wraps=run_artifact._publish_staging_directory)

    def write_then_maybe_fail(path: Path, payload: bytes) -> None:
        path.write_bytes(payload)
        written_paths.append(path)
        if (failure_stage == "data_file" and path.name == "data.npz") or (
            failure_stage == "manifest_file" and path.name == "manifest.json"
        ):
            raise injected_error

    def sync_then_maybe_fail(directory: Path) -> None:
        synced_directories.append(directory)
        if failure_stage == "staging_directory" and len(synced_directories) == 1:
            raise injected_error

    monkeypatch.setattr(
        run_artifact, "_write_bytes_and_fsync", write_then_maybe_fail, raising=False
    )
    monkeypatch.setattr(run_artifact, "_fsync_directory", sync_then_maybe_fail, raising=False)
    monkeypatch.setattr(run_artifact, "_publish_staging_directory", publish_mock)

    with pytest.raises(OSError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert error.value is injected_error
    publish_mock.assert_not_called()
    expected_written_names = ["data.npz"]
    if failure_stage != "data_file":
        expected_written_names.append("manifest.json")
    assert [path.name for path in written_paths] == expected_written_names
    if failure_stage == "staging_directory":
        assert len(synced_directories) == 1
        assert synced_directories[0].parent == tmp_path
    else:
        assert synced_directories == []
    assert not run_directory.exists()
    assert not list(tmp_path.iterdir())
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == original_manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])


def test_save_run_directory_preserves_published_artifact_when_parent_fsync_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_directory = tmp_path / "run-0001"
    manifest = _unbound_save_manifest()
    original_manifest_bytes = encode_run_manifest(manifest)
    data = _artifact_data(_writable_source_arrays())
    original_data_arrays = {name: getattr(data, name).copy() for name in _ARRAY_FIELDS}
    injected_error = OSError("injected parent directory synchronization failure")
    events: list[str] = []
    written_paths: list[Path] = []
    synced_directories: list[Path] = []
    published_pairs: list[tuple[Path, Path]] = []
    real_publish = run_artifact._publish_staging_directory

    def observe_write(path: Path, payload: bytes) -> None:
        events.append("write_data" if path.name == "data.npz" else "write_manifest")
        written_paths.append(path)
        path.write_bytes(payload)

    def sync_then_fail_for_parent(directory: Path) -> None:
        synced_directories.append(directory)
        if len(synced_directories) == 1:
            events.append("sync_staging")
        else:
            events.append("sync_parent")
            assert directory == run_directory.parent
            raise injected_error

    def observe_publish(staging_directory: Path, destination: Path) -> None:
        events.append("publish")
        published_pairs.append((staging_directory, destination))
        real_publish(staging_directory, destination)

    monkeypatch.setattr(run_artifact, "_write_bytes_and_fsync", observe_write, raising=False)
    monkeypatch.setattr(run_artifact, "_fsync_directory", sync_then_fail_for_parent, raising=False)
    monkeypatch.setattr(run_artifact, "_publish_staging_directory", observe_publish)

    with pytest.raises(OSError) as error:
        run_artifact.save_run_directory(run_directory, manifest, data)

    assert error.value is injected_error
    assert events == ["write_data", "write_manifest", "sync_staging", "publish", "sync_parent"]
    assert len(published_pairs) == 1
    staging_directory, destination = published_pairs[0]
    assert destination == run_directory
    assert synced_directories == [staging_directory, run_directory.parent]
    assert written_paths == [staging_directory / "data.npz", staging_directory / "manifest.json"]
    assert not staging_directory.exists()
    assert set(tmp_path.iterdir()) == {run_directory}
    data_path = run_directory / "data.npz"
    manifest_path = run_directory / "manifest.json"
    assert set(run_directory.iterdir()) == {data_path, manifest_path}
    data_bytes = data_path.read_bytes()
    manifest_bytes = manifest_path.read_bytes()
    loaded_manifest, loaded_data = run_artifact.load_run_directory(run_directory)
    assert loaded_manifest.data_npz_sha256 == hashlib.sha256(data_bytes).hexdigest()
    assert encode_run_manifest(loaded_manifest) == manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(loaded_data, name), getattr(data, name))
    assert manifest.data_npz_sha256 is None
    assert encode_run_manifest(manifest) == original_manifest_bytes
    for name in _ARRAY_FIELDS:
        np.testing.assert_array_equal(getattr(data, name), original_data_arrays[name])
