"""Owned, immutable in-memory arrays for one run artifact."""

import ctypes
import errno
import os
from dataclasses import dataclass, fields, replace
from hashlib import sha256
from io import BytesIO
from pathlib import Path
from shutil import rmtree
from tempfile import mkdtemp
from typing import Final, Literal, cast

import numpy as np
from numpy.typing import NDArray

from .run_configuration import SensorSchedule
from .run_manifest import RunManifest, decode_run_manifest, encode_run_manifest
from .sensor_scheduling import _time_comparison_tolerance_s
from .validation import validate_rigid_body_state_history

_FLOAT64_DTYPE: Final[np.dtype[np.float64]] = np.dtype("<f8")
_INT64_DTYPE: Final[np.dtype[np.int64]] = np.dtype("<i8")
_DELIVERY_SENSOR_ID_ACCELEROMETER: Final[int] = 1
_DELIVERY_SENSOR_ID_GYROSCOPE: Final[int] = 2
_DELIVERY_SENSOR_ID_LOCAL_POSITION: Final[int] = 3
_DELIVERY_SENSOR_ID_BAROMETRIC_ALTITUDE: Final[int] = 4
_AT_FDCWD: Final[int] = -100
_RENAME_NOREPLACE: Final[int] = 1
_RUN_DIRECTORY_ENTRY_NAMES: Final[frozenset[str]] = frozenset({"data.npz", "manifest.json"})
type _DeliveredRecord = tuple[int, int, int, int]
type _GlobalDeliveryRecord = tuple[int, int, int]
_DATA_NPZ_MEMBER_NAMES: Final[frozenset[str]] = frozenset(
    {
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
    }
)


def _require_static_shape[ScalarT: np.generic](
    value: NDArray[ScalarT], *, field_name: str, width: Literal[3, 4] | None
) -> None:
    """Require only a vector or a matrix with one fixed trailing width."""
    if width is None:
        if value.ndim != 1:
            raise ValueError(f"{field_name} must have shape (n,)")
    elif value.ndim != 2 or value.shape[1] != width:
        raise ValueError(f"{field_name} must have shape (n, {width})")


def _validate_float_array(
    value: object, *, field_name: str, width: Literal[3, 4] | None
) -> NDArray[np.float64]:
    """Require a finite little-endian float64 array and static shape."""
    if type(value) is not np.ndarray:
        raise ValueError(f"{field_name} must be a NumPy array")
    array = cast(NDArray[np.float64], value)
    if array.dtype != _FLOAT64_DTYPE:
        raise ValueError(f"{field_name} must have dtype little-endian float64")
    _require_static_shape(array, field_name=field_name, width=width)
    _require_finite_values(field_name, array)
    return array


def _validate_integer_array(value: object, *, field_name: str) -> NDArray[np.int64]:
    """Require an exact little-endian int64 vector."""
    if type(value) is not np.ndarray:
        raise ValueError(f"{field_name} must be a NumPy array")
    array = cast(NDArray[np.int64], value)
    if array.dtype != _INT64_DTYPE:
        raise ValueError(f"{field_name} must have dtype little-endian int64")
    _require_static_shape(array, field_name=field_name, width=None)
    return array


def _owned_read_only_copy[ScalarT: np.generic](value: NDArray[ScalarT]) -> NDArray[ScalarT]:
    """Copy one array in C order and freeze the owned copy."""
    copied = value.copy(order="C")
    copied.setflags(write=False)
    return copied


def _require_equal_leading_dimensions(
    arrays: tuple[NDArray[np.float64] | NDArray[np.int64], ...],
    *,
    error_message: str,
) -> None:
    """Require only equal row counts in one explicitly supplied group."""
    leading_dimension = arrays[0].shape[0]
    if any(array.shape[0] != leading_dimension for array in arrays[1:]):
        raise ValueError(error_message)


def _require_consecutive_zero_based_indices(field_name: str, value: NDArray[np.int64]) -> None:
    """Require one stream index vector to follow its zero-based row order."""
    expected = np.arange(value.shape[0], dtype=_INT64_DTYPE)
    if not np.array_equal(value, expected):
        raise ValueError(f"{field_name} must equal consecutive zero-based indices")


def _require_finite_values(field_name: str, value: NDArray[np.float64]) -> None:
    """Require every entry in one floating payload or metadata array to be finite."""
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{field_name} must contain only finite values")


def _require_delivered_truth_indices_in_range(
    field_name: str, value: NDArray[np.int64], final_truth_index: int
) -> None:
    """Require pending or noninitial truth-row indices for one sensor stream."""
    valid = (value == -1) | ((value >= 1) & (value <= final_truth_index))
    if not np.all(valid):
        raise ValueError(
            f"{field_name} entries must be -1 or truth indices from 1 through {final_truth_index}"
        )


def _require_stable_delivery_sensor_ids(value: NDArray[np.int64]) -> None:
    """Require each global delivery row to name one stable sensor channel."""
    valid = (
        (value == _DELIVERY_SENSOR_ID_ACCELEROMETER)
        | (value == _DELIVERY_SENSOR_ID_GYROSCOPE)
        | (value == _DELIVERY_SENSOR_ID_LOCAL_POSITION)
        | (value == _DELIVERY_SENSOR_ID_BAROMETRIC_ALTITUDE)
    )
    if not np.all(valid):
        raise ValueError("delivery_sensor_id entries must be one of: 1, 2, 3, 4")


def _delivered_stream_records(
    sensor_id: int,
    sequence_indices: NDArray[np.int64],
    observation_indices: NDArray[np.int64],
    delivered_at_truth_indices: NDArray[np.int64],
) -> list[_DeliveredRecord]:
    """Describe only delivered observations from one validated sensor stream."""
    records: list[_DeliveredRecord] = []
    for sequence_index, observation_index, delivered_at_truth_index in zip(
        sequence_indices, observation_indices, delivered_at_truth_indices, strict=True
    ):
        if delivered_at_truth_index == -1:
            continue
        records.append(
            (
                int(delivered_at_truth_index),
                sensor_id,
                int(sequence_index),
                int(observation_index),
            )
        )
    return records


@dataclass(frozen=True, slots=True, eq=False)
class RunArtifactData:
    """Hold 35 stable run arrays as owned, read-only copies with finite float payloads."""

    truth_time_s: NDArray[np.float64]
    truth_position_history_W: NDArray[np.float64]
    truth_velocity_history_W: NDArray[np.float64]
    truth_q_history_WB: NDArray[np.float64]
    truth_omega_history_B: NDArray[np.float64]
    commanded_rotor_omega: NDArray[np.float64]

    accelerometer_sequence_index: NDArray[np.int64]
    accelerometer_observation_index: NDArray[np.int64]
    accelerometer_acquisition_time_s: NDArray[np.float64]
    accelerometer_delivery_time_s: NDArray[np.float64]
    accelerometer_delivered_at_truth_index: NDArray[np.int64]
    accelerometer_measurements_B: NDArray[np.float64]

    gyroscope_sequence_index: NDArray[np.int64]
    gyroscope_observation_index: NDArray[np.int64]
    gyroscope_acquisition_time_s: NDArray[np.float64]
    gyroscope_delivery_time_s: NDArray[np.float64]
    gyroscope_delivered_at_truth_index: NDArray[np.int64]
    gyroscope_measurements_B: NDArray[np.float64]

    local_position_sequence_index: NDArray[np.int64]
    local_position_observation_index: NDArray[np.int64]
    local_position_acquisition_time_s: NDArray[np.float64]
    local_position_delivery_time_s: NDArray[np.float64]
    local_position_delivered_at_truth_index: NDArray[np.int64]
    local_position_measurements_W: NDArray[np.float64]

    barometric_altitude_sequence_index: NDArray[np.int64]
    barometric_altitude_observation_index: NDArray[np.int64]
    barometric_altitude_acquisition_time_s: NDArray[np.float64]
    barometric_altitude_delivery_time_s: NDArray[np.float64]
    barometric_altitude_delivered_at_truth_index: NDArray[np.int64]
    barometric_altitude_measurements: NDArray[np.float64]

    accelerometer_bias_history_B: NDArray[np.float64]
    gyroscope_bias_history_B: NDArray[np.float64]

    delivery_sensor_id: NDArray[np.int64]
    delivery_sequence_index: NDArray[np.int64]
    delivery_observation_index: NDArray[np.int64]

    def __post_init__(self) -> None:
        """Validate and install an owned, read-only copy of each array."""
        object.__setattr__(
            self,
            "truth_time_s",
            _owned_read_only_copy(
                _validate_float_array(
                    self.truth_time_s,
                    field_name="truth_time_s",
                    width=None,
                )
            ),
        )
        object.__setattr__(
            self,
            "truth_position_history_W",
            _owned_read_only_copy(
                _validate_float_array(
                    self.truth_position_history_W,
                    field_name="truth_position_history_W",
                    width=3,
                )
            ),
        )
        object.__setattr__(
            self,
            "truth_velocity_history_W",
            _owned_read_only_copy(
                _validate_float_array(
                    self.truth_velocity_history_W,
                    field_name="truth_velocity_history_W",
                    width=3,
                )
            ),
        )
        object.__setattr__(
            self,
            "truth_q_history_WB",
            _owned_read_only_copy(
                _validate_float_array(
                    self.truth_q_history_WB,
                    field_name="truth_q_history_WB",
                    width=4,
                )
            ),
        )
        object.__setattr__(
            self,
            "truth_omega_history_B",
            _owned_read_only_copy(
                _validate_float_array(
                    self.truth_omega_history_B,
                    field_name="truth_omega_history_B",
                    width=3,
                )
            ),
        )
        object.__setattr__(
            self,
            "commanded_rotor_omega",
            _owned_read_only_copy(
                _validate_float_array(
                    self.commanded_rotor_omega,
                    field_name="commanded_rotor_omega",
                    width=4,
                )
            ),
        )

        object.__setattr__(
            self,
            "accelerometer_sequence_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.accelerometer_sequence_index,
                    field_name="accelerometer_sequence_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "accelerometer_observation_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.accelerometer_observation_index,
                    field_name="accelerometer_observation_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "accelerometer_acquisition_time_s",
            _owned_read_only_copy(
                _validate_float_array(
                    self.accelerometer_acquisition_time_s,
                    field_name="accelerometer_acquisition_time_s",
                    width=None,
                )
            ),
        )
        object.__setattr__(
            self,
            "accelerometer_delivery_time_s",
            _owned_read_only_copy(
                _validate_float_array(
                    self.accelerometer_delivery_time_s,
                    field_name="accelerometer_delivery_time_s",
                    width=None,
                )
            ),
        )
        object.__setattr__(
            self,
            "accelerometer_delivered_at_truth_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.accelerometer_delivered_at_truth_index,
                    field_name="accelerometer_delivered_at_truth_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "accelerometer_measurements_B",
            _owned_read_only_copy(
                _validate_float_array(
                    self.accelerometer_measurements_B,
                    field_name="accelerometer_measurements_B",
                    width=3,
                )
            ),
        )

        object.__setattr__(
            self,
            "gyroscope_sequence_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.gyroscope_sequence_index,
                    field_name="gyroscope_sequence_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "gyroscope_observation_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.gyroscope_observation_index,
                    field_name="gyroscope_observation_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "gyroscope_acquisition_time_s",
            _owned_read_only_copy(
                _validate_float_array(
                    self.gyroscope_acquisition_time_s,
                    field_name="gyroscope_acquisition_time_s",
                    width=None,
                )
            ),
        )
        object.__setattr__(
            self,
            "gyroscope_delivery_time_s",
            _owned_read_only_copy(
                _validate_float_array(
                    self.gyroscope_delivery_time_s,
                    field_name="gyroscope_delivery_time_s",
                    width=None,
                )
            ),
        )
        object.__setattr__(
            self,
            "gyroscope_delivered_at_truth_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.gyroscope_delivered_at_truth_index,
                    field_name="gyroscope_delivered_at_truth_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "gyroscope_measurements_B",
            _owned_read_only_copy(
                _validate_float_array(
                    self.gyroscope_measurements_B,
                    field_name="gyroscope_measurements_B",
                    width=3,
                )
            ),
        )

        object.__setattr__(
            self,
            "local_position_sequence_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.local_position_sequence_index,
                    field_name="local_position_sequence_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "local_position_observation_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.local_position_observation_index,
                    field_name="local_position_observation_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "local_position_acquisition_time_s",
            _owned_read_only_copy(
                _validate_float_array(
                    self.local_position_acquisition_time_s,
                    field_name="local_position_acquisition_time_s",
                    width=None,
                )
            ),
        )
        object.__setattr__(
            self,
            "local_position_delivery_time_s",
            _owned_read_only_copy(
                _validate_float_array(
                    self.local_position_delivery_time_s,
                    field_name="local_position_delivery_time_s",
                    width=None,
                )
            ),
        )
        object.__setattr__(
            self,
            "local_position_delivered_at_truth_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.local_position_delivered_at_truth_index,
                    field_name="local_position_delivered_at_truth_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "local_position_measurements_W",
            _owned_read_only_copy(
                _validate_float_array(
                    self.local_position_measurements_W,
                    field_name="local_position_measurements_W",
                    width=3,
                )
            ),
        )

        object.__setattr__(
            self,
            "barometric_altitude_sequence_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.barometric_altitude_sequence_index,
                    field_name="barometric_altitude_sequence_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "barometric_altitude_observation_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.barometric_altitude_observation_index,
                    field_name="barometric_altitude_observation_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "barometric_altitude_acquisition_time_s",
            _owned_read_only_copy(
                _validate_float_array(
                    self.barometric_altitude_acquisition_time_s,
                    field_name="barometric_altitude_acquisition_time_s",
                    width=None,
                )
            ),
        )
        object.__setattr__(
            self,
            "barometric_altitude_delivery_time_s",
            _owned_read_only_copy(
                _validate_float_array(
                    self.barometric_altitude_delivery_time_s,
                    field_name="barometric_altitude_delivery_time_s",
                    width=None,
                )
            ),
        )
        object.__setattr__(
            self,
            "barometric_altitude_delivered_at_truth_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.barometric_altitude_delivered_at_truth_index,
                    field_name="barometric_altitude_delivered_at_truth_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "barometric_altitude_measurements",
            _owned_read_only_copy(
                _validate_float_array(
                    self.barometric_altitude_measurements,
                    field_name="barometric_altitude_measurements",
                    width=None,
                )
            ),
        )

        object.__setattr__(
            self,
            "accelerometer_bias_history_B",
            _owned_read_only_copy(
                _validate_float_array(
                    self.accelerometer_bias_history_B,
                    field_name="accelerometer_bias_history_B",
                    width=3,
                )
            ),
        )
        object.__setattr__(
            self,
            "gyroscope_bias_history_B",
            _owned_read_only_copy(
                _validate_float_array(
                    self.gyroscope_bias_history_B,
                    field_name="gyroscope_bias_history_B",
                    width=3,
                )
            ),
        )

        object.__setattr__(
            self,
            "delivery_sensor_id",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.delivery_sensor_id,
                    field_name="delivery_sensor_id",
                )
            ),
        )
        object.__setattr__(
            self,
            "delivery_sequence_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.delivery_sequence_index,
                    field_name="delivery_sequence_index",
                )
            ),
        )
        object.__setattr__(
            self,
            "delivery_observation_index",
            _owned_read_only_copy(
                _validate_integer_array(
                    self.delivery_observation_index,
                    field_name="delivery_observation_index",
                )
            ),
        )

        _require_equal_leading_dimensions(
            (
                self.truth_time_s,
                self.truth_position_history_W,
                self.truth_velocity_history_W,
                self.truth_q_history_WB,
                self.truth_omega_history_B,
            ),
            error_message="truth history fields must have equal leading dimensions",
        )
        truth_rows = self.truth_time_s.shape[0]
        if self.accelerometer_bias_history_B.shape[0] != truth_rows:
            raise ValueError(
                f"accelerometer_bias_history_B must have {truth_rows} rows to match truth history"
            )
        if self.gyroscope_bias_history_B.shape[0] != truth_rows:
            raise ValueError(
                f"gyroscope_bias_history_B must have {truth_rows} rows to match truth history"
            )

        expected_command_rows = truth_rows - 1
        if self.commanded_rotor_omega.shape[0] != expected_command_rows:
            raise ValueError(
                "commanded_rotor_omega must have "
                f"{expected_command_rows} rows to match truth transitions"
            )

        _require_equal_leading_dimensions(
            (
                self.accelerometer_sequence_index,
                self.accelerometer_observation_index,
                self.accelerometer_acquisition_time_s,
                self.accelerometer_delivery_time_s,
                self.accelerometer_delivered_at_truth_index,
                self.accelerometer_measurements_B,
            ),
            error_message="accelerometer fields must have equal leading dimensions",
        )
        _require_equal_leading_dimensions(
            (
                self.gyroscope_sequence_index,
                self.gyroscope_observation_index,
                self.gyroscope_acquisition_time_s,
                self.gyroscope_delivery_time_s,
                self.gyroscope_delivered_at_truth_index,
                self.gyroscope_measurements_B,
            ),
            error_message="gyroscope fields must have equal leading dimensions",
        )
        _require_equal_leading_dimensions(
            (
                self.local_position_sequence_index,
                self.local_position_observation_index,
                self.local_position_acquisition_time_s,
                self.local_position_delivery_time_s,
                self.local_position_delivered_at_truth_index,
                self.local_position_measurements_W,
            ),
            error_message="local_position fields must have equal leading dimensions",
        )
        _require_equal_leading_dimensions(
            (
                self.barometric_altitude_sequence_index,
                self.barometric_altitude_observation_index,
                self.barometric_altitude_acquisition_time_s,
                self.barometric_altitude_delivery_time_s,
                self.barometric_altitude_delivered_at_truth_index,
                self.barometric_altitude_measurements,
            ),
            error_message="barometric_altitude fields must have equal leading dimensions",
        )
        _require_equal_leading_dimensions(
            (
                self.delivery_sensor_id,
                self.delivery_sequence_index,
                self.delivery_observation_index,
            ),
            error_message="delivery table fields must have equal leading dimensions",
        )

        _require_consecutive_zero_based_indices(
            "accelerometer_sequence_index", self.accelerometer_sequence_index
        )
        _require_consecutive_zero_based_indices(
            "accelerometer_observation_index", self.accelerometer_observation_index
        )
        _require_consecutive_zero_based_indices(
            "gyroscope_sequence_index", self.gyroscope_sequence_index
        )
        _require_consecutive_zero_based_indices(
            "gyroscope_observation_index", self.gyroscope_observation_index
        )
        _require_consecutive_zero_based_indices(
            "local_position_sequence_index", self.local_position_sequence_index
        )
        _require_consecutive_zero_based_indices(
            "local_position_observation_index", self.local_position_observation_index
        )
        _require_consecutive_zero_based_indices(
            "barometric_altitude_sequence_index", self.barometric_altitude_sequence_index
        )
        _require_consecutive_zero_based_indices(
            "barometric_altitude_observation_index", self.barometric_altitude_observation_index
        )

        _require_finite_values(
            "accelerometer_acquisition_time_s", self.accelerometer_acquisition_time_s
        )
        _require_finite_values("accelerometer_delivery_time_s", self.accelerometer_delivery_time_s)
        _require_finite_values("gyroscope_acquisition_time_s", self.gyroscope_acquisition_time_s)
        _require_finite_values("gyroscope_delivery_time_s", self.gyroscope_delivery_time_s)
        _require_finite_values(
            "local_position_acquisition_time_s", self.local_position_acquisition_time_s
        )
        _require_finite_values(
            "local_position_delivery_time_s", self.local_position_delivery_time_s
        )
        _require_finite_values(
            "barometric_altitude_acquisition_time_s", self.barometric_altitude_acquisition_time_s
        )
        _require_finite_values(
            "barometric_altitude_delivery_time_s", self.barometric_altitude_delivery_time_s
        )

        final_truth_index = self.truth_time_s.shape[0] - 1
        _require_delivered_truth_indices_in_range(
            "accelerometer_delivered_at_truth_index",
            self.accelerometer_delivered_at_truth_index,
            final_truth_index,
        )
        _require_delivered_truth_indices_in_range(
            "gyroscope_delivered_at_truth_index",
            self.gyroscope_delivered_at_truth_index,
            final_truth_index,
        )
        _require_delivered_truth_indices_in_range(
            "local_position_delivered_at_truth_index",
            self.local_position_delivered_at_truth_index,
            final_truth_index,
        )
        _require_delivered_truth_indices_in_range(
            "barometric_altitude_delivered_at_truth_index",
            self.barometric_altitude_delivered_at_truth_index,
            final_truth_index,
        )

        _require_stable_delivery_sensor_ids(self.delivery_sensor_id)
        expected_records: list[_DeliveredRecord] = []
        expected_records.extend(
            _delivered_stream_records(
                _DELIVERY_SENSOR_ID_ACCELEROMETER,
                self.accelerometer_sequence_index,
                self.accelerometer_observation_index,
                self.accelerometer_delivered_at_truth_index,
            )
        )
        expected_records.extend(
            _delivered_stream_records(
                _DELIVERY_SENSOR_ID_GYROSCOPE,
                self.gyroscope_sequence_index,
                self.gyroscope_observation_index,
                self.gyroscope_delivered_at_truth_index,
            )
        )
        expected_records.extend(
            _delivered_stream_records(
                _DELIVERY_SENSOR_ID_LOCAL_POSITION,
                self.local_position_sequence_index,
                self.local_position_observation_index,
                self.local_position_delivered_at_truth_index,
            )
        )
        expected_records.extend(
            _delivered_stream_records(
                _DELIVERY_SENSOR_ID_BAROMETRIC_ALTITUDE,
                self.barometric_altitude_sequence_index,
                self.barometric_altitude_observation_index,
                self.barometric_altitude_delivered_at_truth_index,
            )
        )

        actual_records: list[_GlobalDeliveryRecord] = [
            (int(sensor_id), int(sequence_index), int(observation_index))
            for sensor_id, sequence_index, observation_index in zip(
                self.delivery_sensor_id,
                self.delivery_sequence_index,
                self.delivery_observation_index,
                strict=True,
            )
        ]
        expected_members: list[_GlobalDeliveryRecord] = [
            (sensor_id, sequence_index, observation_index)
            for _, sensor_id, sequence_index, observation_index in expected_records
        ]
        if sorted(actual_records) != sorted(expected_members):
            raise ValueError(
                "delivery table must contain each delivered stream observation exactly once"
            )

        ordered_records = sorted(expected_records, key=lambda record: record[:3])
        expected_order: list[_GlobalDeliveryRecord] = [
            (sensor_id, sequence_index, observation_index)
            for _, sensor_id, sequence_index, observation_index in ordered_records
        ]
        if actual_records != expected_order:
            raise ValueError(
                "delivery table must be ordered by delivered truth index, "
                "sensor ID, and sequence index"
            )


def _require_configured_sensor_acquisition_count(
    stream_name: str,
    actual_count: int,
    schedule: SensorSchedule,
    number_of_steps: int,
    truth_time_step_s: float,
) -> None:
    """Require one stream's length to match its configured truth-grid acquisitions."""
    sample_stride = int(round(schedule.sample_period_s / truth_time_step_s))
    expected_count = number_of_steps // sample_stride
    if actual_count != expected_count:
        raise ValueError(f"{stream_name} stream length must equal the configured acquisition count")


def _require_configured_sensor_timestamps(
    stream_name: str,
    acquisition_time_s: NDArray[np.float64],
    delivery_time_s: NDArray[np.float64],
    schedule: SensorSchedule,
    number_of_steps: int,
    truth_time_step_s: float,
) -> None:
    """Require stored sensor times to match the configured acquisition schedule."""
    sample_stride = int(round(schedule.sample_period_s / truth_time_step_s))
    expected_count = number_of_steps // sample_stride
    expected_rows = np.arange(1, expected_count + 1, dtype=_INT64_DTYPE) * sample_stride
    expected_acquisition_time_s = expected_rows.astype(_FLOAT64_DTYPE) * truth_time_step_s
    expected_delivery_time_s = expected_acquisition_time_s + schedule.delivery_delay_s

    if not all(
        abs(float(actual) - float(expected))
        <= _time_comparison_tolerance_s(float(actual), float(expected))
        for actual, expected in zip(acquisition_time_s, expected_acquisition_time_s, strict=True)
    ):
        raise ValueError(
            f"{stream_name}_acquisition_time_s must equal the configured acquisition schedule"
        )
    if not all(
        abs(float(actual) - float(expected))
        <= _time_comparison_tolerance_s(float(actual), float(expected))
        for actual, expected in zip(delivery_time_s, expected_delivery_time_s, strict=True)
    ):
        raise ValueError(
            f"{stream_name}_delivery_time_s must equal configured acquisition times "
            "plus delivery delay"
        )


def _require_configured_delivery_truth_indices(
    field_name: str,
    delivered_at_truth_index: NDArray[np.int64],
    delivery_time_s: NDArray[np.float64],
    truth_time_s: NDArray[np.float64],
) -> None:
    """Require each delivery row to be the first reached truth update, or pending."""
    expected_indices = np.full(delivery_time_s.shape, -1, dtype=_INT64_DTYPE)
    for observation_index, scheduled_delivery_time_s in enumerate(delivery_time_s):
        for truth_index in range(1, truth_time_s.shape[0]):
            current_time_s = float(truth_time_s[truth_index])
            scheduled_time_s = float(scheduled_delivery_time_s)
            tolerance_s = _time_comparison_tolerance_s(scheduled_time_s, current_time_s)
            if scheduled_time_s <= current_time_s + tolerance_s:
                expected_indices[observation_index] = truth_index
                break

    if not np.array_equal(delivered_at_truth_index, expected_indices):
        raise ValueError(f"{field_name} must match configured delivery times and truth updates")


def _validate_run_artifact_against_manifest(manifest: RunManifest, data: RunArtifactData) -> None:
    """Require configured numerics and initial values to match artifact data."""
    configuration = manifest.run_configuration
    number_of_steps = configuration.numerics.number_of_steps
    truth_time_step_s = configuration.numerics.truth_time_step_s

    if data.truth_time_s.shape[0] != number_of_steps + 1:
        raise ValueError(
            "truth_time_s length must equal configuration.numerics.number_of_steps + 1"
        )
    if data.commanded_rotor_omega.shape[0] != number_of_steps:
        raise ValueError(
            "commanded_rotor_omega length must equal configuration.numerics.number_of_steps"
        )

    expected_truth_time_s = np.arange(number_of_steps + 1, dtype=_FLOAT64_DTYPE) * truth_time_step_s
    if not np.array_equal(data.truth_time_s, expected_truth_time_s):
        raise ValueError("truth_time_s must equal the configured truth-time grid")

    expected_command = np.broadcast_to(
        configuration.rotor_speed_input.rotor_omega, (number_of_steps, 4)
    )
    if data.commanded_rotor_omega.shape != expected_command.shape or not np.array_equal(
        data.commanded_rotor_omega, expected_command
    ):
        raise ValueError(
            "commanded_rotor_omega rows must equal configuration.rotor_speed_input.rotor_omega"
        )

    _require_configured_sensor_acquisition_count(
        "accelerometer",
        data.accelerometer_sequence_index.shape[0],
        configuration.sensor_schedules.accelerometer,
        number_of_steps,
        truth_time_step_s,
    )
    _require_configured_sensor_acquisition_count(
        "gyroscope",
        data.gyroscope_sequence_index.shape[0],
        configuration.sensor_schedules.gyroscope,
        number_of_steps,
        truth_time_step_s,
    )
    _require_configured_sensor_acquisition_count(
        "local_position",
        data.local_position_sequence_index.shape[0],
        configuration.sensor_schedules.local_position,
        number_of_steps,
        truth_time_step_s,
    )
    _require_configured_sensor_acquisition_count(
        "barometric_altitude",
        data.barometric_altitude_sequence_index.shape[0],
        configuration.sensor_schedules.barometric_altitude,
        number_of_steps,
        truth_time_step_s,
    )

    _require_configured_sensor_timestamps(
        "accelerometer",
        data.accelerometer_acquisition_time_s,
        data.accelerometer_delivery_time_s,
        configuration.sensor_schedules.accelerometer,
        number_of_steps,
        truth_time_step_s,
    )
    _require_configured_sensor_timestamps(
        "gyroscope",
        data.gyroscope_acquisition_time_s,
        data.gyroscope_delivery_time_s,
        configuration.sensor_schedules.gyroscope,
        number_of_steps,
        truth_time_step_s,
    )
    _require_configured_sensor_timestamps(
        "local_position",
        data.local_position_acquisition_time_s,
        data.local_position_delivery_time_s,
        configuration.sensor_schedules.local_position,
        number_of_steps,
        truth_time_step_s,
    )
    _require_configured_sensor_timestamps(
        "barometric_altitude",
        data.barometric_altitude_acquisition_time_s,
        data.barometric_altitude_delivery_time_s,
        configuration.sensor_schedules.barometric_altitude,
        number_of_steps,
        truth_time_step_s,
    )

    _require_configured_delivery_truth_indices(
        "accelerometer_delivered_at_truth_index",
        data.accelerometer_delivered_at_truth_index,
        data.accelerometer_delivery_time_s,
        data.truth_time_s,
    )
    _require_configured_delivery_truth_indices(
        "gyroscope_delivered_at_truth_index",
        data.gyroscope_delivered_at_truth_index,
        data.gyroscope_delivery_time_s,
        data.truth_time_s,
    )
    _require_configured_delivery_truth_indices(
        "local_position_delivered_at_truth_index",
        data.local_position_delivered_at_truth_index,
        data.local_position_delivery_time_s,
        data.truth_time_s,
    )
    _require_configured_delivery_truth_indices(
        "barometric_altitude_delivered_at_truth_index",
        data.barometric_altitude_delivered_at_truth_index,
        data.barometric_altitude_delivery_time_s,
        data.truth_time_s,
    )

    validate_rigid_body_state_history(
        data.truth_time_s,
        data.truth_position_history_W,
        data.truth_velocity_history_W,
        data.truth_q_history_WB,
        data.truth_omega_history_B,
    )
    # Recheck at persistence as well: read-only flags are an ownership contract,
    # not a security boundary against deliberate mutation of a NumPy buffer.
    for field in fields(data):
        values = getattr(data, field.name)
        if values.dtype == _FLOAT64_DTYPE:
            _require_finite_values(field.name, values)

    if not np.array_equal(
        data.truth_position_history_W[0], configuration.initial_truth_state.position_W
    ):
        raise ValueError(
            "truth_position_history_W row 0 must equal configuration.initial_truth_state.position_W"
        )
    if not np.array_equal(
        data.truth_velocity_history_W[0], configuration.initial_truth_state.velocity_W
    ):
        raise ValueError(
            "truth_velocity_history_W row 0 must equal configuration.initial_truth_state.velocity_W"
        )
    if not np.array_equal(data.truth_q_history_WB[0], configuration.initial_truth_state.q_WB):
        raise ValueError(
            "truth_q_history_WB row 0 must equal configuration.initial_truth_state.q_WB"
        )
    if not np.array_equal(data.truth_omega_history_B[0], configuration.initial_truth_state.omega_B):
        raise ValueError(
            "truth_omega_history_B row 0 must equal configuration.initial_truth_state.omega_B"
        )
    if not np.array_equal(
        data.accelerometer_bias_history_B[0],
        configuration.truth.imu.initial_accelerometer_bias_B,
    ):
        raise ValueError(
            "accelerometer_bias_history_B row 0 must equal "
            "configuration.truth.imu.initial_accelerometer_bias_B"
        )
    if not np.array_equal(
        data.gyroscope_bias_history_B[0], configuration.truth.imu.initial_gyroscope_bias_B
    ):
        raise ValueError(
            "gyroscope_bias_history_B row 0 must equal "
            "configuration.truth.imu.initial_gyroscope_bias_B"
        )


def _encode_data_npz(data: RunArtifactData) -> bytes:
    """Encode the stable 35-array payload as uncompressed in-memory NPZ bytes."""
    buffer = BytesIO()
    np.savez(
        buffer,
        truth_time_s=data.truth_time_s,
        truth_position_history_W=data.truth_position_history_W,
        truth_velocity_history_W=data.truth_velocity_history_W,
        truth_q_history_WB=data.truth_q_history_WB,
        truth_omega_history_B=data.truth_omega_history_B,
        commanded_rotor_omega=data.commanded_rotor_omega,
        accelerometer_sequence_index=data.accelerometer_sequence_index,
        accelerometer_observation_index=data.accelerometer_observation_index,
        accelerometer_acquisition_time_s=data.accelerometer_acquisition_time_s,
        accelerometer_delivery_time_s=data.accelerometer_delivery_time_s,
        accelerometer_delivered_at_truth_index=data.accelerometer_delivered_at_truth_index,
        accelerometer_measurements_B=data.accelerometer_measurements_B,
        gyroscope_sequence_index=data.gyroscope_sequence_index,
        gyroscope_observation_index=data.gyroscope_observation_index,
        gyroscope_acquisition_time_s=data.gyroscope_acquisition_time_s,
        gyroscope_delivery_time_s=data.gyroscope_delivery_time_s,
        gyroscope_delivered_at_truth_index=data.gyroscope_delivered_at_truth_index,
        gyroscope_measurements_B=data.gyroscope_measurements_B,
        local_position_sequence_index=data.local_position_sequence_index,
        local_position_observation_index=data.local_position_observation_index,
        local_position_acquisition_time_s=data.local_position_acquisition_time_s,
        local_position_delivery_time_s=data.local_position_delivery_time_s,
        local_position_delivered_at_truth_index=data.local_position_delivered_at_truth_index,
        local_position_measurements_W=data.local_position_measurements_W,
        barometric_altitude_sequence_index=data.barometric_altitude_sequence_index,
        barometric_altitude_observation_index=data.barometric_altitude_observation_index,
        barometric_altitude_acquisition_time_s=data.barometric_altitude_acquisition_time_s,
        barometric_altitude_delivery_time_s=data.barometric_altitude_delivery_time_s,
        barometric_altitude_delivered_at_truth_index=data.barometric_altitude_delivered_at_truth_index,
        barometric_altitude_measurements=data.barometric_altitude_measurements,
        accelerometer_bias_history_B=data.accelerometer_bias_history_B,
        gyroscope_bias_history_B=data.gyroscope_bias_history_B,
        delivery_sensor_id=data.delivery_sensor_id,
        delivery_sequence_index=data.delivery_sequence_index,
        delivery_observation_index=data.delivery_observation_index,
    )
    return buffer.getvalue()


def _decode_data_npz(payload: bytes) -> RunArtifactData:
    """Decode the trusted in-memory NPZ payload into owned run arrays."""
    with cast(np.lib.npyio.NpzFile, np.load(BytesIO(payload), allow_pickle=False)) as archive:
        if set(archive.files) != _DATA_NPZ_MEMBER_NAMES:
            raise ValueError("data.npz member names must match the run artifact schema exactly")
        return RunArtifactData(
            truth_time_s=archive["truth_time_s"],
            truth_position_history_W=archive["truth_position_history_W"],
            truth_velocity_history_W=archive["truth_velocity_history_W"],
            truth_q_history_WB=archive["truth_q_history_WB"],
            truth_omega_history_B=archive["truth_omega_history_B"],
            commanded_rotor_omega=archive["commanded_rotor_omega"],
            accelerometer_sequence_index=archive["accelerometer_sequence_index"],
            accelerometer_observation_index=archive["accelerometer_observation_index"],
            accelerometer_acquisition_time_s=archive["accelerometer_acquisition_time_s"],
            accelerometer_delivery_time_s=archive["accelerometer_delivery_time_s"],
            accelerometer_delivered_at_truth_index=archive[
                "accelerometer_delivered_at_truth_index"
            ],
            accelerometer_measurements_B=archive["accelerometer_measurements_B"],
            gyroscope_sequence_index=archive["gyroscope_sequence_index"],
            gyroscope_observation_index=archive["gyroscope_observation_index"],
            gyroscope_acquisition_time_s=archive["gyroscope_acquisition_time_s"],
            gyroscope_delivery_time_s=archive["gyroscope_delivery_time_s"],
            gyroscope_delivered_at_truth_index=archive["gyroscope_delivered_at_truth_index"],
            gyroscope_measurements_B=archive["gyroscope_measurements_B"],
            local_position_sequence_index=archive["local_position_sequence_index"],
            local_position_observation_index=archive["local_position_observation_index"],
            local_position_acquisition_time_s=archive["local_position_acquisition_time_s"],
            local_position_delivery_time_s=archive["local_position_delivery_time_s"],
            local_position_delivered_at_truth_index=archive[
                "local_position_delivered_at_truth_index"
            ],
            local_position_measurements_W=archive["local_position_measurements_W"],
            barometric_altitude_sequence_index=archive["barometric_altitude_sequence_index"],
            barometric_altitude_observation_index=archive["barometric_altitude_observation_index"],
            barometric_altitude_acquisition_time_s=archive[
                "barometric_altitude_acquisition_time_s"
            ],
            barometric_altitude_delivery_time_s=archive["barometric_altitude_delivery_time_s"],
            barometric_altitude_delivered_at_truth_index=archive[
                "barometric_altitude_delivered_at_truth_index"
            ],
            barometric_altitude_measurements=archive["barometric_altitude_measurements"],
            accelerometer_bias_history_B=archive["accelerometer_bias_history_B"],
            gyroscope_bias_history_B=archive["gyroscope_bias_history_B"],
            delivery_sensor_id=archive["delivery_sensor_id"],
            delivery_sequence_index=archive["delivery_sequence_index"],
            delivery_observation_index=archive["delivery_observation_index"],
        )


def _write_bytes_and_fsync(path: Path, payload: bytes) -> None:
    """Write a complete file and synchronize its contents to storage."""
    path.write_bytes(payload)
    file_descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(file_descriptor)
    finally:
        os.close(file_descriptor)


def _fsync_directory(directory: Path) -> None:
    """Synchronize directory entries to storage."""
    file_descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(file_descriptor)
    finally:
        os.close(file_descriptor)


def _publish_staging_directory(staging_directory: Path, run_directory: Path) -> None:
    """Atomically publish one directory without replacing an existing path."""
    libc = ctypes.CDLL(None, use_errno=True)
    try:
        renameat2 = libc.renameat2
    except AttributeError:
        raise RuntimeError(
            "atomic no-replace publication unavailable: libc has no renameat2"
        ) from None

    renameat2.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    renameat2.restype = ctypes.c_int
    result = renameat2(
        _AT_FDCWD,
        os.fsencode(staging_directory),
        _AT_FDCWD,
        os.fsencode(run_directory),
        _RENAME_NOREPLACE,
    )
    if result == 0:
        return

    error_number = ctypes.get_errno()
    if error_number == errno.EEXIST:
        raise FileExistsError(f"run directory already exists: {run_directory}")
    raise OSError(error_number, os.strerror(error_number), run_directory)


def save_run_directory(
    run_directory: Path,
    manifest: RunManifest,
    data: RunArtifactData,
) -> RunManifest:
    """Publish a completed data archive and bound manifest in a new run directory."""
    if run_directory.exists():
        raise FileExistsError(f"run directory already exists: {run_directory}")

    _validate_run_artifact_against_manifest(manifest, data)
    data_npz_bytes = _encode_data_npz(data)
    data_npz_sha256 = sha256(data_npz_bytes).hexdigest()
    bound_manifest = replace(manifest, data_npz_sha256=data_npz_sha256)
    manifest_bytes = encode_run_manifest(bound_manifest)

    staging_directory = Path(
        mkdtemp(prefix=f".{run_directory.name}.staging-", dir=run_directory.parent)
    )
    try:
        _write_bytes_and_fsync(staging_directory / "data.npz", data_npz_bytes)
        _write_bytes_and_fsync(staging_directory / "manifest.json", manifest_bytes)
        _fsync_directory(staging_directory)
        _publish_staging_directory(staging_directory, run_directory)
        _fsync_directory(run_directory.parent)
    finally:
        if staging_directory.exists():
            rmtree(staging_directory)
    return bound_manifest


def load_run_directory(run_directory: Path) -> tuple[RunManifest, RunArtifactData]:
    """Load a saved run after verifying the data archive's SHA-256 digest."""
    if not run_directory.is_dir():
        raise ValueError("run directory must be a directory")
    if {entry.name for entry in run_directory.iterdir()} != _RUN_DIRECTORY_ENTRY_NAMES:
        raise ValueError("run directory entries must be exactly: data.npz, manifest.json")

    manifest_bytes = (run_directory / "manifest.json").read_bytes()
    manifest = decode_run_manifest(manifest_bytes)
    expected_sha256 = manifest.data_npz_sha256
    if expected_sha256 is None:
        raise ValueError("manifest.json must bind data.npz")

    data_npz_bytes = (run_directory / "data.npz").read_bytes()
    actual_sha256 = sha256(data_npz_bytes).hexdigest()
    if actual_sha256 != expected_sha256:
        raise ValueError("data.npz SHA-256 does not match manifest")
    data = _decode_data_npz(data_npz_bytes)
    _validate_run_artifact_against_manifest(manifest, data)
    return manifest, data
