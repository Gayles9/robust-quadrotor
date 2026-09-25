"""Lossless, pickle-free histories for the estimated-feedback experiment only."""

import hashlib
import io
import json
from dataclasses import asdict, fields
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import canonical_json
from experiments.position_control_validation import _unique_object
from quadrotor_math.eskf import EskfMeasurementUpdate, EskfNominalState
from quadrotor_math.eskf_innovation import EskfInnovation
from quadrotor_math.eskf_replay import (
    EskfObservationKind,
    EskfReplayEvent,
    EskfReplayInput,
    EskfReplayObservation,
    EskfReplayResult,
    EskfReplayStatus,
)
from quadrotor_math.estimated_mission import EstimatedMissionResult
from quadrotor_math.mission_simulation import MissionResult

HISTORY_CHUNK_ROWS = 4096


def plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def pack_history(result: EstimatedMissionResult, baseline: MissionResult) -> dict[str, Any]:
    """Keep full numerical histories and all event/update/innovation diagnostics."""
    arrays = {}
    for prefix, mission in (("mission_", result.mission), ("baseline_", baseline)):
        for field in fields(MissionResult):
            if field.name != "abort_reason":
                arrays[prefix + field.name] = getattr(mission, field.name)
    for field in fields(EskfNominalState):
        arrays["estimate_" + field.name] = np.array(
            [getattr(s, field.name) for s in result.estimates.states]
        )
    arrays["covariances"] = result.estimates.covariances
    arrays["specific_force_measurements_B"] = result.measurements.specific_force_measurements_B
    arrays["angular_velocity_measurements_B"] = result.measurements.angular_velocity_measurements_B
    for field in fields(EstimatedMissionResult):
        if field.name not in ("mission", "measurements", "estimates"):
            arrays[field.name] = getattr(result, field.name)
    metadata = {
        "mission_abort_reason": result.mission.abort_reason,
        "baseline_abort_reason": baseline.abort_reason,
        "events": [plain(asdict(e)) for e in result.estimates.events],
    }
    arrays["metadata_json"] = np.frombuffer(canonical_json(metadata), dtype=np.uint8).copy()
    return arrays


def _state(value: dict[str, Any]) -> EskfNominalState:
    return EskfNominalState(**{key: np.array(item) for key, item in value.items()})


def _event(value: dict[str, Any]) -> EskfReplayEvent:
    obs = value["observation"]
    observation = EskfReplayObservation(
        EskfObservationKind(obs["kind"]),
        obs["observation_index"],
        obs["acquisition_index"],
        obs["delivery_index"],
        np.array(obs["measurement"]),
    )
    update = None
    if value["update"] is not None:
        data = value["update"]
        update = EskfMeasurementUpdate(
            nominal_state=_state(data["nominal_state"]),
            **{key: np.array(item) for key, item in data.items() if key != "nominal_state"},
        )
    innovation = None
    if value["innovation"] is not None:
        data = value["innovation"]
        innovation = EskfInnovation(
            np.array(data["innovation"]), np.array(data["innovation_covariance"])
        )
        if canonical_json(plain(asdict(innovation))) != canonical_json(data):
            raise ValueError("stored innovation diagnostics do not match their defining data")
    result = EskfReplayEvent(
        observation, EskfReplayStatus(value["status"]), update, innovation, value["nis_threshold"]
    )
    if canonical_json(plain(asdict(result))) != canonical_json(value):
        raise ValueError("event fields do not match canonical schema")
    return result


def unpack_history(arrays: dict[str, Any]) -> tuple[EstimatedMissionResult, MissionResult]:
    """Reconstruct all public contracts; reject extra, missing or lossy fields."""
    raw = arrays["metadata_json"]
    if not isinstance(raw, np.ndarray) or raw.ndim != 1 or raw.dtype != np.uint8:
        raise ValueError("metadata_json must be a uint8 vector")
    metadata = json.loads(raw.tobytes(), object_pairs_hook=_unique_object)
    canonical_json(metadata)

    def mission(prefix: str) -> MissionResult:
        return MissionResult(
            **{
                f.name: metadata[prefix + f.name]
                if f.name == "abort_reason"
                else arrays[prefix + f.name]
                for f in fields(MissionResult)
            }
        )

    current, baseline = mission("mission_"), mission("baseline_")
    events = tuple(_event(e) for e in metadata["events"])
    data = EskfReplayInput(
        current.time_s,
        arrays["specific_force_measurements_B"],
        arrays["angular_velocity_measurements_B"],
        tuple(e.observation for e in events),
    )
    states = tuple(
        EskfNominalState(
            **{f.name: arrays["estimate_" + f.name][k] for f in fields(EskfNominalState)}
        )
        for k in range(len(current.time_s))
    )
    estimates = EskfReplayResult(current.time_s, states, arrays["covariances"], events)
    result = EstimatedMissionResult(
        current,
        data,
        estimates,
        **{
            f.name: arrays[f.name]
            for f in fields(EstimatedMissionResult)
            if f.name not in ("mission", "measurements", "estimates")
        },
    )
    canonical = pack_history(result, baseline)
    if arrays.keys() != canonical.keys() or any(
        arrays[key].dtype != canonical[key].dtype or not np.array_equal(arrays[key], canonical[key])
        for key in arrays
    ):
        raise ValueError("history arrays must exactly match the canonical schema and dtypes")
    return result, baseline


def save_history(directory: Path, index: int, arrays: dict[str, Any]) -> list[dict[str, str]]:
    """Separate the large full-rate covariance into bounded, lossless NPZ chunks."""
    covariance = arrays["covariances"]
    parts: list[tuple[str, dict[str, Any]]] = [
        (f"trial-{index:03d}-data.npz", {k: v for k, v in arrays.items() if k != "covariances"})
    ]
    parts.extend(
        (
            f"trial-{index:03d}-cov-{j:03d}.npz",
            {"covariances": covariance[start : start + HISTORY_CHUNK_ROWS]},
        )
        for j, start in enumerate(range(0, len(covariance), HISTORY_CHUNK_ROWS))
    )
    records = []
    for name, values in parts:
        path = directory / name
        with path.open("xb") as stream:
            np.savez_compressed(stream, **values)
        records.append({"file": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    return records


def load_history(directory: Path, index: int, records: list[dict[str, str]]) -> dict[str, Any]:
    """Check exact byte digests before decoding those same bytes, without pickle."""
    if not isinstance(records, list) or len(records) < 2:
        raise ValueError("history requires data and at least one covariance chunk")
    arrays: dict[str, Any] = {}
    chunks: list[NDArray[np.float64]] = []
    for j, record in enumerate(records):
        expected = (
            f"trial-{index:03d}-data.npz" if j == 0 else f"trial-{index:03d}-cov-{j - 1:03d}.npz"
        )
        if set(record) != {"file", "sha256"} or record["file"] != expected:
            raise ValueError("unexpected history filename or fields")
        data = (directory / expected).read_bytes()
        if hashlib.sha256(data).hexdigest() != record["sha256"]:
            raise ValueError("history byte digest mismatch")
        with np.load(io.BytesIO(data), allow_pickle=False) as archive:
            if len(archive.files) != len(set(archive.files)):
                raise ValueError("duplicate NPZ array")
            loaded = {key: archive[key] for key in archive.files}
        if j == 0:
            arrays = loaded
            if "covariances" in arrays:
                raise ValueError("covariances belong only in the declared covariance chunks")
            n = len(arrays["mission_time_s"])
            if len(records) != 1 + (n + HISTORY_CHUNK_ROWS - 1) // HISTORY_CHUNK_ROWS:
                raise ValueError("wrong covariance chunk count")
        else:
            expected_rows = min(HISTORY_CHUNK_ROWS, n - (j - 1) * HISTORY_CHUNK_ROWS)
            if set(loaded) != {"covariances"} or loaded["covariances"].shape != (
                expected_rows,
                15,
                15,
            ):
                raise ValueError("wrong covariance chunk shape")
            chunks.append(loaded["covariances"])
    arrays["covariances"] = np.concatenate(chunks)
    return arrays
