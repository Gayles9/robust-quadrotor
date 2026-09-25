"""Deterministic measurement-only ESKF execution with explicit epoch contracts.

This module has no access to plant truth, run configuration, scheduler or RNG.
See ADR 0006 for interval semantics and the deliberately limited stale-data policy.
"""

from dataclasses import dataclass, replace
from enum import Enum, IntEnum

import numpy as np
from numpy.typing import NDArray

from .eskf import (
    EskfMeasurementUpdate,
    EskfNominalState,
    _validated_float64_matrix,
    eskf_barometric_altitude_measurement_model,
    eskf_local_position_measurement_model,
    predict_eskf,
    update_eskf_barometric_altitude,
    update_eskf_local_position,
)
from .eskf_endpoint import (
    EskfEndpointState,
    EskfSampledImuNoise,
    initialize_eskf_endpoint,
    predict_eskf_endpoint,
    update_eskf_endpoint,
)
from .eskf_innovation import (
    EskfInnovation,
    EskfInnovationPolicy,
    _positive_nis_threshold,
    compute_eskf_linear_innovation,
)


class EskfObservationKind(IntEnum):
    """Sensor IDs and same-epoch fusion order, matching the run delivery table."""

    LOCAL_POSITION = 3
    BAROMETRIC_ALTITUDE = 4


class EskfReplayStatus(Enum):
    """One exhaustive replay disposition for every supplied observation."""

    FUSED = "fused"
    REJECTED = "rejected"
    STALE = "stale"
    DISABLED = "disabled"
    PENDING = "pending"


def _owned_array(
    name: str, values: NDArray[np.float64], shape: tuple[int, ...]
) -> NDArray[np.float64]:
    if not isinstance(values, np.ndarray) or values.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    if values.dtype.kind not in "fiu":
        raise ValueError(f"{name} must contain real numeric values")
    with np.errstate(over="ignore", invalid="ignore"):
        owned = np.array(values, dtype=np.float64, order="C", copy=True)
    if not np.all(np.isfinite(owned)):
        raise ValueError(f"{name} must contain only finite values")
    owned.flags.writeable = False
    return owned


def _owned_covariance(name: str, values: NDArray[np.float64], size: int) -> NDArray[np.float64]:
    checked = _owned_array(name, values, (size, size))
    owned = _validated_float64_matrix(
        name, checked, (size, size), symmetric_positive_semidefinite=True
    )
    owned.flags.writeable = False
    return owned


def _scalar(name: str, value: float, *, nonnegative: bool = False) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError(f"{name} must be a finite real scalar")
    try:
        scalar = float(value)
    except OverflowError:
        raise ValueError(f"{name} must be finite") from None
    if not np.isfinite(scalar) or (nonnegative and scalar < 0.0):
        raise ValueError(f"{name} must be finite" + (" and nonnegative" if nonnegative else ""))
    return scalar


def _time_vector(time_s: NDArray[np.float64]) -> NDArray[np.float64]:
    if not isinstance(time_s, np.ndarray) or time_s.ndim != 1 or not time_s.size:
        raise ValueError("time_s must be a nonempty vector")
    owned = _owned_array("time_s", time_s, (time_s.size,))
    if owned[0] < 0.0 or np.any(owned[1:] <= owned[:-1]):
        raise ValueError("time_s must be nonnegative and strictly increasing")
    return owned


@dataclass(frozen=True, slots=True, eq=False)
class EskfReplayObservation:
    """One measured value at an acquisition epoch and its actual delivery epoch.

    Indices are zero-based into the replay time vector, not truth-history rows.
    ``delivery_index == -1`` means pending beyond the replay horizon.
    ``observation_index`` identifies the source row within this sensor stream.
    Measurement is (3,) NED position or (1,) positive-up altitude, in metres.
    """

    kind: EskfObservationKind
    observation_index: int
    acquisition_index: int
    delivery_index: int
    measurement: NDArray[np.float64]

    def __post_init__(self) -> None:
        if not isinstance(self.kind, EskfObservationKind):
            raise TypeError("kind must be an EskfObservationKind")
        for name, value, minimum in (
            ("observation_index", self.observation_index, 0),
            ("acquisition_index", self.acquisition_index, 0),
            ("delivery_index", self.delivery_index, -1),
        ):
            if type(value) is not int or value < minimum:
                raise ValueError(f"{name} must be a non-Boolean integer >= {minimum}")
        if self.delivery_index != -1 and self.delivery_index < self.acquisition_index:
            raise ValueError("delivery_index cannot precede acquisition_index")
        width = 3 if self.kind is EskfObservationKind.LOCAL_POSITION else 1
        object.__setattr__(
            self, "measurement", _owned_array("measurement", self.measurement, (width,))
        )


def _owned_observations(
    observations: tuple[EskfReplayObservation, ...], n: int
) -> tuple[EskfReplayObservation, ...]:
    """Validate source identity and timing, then return owned canonical order."""
    owned: list[EskfReplayObservation] = []
    for observation in observations:
        if not isinstance(observation, EskfReplayObservation):
            raise TypeError("observations must contain EskfReplayObservation values")
        if observation.acquisition_index >= n or observation.delivery_index >= n:
            raise ValueError("observation epoch index outside time_s")
        owned.append(replace(observation))
    for kind in EskfObservationKind:
        stream = sorted((o for o in owned if o.kind is kind), key=lambda o: o.observation_index)
        if [o.observation_index for o in stream] != list(range(len(stream))):
            raise ValueError("observation_index must be consecutive and unique within each sensor")
        if any(
            b.acquisition_index <= a.acquisition_index
            for a, b in zip(stream, stream[1:], strict=False)
        ):
            raise ValueError("sensor acquisition indices must be strictly increasing")
    owned.sort(
        key=lambda o: (
            n if o.delivery_index == -1 else o.delivery_index,
            o.kind.value,
            o.observation_index,
        )
    )
    return tuple(owned)


@dataclass(frozen=True, slots=True, eq=False)
class EskfReplayInput:
    """Own paired IMU samples and independent position/altitude observations.

    IMU rows (n,3) are simultaneous and available at their time_s row. Specific
    force is FRD m/s²; angular velocity is FRD rad/s. By default row k is left-held
    over [time_s[k], time_s[k+1]). With explicit sampled_imu_noise, adjacent rows
    are instantaneous endpoints and the final row closes the final interval.
    All acquisitions must lie in this horizon. Each sensor's source row IDs
    must be consecutive from zero and acquisitions strictly increasing.
    """

    time_s: NDArray[np.float64]
    specific_force_measurements_B: NDArray[np.float64]
    angular_velocity_measurements_B: NDArray[np.float64]
    observations: tuple[EskfReplayObservation, ...] = ()

    def __post_init__(self) -> None:
        times = _time_vector(self.time_s)
        n = times.size
        for name in ("specific_force_measurements_B", "angular_velocity_measurements_B"):
            object.__setattr__(self, name, _owned_array(name, getattr(self, name), (n, 3)))
        owned = _owned_observations(self.observations, n)
        object.__setattr__(self, "time_s", times)
        object.__setattr__(self, "observations", owned)


@dataclass(frozen=True, slots=True, eq=False)
class EskfReplayConfiguration:
    """Explicit estimator assumptions, independent of simulated truth.

    The prior is at initial_time_s, exactly equal to the input's first epoch.
    P is (15,15); continuous Q_c is (12,12) in ADR 0004 noise order and spectral
    density units. Position R (3,3) and altitude variance are per-observation m².
    Biases and altitude datum are assumed metres; gravity is positive NED m/s².
    Fusion flags disable fresh updates without disabling observation validation.
    innovation_policy=None preserves unscored execution. An explicit policy
    records pre-update NIS and optionally gates each fresh enabled sensor.
    sampled_imu_noise selects endpoint propagation (ADR 0010); the unused
    continuous_noise_covariance must then be zero. The prior must be independent
    of the first IMU sample noise. The physical output remains 15-dimensional.
    """

    initial_time_s: float
    initial_state: EskfNominalState
    initial_covariance: NDArray[np.float64]
    gravity_acceleration: float
    continuous_noise_covariance: NDArray[np.float64]
    local_position_bias_W: NDArray[np.float64]
    local_position_noise_covariance_W: NDArray[np.float64]
    barometric_reference_altitude: float
    barometric_altitude_bias: float
    barometric_altitude_noise_variance: float
    fuse_local_position: bool = True
    fuse_barometric_altitude: bool = True
    innovation_policy: EskfInnovationPolicy | None = None
    sampled_imu_noise: EskfSampledImuNoise | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.initial_state, EskfNominalState):
            raise TypeError("initial_state must be an EskfNominalState")
        if self.innovation_policy is not None:
            if not isinstance(self.innovation_policy, EskfInnovationPolicy):
                raise TypeError("innovation_policy must be an EskfInnovationPolicy or None")
            object.__setattr__(self, "innovation_policy", replace(self.innovation_policy))
        for name in ("fuse_local_position", "fuse_barometric_altitude"):
            if type(getattr(self, name)) is not bool:
                raise TypeError(f"{name} must be a bool")
        for name, nonnegative in (
            ("initial_time_s", True),
            ("gravity_acceleration", True),
            ("barometric_reference_altitude", False),
            ("barometric_altitude_bias", False),
            ("barometric_altitude_noise_variance", True),
        ):
            object.__setattr__(
                self, name, _scalar(name, getattr(self, name), nonnegative=nonnegative)
            )
        if self.gravity_acceleration == 0.0:
            raise ValueError("gravity_acceleration must be positive")
        for name, size in (
            ("initial_covariance", 15),
            ("continuous_noise_covariance", 12),
            ("local_position_noise_covariance_W", 3),
        ):
            object.__setattr__(self, name, _owned_covariance(name, getattr(self, name), size))
        if self.sampled_imu_noise is not None:
            if not isinstance(self.sampled_imu_noise, EskfSampledImuNoise):
                raise TypeError("sampled_imu_noise must be an EskfSampledImuNoise or None")
            if np.any(self.continuous_noise_covariance):
                raise ValueError("continuous_noise_covariance must be zero with sampled_imu_noise")
            object.__setattr__(self, "sampled_imu_noise", replace(self.sampled_imu_noise))
        object.__setattr__(
            self,
            "local_position_bias_W",
            _owned_array("local_position_bias_W", self.local_position_bias_W, (3,)),
        )
        object.__setattr__(self, "initial_state", replace(self.initial_state))


@dataclass(frozen=True, slots=True, eq=False)
class EskfReplayEvent:
    """One observation disposition and independently owned optional diagnostics.

    Only FUSED carries an update. With scoring enabled, FUSED and REJECTED carry
    pre-correction innovation diagnostics; REJECTED requires NIS > nis_threshold.
    Legacy unscored FUSED events remain valid. Other dispositions are unscored.
    """

    observation: EskfReplayObservation
    status: EskfReplayStatus
    update: EskfMeasurementUpdate | None = None
    innovation: EskfInnovation | None = None
    nis_threshold: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.observation, EskfReplayObservation):
            raise TypeError("observation must be an EskfReplayObservation")
        if not isinstance(self.status, EskfReplayStatus):
            raise TypeError("status must be an EskfReplayStatus")
        observation = self.observation
        if (self.status is EskfReplayStatus.PENDING) != (observation.delivery_index == -1):
            raise ValueError("PENDING status must match pending delivery")
        if observation.delivery_index != -1:
            stale = observation.delivery_index > observation.acquisition_index
            if (self.status is EskfReplayStatus.STALE) != stale:
                raise ValueError("STALE status must match an older acquisition epoch")
        if self.status is EskfReplayStatus.FUSED:
            if not isinstance(self.update, EskfMeasurementUpdate):
                raise ValueError("FUSED event requires an EskfMeasurementUpdate")
            if self.update.innovation.shape != observation.measurement.shape:
                raise ValueError("update dimension must match observation")
            object.__setattr__(self, "update", replace(self.update))
        elif self.update is not None:
            raise ValueError("unfused event cannot contain an update")
        if self.nis_threshold is not None:
            object.__setattr__(
                self, "nis_threshold", _positive_nis_threshold("nis_threshold", self.nis_threshold)
            )
        if self.innovation is not None:
            if not isinstance(self.innovation, EskfInnovation):
                raise TypeError("innovation must be an EskfInnovation or None")
            if self.status not in (EskfReplayStatus.FUSED, EskfReplayStatus.REJECTED):
                raise ValueError("only FUSED or REJECTED events can contain innovation diagnostics")
            diagnostic = replace(self.innovation)
            if diagnostic.innovation.shape != observation.measurement.shape:
                raise ValueError("innovation dimension must match observation")
            if self.update is not None and not (
                np.array_equal(diagnostic.innovation, self.update.innovation)
                and np.array_equal(
                    diagnostic.innovation_covariance, self.update.innovation_covariance
                )
            ):
                raise ValueError("innovation diagnostics must match the update")
            rejected = (
                self.nis_threshold is not None
                and diagnostic.normalized_innovation_squared > self.nis_threshold
            )
            if (self.status is EskfReplayStatus.REJECTED) != rejected:
                raise ValueError("REJECTED status requires NIS strictly above its threshold")
            object.__setattr__(self, "innovation", diagnostic)
        elif self.nis_threshold is not None or self.status is EskfReplayStatus.REJECTED:
            raise ValueError("a gate threshold or REJECTED status requires innovation diagnostics")
        object.__setattr__(self, "observation", replace(observation))


@dataclass(frozen=True, slots=True, eq=False)
class EskfReplayResult:
    """Post-update state and covariance at each epoch, with all event dispositions.

    History covariances have shape (n,15,15). Full per-fusion diagnostics retain
    the state after that individual update, before any later same-epoch update.
    Delivered events are chronological; pending events follow them. This is an
    in-memory result, not a new persisted run-artifact schema.
    """

    time_s: NDArray[np.float64]
    states: tuple[EskfNominalState, ...]
    covariances: NDArray[np.float64]
    events: tuple[EskfReplayEvent, ...]

    def __post_init__(self) -> None:
        times = _time_vector(self.time_s)
        if len(self.states) != times.size or not all(
            isinstance(s, EskfNominalState) for s in self.states
        ):
            raise ValueError("states must contain one EskfNominalState per epoch")
        covariances = np.array(
            _owned_array("covariances", self.covariances, (times.size, 15, 15)), copy=True
        )
        for index, covariance in enumerate(covariances):
            covariances[index] = _owned_covariance("covariances", covariance, 15)
        covariances.flags.writeable = False
        events: list[EskfReplayEvent] = []
        for event in self.events:
            if not isinstance(event, EskfReplayEvent):
                raise TypeError("events must contain EskfReplayEvent values")
            observation = event.observation
            if (
                observation.acquisition_index >= times.size
                or observation.delivery_index >= times.size
            ):
                raise ValueError("event epoch index outside time_s")
            events.append(replace(event))
        checked = _owned_observations(tuple(e.observation for e in events), times.size)
        if [(e.observation.kind, e.observation.observation_index) for e in events] != [
            (o.kind, o.observation_index) for o in checked
        ]:
            raise ValueError("events must follow canonical delivery order")
        object.__setattr__(self, "time_s", times)
        object.__setattr__(self, "states", tuple(replace(s) for s in self.states))
        object.__setattr__(self, "covariances", covariances)
        object.__setattr__(self, "events", tuple(events))


def _correct_eskf_epoch(
    state: EskfNominalState,
    covariance: NDArray[np.float64],
    endpoint: EskfEndpointState | None,
    observations: tuple[EskfReplayObservation, ...],
    index: int,
    configuration: EskfReplayConfiguration,
) -> tuple[
    EskfNominalState,
    NDArray[np.float64],
    EskfEndpointState | None,
    tuple[EskfReplayEvent, ...],
]:
    """Shared ordered correction; replay/online callers validate delivery identities."""
    events: list[EskfReplayEvent] = []
    for observation in observations:
        if observation.acquisition_index != index:
            events.append(EskfReplayEvent(observation, EskfReplayStatus.STALE))
            continue
        position = observation.kind is EskfObservationKind.LOCAL_POSITION
        enabled = (
            configuration.fuse_local_position
            if position
            else configuration.fuse_barometric_altitude
        )
        if not enabled:
            events.append(EskfReplayEvent(observation, EskfReplayStatus.DISABLED))
            continue
        innovation = None
        threshold = None
        if configuration.innovation_policy is not None:
            policy = configuration.innovation_policy
            if position:
                predicted, jacobian = eskf_local_position_measurement_model(
                    state, configuration.local_position_bias_W
                )
                noise = configuration.local_position_noise_covariance_W
                threshold = policy.local_position_nis_threshold
            else:
                predicted, jacobian = eskf_barometric_altitude_measurement_model(
                    state,
                    configuration.barometric_reference_altitude,
                    configuration.barometric_altitude_bias,
                )
                noise = np.array([[configuration.barometric_altitude_noise_variance]])
                threshold = policy.barometric_altitude_nis_threshold
            innovation = compute_eskf_linear_innovation(
                covariance, observation.measurement, predicted, jacobian, noise
            )
            if threshold is not None and innovation.normalized_innovation_squared > threshold:
                events.append(
                    EskfReplayEvent(
                        observation,
                        EskfReplayStatus.REJECTED,
                        innovation=innovation,
                        nis_threshold=threshold,
                    )
                )
                continue
        if endpoint is not None:
            if position:
                predicted, jacobian = eskf_local_position_measurement_model(
                    state, configuration.local_position_bias_W
                )
                noise = configuration.local_position_noise_covariance_W
            else:
                predicted, jacobian = eskf_barometric_altitude_measurement_model(
                    state,
                    configuration.barometric_reference_altitude,
                    configuration.barometric_altitude_bias,
                )
                noise = np.array([[configuration.barometric_altitude_noise_variance]])
            endpoint, update = update_eskf_endpoint(
                endpoint, observation.measurement, predicted, jacobian, noise
            )
        elif position:
            update = update_eskf_local_position(
                state,
                covariance,
                observation.measurement,
                configuration.local_position_noise_covariance_W,
                configuration.local_position_bias_W,
            )
        else:
            update = update_eskf_barometric_altitude(
                state,
                covariance,
                float(observation.measurement[0]),
                configuration.barometric_altitude_noise_variance,
                configuration.barometric_reference_altitude,
                configuration.barometric_altitude_bias,
            )
        state, covariance = update.nominal_state, update.covariance
        events.append(
            EskfReplayEvent(observation, EskfReplayStatus.FUSED, update, innovation, threshold)
        )
    return state, covariance, endpoint, tuple(events)


def replay_eskf(data: EskfReplayInput, configuration: EskfReplayConfiguration) -> EskfReplayResult:
    """Execute a causal replay with the explicitly configured IMU propagation.

    No prediction precedes the first epoch. Later epochs predict using row k-1
    (default) or endpoints k-1/k (sampled_imu_noise), then fuse fresh position
    followed by altitude. Older acquisitions are STALE;
    undelivered observations are PENDING. A disabled fresh update is DISABLED.
    An explicit innovation policy scores fresh enabled observations before any
    correction; NIS strictly above its threshold is REJECTED without changing
    state/covariance. Each later sensor uses the actual preceding posterior.
    Invalid/unsolvable arithmetic raises ValueError, not a rejection event. No
    partial result is returned on failure and no external state/RNG is changed.
    """
    if not isinstance(data, EskfReplayInput) or not isinstance(
        configuration, EskfReplayConfiguration
    ):
        raise TypeError("replay requires EskfReplayInput and EskfReplayConfiguration")
    if configuration.initial_time_s != float(data.time_s[0]):
        raise ValueError("initial_time_s must exactly equal the first input epoch")
    state = replace(configuration.initial_state)
    covariance = configuration.initial_covariance
    endpoint = (
        initialize_eskf_endpoint(state, covariance, configuration.sampled_imu_noise)
        if configuration.sampled_imu_noise is not None
        else None
    )
    states: list[EskfNominalState] = []
    covariances = np.empty((data.time_s.size, 15, 15), dtype=np.float64)
    events: list[EskfReplayEvent] = []
    delivered: dict[int, list[EskfReplayObservation]] = {}
    for observation in data.observations:
        delivered.setdefault(observation.delivery_index, []).append(observation)
    for index, time_s in enumerate(data.time_s):
        if index and endpoint is not None and configuration.sampled_imu_noise is not None:
            endpoint = predict_eskf_endpoint(
                endpoint,
                data.specific_force_measurements_B[index - 1],
                data.angular_velocity_measurements_B[index - 1],
                data.specific_force_measurements_B[index],
                data.angular_velocity_measurements_B[index],
                configuration.gravity_acceleration,
                configuration.sampled_imu_noise,
                float(time_s - data.time_s[index - 1]),
            )
            state, covariance = endpoint.nominal_state, endpoint.joint_covariance[:15, :15]
        elif index:
            state, covariance = predict_eskf(
                state,
                covariance,
                data.specific_force_measurements_B[index - 1],
                data.angular_velocity_measurements_B[index - 1],
                configuration.gravity_acceleration,
                configuration.continuous_noise_covariance,
                float(time_s - data.time_s[index - 1]),
            )
        state, covariance, endpoint, epoch_events = _correct_eskf_epoch(
            state, covariance, endpoint, tuple(delivered.get(index, ())), index, configuration
        )
        events.extend(epoch_events)
        states.append(state)
        covariances[index] = covariance
    events.extend(EskfReplayEvent(o, EskfReplayStatus.PENDING) for o in delivered.get(-1, ()))
    return EskfReplayResult(data.time_s, tuple(states), covariances, tuple(events))
