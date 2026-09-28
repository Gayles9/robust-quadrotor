"""Causal, passive availability diagnostics for accepted ESKF observations (ADR 0021)."""

from dataclasses import dataclass, replace
from enum import Enum
from math import isfinite

from .eskf_replay import EskfObservationKind, EskfReplayEvent, EskfReplayStatus, _scalar


class ObservationHealthState(Enum):
    """Availability of one observation stream, not estimator accuracy."""

    WAITING = "waiting"
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    LOST = "lost"
    RECOVERING = "recovering"
    DISABLED = "disabled"


class ObservationHealthReason(Enum):
    STARTUP = "startup"
    FRESH = "fresh"
    AGE_WARNING = "age_warning"
    AGE_LOSS = "age_loss"
    REJECTION_RUN = "rejection_run"
    RECOVERY = "recovery"
    DISABLED = "disabled"


@dataclass(frozen=True, slots=True)
class ObservationHealthPolicy:
    """Explicit schedule-based thresholds for one sensor, in elapsed seconds.

    Warning/loss ages are period_count * sample_period + delivery_delay +
    check_interval. Equality stays within the threshold. Recent evidence is
    the last evidence_window_size deliveries, not a rolling time window.
    These settings do not establish a safe-flight deadline.
    """

    sample_period_s: float
    delivery_delay_s: float
    check_interval_s: float
    warning_periods: int
    lost_periods: int
    rejection_limit: int
    recovery_acceptances: int
    evidence_window_size: int
    enabled: bool = True

    def __post_init__(self) -> None:
        for name in ("sample_period_s", "delivery_delay_s", "check_interval_s"):
            value = _scalar(name, getattr(self, name), nonnegative=True)
            if name != "delivery_delay_s" and value == 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        for name in (
            "warning_periods",
            "lost_periods",
            "rejection_limit",
            "recovery_acceptances",
            "evidence_window_size",
        ):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"{name} must be a positive non-Boolean integer")
        if self.lost_periods <= self.warning_periods:
            raise ValueError("lost_periods must exceed warning_periods")
        if self.recovery_acceptances < 2:
            raise ValueError("recovery_acceptances must be at least two")
        if self.evidence_window_size < max(self.rejection_limit, self.recovery_acceptances):
            raise ValueError("evidence window must include rejection and recovery counts")
        if type(self.enabled) is not bool:
            raise TypeError("enabled must be bool")
        try:
            warning, loss = self.warning_age_s, self.lost_age_s
        except OverflowError:
            raise ValueError("derived health ages must be finite") from None
        if not (isfinite(warning) and isfinite(loss) and 0 < warning < loss):
            raise ValueError("derived health ages must be finite and strictly ordered")

    @property
    def warning_age_s(self) -> float:
        return (
            self.warning_periods * self.sample_period_s
            + self.delivery_delay_s
            + self.check_interval_s
        )

    @property
    def lost_age_s(self) -> float:
        return (
            self.lost_periods * self.sample_period_s + self.delivery_delay_s + self.check_interval_s
        )


@dataclass(frozen=True, slots=True)
class ObservationHealthConfiguration:
    local_position: ObservationHealthPolicy
    barometric_altitude: ObservationHealthPolicy

    def __post_init__(self) -> None:
        for name in ("local_position", "barometric_altitude"):
            policy = getattr(self, name)
            if not isinstance(policy, ObservationHealthPolicy):
                raise TypeError(f"{name} must be ObservationHealthPolicy")
            object.__setattr__(self, name, replace(policy))


@dataclass(frozen=True, slots=True)
class ObservationStreamHealth:
    """Scalar-only output record; silence changes age, not delivery counts."""

    kind: EskfObservationKind
    state: ObservationHealthState
    reason: ObservationHealthReason
    last_accepted_acquisition_time_s: float | None = None
    last_accepted_delivery_time_s: float | None = None
    accepted_age_s: float | None = None
    last_acquisition_time_s: float | None = None
    last_delivery_time_s: float | None = None
    consecutive_acceptances: int = 0
    consecutive_rejections: int = 0
    recent_statuses: tuple[EskfReplayStatus, ...] = ()

    @property
    def recent_accepted_count(self) -> int:
        return self.recent_statuses.count(EskfReplayStatus.FUSED)

    @property
    def recent_rejected_count(self) -> int:
        return self.recent_statuses.count(EskfReplayStatus.REJECTED)


@dataclass(frozen=True, slots=True)
class ObservationHealthTransition:
    kind: EskfObservationKind
    previous: ObservationHealthState | None
    current: ObservationHealthState
    time_s: float


@dataclass(frozen=True, slots=True)
class ObservationHealthSnapshot:
    """Both streams after all current deliveries, with state changes only."""

    epoch_index: int
    time_s: float
    local_position: ObservationStreamHealth
    barometric_altitude: ObservationStreamHealth
    transitions: tuple[ObservationHealthTransition, ...]


def _advance_stream(
    previous: ObservationStreamHealth,
    policy: ObservationHealthPolicy,
    initial_time_s: float,
    time_s: float,
    evidence: tuple[tuple[EskfReplayStatus, float], ...],
) -> ObservationStreamHealth:
    accepted = previous.last_accepted_acquisition_time_s
    accepted_delivery = previous.last_accepted_delivery_time_s
    acquired, delivered = previous.last_acquisition_time_s, previous.last_delivery_time_s
    accept_run, reject_run = previous.consecutive_acceptances, previous.consecutive_rejections
    recent = previous.recent_statuses
    old_age = time_s - (initial_time_s if accepted is None else accepted)
    recovering = previous.state in (
        ObservationHealthState.DEGRADED,
        ObservationHealthState.LOST,
        ObservationHealthState.RECOVERING,
    )
    if old_age > policy.warning_age_s:
        recovering = True
        accept_run = 0
    for status, acquisition_time in evidence:
        acquired, delivered = acquisition_time, time_s
        recent = (*recent, status)[-policy.evidence_window_size :]
        if status is EskfReplayStatus.FUSED:
            accepted, accepted_delivery = acquisition_time, time_s
            accept_run += 1
        else:
            accept_run = 0
        reject_run = reject_run + 1 if status is EskfReplayStatus.REJECTED else 0
    age = time_s - (initial_time_s if accepted is None else accepted)
    if not policy.enabled:
        state, reason = ObservationHealthState.DISABLED, ObservationHealthReason.DISABLED
    elif age > policy.lost_age_s:
        state, reason = ObservationHealthState.LOST, ObservationHealthReason.AGE_LOSS
    elif age > policy.warning_age_s:
        state, reason = ObservationHealthState.DEGRADED, ObservationHealthReason.AGE_WARNING
    elif reject_run >= policy.rejection_limit:
        state, reason = ObservationHealthState.DEGRADED, ObservationHealthReason.REJECTION_RUN
    elif recovering and accept_run < policy.recovery_acceptances:
        state = ObservationHealthState.RECOVERING if accept_run else ObservationHealthState.DEGRADED
        reason = ObservationHealthReason.RECOVERY
    elif accepted is None:
        state, reason = ObservationHealthState.WAITING, ObservationHealthReason.STARTUP
    else:
        state, reason = ObservationHealthState.HEALTHY, ObservationHealthReason.FRESH
    return ObservationStreamHealth(
        previous.kind,
        state,
        reason,
        accepted,
        accepted_delivery,
        None if accepted is None else time_s - accepted,
        acquired,
        delivered,
        accept_run,
        reject_run,
        recent,
    )


class ObservationHealthMonitor:
    """Transactional observer of every ESKF epoch; never an estimator/controller.

    step receives only delivered events at the current epoch. Acquisition times
    are resolved from previously observed clock epochs. Delayed/out-of-order
    stale observations are evidence of delivery, not fresh usable information.
    A reset begins a new stream at epoch zero. Not a concurrent/thread-safe API.
    """

    def __init__(
        self, configuration: ObservationHealthConfiguration, *, initial_time_s: float = 0.0
    ) -> None:
        if not isinstance(configuration, ObservationHealthConfiguration):
            raise TypeError("configuration must be ObservationHealthConfiguration")
        self._configuration = replace(configuration)
        self.reset(initial_time_s=initial_time_s)

    def reset(self, *, initial_time_s: float = 0.0) -> None:
        """Clear clock, identities, streaks and evidence; validate before changing anything."""
        initial = _scalar("initial_time_s", initial_time_s, nonnegative=True)
        for policy in (self._configuration.local_position, self._configuration.barometric_altitude):
            if (
                not isfinite(initial + policy.lost_age_s)
                or initial + policy.check_interval_s <= initial
            ):
                raise ValueError("initial clock must resolve the configured health intervals")
        self._initial_time_s = initial
        self._times: list[float] = []
        self._identities: set[tuple[EskfObservationKind, int]] = set()
        self._acquisitions: set[tuple[EskfObservationKind, int]] = set()
        self._latest: ObservationHealthSnapshot | None = None
        self._history: list[ObservationHealthSnapshot] = []

    @property
    def configuration(self) -> ObservationHealthConfiguration:
        """Immutable policy; reconfiguration requires a new monitor."""
        return self._configuration

    @property
    def initial_time_s(self) -> float:
        return self._initial_time_s

    @property
    def history(self) -> tuple[ObservationHealthSnapshot, ...]:
        """Immutable observed prefix, including terminal epochs when used in missions."""
        return tuple(self._history)

    @property
    def latest(self) -> ObservationHealthSnapshot | None:
        """Immutable scalar-only snapshot, or None before the first epoch/after reset."""
        return self._latest

    def step(
        self, time_s: float, events: tuple[EskfReplayEvent, ...] = ()
    ) -> ObservationHealthSnapshot:
        """Observe one complete epoch. Invalid input consumes no clock or identity."""
        time = _scalar("time_s", time_s, nonnegative=True)
        index = len(self._times)
        if (index == 0 and time != self._initial_time_s) or (index > 0 and time <= self._times[-1]):
            raise ValueError("clock must start at initial_time_s and strictly increase")
        if not isinstance(events, tuple) or not all(isinstance(e, EskfReplayEvent) for e in events):
            raise TypeError("events must be a tuple of EskfReplayEvent")
        keys = [(e.observation.kind.value, e.observation.observation_index) for e in events]
        if keys != sorted(keys):
            raise ValueError("events must follow canonical same-epoch ordering")
        identities = {(e.observation.kind, e.observation.observation_index) for e in events}
        acquisitions = {(e.observation.kind, e.observation.acquisition_index) for e in events}
        if (
            len(identities) != len(events)
            or len(acquisitions) != len(events)
            or identities & self._identities
            or acquisitions & self._acquisitions
        ):
            raise ValueError("duplicate observation identity or acquisition epoch")
        evidence: dict[EskfObservationKind, list[tuple[EskfReplayStatus, float]]] = {
            kind: [] for kind in EskfObservationKind
        }
        policies = (self._configuration.local_position, self._configuration.barometric_altitude)
        for event in events:
            observation = event.observation
            if observation.delivery_index != index or observation.acquisition_index > index:
                raise ValueError("events must be delivered now at a known acquisition epoch")
            policy = policies[0 if observation.kind is EskfObservationKind.LOCAL_POSITION else 1]
            if (policy.enabled and event.status is EskfReplayStatus.DISABLED) or (
                not policy.enabled
                and event.status in (EskfReplayStatus.FUSED, EskfReplayStatus.REJECTED)
            ):
                raise ValueError("event disposition must agree with the configured fusion flag")
            acquired = (
                time
                if observation.acquisition_index == index
                else self._times[observation.acquisition_index]
            )
            evidence[observation.kind].append((event.status, acquired))
        previous_streams = (
            (self._latest.local_position, self._latest.barometric_altitude)
            if self._latest is not None
            else tuple(
                ObservationStreamHealth(
                    kind, ObservationHealthState.WAITING, ObservationHealthReason.STARTUP
                )
                for kind in EskfObservationKind
            )
        )
        streams: list[ObservationStreamHealth] = []
        transitions: list[ObservationHealthTransition] = []
        for previous, policy in zip(previous_streams, policies, strict=True):
            current = _advance_stream(
                previous, policy, self._initial_time_s, time, tuple(evidence[previous.kind])
            )
            old_state = None if self._latest is None else previous.state
            if current.state is not old_state:
                transitions.append(
                    ObservationHealthTransition(previous.kind, old_state, current.state, time)
                )
            streams.append(current)
        result = ObservationHealthSnapshot(index, time, streams[0], streams[1], tuple(transitions))
        self._times.append(time)
        self._identities.update(identities)
        self._acquisitions.update(acquisitions)
        self._latest = result
        self._history.append(result)
        return result
