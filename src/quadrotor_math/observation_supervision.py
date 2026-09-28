"""Explicit, causal numerical-abort budgets for observation availability (ADR 0022)."""

from dataclasses import dataclass, replace

from .eskf_replay import _scalar
from .observation_health import ObservationHealthMonitor, ObservationHealthState


@dataclass(frozen=True, slots=True)
class ObservationSupervisionPolicy:
    """Unhealthy-time budgets in seconds, supplied explicitly by the caller.

    Position is required in all active mission phases, including initialization
    hold. Altitude is independently required unless its timeout is None. These
    are simulation response budgets, not validated safe-flight deadlines.
    """

    local_position_timeout_s: float
    barometric_altitude_timeout_s: float | None

    def __post_init__(self) -> None:
        for name in ("local_position_timeout_s", "barometric_altitude_timeout_s"):
            value = getattr(self, name)
            if name == "barometric_altitude_timeout_s" and value is None:
                continue
            value = _scalar(name, value, nonnegative=True)
            if value == 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)


@dataclass(frozen=True, slots=True)
class ObservationSupervisionDecision:
    """Scalar-only output after one complete monitor epoch; abort stays latched."""

    epoch_index: int
    time_s: float
    local_position_unhealthy_since_s: float | None
    barometric_altitude_unhealthy_since_s: float | None
    abort_reason: str | None


class ObservationSupervisor:
    """Consume each bound monitor snapshot once, after its current deliveries.

    WAITING, DEGRADED, LOST and RECOVERING spend the same continuous budget.
    Only HEALTHY before expiry clears it; recovery at the deadline is too late.
    Required-but-disabled streams fail at construction. Optional altitude never
    vetoes the mission. No truth, estimate, command or fault labels are inputs.

    Caller-owned history grows with the run; this is a simulation interface,
    not a bounded-memory embedded implementation. Reset the monitor first, then
    this supervisor, before reuse. A failed call consumes no supervisor state.
    """

    def __init__(
        self, monitor: ObservationHealthMonitor, policy: ObservationSupervisionPolicy
    ) -> None:
        if not isinstance(monitor, ObservationHealthMonitor):
            raise TypeError("monitor must be ObservationHealthMonitor")
        if not isinstance(policy, ObservationSupervisionPolicy):
            raise TypeError("policy must be ObservationSupervisionPolicy")
        self._monitor = monitor
        self._policy = replace(policy)
        if not monitor.configuration.local_position.enabled:
            raise ValueError("required local position fusion cannot be disabled")
        if (
            policy.barometric_altitude_timeout_s is not None
            and not monitor.configuration.barometric_altitude.enabled
        ):
            raise ValueError("required barometric altitude fusion cannot be disabled")
        self._history: list[ObservationSupervisionDecision] = []
        self.reset()

    @property
    def monitor(self) -> ObservationHealthMonitor:
        return self._monitor

    @property
    def policy(self) -> ObservationSupervisionPolicy:
        return self._policy

    @property
    def history(self) -> tuple[ObservationSupervisionDecision, ...]:
        return tuple(self._history)

    @property
    def latest(self) -> ObservationSupervisionDecision | None:
        return self._history[-1] if self._history else None

    @property
    def abort_reason(self) -> str | None:
        return None if self.latest is None else self.latest.abort_reason

    def reset(self) -> None:
        """Clear all response memory, only after the bound monitor is fresh."""
        if self.monitor.latest is not None:
            raise ValueError("reset requires a fresh observation health monitor")
        self._history.clear()

    def step(self) -> ObservationSupervisionDecision:
        """Check expiry before clearing recovered timers; position wins ties."""
        snapshot = self.monitor.latest
        if snapshot is None or snapshot.epoch_index != len(self._history):
            raise ValueError("consume every observation health epoch exactly once in order")
        previous = self.latest
        if previous is not None and snapshot.time_s <= previous.time_s:
            raise ValueError("observation supervision time must strictly increase")
        timers: list[float | None] = []
        reason = self.abort_reason
        for stream, timeout, since, label in (
            (
                snapshot.local_position,
                self.policy.local_position_timeout_s,
                None if previous is None else previous.local_position_unhealthy_since_s,
                "local_position",
            ),
            (
                snapshot.barometric_altitude,
                self.policy.barometric_altitude_timeout_s,
                None if previous is None else previous.barometric_altitude_unhealthy_since_s,
                "barometric_altitude",
            ),
        ):
            if timeout is None:
                timers.append(None)
                continue
            if since is None and stream.state is not ObservationHealthState.HEALTHY:
                since = snapshot.time_s
            expired = since is not None and snapshot.time_s >= since + timeout
            if expired and reason is None:
                reason = f"observation_{label}_timeout"
            if stream.state is ObservationHealthState.HEALTHY and not expired:
                since = None
            timers.append(since)
        decision = ObservationSupervisionDecision(
            snapshot.epoch_index, snapshot.time_s, timers[0], timers[1], reason
        )
        self._history.append(decision)
        return decision
