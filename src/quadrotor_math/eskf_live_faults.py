"""Causal simulation fault delivery, reconciled with the offline injector (ADR 0023)."""

from dataclasses import replace

import numpy as np

from .eskf_faults import (
    EskfFaultInjection,
    EskfObservationFault,
    inject_eskf_observation_faults,
)
from .eskf_replay import EskfObservationKind, EskfReplayInput, EskfReplayObservation, _scalar


def _same_observations(
    left: tuple[EskfReplayObservation, ...], right: tuple[EskfReplayObservation, ...]
) -> bool:
    return len(left) == len(right) and all(
        (a.kind, a.observation_index, a.acquisition_index, a.delivery_index)
        == (b.kind, b.observation_index, b.acquisition_index, b.delivery_index)
        and np.array_equal(a.measurement, b.measurement)
        for a, b in zip(left, right, strict=True)
    )


class EskfLiveObservationFaults:
    """Transform nominal arrivals before the online ESKF, without truth or RNG.

    Source arrivals must have consecutive per-sensor identities in acquisition
    order, as produced by the fixed-rate mission schedulers. Survivors are
    reindexed on nominal arrival, before fault-induced reordering. Integer
    delays count estimator epochs; each call must supply the next observed
    clock. Only due arrivals are emitted, in canonical sensor/identity order.

    finish() is evidence-only: it accounts for all acquired sources (including
    nominally pending/dropout records), reconciles actual delivery against the
    pure offline injector and seals the run. Planned faults beyond a terminated
    run remain in faults, but are not invented as acquired records. Both fault
    labels and the exhaustive injection ledger stay outside the estimator.

    Invalid calls are atomic. Reset before reuse. Histories grow with run length;
    this diagnostic simulation interface is not an embedded transport driver.
    """

    def __init__(
        self, faults: tuple[EskfObservationFault, ...] = (), *, initial_time_s: float = 0.0
    ) -> None:
        if not isinstance(faults, tuple) or not all(
            isinstance(f, EskfObservationFault) for f in faults
        ):
            raise TypeError("faults must be a tuple of EskfObservationFault values")
        self._faults = tuple(replace(f) for f in faults)
        self._plan = {(f.kind, f.observation_index): f for f in self._faults}
        if len(self._plan) != len(self._faults):
            raise ValueError("fault source identities must be unique")
        self._initial_time_s = _scalar("initial_time_s", initial_time_s, nonnegative=True)
        self.reset()

    @property
    def faults(self) -> tuple[EskfObservationFault, ...]:
        return self._faults

    @property
    def initial_time_s(self) -> float:
        return self._initial_time_s

    @property
    def epoch_count(self) -> int:
        return len(self._times)

    @property
    def injection(self) -> EskfFaultInjection | None:
        return self._injection

    @property
    def delivered(self) -> tuple[EskfReplayObservation, ...]:
        return tuple(self._delivered)

    def reset(self) -> None:
        self._times: list[float] = []
        self._sources: list[EskfReplayObservation] = []
        self._delivered: list[EskfReplayObservation] = []
        self._pending: list[EskfReplayObservation] = []
        self._next_source = dict.fromkeys(EskfObservationKind, 0)
        self._next_output = dict.fromkeys(EskfObservationKind, 0)
        self._last_acquisition = dict.fromkeys(EskfObservationKind, -1)
        self._injection: EskfFaultInjection | None = None

    def step(
        self, time_s: float, arrivals: tuple[EskfReplayObservation, ...] = ()
    ) -> tuple[EskfReplayObservation, ...]:
        """Receive nominal arrivals, then emit faulted observations due now."""
        time = _scalar("time_s", time_s, nonnegative=True)
        if self.injection is not None:
            raise ValueError("fault delivery is finished; reset before reuse")
        if (not self._times and time != self.initial_time_s) or (
            self._times and time <= self._times[-1]
        ):
            raise ValueError("fault clock must start at its prior and strictly increase")
        if not isinstance(arrivals, tuple) or not all(
            isinstance(o, EskfReplayObservation) for o in arrivals
        ):
            raise TypeError("arrivals must be a tuple of observations")
        if arrivals != tuple(sorted(arrivals, key=lambda o: (o.kind, o.observation_index))):
            raise ValueError("arrivals must be in canonical order")
        k = self.epoch_count
        source_ids, output_ids = self._next_source.copy(), self._next_output.copy()
        acquired = self._last_acquisition.copy()
        pending = list(self._pending)
        for obs in arrivals:
            if (
                obs.delivery_index != k
                or obs.acquisition_index > k
                or obs.observation_index != source_ids[obs.kind]
                or obs.acquisition_index <= acquired[obs.kind]
            ):
                raise ValueError("nominal arrivals must follow current epoch and source order")
            source_ids[obs.kind] += 1
            acquired[obs.kind] = obs.acquisition_index
            fault = self._plan.get((obs.kind, obs.observation_index))
            if fault is not None and fault.dropout:
                continue
            value = obs.measurement
            if fault is not None and fault.offset is not None:
                try:
                    with np.errstate(over="raise", invalid="raise"):
                        value = value + fault.offset
                except FloatingPointError:
                    raise ValueError("faulted measurement must remain finite") from None
            pending.append(
                EskfReplayObservation(
                    obs.kind,
                    output_ids[obs.kind],
                    obs.acquisition_index,
                    k + (0 if fault is None else fault.delay_steps),
                    value,
                )
            )
            output_ids[obs.kind] += 1
        delivered = tuple(
            sorted(
                (o for o in pending if o.delivery_index == k),
                key=lambda o: (o.kind, o.observation_index),
            )
        )
        self._pending = [o for o in pending if o.delivery_index > k]
        self._times.append(time)
        self._sources.extend(arrivals)
        self._delivered.extend(delivered)
        self._next_source, self._next_output, self._last_acquisition = (
            source_ids,
            output_ids,
            acquired,
        )
        return delivered

    def finish(self, source: EskfReplayInput) -> EskfFaultInjection:
        """Reconcile delivered/pending/dropped sources without feeding back labels."""
        if self.injection is not None:
            raise ValueError("fault delivery is already finished")
        if not isinstance(source, EskfReplayInput):
            raise TypeError("source must be EskfReplayInput")
        if not np.array_equal(source.time_s, self._times) or not _same_observations(
            tuple(o for o in source.observations if o.delivery_index != -1), tuple(self._sources)
        ):
            raise ValueError("source ledger must match the observed clock and nominal arrivals")
        identities = {(o.kind, o.observation_index) for o in source.observations}
        reached = tuple(f for f in self.faults if (f.kind, f.observation_index) in identities)
        injection = inject_eskf_observation_faults(source, reached)
        if not _same_observations(
            tuple(o for o in injection.measurements.observations if o.delivery_index != -1),
            self.delivered,
        ):
            raise ValueError("live fault delivery differs from the pure offline injector")
        self._injection = injection
        return injection
