"""Pure observation fault injection, with labels kept outside measurement replay.

An injector consumes no truth or RNG. Callers construct explicit faults (including
bursts) before injection. Original source identities survive in a separate ledger;
surviving observation indices are rebuilt to satisfy the replay stream contract.
"""

from dataclasses import dataclass, replace

import numpy as np
from numpy.typing import NDArray

from .eskf_replay import (
    EskfObservationKind,
    EskfReplayInput,
    EskfReplayObservation,
    _owned_array,
)


@dataclass(frozen=True, slots=True, eq=False)
class EskfObservationFault:
    """Alter one source observation identified by (kind, observation_index).

    A dropout removes the observation entirely. Otherwise add an optional offset
    (3,) in NED position metres or (1,) in positive-up altitude metres, then add
    delay_steps to its existing delivery epoch. Acquisitions never change. Pending
    deliveries remain pending. Dropout cannot be combined with an offset or delay.
    """

    kind: EskfObservationKind
    observation_index: int
    dropout: bool = False
    delay_steps: int = 0
    offset: NDArray[np.float64] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.kind, EskfObservationKind):
            raise TypeError("kind must be an EskfObservationKind")
        for name in ("observation_index", "delay_steps"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 0:
                raise ValueError(f"{name} must be a nonnegative non-Boolean integer")
        if type(self.dropout) is not bool:
            raise TypeError("dropout must be a bool")
        if self.dropout and (self.delay_steps or self.offset is not None):
            raise ValueError("dropout cannot also delay or offset an observation")
        if self.offset is not None:
            width = 3 if self.kind is EskfObservationKind.LOCAL_POSITION else 1
            object.__setattr__(self, "offset", _owned_array("offset", self.offset, (width,)))


@dataclass(frozen=True, slots=True, eq=False)
class EskfFaultRecord:
    """An owned source observation, its explicit fault and optional resulting row."""

    source: EskfReplayObservation
    fault: EskfObservationFault
    output: EskfReplayObservation | None

    def __post_init__(self) -> None:
        if not isinstance(self.source, EskfReplayObservation) or not isinstance(
            self.fault, EskfObservationFault
        ):
            raise TypeError("source and fault must be observation/fault values")
        if (self.source.kind, self.source.observation_index) != (
            self.fault.kind,
            self.fault.observation_index,
        ):
            raise ValueError("fault identity must match source")
        if self.fault.dropout != (self.output is None):
            raise ValueError("only dropped observations have no output")
        if self.output is not None:
            if not isinstance(self.output, EskfReplayObservation):
                raise TypeError("output must be an observation or None")
            if (self.output.kind, self.output.acquisition_index) != (
                self.source.kind,
                self.source.acquisition_index,
            ):
                raise ValueError("output must preserve sensor kind and acquisition")
            object.__setattr__(self, "output", replace(self.output))
        object.__setattr__(self, "source", replace(self.source))
        object.__setattr__(self, "fault", replace(self.fault))


@dataclass(frozen=True, slots=True, eq=False)
class EskfFaultInjection:
    """Measurement-only replay input and a separate exhaustive source ledger.

    Produced by inject_eskf_observation_faults; the ledger must never enter the
    filter. Even unfaulted sources have a record. Ledger order is source identity
    (kind, index), while the replay input remains ordered by delivery.
    """

    measurements: EskfReplayInput
    records: tuple[EskfFaultRecord, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.measurements, EskfReplayInput):
            raise TypeError("measurements must be an EskfReplayInput")
        data = replace(self.measurements)
        records = []
        actual = {(o.kind, o.observation_index): o for o in data.observations}
        seen: set[tuple[EskfObservationKind, int]] = set()
        for record in self.records:
            if not isinstance(record, EskfFaultRecord):
                raise TypeError("records must contain EskfFaultRecord values")
            record = replace(record)
            records.append(record)
            if record.output is None:
                continue
            key = (record.output.kind, record.output.observation_index)
            if key in seen or key not in actual:
                raise ValueError("each output identity must occur exactly once in the ledger")
            seen.add(key)
            observation = actual[key]
            expected_delivery = record.source.delivery_index
            if expected_delivery != -1:
                expected_delivery += record.fault.delay_steps
                if expected_delivery >= data.time_s.size:
                    expected_delivery = -1
            try:
                with np.errstate(over="raise", invalid="raise"):
                    expected_measurement = (
                        record.source.measurement
                        if record.fault.offset is None
                        else record.source.measurement + record.fault.offset
                    )
            except FloatingPointError:
                raise ValueError("faulted measurement must remain finite") from None
            if (
                observation.acquisition_index != record.source.acquisition_index
                or observation.delivery_index != expected_delivery
                or record.output.delivery_index != expected_delivery
                or not np.array_equal(observation.measurement, expected_measurement)
                or not np.array_equal(record.output.measurement, expected_measurement)
            ):
                raise ValueError("ledger must exactly describe supplied measurements and delivery")
        if seen != set(actual):
            raise ValueError("ledger must account for every supplied observation")
        # Validate original source identities/epochs even for dropped observations.
        replace(data, observations=tuple(r.source for r in records))
        records.sort(key=lambda r: (r.source.kind, r.source.observation_index))
        for kind in EskfObservationKind:
            indices = [
                r.output.observation_index
                for r in records
                if r.source.kind is kind and r.output is not None
            ]
            if indices != list(range(len(indices))):
                raise ValueError("survivors must retain acquisition order when reindexed")
        object.__setattr__(self, "measurements", data)
        object.__setattr__(self, "records", tuple(records))


def inject_eskf_observation_faults(
    data: EskfReplayInput, faults: tuple[EskfObservationFault, ...]
) -> EskfFaultInjection:
    """Copy input and apply explicit faults atomically; never mutate caller data.

    Unknown or duplicate source IDs, invalid offsets, and finite arithmetic overflow
    raise ValueError. A delayed observation beyond the horizon has delivery_index=-1.
    Existing delays are additive. No IMU, clock, noise model or truth value changes.
    """
    if not isinstance(data, EskfReplayInput):
        raise TypeError("data must be an EskfReplayInput")
    data = replace(data)
    plan: dict[tuple[EskfObservationKind, int], EskfObservationFault] = {}
    identities = {(o.kind, o.observation_index) for o in data.observations}
    for fault in faults:
        if not isinstance(fault, EskfObservationFault):
            raise TypeError("faults must contain EskfObservationFault values")
        fault = replace(fault)
        key = (fault.kind, fault.observation_index)
        if key not in identities or key in plan:
            raise ValueError("fault source identity must exist and occur only once")
        plan[key] = fault
    records: list[EskfFaultRecord] = []
    outputs: list[EskfReplayObservation] = []
    indices = dict.fromkeys(EskfObservationKind, 0)
    for source in sorted(data.observations, key=lambda o: (o.kind, o.observation_index)):
        fault = plan.get(
            (source.kind, source.observation_index),
            EskfObservationFault(source.kind, source.observation_index),
        )
        output = None
        if not fault.dropout:
            measurement = source.measurement
            if fault.offset is not None:
                try:
                    with np.errstate(over="raise", invalid="raise"):
                        measurement = measurement + fault.offset
                except FloatingPointError:
                    raise ValueError("faulted measurement must remain finite") from None
            delivery = source.delivery_index
            if delivery != -1:
                delivery += fault.delay_steps
                if delivery >= data.time_s.size:
                    delivery = -1
            output = EskfReplayObservation(
                source.kind,
                indices[source.kind],
                source.acquisition_index,
                delivery,
                measurement,
            )
            indices[source.kind] += 1
            outputs.append(output)
        records.append(EskfFaultRecord(source, fault, output))
    return EskfFaultInjection(replace(data, observations=tuple(outputs)), tuple(records))
