"""Authenticate and reconstruct integrated faulted feedback evidence (ADR 0023)."""

import hashlib
import json
from collections import defaultdict
from dataclasses import asdict, fields
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import canonical_json
from experiments.estimated_feedback_evidence import plain
from experiments.position_control_validation import _unique_object
from experiments.robustness_protocol import configuration, faults, policies
from quadrotor_math.attitude_control import attitude_error_body, compute_attitude_control
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_faults import inject_eskf_observation_faults
from quadrotor_math.eskf_replay import (
    EskfObservationKind,
    EskfReplayEvent,
    EskfReplayInput,
    EskfReplayObservation,
    replay_eskf,
)
from quadrotor_math.eskf_run_replay import _first_delivery_index
from quadrotor_math.estimated_mission import EstimatedMissionResult
from quadrotor_math.missions import (
    MissionPhase,
    MissionState,
    advance_mission,
    mission_guard_reason,
    mission_reference,
)
from quadrotor_math.observation_health import ObservationHealthMonitor, ObservationHealthState
from quadrotor_math.observation_supervision import ObservationSupervisor
from quadrotor_math.position_control import compute_position_control
from quadrotor_math.vertical_compensation import (
    VerticalCompensationPolicy,
    VerticalIntegralCompensator,
)


def ensure(condition: object, message: str) -> None:
    if not condition:
        raise ValueError(message)


def equal_json(actual: Any, expected: Any, message: str) -> None:
    ensure(canonical_json(plain(actual)) == canonical_json(plain(expected)), message)


def save_json(directory: Path, name: str, value: Any) -> dict[str, str]:
    data = canonical_json(plain(value))
    with (directory / name).open("xb") as stream:
        stream.write(data)
    return {"file": name, "sha256": hashlib.sha256(data).hexdigest()}


def load_json(directory: Path, name: str, record: dict[str, str]) -> Any:
    ensure(set(record) == {"file", "sha256"} and record["file"] == name, "unexpected JSON record")
    data = (directory / name).read_bytes()
    ensure(hashlib.sha256(data).hexdigest() == record["sha256"], "JSON byte digest mismatch")
    value = json.loads(data, object_pairs_hook=_unique_object)
    canonical_json(value)
    return value


def observation(value: dict[str, Any]) -> EskfReplayObservation:
    return EskfReplayObservation(
        EskfObservationKind(value["kind"]),
        value["observation_index"],
        value["acquisition_index"],
        value["delivery_index"],
        np.array(value["measurement"]),
    )


def audit_result(
    case: str,
    partition: str,
    mode: str,
    result: EstimatedMissionResult,
    diagnostic: dict[str, Any],
    *,
    compensation_policy: VerticalCompensationPolicy | None = None,
) -> tuple[ObservationHealthMonitor, ObservationSupervisor]:
    """Replay measurements, original fault sources, health, guards and all commands."""
    ensure(mode in ("off", "on"), "unknown supervision mode")
    ensure(
        set(diagnostic)
        == {"fault_records", "health", "supervision"}
        | ({"vertical"} if compensation_policy is not None else set()),
        "diagnostic schema mismatch",
    )
    args = configuration(case, partition)
    mission, estimates = result.mission, result.estimates
    times, dt = mission.time_s, args["numerics"].time_step_s
    ensure(np.array_equal(times, np.arange(len(times)) * dt), "noncanonical mission clock")
    for name in ("position_W", "velocity_W", "q_WB", "omega_B"):
        ensure(
            np.array_equal(getattr(mission, name)[0], getattr(args["initial_state"], name)),
            "true initialization mismatch",
        )
    ensure(
        np.array_equal(mission.actual_rotor_omega[0], args["initial_actual_rotor_omega"]),
        "initial motor mismatch",
    )
    for clock, stride in (
        (mission.control_time_s, args["numerics"].attitude_stride),
        (mission.position_control_time_s, args["numerics"].position_stride),
    ):
        ensure(np.array_equal(clock, times[:-1:stride]), "terminal or off-grid command")

    # Decode original measured sources, including ones dropped before the ESKF.
    source = EskfReplayInput(
        times,
        result.measurements.specific_force_measurements_B,
        result.measurements.angular_velocity_measurements_B,
        tuple(observation(r["source"]) for r in diagnostic["fault_records"]),
    )
    for kind, schedule in zip(
        EskfObservationKind,
        (args["sensors"].local_position_schedule, args["sensors"].barometric_altitude_schedule),
        strict=True,
    ):
        expected_indices = list(
            range(
                round(schedule.sample_period_s / dt),
                len(times),
                round(schedule.sample_period_s / dt),
            )
        )
        rows = sorted(
            (o for o in source.observations if o.kind is kind), key=lambda o: o.observation_index
        )
        ensure(
            [o.acquisition_index for o in rows] == expected_indices,
            "incomplete original acquisition ledger",
        )
        for o in rows:
            nominal_time = float(times[o.acquisition_index]) + schedule.delivery_delay_s
            ensure(
                o.delivery_index == _first_delivery_index(times, nominal_time),
                "nominal source delivery mismatch",
            )
    identities = {(o.kind, o.observation_index) for o in source.observations}
    reached = tuple(
        f for f in faults(case, partition, args) if (f.kind, f.observation_index) in identities
    )
    injection = inject_eskf_observation_faults(source, reached)
    equal_json(
        diagnostic["fault_records"],
        [asdict(r) for r in injection.records],
        "fault ledger differs from source and frozen plan",
    )
    equal_json(
        [asdict(o) for o in result.measurements.observations],
        [asdict(o) for o in injection.measurements.observations],
        "faulted input differs from exhaustive ledger",
    )
    scheduled = {}
    for row in injection.records:
        if row.output is None:
            continue
        schedule = (
            args["sensors"].local_position_schedule
            if row.source.kind is EskfObservationKind.LOCAL_POSITION
            else args["sensors"].barometric_altitude_schedule
        )
        scheduled[row.output.kind, row.output.observation_index] = (
            float(times[row.source.acquisition_index])
            + schedule.delivery_delay_s
            + row.fault.delay_steps * dt
        )
    ensure(
        np.array_equal(
            result.scheduled_observation_delivery_time_s,
            np.array(
                [scheduled[o.kind, o.observation_index] for o in result.measurements.observations]
            ),
        ),
        "effective schedule mismatch",
    )

    replay = replay_eskf(result.measurements, args["estimator_configuration"])
    ensure(
        np.array_equal(replay.covariances, estimates.covariances), "ESKF covariance replay mismatch"
    )
    for actual, expected in zip(estimates.states, replay.states, strict=True):
        ensure(
            all(
                np.array_equal(getattr(actual, f.name), getattr(expected, f.name))
                for f in fields(EskfNominalState)
            ),
            "ESKF state replay mismatch",
        )
    equal_json(
        [asdict(e) for e in estimates.events],
        [asdict(e) for e in replay.events],
        "ESKF event replay mismatch",
    )

    health_config, response = policies(args, partition)
    health = ObservationHealthMonitor(health_config)
    observation_supervisor = ObservationSupervisor(health, response)
    by_epoch: dict[int, list[EskfReplayEvent]] = defaultdict(list)
    for event in estimates.events:
        if event.observation.delivery_index != -1:
            by_epoch[event.observation.delivery_index].append(event)
    state = MissionState()
    outer_row = inner_row = 0
    compensation = (
        None
        if compensation_policy is None
        else VerticalIntegralCompensator(args["position_controller"], compensation_policy)
    )
    previous_inner_limited = False
    for k, time in enumerate(times):
        health.step(float(time), tuple(by_epoch[k]))
        decision = observation_supervisor.step()
        estimate = estimates.states[k]
        reference, _ = mission_reference(args["plan"], float(time))
        for name in ("position_W", "velocity_W", "acceleration_W"):
            ensure(
                np.array_equal(getattr(mission, "reference_" + name)[k], getattr(reference, name)),
                "reference mismatch",
            )
        truth_guard = mission_guard_reason(mission.position_W[k], mission.q_WB[k], args["safety"])
        estimate_guard = mission_guard_reason(estimate.position_W, estimate.q_WB, args["safety"])
        reason = (
            "truth_" + truth_guard
            if truth_guard
            else "estimate_" + estimate_guard
            if estimate_guard
            else decision.abort_reason
            if mode == "on"
            else None
        )
        state = advance_mission(
            state, args["plan"], float(time), estimate.position_W, estimate.velocity_W, reason
        )
        if state.phase not in (MissionPhase.COMPLETE, MissionPhase.ABORT):
            if k % args["numerics"].position_stride == 0:
                if compensation is None:
                    held = compute_position_control(
                        estimate.position_W,
                        estimate.velocity_W,
                        reference,
                        args["position_controller"],
                    )
                else:
                    assert health.latest is not None
                    held = compensation.step(
                        float(time),
                        estimate.position_W,
                        estimate.velocity_W,
                        reference,
                        observations_healthy=(
                            health.latest.local_position.state is ObservationHealthState.HEALTHY
                            and health.latest.barometric_altitude.state
                            is ObservationHealthState.HEALTHY
                        ),
                        previous_inner_limited=previous_inner_limited,
                    )
            if (
                np.linalg.norm(attitude_error_body(estimate.q_WB, held.q_reference_WB))
                > args["attitude_controller"].maximum_attitude_error_rad
            ):
                state = MissionState(MissionPhase.ABORT, float(time), reason="attitude_domain")
        ensure(mission.phase[k] == state.phase, "phase or observation guard mismatch")
        if state.phase in (MissionPhase.COMPLETE, MissionPhase.ABORT):
            ensure(
                k == len(times) - 1 and state.reason == mission.abort_reason,
                "terminal disposition mismatch",
            )
            break
        if k % args["numerics"].position_stride == 0:
            outer_values: dict[str, Any] = {
                "requested_acceleration_W": held.requested_acceleration_W,
                "feasible_acceleration_W": held.feasible_acceleration_W,
                "outer_limit_flags": [
                    held.acceleration_limited,
                    held.tilt_limited,
                    held.thrust_limited,
                ],
            }
            for name, value in outer_values.items():
                ensure(
                    np.array_equal(getattr(mission, name)[outer_row], value),
                    "outer control dataflow mismatch",
                )
            outer_row += 1
        if k % args["numerics"].attitude_stride == 0:
            control = compute_attitude_control(
                estimate.q_WB,
                result.angular_velocity_estimate_B[k],
                held.q_reference_WB,
                held.collective_thrust,
                args["attitude_controller"],
            )
            previous_inner_limited = bool(
                control.rate_limited
                or control.moment_limited
                or control.allocation.moment_scale < 1
            )
            values: dict[str, Any] = {
                "q_reference_WB": held.q_reference_WB,
                "collective_thrust": held.collective_thrust,
                "commanded_rotor_omega": control.allocation.commanded_rotor_omega,
                "moment_requested_B": control.moment_requested_B,
                "moment_limited_B": control.moment_limited_B,
                "allocated_moment_B": control.allocation.allocated_moment_B,
                "desired_omega_B": control.desired_omega_B,
                "moment_scale": control.allocation.moment_scale,
                "inner_limit_flags": [
                    control.rate_limited,
                    control.moment_limited,
                    control.allocation.moment_scale < 1,
                ],
            }
            for name, value in values.items():
                ensure(
                    np.array_equal(getattr(mission, name)[inner_row], value),
                    "inner control dataflow mismatch",
                )
            inner_row += 1
    equal_json(
        diagnostic["health"], [asdict(s) for s in health.history], "health reconstruction mismatch"
    )
    equal_json(
        diagnostic["supervision"],
        [asdict(d) for d in observation_supervisor.history] if mode == "on" else [],
        "supervisor reconstruction mismatch",
    )
    if compensation is not None:
        equal_json(
            diagnostic["vertical"],
            [asdict(s) for s in compensation.history],
            "vertical compensation reconstruction mismatch",
        )
    return health, observation_supervisor
