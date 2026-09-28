"""Frozen original-cascade response/maneuver campaign; no tuning inputs (ADR 0023)."""

from dataclasses import asdict, replace
from typing import Any

import numpy as np

from experiments.estimated_feedback_evidence import plain
from experiments.estimated_feedback_validation import make_configuration as original_configuration
from quadrotor_math.eskf_faults import EskfObservationFault
from quadrotor_math.eskf_replay import EskfObservationKind
from quadrotor_math.missions import MissionPhase, MissionSegment, ReferenceKind
from quadrotor_math.observation_health import (
    ObservationHealthConfiguration,
    ObservationHealthPolicy,
)
from quadrotor_math.observation_supervision import ObservationSupervisionPolicy
from quadrotor_math.run_configuration import SensorSchedule

CASES = (
    "nominal_hover",
    "nominal_tracking",
    "position_dropout",
    "position_rejection",
    "position_delay",
    "altitude_dropout",
    "altitude_rejection",
    "altitude_delay",
    "position_recovery",
    "landing_position_dropout",
    "wind_tracking",
    "mass_tracking",
)


def planned_cases(partition: str) -> tuple[str, ...]:
    if partition not in ("smoke", "campaign"):
        raise ValueError("partition must be smoke or campaign")
    return ("nominal_tracking", "position_dropout") if partition == "smoke" else CASES


def configuration(case: str, partition: str) -> dict[str, Any]:
    planned_cases(partition)
    if case not in CASES:
        raise ValueError("unknown robustness case")
    hover = case == "nominal_hover"
    args = original_configuration(
        {
            "case": "smoke" if partition == "smoke" else "hover",
            "seed": 30 if hover else 31,
            "noiseless": False,
        }
    )
    if partition == "smoke":
        args["plan"] = replace(
            args["plan"], segments=tuple(replace(s, duration_s=0.1) for s in args["plan"].segments)
        )
        args["sensors"] = replace(
            args["sensors"],
            local_position_schedule=SensorSchedule(0.02, 0),
            barometric_altitude_schedule=SensorSchedule(0.01, 0),
        )
    else:
        point = np.zeros(3)
        segments = []

        def add(
            phase: MissionPhase, duration: float, end: list[float], kind: ReferenceKind
        ) -> None:
            nonlocal point
            target = np.array(end)
            segments.append(MissionSegment(phase, duration, point, target, kind))
            point = target

        add(MissionPhase.INITIALIZE, 1, [0, 0, 0], ReferenceKind.HOLD)
        add(MissionPhase.TAKEOFF, 4, [0, 0, -1], ReferenceKind.SMOOTH)
        add(MissionPhase.TRACK, 6 if hover else 1, [0, 0, -1], ReferenceKind.HOLD)
        if not hover:
            add(MissionPhase.TRACK, 6, [0.75, 0, -1], ReferenceKind.SMOOTH)
            add(MissionPhase.TRACK, 1, [0.75, 0, -1], ReferenceKind.HOLD)
        add(MissionPhase.LAND, 4, [0 if hover else 0.75, 0, 0], ReferenceKind.SMOOTH)
        args["plan"] = replace(args["plan"], segments=tuple(segments))
    if case == "wind_tracking":
        args["truth_body"] = replace(
            args["truth_body"], quadratic_drag_coefficient_B=np.array([0.1, 0.1, 0.15])
        )
        args["truth_world"] = replace(args["truth_world"], wind_velocity_W=np.array([0.5, -0.3, 0]))
    if case == "mass_tracking":
        args["truth_body"] = replace(args["truth_body"], mass=1.1)
    return args


def policies(
    args: dict[str, Any], partition: str
) -> tuple[ObservationHealthConfiguration, ObservationSupervisionPolicy]:
    planned_cases(partition)
    sensors = args["sensors"]
    health = ObservationHealthConfiguration(
        *(
            ObservationHealthPolicy(
                s.sample_period_s, s.delivery_delay_s, args["numerics"].time_step_s, 2, 4, 3, 2, 5
            )
            for s in (sensors.local_position_schedule, sensors.barometric_altitude_schedule)
        )
    )
    response = (
        ObservationSupervisionPolicy(0.08, 0.06)
        if partition == "smoke"
        else ObservationSupervisionPolicy(0.6, 0.2)
    )
    return health, response


def fault_spec(case: str, partition: str) -> tuple[EskfObservationKind, float, float, str] | None:
    if case.startswith("nominal") or case in ("wind_tracking", "mass_tracking"):
        return None
    kind = (
        EskfObservationKind.BAROMETRIC_ALTITUDE
        if case.startswith("altitude")
        else EskfObservationKind.LOCAL_POSITION
    )
    start, end = (
        (13.2, 15.2)
        if case.startswith("landing")
        else (6.0, 6.4 if case.endswith("recovery") else 8.0)
    )
    if partition == "smoke":
        start, end = 0.1, 0.14 if case.endswith("recovery") else 0.3
    mode = "dropout" if case.endswith("recovery") else case.rsplit("_", 1)[-1]
    return kind, start, end, mode


def faults(case: str, partition: str, args: dict[str, Any]) -> tuple[EskfObservationFault, ...]:
    spec = fault_spec(case, partition)
    if spec is None:
        return ()
    kind, start, end, mode = spec
    schedule = (
        args["sensors"].local_position_schedule
        if kind is EskfObservationKind.LOCAL_POSITION
        else args["sensors"].barometric_altitude_schedule
    )
    dt = args["numerics"].time_step_s
    stride = round(schedule.sample_period_s / dt)
    result = []
    for i in range(int(np.ceil(end / schedule.sample_period_s))):
        time = (i + 1) * stride * dt
        if start <= time < end:
            offset = (
                np.array([5.0, 0, 0])
                if kind is EskfObservationKind.LOCAL_POSITION
                else np.array([5.0])
            )
            result.append(
                EskfObservationFault(
                    kind,
                    i,
                    dropout=mode == "dropout",
                    delay_steps=100 if mode == "delay" else 0,
                    offset=offset if mode == "rejection" else None,
                )
            )
    return tuple(result)


def protocol(partition: str) -> dict[str, Any]:
    cases = []
    for name in planned_cases(partition):
        args = configuration(name, partition)
        health, response = policies(args, partition)
        cases.append(
            {
                "name": name,
                "configuration": plain(
                    {
                        k: asdict(v) if hasattr(v, "__dataclass_fields__") else v
                        for k, v in args.items()
                    }
                ),
                "health": plain(asdict(health)),
                "response": asdict(response),
                "faults": plain([asdict(f) for f in faults(name, partition, args)]),
            }
        )
    return {
        "name": "integrated_observation_robustness",
        "version": 1,
        "partition": partition,
        "cases": cases,
        "modes": ["off", "on"],
        "flight_limits": {
            "rmse_m": 0.15,
            "final_position_m": 0.15,
            "final_speed_m_s": 0.15,
            "hover_peak_m": 0.08,
            "hover_window_s": [5.0, 11.0],
        },
        "response_limit": (
            "first sampled epoch at unhealthy origin + budget; "
            "onset ceiling warning_age + budget + 2*h"
        ),
        "semantics": (
            "Original cascade and original prior, no tuning; known seeds 30/31; "
            "numerical abort and flight completion scored separately; "
            "smoke is not maneuver qualification."
        ),
    }
