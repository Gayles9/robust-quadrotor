"""Frozen paired fault/mismatch campaign with authenticated replay (ADR 0023).

python -m experiments.robustness_validation --partition campaign --workers 2 --output NEW_DIR
A valid failed campaign is saved and exits 1. Evidence/implementation errors raise.
"""

import argparse
import hashlib
import json
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, fields
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.estimated_feedback_evidence import (
    load_history,
    pack_history,
    plain,
    save_history,
    unpack_history,
)
from experiments.position_control_validation import _unique_object
from experiments.robustness_evidence import audit_result, ensure, equal_json, load_json, save_json
from experiments.robustness_protocol import (
    configuration,
    fault_spec,
    faults,
    planned_cases,
    policies,
    protocol,
)
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_live_faults import EskfLiveObservationFaults
from quadrotor_math.eskf_replay import EskfObservationKind
from quadrotor_math.estimated_mission import EstimatedMissionResult, simulate_estimated_mission
from quadrotor_math.mission_simulation import MissionResult
from quadrotor_math.missions import MissionPhase
from quadrotor_math.observation_health import ObservationHealthMonitor, ObservationHealthState
from quadrotor_math.observation_supervision import ObservationSupervisor
from quadrotor_math.run_manifest import capture_software_provenance


def prefix_equal(off: EstimatedMissionResult, on: EstimatedMissionResult) -> bool:
    n = len(on.mission.time_s)
    if n > len(off.mission.time_s):
        return False
    for f in fields(MissionResult):
        if f.name == "abort_reason":
            continue
        actual, expected = getattr(on.mission, f.name), getattr(off.mission, f.name)
        if f.name == "phase":
            actual = actual[:-1]
        if not np.array_equal(actual, expected[: len(actual)]):
            return False
    if not np.array_equal(on.estimates.covariances, off.estimates.covariances[:n]):
        return False
    for a, b in zip(on.estimates.states, off.estimates.states[:n], strict=True):
        if any(
            not np.array_equal(getattr(a, f.name), getattr(b, f.name))
            for f in fields(EskfNominalState)
        ):
            return False
    for name in (
        "angular_velocity_estimate_B",
        "imu_noise_mean_B",
        "true_accelerometer_bias_B",
        "true_gyroscope_bias_B",
    ):
        if not np.array_equal(getattr(on, name), getattr(off, name)[:n]):
            return False
    for name in ("specific_force_measurements_B", "angular_velocity_measurements_B"):
        if not np.array_equal(getattr(on.measurements, name), getattr(off.measurements, name)[:n]):
            return False
    left = [asdict(e) for e in on.estimates.events if e.observation.delivery_index != -1]
    right = [asdict(e) for e in off.estimates.events if 0 <= e.observation.delivery_index < n]
    return canonical_json(plain(left)) == canonical_json(plain(right))


def rms(values: Any) -> float:
    return float(np.sqrt(np.mean(np.sum(np.asarray(values) ** 2, axis=1))))


def score_pair(
    case: str,
    partition: str,
    results: dict[str, EstimatedMissionResult],
    health: dict[str, ObservationHealthMonitor],
    diagnostics: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    args = configuration(case, partition)
    off, on = results["off"], results["on"]
    n = min(len(off.mission.time_s), len(on.mission.time_s))
    spec = fault_spec(case, partition)
    transient = case.endswith("recovery")
    persistent = spec is not None and not transient
    limits = protocol(partition)["flight_limits"]
    metrics: dict[str, Any] = {
        "common_horizon_s": float(on.mission.time_s[n - 1]),
        "common_prefix_equal": prefix_equal(off, on),
        "modes": {},
    }
    for mode, result in results.items():
        latest_health = health[mode].latest
        assert latest_health is not None
        m = result.mission
        error = m.position_W - m.reference_position_W
        estimation = np.array([s.position_W for s in result.estimates.states]) - m.position_W
        terminal_time = float(m.time_s[-1])
        records = diagnostics[mode]["fault_records"]
        affected = [
            r
            for r in records
            if r["fault"]["dropout"]
            or r["fault"]["delay_steps"]
            or r["fault"]["offset"] is not None
        ]
        row: dict[str, Any] = {
            "terminal_time_s": terminal_time,
            "terminal_phase": MissionPhase(int(m.phase[-1])).name,
            "abort_reason": m.abort_reason,
            "completed": bool(m.phase[-1] == MissionPhase.COMPLETE),
            "tracking_rmse_m": rms(error),
            "tracking_rmse_common_m": rms(error[:n]),
            "estimation_rmse_common_m": rms(estimation[:n]),
            "final_position_error_m": float(np.linalg.norm(error[-1])),
            "final_true_speed_m_s": float(np.linalg.norm(m.velocity_W[-1])),
            "last_command_time_s": float(m.control_time_s[-1]) if len(m.control_time_s) else None,
            "event_counts": dict(
                Counter(
                    f"{e.observation.kind.name}:{e.status.value}" for e in result.estimates.events
                )
            ),
            "planned_fault_sources": len(faults(case, partition, args)),
            "acquired_fault_sources": len(affected),
            "delivered_fault_sources": sum(r["source"]["delivery_index"] != -1 for r in affected),
            "dropped_sources": sum(r["output"] is None for r in records),
            "terminal_accepted_age_s": {
                "local_position": latest_health.local_position.accepted_age_s,
                "barometric_altitude": latest_health.barometric_altitude.accepted_age_s,
            },
        }
        row["unacquired_fault_sources"] = (
            row["planned_fault_sources"] - row["acquired_fault_sources"]
        )
        row["flight_required"] = not persistent
        conditions = {
            "complete": row["completed"],
            "rmse": row["tracking_rmse_m"] <= limits["rmse_m"],
            "final_position": row["final_position_error_m"] <= limits["final_position_m"],
            "final_speed": row["final_true_speed_m_s"] <= limits["final_speed_m_s"],
        }
        if case == "nominal_hover" and partition == "campaign":
            mask = (m.time_s >= 5) & (m.time_s <= 11)
            row["hover_peak_m"] = (
                float(np.max(np.linalg.norm(error[mask], axis=1))) if np.any(mask) else None
            )
            conditions["full_hover"] = (
                terminal_time >= 11
                and row["hover_peak_m"] is not None
                and row["hover_peak_m"] <= limits["hover_peak_m"]
            )
        row["flight_conditions"] = conditions
        row["flight_passed"] = all(conditions.values())
        metrics["modes"][mode] = row

    response_conditions = {"common_prefix": metrics["common_prefix_equal"]}
    if not persistent:
        response_conditions["no_observation_abort"] = not (
            on.mission.abort_reason or ""
        ).startswith("observation_")
        before, after = pack_history(off, off.mission), pack_history(on, on.mission)
        response_conditions["complete_payload_parity"] = all(
            before[k].shape == after[k].shape
            and before[k].dtype == after[k].dtype
            and before[k].tobytes() == after[k].tobytes()
            for k in before
        )
    if spec is not None:
        kind, onset, _, _ = spec
        label = (
            "local_position"
            if kind is EskfObservationKind.LOCAL_POSITION
            else "barometric_altitude"
        )
        history = health["on"].history
        earlier = [s for s in history if s.time_s < onset]
        established = bool(
            earlier and getattr(earlier[-1], label).state is ObservationHealthState.HEALTHY
        )
        bad = next(
            (
                s
                for s in history
                if s.time_s >= onset
                and getattr(s, label).state
                in (
                    ObservationHealthState.DEGRADED,
                    ObservationHealthState.LOST,
                    ObservationHealthState.RECOVERING,
                )
            ),
            None,
        )
        recovered = next(
            (
                s
                for s in history
                if bad is not None
                and s.time_s > bad.time_s
                and getattr(s, label).state is ObservationHealthState.HEALTHY
            ),
            None,
        )
        metrics["fault_onset_s"] = onset
        metrics["detected_unhealthy_s"] = None if bad is None else bad.time_s
        metrics["confirmed_recovery_s"] = None if recovered is None else recovered.time_s
        response_conditions["health_before_fault"] = established
        response_conditions["actual_fault_exposure"] = (
            metrics["modes"]["on"]["delivered_fault_sources"] > 0
        )
        if transient:
            response_conditions["degradation_and_recovery"] = (
                bad is not None and recovered is not None
            )
        else:
            health_policy, policy = policies(args, partition)
            budget = (
                policy.local_position_timeout_s
                if label == "local_position"
                else policy.barometric_altitude_timeout_s
            )
            assert budget is not None
            deadline = None if bad is None else bad.time_s + budget
            expected = (
                None
                if deadline is None
                else next((s.time_s for s in history if s.time_s >= deadline), None)
            )
            terminal = float(on.mission.time_s[-1])
            latency = terminal - onset
            ceiling = (
                getattr(health_policy, label).warning_age_s
                + budget
                + 2 * args["numerics"].time_step_s
            )
            metrics.update(
                {
                    "response_deadline_s": deadline,
                    "response_latency_s": latency,
                    "response_ceiling_s": ceiling,
                }
            )
            response_conditions.update(
                {
                    "observation_reason": on.mission.abort_reason == f"observation_{label}_timeout",
                    "first_expired_epoch": expected is not None and terminal == expected,
                    "onset_bound": 0 <= latency <= ceiling,
                    "command_cutoff": bool(
                        np.all(on.mission.control_time_s < terminal)
                        and np.all(on.mission.position_control_time_s < terminal)
                    ),
                }
            )
    metrics["response_conditions"] = response_conditions
    metrics["response_passed"] = all(response_conditions.values())
    metrics["flight_required"] = not persistent
    metrics["flight_passed"] = all(r["flight_passed"] for r in metrics["modes"].values())
    return metrics


def audit_case(directory: Path, case: str, partition: str, row: dict[str, Any]) -> dict[str, Any]:
    ensure(row["name"] == case and set(row["modes"]) == {"off", "on"}, "case/mode ledger mismatch")
    results: dict[str, EstimatedMissionResult] = {}
    health: dict[str, ObservationHealthMonitor] = {}
    diagnostics: dict[str, dict[str, Any]] = {}
    for index, mode in enumerate(("off", "on")):
        item = row["modes"][mode]
        if item["status"] == "error":
            ensure(
                set(item) == {"status", "error", "diagnostic"}
                and isinstance(item["error"], str)
                and item["error"],
                "failure must retain its diagnostic",
            )
            diagnostic = load_json(directory, f"failure-{mode}.json", item["diagnostic"])
            ensure(
                set(diagnostic) == {"health", "supervision", "delivered"},
                "failure diagnostic schema mismatch",
            )
            continue
        ensure(
            item["status"] == "ok" and set(item) == {"status", "history", "diagnostic"},
            "mode schema mismatch",
        )
        result, duplicate = unpack_history(load_history(directory, index, item["history"]))
        for f in fields(MissionResult):
            ensure(
                np.array_equal(getattr(result.mission, f.name), getattr(duplicate, f.name)),
                "archive companion mission mismatch",
            )
        diagnostic = load_json(directory, f"diagnostic-{mode}.json", item["diagnostic"])
        h, _ = audit_result(case, partition, mode, result, diagnostic)
        results[mode], health[mode], diagnostics[mode] = result, h, diagnostic
    if len(results) != 2:
        return {
            "response_passed": False,
            "flight_required": fault_spec(case, partition) is None or case.endswith("recovery"),
            "flight_passed": False,
            "numerical_failure": True,
        }
    return score_pair(case, partition, results, health, diagnostics)


def _execute_case(task: tuple[str, str, str]) -> dict[str, Any]:
    case, partition, root = task
    directory = Path(root) / case
    directory.mkdir()
    row: dict[str, Any] = {"name": case, "modes": {}}
    for index, mode in enumerate(("off", "on")):
        args = configuration(case, partition)
        h, response = policies(args, partition)
        health = ObservationHealthMonitor(h)
        supervisor = ObservationSupervisor(health, response)
        channel = EskfLiveObservationFaults(faults(case, partition, args))
        try:
            result = simulate_estimated_mission(
                **args,
                observation_health=health,
                observation_supervision=supervisor if mode == "on" else None,
                observation_faults=channel,
            )
        except (ValueError, FloatingPointError, np.linalg.LinAlgError, RuntimeError) as error:
            row["modes"][mode] = {
                "status": "error",
                "error": f"{type(error).__name__}: {error}",
                "diagnostic": save_json(
                    directory,
                    f"failure-{mode}.json",
                    {
                        "health": [asdict(s) for s in health.history],
                        "supervision": [asdict(d) for d in supervisor.history],
                        "delivered": [asdict(o) for o in channel.delivered],
                    },
                ),
            }
        else:
            assert channel.injection is not None
            row["modes"][mode] = {
                "status": "ok",
                "history": save_history(directory, index, pack_history(result, result.mission)),
                "diagnostic": save_json(
                    directory,
                    f"diagnostic-{mode}.json",
                    {
                        "fault_records": [asdict(r) for r in channel.injection.records],
                        "health": [asdict(s) for s in health.history],
                        "supervision": [asdict(d) for d in supervisor.history],
                    },
                ),
            }
        print(
            json.dumps({"case": case, "mode": mode, "status": row["modes"][mode]["status"]}),
            flush=True,
        )
    row["metrics"] = audit_case(directory, case, partition, row)
    print(
        json.dumps(
            {
                "case": case,
                "audited": True,
                "response_passed": row["metrics"]["response_passed"],
                "flight_passed": row["metrics"]["flight_passed"],
            }
        ),
        flush=True,
    )
    return row


def summarize(partition: str, rows: list[dict[str, Any]]) -> dict[str, Any]:
    ensure(
        [r["name"] for r in rows] == list(planned_cases(partition)),
        "all planned cases must occur exactly once in order",
    )
    failures = sum(item["status"] == "error" for r in rows for item in r["modes"].values())
    responses = sum(r["metrics"]["response_passed"] for r in rows)
    required = [r for r in rows if r["metrics"]["flight_required"]]
    flights = sum(r["metrics"]["flight_passed"] for r in required)
    return {
        "cases": len(rows),
        "executions": 2 * len(rows),
        "numerical_failures": failures,
        "response_passes": responses,
        "required_flight_cases": len(required),
        "flight_passes": flights,
        "passed": failures == 0 and responses == len(rows) and flights == len(required),
    }


def run_campaign(directory: Path, partition: str = "campaign", workers: int = 1) -> dict[str, Any]:
    ensure(type(workers) is int and 1 <= workers <= 2, "workers must be one or two")
    definition = protocol(partition)
    directory.mkdir(parents=True, exist_ok=False)
    save_json(directory, "protocol.json", definition)
    root = Path(__file__).resolve().parents[1]
    provenance, source = asdict(capture_software_provenance(root)), source_sha256(root)
    tasks = [(case, partition, str(directory)) for case in planned_cases(partition)]
    if workers == 1:
        rows = [_execute_case(task) for task in tasks]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(_execute_case, tasks))
    report = {
        "protocol": definition,
        "protocol_sha256": hashlib.sha256(canonical_json(definition)).hexdigest(),
        "software": provenance,
        "source_sha256": source,
        "cases": rows,
        "summary": summarize(partition, rows),
    }
    ensure(source_sha256(root) == source, "execution source changed during the campaign")
    save_json(directory, "report.json", report)
    return report


def verify_report(directory: Path) -> dict[str, Any]:
    report: dict[str, Any] = json.loads(
        (directory / "report.json").read_bytes(), object_pairs_hook=_unique_object
    )
    ensure(
        set(report)
        == {"protocol", "protocol_sha256", "software", "source_sha256", "cases", "summary"},
        "report schema mismatch",
    )
    ensure(
        report["source_sha256"] == source_sha256(Path(__file__).resolve().parents[1]),
        "verification requires the recorded execution source",
    )
    software = report["software"]
    ensure(
        isinstance(software, dict)
        and set(software)
        == {
            "package_version",
            "python_version",
            "numpy_version",
            "git_commit_sha",
            "git_worktree_clean",
        }
        and all(
            isinstance(software[key], str) and software[key]
            for key in ("package_version", "python_version", "numpy_version", "git_commit_sha")
        )
        and len(software["git_commit_sha"]) == 40
        and all(c in "0123456789abcdef" for c in software["git_commit_sha"])
        and type(software["git_worktree_clean"]) is bool,
        "invalid software provenance",
    )
    definition = protocol(report["protocol"]["partition"])
    equal_json(report["protocol"], definition, "protocol differs from frozen definition")
    ensure(
        report["protocol_sha256"] == hashlib.sha256(canonical_json(definition)).hexdigest(),
        "protocol digest mismatch",
    )
    equal_json(
        json.loads((directory / "protocol.json").read_bytes(), object_pairs_hook=_unique_object),
        definition,
        "protocol checkpoint mismatch",
    )
    partition = definition["partition"]
    ensure(
        [r["name"] for r in report["cases"]] == list(planned_cases(partition)),
        "missing or reordered cases",
    )
    for row in report["cases"]:
        ensure(set(row) == {"name", "modes", "metrics"}, "case schema mismatch")
        recomputed = audit_case(directory / row["name"], row["name"], partition, row)
        equal_json(
            row["metrics"], recomputed, "metrics differ from authenticated complete histories"
        )
    summary = summarize(partition, report["cases"])
    equal_json(report["summary"], summary, "summary differs from all planned outcomes")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", choices=("smoke", "campaign"), default="campaign")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify", type=Path)
    args = parser.parse_args()
    if (args.output is None) == (args.verify is None):
        parser.error("supply exactly one of --output and --verify")
    report = (
        verify_report(args.verify)
        if args.verify
        else run_campaign(args.output, args.partition, args.workers)
    )
    print(json.dumps(report["summary"], indent=2))
    return 0 if report["summary"]["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
