"""ADR 0031: 34 frozen independent supported-start flights with saved replay."""

import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np

from experiments import supported_validation_protocol as protocol
from experiments.attitude_control_validation import source_sha256
from experiments.estimated_feedback_evidence import (
    load_history,
    pack_history,
    save_history,
    unpack_history,
)
from experiments.robustness_evidence import ensure, equal_json, load_json, save_json
from experiments.robustness_protocol import faults, policies
from experiments.supported_start import PreparedStart, audit, configuration_for, plain_configuration
from experiments.supported_start_validation import score
from quadrotor_math.estimated_mission import EstimatedMissionResult
from quadrotor_math.prearm_alignment import PrearmStatus
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0031-independent-supported-start-validation.md"


def definition() -> dict[str, Any]:
    scenarios = []
    for job in protocol.jobs():
        args = protocol.configuration(job, "campaign")
        health, response = policies(args, "campaign")
        scenarios.append(
            dict(
                **asdict(job),
                name=job.name,
                modes=protocol.modes(job),
                configuration={k: plain_configuration(v) for k, v in args.items()},
                health=asdict(health),
                response=asdict(response),
                faults=[asdict(f) for f in faults(job.case, "campaign", args)],
            )
        )
    return dict(
        design="ADR 0031 independent supported-start validation",
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        jobs=scenarios,
        flights=34,
        prearm_time_s=0.5025,
        limits=dict(
            hover_peak_m=0.08,
            hover_window_s=[5.0, 11.0],
            rmse_m=0.15,
            final_position_m=0.15,
            final_speed_m_s=0.15,
        ),
        reserved_geometric_seeds_opened=False,
    )


def comparison(
    prepared: PreparedStart,
    row: dict[str, Any],
    results: dict[str, EstimatedMissionResult],
    diagnostics: dict[str, dict[str, Any]],
) -> None:
    protocol.common_bias_prefix(results)
    if prepared.case in protocol.CLEAN:
        row["comparison"] = protocol.clean_comparison(
            prepared.case,
            row["modes"]["unaligned"]["metrics"],
            row["modes"]["aligned"]["metrics"],
        )
    else:
        response = protocol.response_metrics(prepared, results, diagnostics)
        row["response"] = response
        conditions = dict(response["response_conditions"])
        if response["flight_required"]:
            conditions["both_recovery_flights"] = response["flight_passed"]
        row["comparison"] = dict(conditions=conditions, passed=all(conditions.values()))


def execute_job(job: protocol.Job, output: Path, partition: str = "campaign") -> dict[str, Any]:
    directory = output / job.name
    directory.mkdir()
    prepared = protocol.prepare_job(job, partition)
    support_payload: dict[str, Any] = dict(prepared.evidence)
    with (directory / "support.npz").open("xb") as stream:
        np.savez_compressed(stream, **support_payload)
    row: dict[str, Any] = dict(
        **asdict(job),
        name=job.name,
        status=prepared.status.value,
        reasons=prepared.reasons,
        support=dict(
            file="support.npz",
            sha256=hashlib.sha256((directory / "support.npz").read_bytes()).hexdigest(),
        ),
        modes={},
    )
    if prepared.status is not PrearmStatus.RELEASED:
        save_json(directory, "case.json", row)
        print(
            json.dumps(dict(job=job.name, status=prepared.status.value, reasons=prepared.reasons)),
            flush=True,
        )
        return row
    assert prepared.release is not None
    row["release"] = save_json(directory, "release.json", asdict(prepared.release))
    protocol.paired_initialization(prepared)
    results, diagnostics = {}, {}
    for index, mode in enumerate(protocol.modes(job)):
        args = configuration_for(prepared, protocol.alignment_mode(mode))
        config = save_json(
            directory,
            f"configuration-{mode}.json",
            {k: plain_configuration(v) for k, v in args.items()},
        )
        try:
            result, diagnostic = protocol.execute(prepared, mode)
            history = save_history(directory, index, pack_history(result, result.mission))
            saved_diagnostic = save_json(directory, f"diagnostic-{mode}.json", diagnostic)
            arrays = load_history(directory, index, history)
            restored, _ = unpack_history(arrays)
            decoded = load_json(directory, f"diagnostic-{mode}.json", saved_diagnostic)
            checked = audit(
                prepared,
                protocol.alignment_mode(mode),
                restored,
                decoded,
                supervision="off" if mode == "off" else "on",
            )
            metrics = score(arrays, job.case)
            metrics["abort_reason"] = restored.mission.abort_reason
            row["modes"][mode] = dict(
                status="ok",
                configuration=config,
                history=history,
                diagnostic=saved_diagnostic,
                audit=checked,
                metrics=metrics,
            )
            results[mode], diagnostics[mode] = restored, decoded
            print(json.dumps(dict(job=job.name, mode=mode, metrics=metrics)), flush=True)
        except (ValueError, RuntimeError, FloatingPointError, np.linalg.LinAlgError) as error:
            row["modes"][mode] = dict(
                status="implementation_or_evidence_failure",
                error=f"{type(error).__name__}: {error}",
            )
            save_json(directory, "failure.json", row)
            raise
    comparison(prepared, row, results, diagnostics)
    save_json(directory, "case.json", row)
    print(json.dumps(dict(job=job.name, comparison=row["comparison"]["passed"])), flush=True)
    return row


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    jobs = protocol.jobs()
    ensure([r["name"] for r in rows] == [j.name for j in jobs], "job ledger mismatch")
    valid = [
        r["status"] == "released"
        and set(r["modes"]) == set(protocol.modes(j))
        and all(m["status"] == "ok" for m in r["modes"].values())
        for j, r in zip(jobs, rows, strict=True)
    ]
    passed = [v and r["comparison"]["passed"] for v, r in zip(valid, rows, strict=True)]
    return dict(
        planned_pairs=17,
        planned_flights=34,
        flights=sum(len(r["modes"]) for r in rows),
        rejected_acquisitions=sum(r["status"] != "released" for r in rows),
        complete_evidence=all(valid),
        clean_comparisons_passed=sum(passed[:9]),
        fault_comparisons_passed=sum(passed[9:]),
        integration_go=all(passed),
        mass_qualified=False,
        hardware_qualified=False,
        population_reliability_qualified=False,
    )


def verify_job(
    directory: Path, job: protocol.Job, row: dict[str, Any], partition: str = "campaign"
) -> dict[str, Any]:
    target = directory / job.name
    ensure(
        (row["case"], row["seed"], row["name"]) == (job.case, job.seed, job.name),
        "case identity mismatch",
    )
    equal_json(json.loads((target / "case.json").read_bytes()), row, "case checkpoint mismatch")
    prepared = protocol.prepare_job(job, partition)
    ensure(prepared.status.value == row["status"], "support disposition mismatch")
    equal_json(prepared.reasons, row["reasons"], "support rejection mismatch")
    ensure(row["support"]["file"] == "support.npz", "support path mismatch")
    ensure(
        hashlib.sha256((target / "support.npz").read_bytes()).hexdigest()
        == row["support"]["sha256"],
        "support digest mismatch",
    )
    with np.load(target / "support.npz", allow_pickle=False) as saved:
        ensure(set(saved.files) == set(prepared.evidence), "support field mismatch")
        for key, value in prepared.evidence.items():
            ensure(np.array_equal(saved[key], value), "support reconstruction mismatch")
    if prepared.release is None:
        ensure(not row["modes"], "rejected acquisition flew")
        return row
    equal_json(
        load_json(target, "release.json", row["release"]),
        asdict(prepared.release),
        "release mismatch",
    )
    ensure(set(row["modes"]) == set(protocol.modes(job)), "mode ledger mismatch")
    protocol.paired_initialization(prepared)
    results, diagnostics = {}, {}
    for index, mode in enumerate(protocol.modes(job)):
        item = row["modes"][mode]
        ensure(item["status"] == "ok", "incomplete flight evidence")
        args = configuration_for(prepared, protocol.alignment_mode(mode))
        equal_json(
            load_json(target, f"configuration-{mode}.json", item["configuration"]),
            {k: plain_configuration(v) for k, v in args.items()},
            "configuration mismatch",
        )
        arrays = load_history(target, index, item["history"])
        result, _ = unpack_history(arrays)
        diagnostic = load_json(target, f"diagnostic-{mode}.json", item["diagnostic"])
        checked = audit(
            prepared,
            protocol.alignment_mode(mode),
            result,
            diagnostic,
            supervision="off" if mode == "off" else "on",
        )
        equal_json(checked, item["audit"], "audit mismatch")
        metrics = score(arrays, job.case)
        metrics["abort_reason"] = result.mission.abort_reason
        equal_json(metrics, item["metrics"], "full-flight score mismatch")
        results[mode], diagnostics[mode] = result, diagnostic
    recomputed: dict[str, Any] = {"modes": row["modes"]}
    comparison(prepared, recomputed, results, diagnostics)
    for key in ("comparison",) + (("response",) if job.case in protocol.FAULTS else ()):
        equal_json(recomputed[key], row[key], key + " mismatch")
    return row


def run(output: Path, workers: int = 4) -> dict[str, Any]:
    ensure(type(workers) is int and 1 <= workers <= 4, "one to four worker processes required")
    output.mkdir(parents=True, exist_ok=False)
    frozen = definition()
    source = source_sha256(ROOT)
    software = asdict(capture_software_provenance(ROOT))
    save_json(output, "protocol.json", frozen)
    execute = partial(execute_job, output=output)
    if workers == 1:
        rows = [execute(job) for job in protocol.jobs()]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(execute, protocol.jobs()))
    ensure(source_sha256(ROOT) == source, "source changed during campaign")
    equal_json(definition(), frozen, "protocol changed during campaign")
    report = dict(
        protocol=frozen,
        source_sha256=source,
        software=software,
        cases=rows,
        summary=summarize(rows),
    )
    save_json(output, "report.json", report)
    return report


def verify(directory: Path) -> dict[str, Any]:
    report: dict[str, Any] = json.loads((directory / "report.json").read_bytes())
    ensure(report["source_sha256"] == source_sha256(ROOT), "execution source mismatch")
    equal_json(report["protocol"], definition(), "frozen protocol mismatch")
    equal_json(
        json.loads((directory / "protocol.json").read_bytes()),
        report["protocol"],
        "protocol checkpoint mismatch",
    )
    # Fail on missing, duplicate or reordered jobs before decoding large payloads.
    summarize(report["cases"])
    for job, row in zip(protocol.jobs(), report["cases"], strict=True):
        verify_job(directory, job, row)
        print(job.name, "full saved replay PASS", flush=True)
    equal_json(summarize(report["cases"]), report["summary"], "summary mismatch")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--output", type=Path)
    action.add_argument("--verify", type=Path)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()
    report = verify(args.verify) if args.verify else run(args.output, args.workers)
    print(json.dumps(report["summary"], indent=2), flush=True)
    return 0 if report["summary"]["integration_go"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
