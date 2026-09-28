"""One frozen vertical candidate against saved ADR 0023 evidence; ADR 0024.

No gains, protocol limits, baseline identity or reserved-seed choices are CLI inputs.
"""

import argparse
import hashlib
import json
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from functools import partial
from pathlib import Path
from typing import Any

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.position_control_validation import _unique_object
from experiments.robustness_evidence import ensure, equal_json, load_json, save_json
from experiments.robustness_protocol import planned_cases
from experiments.robustness_protocol import protocol as base_protocol
from experiments.robustness_validation import _execute_case, audit_case
from experiments.robustness_validation import summarize as base_summary
from quadrotor_math.run_manifest import capture_software_provenance
from quadrotor_math.vertical_compensation import VerticalCompensationPolicy

ROOT = Path(__file__).resolve().parents[1]
BASELINE_REPORT_SHA256 = "ec85a0d6b55acc278acb18b1c7af7b02cb56c6e985dc48eaf66fbc7fd520bb09"
POLICY = VerticalCompensationPolicy(0.5, 1.5, 0.02)


def read_baseline(directory: Path, partition: str) -> tuple[dict[str, Any], bytes]:
    """Pin the previously reconstructed campaign; smoke is re-audited here."""
    raw = (directory / "report.json").read_bytes()
    if partition == "campaign":
        ensure(
            hashlib.sha256(raw).hexdigest() == BASELINE_REPORT_SHA256,
            "baseline differs from the audited campaign",
        )
    report: dict[str, Any] = json.loads(raw, object_pairs_hook=_unique_object)
    equal_json(report["protocol"], base_protocol(partition), "baseline protocol mismatch")
    ensure(
        report["protocol_sha256"]
        == hashlib.sha256(canonical_json(base_protocol(partition))).hexdigest(),
        "baseline protocol digest mismatch",
    )
    ensure(
        [r["name"] for r in report["cases"]] == list(planned_cases(partition)),
        "baseline cases mismatch",
    )
    for row in report["cases"]:
        ensure(set(row["modes"]) == {"off", "on"}, "baseline mode mismatch")
        for index, mode in enumerate(("off", "on")):
            item = row["modes"][mode]
            ensure(item["status"] == "ok", "baseline execution must have complete evidence")
            for j, record in enumerate(item["history"]):
                expected = (
                    f"trial-{index:03d}-data.npz"
                    if j == 0
                    else f"trial-{index:03d}-cov-{j - 1:03d}.npz"
                )
                ensure(
                    set(record) == {"file", "sha256"} and record["file"] == expected,
                    "baseline history filename mismatch",
                )
                ensure(
                    hashlib.sha256((directory / row["name"] / expected).read_bytes()).hexdigest()
                    == record["sha256"],
                    "baseline history byte digest mismatch",
                )
            load_json(directory / row["name"], f"diagnostic-{mode}.json", item["diagnostic"])
        if partition == "smoke":
            equal_json(
                row["metrics"],
                audit_case(directory / row["name"], row["name"], partition, row),
                "baseline smoke metrics differ",
            )
    equal_json(
        report["summary"], base_summary(partition, report["cases"]), "baseline summary mismatch"
    )
    return report, raw


def protocol(partition: str, baseline_digest: str) -> dict[str, Any]:
    ensure(
        partition != "campaign" or baseline_digest == BASELINE_REPORT_SHA256,
        "unexpected campaign baseline",
    )
    return {
        "name": "bounded_vertical_compensation",
        "version": 1,
        "partition": partition,
        "base_protocol": base_protocol(partition),
        "baseline_report_sha256": baseline_digest,
        "compensation_policy": asdict(POLICY),
        "acceptance": (
            "all response gates; mass flight pass; preserve inherited passing flight conditions; "
            "hover peak no increase beyond 1e-12 m"
        ),
        "semantics": (
            "one experimental vertical candidate; no tuning or default promotion; "
            "smoke is not qualification"
        ),
    }


def compare(directory: Path, row: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    metrics = row["metrics"]
    if metrics.get("numerical_failure", False):
        return {
            "conditions": {"complete_pair": False},
            "passed": False,
            "metric_deltas": {},
            "compensation": {},
        }
    conditions: dict[str, bool] = {"response": metrics["response_passed"]}
    deltas: dict[str, Any] = {}
    traces: dict[str, Any] = {}
    for mode in ("off", "on"):
        item = row["modes"][mode]
        if item["status"] != "ok":
            conditions[mode + "_numerical"] = False
            continue
        actual, original = metrics["modes"][mode], baseline["metrics"]["modes"][mode]
        if metrics["flight_required"]:
            for name, passed in original["flight_conditions"].items():
                if passed:
                    conditions[mode + "_retains_" + name] = actual["flight_conditions"][name]
        if row["name"] == "mass_tracking":
            conditions[mode + "_mass_flight"] = actual["flight_passed"]
        if row["name"] == "nominal_hover":
            conditions[mode + "_hover_peak"] = (
                actual["hover_peak_m"] <= original["hover_peak_m"] + 1e-12
            )
        deltas[mode] = {
            name: actual[name] - original[name]
            for name in (
                "tracking_rmse_m",
                "final_position_error_m",
                "final_true_speed_m_s",
                "terminal_time_s",
            )
        }
        trace = load_json(directory, f"diagnostic-{mode}.json", item["diagnostic"])["vertical"]
        values = [
            v
            for sample in trace
            for v in (sample["applied_acceleration"], sample["next_acceleration"])
        ]
        conditions[mode + "_bounded"] = all(abs(v) <= POLICY.maximum_acceleration for v in values)
        traces[mode] = {
            "attempts": len(trace),
            "minimum_acceleration": min(values, default=0),
            "maximum_acceleration": max(values, default=0),
            "terminal_acceleration": trace[-1]["next_acceleration"] if trace else 0,
            "update_counts": dict(Counter(s["update_reason"] for s in trace)),
            "bound_limited_attempts": sum(s["bound_limited"] for s in trace),
        }
    return {
        "conditions": conditions,
        "passed": all(conditions.values()),
        "metric_deltas": deltas,
        "compensation": traces,
    }


def execute(task: tuple[str, str, str], *, baseline: dict[str, Any]) -> dict[str, Any]:
    row = _execute_case(task, compensation_policy=POLICY)
    previous = next(r for r in baseline["cases"] if r["name"] == row["name"])
    row["comparison"] = compare(Path(task[2]) / row["name"], row, previous)
    print(
        json.dumps({"case": row["name"], "comparison_passed": row["comparison"]["passed"]}),
        flush=True,
    )
    return row


def summarize(partition: str, rows: list[dict[str, Any]]) -> dict[str, Any]:
    summary = base_summary(partition, rows)
    summary["flight_campaign_passed"] = summary.pop("passed")
    summary["comparison_passes"] = sum(r["comparison"]["passed"] for r in rows)
    summary["candidate_accepted"] = (
        partition == "campaign"
        and summary["numerical_failures"] == 0
        and summary["comparison_passes"] == len(rows)
    )
    return summary


def run(
    directory: Path, baseline_directory: Path, partition: str, workers: int = 1
) -> dict[str, Any]:
    ensure(type(workers) is int and 1 <= workers <= 2, "workers must be one or two")
    baseline, raw = read_baseline(baseline_directory, partition)
    definition = protocol(partition, hashlib.sha256(raw).hexdigest())
    directory.mkdir(parents=True, exist_ok=False)
    save_json(directory, "protocol.json", definition)
    (directory / "baseline-report.json").write_bytes(raw)
    source, software = source_sha256(ROOT), asdict(capture_software_provenance(ROOT))
    tasks = [(case, partition, str(directory)) for case in planned_cases(partition)]
    worker = partial(execute, baseline=baseline)
    if workers == 1:
        rows = [worker(task) for task in tasks]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(worker, tasks))
    report = {
        "protocol": definition,
        "protocol_sha256": hashlib.sha256(canonical_json(definition)).hexdigest(),
        "source_sha256": source,
        "software": software,
        "cases": rows,
        "summary": summarize(partition, rows),
    }
    ensure(source_sha256(ROOT) == source, "execution source changed during candidate campaign")
    save_json(directory, "report.json", report)
    return report


def verify(directory: Path, baseline_directory: Path) -> dict[str, Any]:
    report: dict[str, Any] = json.loads(
        (directory / "report.json").read_bytes(), object_pairs_hook=_unique_object
    )
    ensure(
        set(report)
        == {"protocol", "protocol_sha256", "source_sha256", "software", "cases", "summary"},
        "report schema mismatch",
    )
    ensure(
        report["source_sha256"] == source_sha256(ROOT),
        "verification requires recorded execution source",
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
            isinstance(software[k], str) and software[k]
            for k in ("package_version", "python_version", "numpy_version", "git_commit_sha")
        )
        and len(software["git_commit_sha"]) == 40
        and all(c in "0123456789abcdef" for c in software["git_commit_sha"])
        and type(software["git_worktree_clean"]) is bool,
        "invalid software provenance",
    )
    partition = report["protocol"]["partition"]
    baseline, raw = read_baseline(baseline_directory, partition)
    ensure((directory / "baseline-report.json").read_bytes() == raw, "baseline checkpoint mismatch")
    definition = protocol(partition, hashlib.sha256(raw).hexdigest())
    equal_json(report["protocol"], definition, "protocol differs from frozen candidate")
    ensure(
        report["protocol_sha256"] == hashlib.sha256(canonical_json(definition)).hexdigest(),
        "protocol digest mismatch",
    )
    equal_json(
        json.loads((directory / "protocol.json").read_bytes(), object_pairs_hook=_unique_object),
        definition,
        "protocol checkpoint mismatch",
    )
    ensure(
        [r["name"] for r in report["cases"]] == list(planned_cases(partition)),
        "missing or reordered candidate cases",
    )
    for row, previous in zip(report["cases"], baseline["cases"], strict=True):
        ensure(
            set(row) == {"name", "modes", "metrics", "comparison"}, "candidate case schema mismatch"
        )
        metrics = audit_case(
            directory / row["name"], row["name"], partition, row, compensation_policy=POLICY
        )
        equal_json(row["metrics"], metrics, "candidate metrics differ from complete histories")
        equal_json(
            row["comparison"],
            compare(directory / row["name"], row, previous),
            "candidate comparison differs",
        )
    equal_json(
        report["summary"], summarize(partition, report["cases"]), "candidate summary differs"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", choices=("smoke", "campaign"), default="campaign")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--baseline", type=Path, required=True)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--output", type=Path)
    action.add_argument("--verify", type=Path)
    args = parser.parse_args()
    report = (
        verify(args.verify, args.baseline)
        if args.verify
        else run(args.output, args.baseline, args.partition, args.workers)
    )
    print(json.dumps(report["summary"], indent=2))
    return 0 if report["summary"]["candidate_accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
