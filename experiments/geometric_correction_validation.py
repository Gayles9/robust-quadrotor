"""ADR 0018 bounded estimator-correction study; original campaign remains intact."""

import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.estimated_feedback_evidence import plain
from experiments.geometric_reimplementation_validation import (
    configuration as original_configuration,
)
from experiments.geometric_reimplementation_validation import execute, jobs, name_of, summarize
from experiments.trajectory_mission_validation import TARGETS

FREQUENCIES = (1.0, 1.5, 2.0)
PROFILES = ((1.0, 0.64), (1.5, 0.64), (2.0, 0.64), (1.0, 1.28), (1.5, 1.28))


def configuration(
    job: dict[str, Any], frequency: float, stiffness: float = 0.64, strategy: str = "rebase"
) -> dict[str, Any]:
    if (
        isinstance(frequency, (bool, np.bool_))
        or isinstance(stiffness, (bool, np.bool_))
        or (frequency, stiffness) not in PROFILES
    ):
        raise ValueError("frequency/stiffness must be a declared candidate")
    if strategy not in ("rebase", "measured") or (
        strategy == "measured" and (frequency, stiffness) != (1.0, 0.64)
    ):
        raise ValueError("measured strategy uses the original gains only")
    config = original_configuration(job)
    if job["controller"] == "geometric":
        config["geometric_controller"] = replace(
            config["geometric_controller"],
            rebase_estimator_corrections=strategy == "rebase",
            use_measured_acceleration=strategy == "measured" and job["mode"] != "true",
            attitude_stiffness=stiffness,
        )
        outer = config["position_controller"]
        config["position_controller"] = replace(
            outer,
            position_gain_W=np.array([frequency**2] * 2 + [outer.position_gain_W[2]]),
            velocity_gain_W=np.array([1.8 * frequency] * 2 + [outer.velocity_gain_W[2]]),
        )
    return config


def planned_jobs(stage: str) -> list[dict[str, Any]]:
    if stage == "development":
        return [jobs()[i] for i in (14, 15, 23, 25)]
    if stage != "qualification":
        raise ValueError("unknown study stage")
    return (
        jobs()
        + [
            dict(mode="hover", case="hover", controller="geometric", seed=seed, repeat=False)
            for seed in range(95000, 95004)
        ]
        + [
            dict(
                mode="estimated",
                case="nominal" if seed % 2 == 0 else "mild_wind",
                controller=controller,
                seed=seed,
                repeat=False,
            )
            for seed in range(96000, 96004)
            for controller in ("cascade", "geometric")
        ]
    )


def comparisons(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_name = {name_of(row["job"]): row for row in rows}
    result = []
    for row in rows:
        j = row["job"]
        if j["mode"] != "estimated" or j["controller"] != "geometric" or j["repeat"]:
            continue
        b = by_name.get(name_of({**j, "controller": "cascade"}), {}).get("metrics", {})
        a = row.get("metrics", {})
        ratio = effort = None
        if b.get("position_rmse_m", 0) > 0 and a.get("position_rmse_m") is not None:
            ratio = a["position_rmse_m"] / b["position_rmse_m"]
        if b.get("moment_effort_N2_m2_s", 0) > 0 and a.get("moment_effort_N2_m2_s") is not None:
            effort = a["moment_effort_N2_m2_s"] / b["moment_effort_N2_m2_s"]
        result.append(
            dict(
                job=j,
                rmse_ratio=ratio,
                effort_ratio=effort,
                passed=bool(
                    a.get("passed")
                    and b.get("passed")
                    and ratio is not None
                    and ratio <= 1
                    and effort is not None
                    and effort <= 2
                ),
            )
        )
    return result


def study_summary(rows: list[dict[str, Any]], stage: str, output: Path) -> dict[str, Any]:
    planned = planned_jobs(stage)
    complete = len(rows) == len(planned) and all(
        row.get("index") == i and row.get("job") == job
        for i, (row, job) in enumerate(zip(rows, planned, strict=False))
    )
    paired = comparisons(rows)
    candidates = [r for r in rows if r["job"]["controller"] == "geometric"]
    candidate_gates = bool(
        complete
        and all(r.get("metrics", {}).get("passed", False) for r in candidates)
        and all(p["passed"] for p in paired)
    )
    result = dict(
        complete_ledger=complete,
        paired_estimated=paired,
        candidate_tracking_and_effort_passed=candidate_gates,
        candidate_qualified=False,
    )
    if stage == "qualification":
        inherited = summarize(rows[: len(jobs())], output)
        result["original_matrix"] = inherited
        result["candidate_qualified"] = bool(
            candidate_gates
            and inherited["true_state_passed"]
            and all(r["mission_arrays_identical"] for r in inherited["repeats"])
        )
    return result


def verify_payloads(rows: list[dict[str, Any]], output: Path) -> dict[str, Any]:
    """Fail closed on missing or damaged flight evidence before qualification."""
    failures = []
    count = 0
    for row in rows:
        if not row.get("files"):
            failures.append(dict(directory=row["directory"], error="no saved payloads"))
        for record in row.get("files", []):
            count += 1
            path = output / row["directory"] / record["file"]
            try:
                actual = hashlib.sha256(path.read_bytes()).hexdigest()
            except OSError as error:
                failures.append(dict(file=str(path.relative_to(output)), error=str(error)))
                continue
            if actual != record["sha256"]:
                failures.append(
                    dict(
                        file=str(path.relative_to(output)), expected=record["sha256"], actual=actual
                    )
                )
    return dict(passed=bool(rows) and not failures, files_checked=count, failures=failures)


def run_item(
    item: tuple[int, dict[str, Any], str, float],
    *,
    stiffness: float = 0.64,
    strategy: str = "rebase",
) -> dict[str, Any]:
    index, job, output, frequency = item
    return execute(
        (index, job, output), config_override=configuration(job, frequency, stiffness, strategy)
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("development", "qualification"), required=True)
    parser.add_argument("--frequency", type=float, choices=FREQUENCIES, required=True)
    parser.add_argument("--stiffness", type=float, choices=(0.64, 1.28), default=0.64)
    parser.add_argument("--strategy", choices=("rebase", "measured"), default="rebase")
    parser.add_argument("--workers", type=int, choices=range(1, 5), default=3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    fingerprint = source_sha256(root)
    planned = planned_jobs(args.stage)
    protocol = dict(
        version=2,
        scope="ADR 0018; unchanged acceptance; no qualification from development cases",
        stage=args.stage,
        frequency_rad_s=args.frequency,
        attitude_stiffness_Nm=args.stiffness,
        strategy=args.strategy,
        jobs=planned,
        configurations=[
            plain(
                {
                    k: asdict(v) if hasattr(v, "__dataclass_fields__") else v
                    for k, v in configuration(
                        j, args.frequency, args.stiffness, args.strategy
                    ).items()
                }
            )
            for j in planned
        ],
        physical_targets=TARGETS,
        maximum_paired_rmse_ratio=1.0,
        maximum_estimated_effort_ratio=2.0,
        full_hover_peak_m=0.08,
        full_hover_window_s=[5.0, 65.0],
        comparator_note=(
            "Cascade retains its original gains; hover uses v2 context. Changing "
            "geometric outer gains is not an isolated control-law comparison."
        ),
    )
    raw = canonical_json(protocol)
    (args.output / "protocol.json").write_bytes(raw + b"\n")
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(
            pool.map(
                partial(run_item, stiffness=args.stiffness, strategy=args.strategy),
                [(i, j, str(args.output), args.frequency) for i, j in enumerate(planned)],
            )
        )
    verification = verify_payloads(rows, args.output)
    summary = study_summary(rows, args.stage, args.output)
    summary["candidate_qualified"] = summary["candidate_qualified"] and verification["passed"]
    report: dict[str, Any] = dict(
        source_sha256=fingerprint,
        source_unchanged=fingerprint == source_sha256(root),
        protocol_sha256=hashlib.sha256(raw).hexdigest(),
        rows=rows,
        payload_verification=verification,
        summary=summary,
    )
    (args.output / "report.json").write_bytes(canonical_json(report) + b"\n")
    print(json.dumps(report["summary"], indent=2), flush=True)
    gate = (
        "candidate_qualified"
        if args.stage == "qualification"
        else "candidate_tracking_and_effort_passed"
    )
    return 0 if report["source_unchanged"] and verification["passed"] and summary[gate] else 1


if __name__ == "__main__":
    raise SystemExit(main())
