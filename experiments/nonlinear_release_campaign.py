"""ADR0036 exactly 25 boundary-only flights against saved baselines; no zero prior."""

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
from experiments.durable_release_evidence import save_history, save_json
from experiments.estimated_feedback_evidence import (
    load_history,
    pack_history,
    unpack_history,
)
from experiments.navigation_feedback_oracle import noise_pairing
from experiments.nonlinear_release import nonlinear_release_prediction
from experiments.nonlinear_release_validation import ADR, ROOT, definition, load_inputs
from experiments.residual_hover_diagnostic import INPUT_SHA, authenticate
from experiments.robustness_evidence import ensure, equal_json, load_json
from experiments.supported_start import audit, plain_configuration
from experiments.supported_start_validation import score
from quadrotor_math.run_manifest import capture_software_provenance


def modes(job: protocol.Job) -> tuple[str, ...]:
    return ("aligned",) if job.case in protocol.CLEAN else ("off", "on")


def clean_comparison(
    case: str, baseline: dict[str, Any], candidate: dict[str, Any]
) -> dict[str, Any]:
    conditions = dict(
        original_flight_limits=candidate["flight_passed"],
        rmse_no_regression=candidate["tracking_rmse_m"] <= baseline["tracking_rmse_m"] + 1e-12,
    )
    if case == "nominal_hover":
        conditions["hover_no_regression"] = (
            candidate["hover_peak_m"] is not None
            and candidate["hover_peak_m"] <= baseline["hover_peak_m"] + 1e-12
        )
    return dict(conditions=conditions, passed=all(conditions.values()))


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    ensure([r["name"] for r in rows] == [j.name for j in protocol.jobs()], "flight job ledger")
    ensure(
        all(set(r["modes"]) == set(modes(j)) for r, j in zip(rows, protocol.jobs(), strict=True)),
        "flight mode ledger",
    )
    return dict(
        scientific_flights=sum(len(r["modes"]) for r in rows),
        clean_comparisons_passed=sum(r["comparison"]["passed"] for r in rows[:9]),
        hover_flights_passed=sum(
            r["modes"]["aligned"]["candidate_metrics"]["flight_passed"]
            for r in rows[:9]
            if r["case"] == "nominal_hover"
        ),
        fault_comparisons_passed=sum(r["comparison"]["passed"] for r in rows[9:]),
        frozen_comparison_passed=all(r["comparison"]["passed"] for r in rows),
        zero_velocity_conditioning=False,
        adopted_as_default=False,
        fresh_validation_complete=False,
        flight_qualification=False,
    )


def execute_job(
    job: protocol.Job,
    *,
    campaign: Path,
    output: Path,
    parent_rows: dict[str, Any],
    verify: bool = False,
) -> dict[str, Any]:
    old = parent_rows[job.name]
    prepared, args, _ = load_inputs(campaign, job, old)
    target = output / job.name
    if verify:
        saved = json.loads((target / "case.json").read_bytes())
    else:
        target.mkdir()
    row: dict[str, Any] = dict(name=job.name, **asdict(job), modes={})
    candidate_results = {}
    candidate_diagnostics = {}
    for index, mode in enumerate(modes(job)):
        baseline_item = old["modes"][mode]
        baseline_index = protocol.modes(job).index(mode)
        baseline_arrays = load_history(
            campaign / job.name, baseline_index, baseline_item["history"]
        )
        baseline, _ = unpack_history(baseline_arrays)
        baseline_diagnostic = load_json(
            campaign / job.name, f"diagnostic-{mode}.json", baseline_item["diagnostic"]
        )
        baseline_audit = audit(
            prepared,
            "aligned",
            baseline,
            baseline_diagnostic,
            supervision="off" if mode == "off" else "on",
        )
        equal_json(baseline_audit, baseline_item["audit"], "baseline complete replay")
        baseline_metrics = score(baseline_arrays, job.case)
        baseline_metrics["abort_reason"] = baseline.mission.abort_reason
        equal_json(baseline_metrics, baseline_item["metrics"], "baseline complete score")
        if verify:
            item = saved["modes"][mode]
            history = item["history"]
            diagnostic_record = item["diagnostic"]
            config_record = item["configuration"]
            configuration = load_json(target, f"configuration-{mode}.json", config_record)
        else:
            configuration = {key: plain_configuration(value) for key, value in args.items()}
            config_record = save_json(target, f"configuration-{mode}.json", configuration)
            # Only this block executes a scientific flight. Save before auditing;
            # a valid result is never rerun to obtain a more favorable score.
            with nonlinear_release_prediction(prepared, "online") as trace:
                result, diagnostic = protocol.execute(prepared, mode)
            diagnostic["release_prediction"] = trace
            history = save_history(target, index, pack_history(result, result.mission))
            diagnostic_record = save_json(target, f"diagnostic-{mode}.json", diagnostic)
            save_json(
                target,
                f"flight-checkpoint-{mode}.json",
                dict(history=history, diagnostic=diagnostic_record, configuration=config_record),
            )
        equal_json(
            configuration,
            {key: plain_configuration(value) for key, value in args.items()},
            "unchanged flight configuration",
        )
        equal_json(
            configuration,
            load_json(
                campaign / job.name, f"configuration-{mode}.json", baseline_item["configuration"]
            ),
            "baseline/candidate configuration identity",
        )
        arrays = load_history(target, index, history)
        restored, _ = unpack_history(arrays)
        diagnostic = load_json(target, f"diagnostic-{mode}.json", diagnostic_record)
        original_diagnostic = {
            key: value for key, value in diagnostic.items() if key != "release_prediction"
        }
        with nonlinear_release_prediction(prepared, "replay") as trace:
            checks = audit(
                prepared,
                "aligned",
                restored,
                original_diagnostic,
                supervision="off" if mode == "off" else "on",
            )
        equal_json(trace, diagnostic["release_prediction"], "complete first-prediction trace")
        ensure(not trace["zero_velocity_conditioning"], "velocity constraint entered flight")
        np.testing.assert_array_equal(
            np.array(trace["input"]["joint_covariance"])[3:6, 3:6],
            args["estimator_configuration"].initial_covariance[3:6, 3:6],
        )
        candidate_metrics = score(arrays, job.case)
        candidate_metrics["abort_reason"] = restored.mission.abort_reason
        row["modes"][mode] = dict(
            configuration=config_record,
            history=history,
            diagnostic=diagnostic_record,
            audit=checks,
            baseline_audit=baseline_audit,
            baseline_metrics=baseline_metrics,
            candidate_metrics=candidate_metrics,
            noise_pairing=noise_pairing(baseline_arrays, arrays, args),
            metric_changes={
                key: candidate_metrics[key] - baseline_metrics[key]
                for key in (
                    "tracking_rmse_m",
                    "final_position_error_m",
                    "final_true_speed_m_s",
                    "terminal_time_s",
                )
            },
        )
        if job.case == "nominal_hover":
            row["modes"][mode]["metric_changes"]["hover_peak_m"] = (
                candidate_metrics["hover_peak_m"] - baseline_metrics["hover_peak_m"]
            )
        candidate_results[mode] = restored
        candidate_diagnostics[mode] = original_diagnostic
        print(
            json.dumps(dict(job=job.name, mode=mode, verify=verify, metrics=candidate_metrics)),
            flush=True,
        )
    if job.case in protocol.CLEAN:
        item = row["modes"]["aligned"]
        row["comparison"] = clean_comparison(
            job.case, item["baseline_metrics"], item["candidate_metrics"]
        )
    else:
        protocol.common_bias_prefix(candidate_results)
        response = protocol.response_metrics(prepared, candidate_results, candidate_diagnostics)
        row["response"] = response
        conditions = dict(response["response_conditions"])
        if response["flight_required"]:
            conditions["both_recovery_flights"] = response["flight_passed"]
        row["comparison"] = dict(conditions=conditions, passed=all(conditions.values()))
    if verify:
        equal_json(saved, row, "complete flight reconstruction")
    else:
        save_json(target, "case.json", row)
    print(job.name, "comparison", row["comparison"]["passed"], flush=True)
    return row


def run(
    campaign: Path, uncertainty: Path, output: Path, workers: int, *, verify: bool = False
) -> dict[str, Any]:
    ensure(type(workers) is int and workers in (1, 2), "one or two worker processes")
    uncertainty_report = json.loads((uncertainty / "report.json").read_bytes())
    equal_json(uncertainty_report["protocol"], definition(), "verified uncertainty source/protocol")
    ensure(
        uncertainty_report["summary"]["boundary_flight_eligible"],
        "original-prior uncertainty gate failed",
    )
    parent = authenticate(campaign)
    frozen = dict(
        design="ADR0036 boundary-only flight comparison",
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        input_report_sha256=INPUT_SHA,
        uncertainty_report_sha256=hashlib.sha256(
            (uncertainty / "report.json").read_bytes()
        ).hexdigest(),
        jobs=[j.name for j in protocol.jobs()],
        planned_flights=25,
        zero_velocity_conditioning=False,
    )
    if verify:
        saved = json.loads((output / "report.json").read_bytes())
        equal_json(saved["protocol"], frozen, "flight protocol")
        equal_json(
            json.loads((output / "protocol.json").read_bytes()), frozen, "protocol checkpoint"
        )
        software = saved["software"]
    else:
        output.mkdir(parents=True, exist_ok=False)
        save_json(output, "protocol.json", frozen)
        software = asdict(capture_software_provenance(ROOT))
    execute = partial(
        execute_job,
        campaign=campaign,
        output=output,
        parent_rows={r["name"]: r for r in parent["cases"]},
        verify=verify,
    )
    try:
        if workers == 1:
            rows = [execute(j) for j in protocol.jobs()]
        else:
            with ProcessPoolExecutor(max_workers=workers) as pool:
                rows = list(pool.map(execute, protocol.jobs()))
        ensure(source_sha256(ROOT) == frozen["source_sha256"], "source changed during campaign")
        report = dict(protocol=frozen, software=software, cases=rows, summary=summarize(rows))
        ensure(report["summary"]["scientific_flights"] == 25, "exact flight count")
        if verify:
            equal_json(saved, report, "complete campaign report")
        else:
            save_json(output, "report.json", report)
    except (
        ValueError,
        RuntimeError,
        FloatingPointError,
        np.linalg.LinAlgError,
        AssertionError,
    ) as error:
        if not verify:
            save_json(
                output,
                "failure.json",
                dict(protocol=frozen, error=f"{type(error).__name__}: {error}"),
            )
        raise
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", required=True, type=Path)
    parser.add_argument("--uncertainty", required=True, type=Path)
    parser.add_argument("--workers", type=int, default=2)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output", type=Path)
    group.add_argument("--verify", type=Path)
    args = parser.parse_args()
    result = run(
        args.campaign,
        args.uncertainty,
        args.verify or args.output,
        args.workers,
        verify=args.verify is not None,
    )
    print(json.dumps(result["summary"], indent=2), flush=True)
    return 0 if result["summary"]["frozen_comparison_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
