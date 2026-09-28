"""ADR 0030: eight frozen supported-release flights and complete evidence replay."""

import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import source_sha256
from experiments.early_flight_diagnostic import REPORT_DIGESTS, authenticate_campaign, down_axis
from experiments.estimated_feedback_evidence import (
    load_history,
    pack_history,
    save_history,
    unpack_history,
)
from experiments.robustness_evidence import ensure, equal_json, load_json, save_json
from experiments.supported_start import (
    CASES,
    MODES,
    audit,
    configuration_for,
    fly,
    plain_configuration,
    prepare,
    verify_noise,
)
from quadrotor_math.missions import MissionPhase
from quadrotor_math.prearm_alignment import PrearmStatus
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0030-supported-start-flight-evaluation.md"


def score(arrays: dict[str, Any], case: str) -> dict[str, Any]:
    time = arrays["mission_time_s"]
    error = arrays["mission_position_W"] - arrays["mission_reference_position_W"]
    norm = np.linalg.norm(error, axis=1)
    complete = bool(arrays["mission_phase"][-1] == MissionPhase.COMPLETE)
    metrics: dict[str, Any] = dict(
        tracking_rmse_m=float(np.sqrt(np.mean(norm**2))),
        final_position_error_m=float(norm[-1]),
        final_true_speed_m_s=float(np.linalg.norm(arrays["mission_velocity_W"][-1])),
        terminal_time_s=float(time[-1]),
        elapsed_including_support_s=float(time[-1] + 0.5025),
        terminal_phase=MissionPhase(int(arrays["mission_phase"][-1])).name,
    )
    conditions = dict(
        complete=complete,
        rmse=metrics["tracking_rmse_m"] <= 0.15,
        final_position=metrics["final_position_error_m"] <= 0.15,
        final_speed=metrics["final_true_speed_m_s"] <= 0.15,
    )
    if case == "nominal_hover":
        mask = (time >= 5) & (time <= 11)
        metrics["hover_peak_m"] = float(norm[mask].max()) if np.any(mask) else None
        conditions["full_hover"] = bool(
            time[-1] >= 11
            and metrics["hover_peak_m"] is not None
            and metrics["hover_peak_m"] <= 0.08
        )
    true_axis, estimated_axis = (
        down_axis(arrays["mission_q_WB"]),
        down_axis(arrays["estimate_q_WB"]),
    )
    axis_error = np.rad2deg(
        np.arctan2(
            np.linalg.norm(np.cross(true_axis, estimated_axis), axis=1),
            np.sum(true_axis * estimated_axis, axis=1),
        )
    )
    metrics["axis_estimation_deg"] = dict(
        initial=float(axis_error[0]),
        early_peak=float(axis_error[time <= 1].max()),
        full_peak=float(axis_error.max()),
    )
    metrics["flight_conditions"], metrics["flight_passed"] = conditions, all(conditions.values())
    return metrics


def compare(
    case: str, original: dict[str, Any], unaligned: dict[str, Any], aligned: dict[str, Any]
) -> dict[str, Any]:
    conditions = {}
    for label, other in (("original", original), ("supported_unaligned", unaligned)):
        for key, passed in other["flight_conditions"].items():
            if passed:
                conditions[label + "_retains_" + key] = aligned["flight_conditions"].get(key, False)
        if case == "nominal_hover":
            peak, ref = aligned.get("hover_peak_m"), other.get("hover_peak_m")
            conditions[label + "_hover_no_regression"] = (
                peak is not None and ref is not None and peak <= ref + 1e-12
            )
    if case == "nominal_hover":
        conditions["aligned_hover_flight"] = aligned["flight_passed"]
    return dict(
        conditions=conditions,
        passed=all(conditions.values()),
        delta_from_supported_unaligned={
            k: aligned[k] - unaligned[k]
            for k in (
                "tracking_rmse_m",
                "final_position_error_m",
                "final_true_speed_m_s",
                "terminal_time_s",
            )
        },
        delta_from_original={
            k: aligned[k] - original[k]
            for k in (
                "tracking_rmse_m",
                "final_position_error_m",
                "final_true_speed_m_s",
                "terminal_time_s",
            )
        },
    )


def _case(case: str, *, output: Path, baseline: Path) -> dict[str, Any]:
    directory = output / case
    directory.mkdir()
    prepared = prepare(case)
    support_payload: dict[str, Any] = dict(prepared.evidence)
    with (directory / "support.npz").open("xb") as stream:
        np.savez_compressed(stream, **support_payload)
    record: dict[str, Any] = dict(
        name=case,
        status=prepared.status.value,
        reasons=prepared.reasons,
        support=dict(
            file="support.npz",
            sha256=hashlib.sha256((directory / "support.npz").read_bytes()).hexdigest(),
        ),
        modes={},
    )
    if prepared.status is not PrearmStatus.RELEASED:
        save_json(directory, "case.json", record)
        return record
    assert prepared.release is not None
    record["release"] = save_json(directory, "release.json", asdict(prepared.release))
    for index, mode in enumerate(MODES):
        args = configuration_for(prepared, mode)
        config_record = save_json(
            directory,
            f"configuration-{mode}.json",
            {k: plain_configuration(v) for k, v in args.items()},
        )
        try:
            result, diagnostic = fly(prepared, mode)
            arrays = pack_history(result, result.mission)
            history = save_history(directory, index, arrays)
            diagnostic_record = save_json(directory, f"diagnostic-{mode}.json", diagnostic)
            # Authenticate/decode the saved bytes before replay and scoring.
            loaded = load_history(directory, index, history)
            restored, _ = unpack_history(loaded)
            checked = audit(
                prepared,
                mode,
                restored,
                load_json(directory, f"diagnostic-{mode}.json", diagnostic_record),
            )
            record["modes"][mode] = dict(
                status="ok",
                configuration=config_record,
                history=history,
                diagnostic=diagnostic_record,
                audit=checked,
                metrics=score(loaded, case),
            )
            print(
                json.dumps(dict(case=case, mode=mode, metrics=record["modes"][mode]["metrics"])),
                flush=True,
            )
        except (ValueError, RuntimeError, FloatingPointError, np.linalg.LinAlgError) as error:
            record["modes"][mode] = dict(
                status="implementation_or_evidence_failure",
                error=f"{type(error).__name__}: {error}",
            )
            save_json(directory, "failure.json", record)
            raise
    original_report = json.loads((baseline / "report.json").read_bytes())
    original = next(x for x in original_report["cases"] if x["name"] == case)
    prior = load_history(baseline / case, 1, original["modes"]["on"]["history"])
    original_result, _ = unpack_history(prior)
    record["original_noise"] = verify_noise(original_result, prepared.original, supported=False)
    record["original"] = original["metrics"]["modes"]["on"]
    record["comparison"] = compare(
        case,
        record["original"],
        record["modes"]["unaligned"]["metrics"],
        record["modes"]["aligned"]["metrics"],
    )
    save_json(directory, "case.json", record)
    return record


def summarize(cases: list[dict[str, Any]]) -> dict[str, Any]:
    complete = len(cases) == 4 and all(
        r["status"] == "released"
        and set(r["modes"]) == set(MODES)
        and all(x["status"] == "ok" for x in r["modes"].values())
        for r in cases
    )
    return dict(
        complete_evidence=complete,
        flights=sum(len(r["modes"]) for r in cases),
        supported_initialization_go=complete and all(r["comparison"]["passed"] for r in cases),
        all_four_aligned_flights_pass=complete
        and all(r["modes"]["aligned"]["metrics"]["flight_passed"] for r in cases),
        hardware_qualified=False,
        fault_campaign_qualified=False,
    )


def run(baseline: Path, output: Path, workers: int = 2) -> dict[str, Any]:
    ensure(workers in (1, 2), "one or two worker processes required")
    authenticate_campaign(baseline, "baseline")
    output.mkdir(parents=True, exist_ok=False)
    source = source_sha256(ROOT)
    definition = dict(
        design="ADR 0030 physically supported-start comparison",
        cases=list(CASES),
        modes=list(MODES),
        baseline_report_sha256=REPORT_DIGESTS["baseline"],
        source_sha256=source,
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        software=asdict(capture_software_provenance(ROOT)),
        prearm_time_s=0.5025,
        new_flights=8,
        operating_condition=(
            "ideal fixture release; zero velocity/rate/rotors; supported left-limit first sample"
        ),
        limits=dict(
            hover_peak_m=0.08,
            rmse_m=0.15,
            final_position_m=0.15,
            final_speed_m_s=0.15,
            hover_window_s=[5.0, 11.0],
        ),
    )
    save_json(output, "protocol.json", definition)
    execute = partial(_case, output=output, baseline=baseline)
    if workers == 1:
        cases = [execute(case) for case in CASES]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            cases = list(pool.map(execute, CASES))
    ensure(source_sha256(ROOT) == source, "source changed during study")
    report = dict(protocol=definition, cases=cases, summary=summarize(cases))
    save_json(output, "report.json", report)
    return report


def verify(directory: Path, baseline: Path) -> dict[str, Any]:
    """Recreate only support data; authenticate and replay every saved flight."""
    authenticate_campaign(baseline, "baseline")
    report: dict[str, Any] = json.loads((directory / "report.json").read_bytes())
    ensure(report["protocol"]["source_sha256"] == source_sha256(ROOT), "source mismatch")
    ensure(
        report["protocol"]["protocol_sha256"] == hashlib.sha256(ADR.read_bytes()).hexdigest(),
        "protocol mismatch",
    )
    equal_json(
        json.loads((directory / "protocol.json").read_bytes()),
        report["protocol"],
        "frozen protocol mismatch",
    )
    ensure([r["name"] for r in report["cases"]] == list(CASES), "case order/count mismatch")
    original_report = json.loads((baseline / "report.json").read_bytes())
    for row in report["cases"]:
        case = row["name"]
        prepared = prepare(case)
        target = directory / case
        equal_json(json.loads((target / "case.json").read_bytes()), row, "case record mismatch")
        ensure(
            hashlib.sha256((target / "support.npz").read_bytes()).hexdigest()
            == row["support"]["sha256"],
            "support digest mismatch",
        )
        with np.load(target / "support.npz", allow_pickle=False) as saved:
            ensure(set(saved.files) == set(prepared.evidence), "support fields")
            for key, value in prepared.evidence.items():
                ensure(np.array_equal(saved[key], value), "support reconstruction mismatch")
        ensure(prepared.status.value == row["status"], "support disposition")
        if prepared.release is None:
            ensure(not row["modes"], "rejected acquisition flew")
            continue
        equal_json(
            load_json(target, "release.json", row["release"]),
            asdict(prepared.release),
            "release mismatch",
        )
        for index, mode in enumerate(MODES):
            item = row["modes"][mode]
            args = configuration_for(prepared, mode)
            equal_json(
                load_json(target, f"configuration-{mode}.json", item["configuration"]),
                {k: plain_configuration(v) for k, v in args.items()},
                "configuration mismatch",
            )
            arrays = load_history(target, index, item["history"])
            result, _ = unpack_history(arrays)
            diagnostic = load_json(target, f"diagnostic-{mode}.json", item["diagnostic"])
            equal_json(
                audit(prepared, mode, result, diagnostic), item["audit"], "saved audit mismatch"
            )
            equal_json(score(arrays, case), item["metrics"], "saved score mismatch")
        original = next(x for x in original_report["cases"] if x["name"] == case)["metrics"][
            "modes"
        ]["on"]
        equal_json(original, row["original"], "original score mismatch")
        equal_json(
            compare(
                case,
                original,
                row["modes"]["unaligned"]["metrics"],
                row["modes"]["aligned"]["metrics"],
            ),
            row["comparison"],
            "comparison mismatch",
        )
        print(case, "full saved replay PASS", flush=True)
    equal_json(summarize(report["cases"]), report["summary"], "summary mismatch")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--output", type=Path)
    action.add_argument("--verify", type=Path)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    report = (
        verify(args.verify, args.baseline)
        if args.verify
        else run(args.baseline, args.output, args.workers)
    )
    print(json.dumps(report["summary"], indent=2), flush=True)
    return 0 if report["summary"]["supported_initialization_go"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
