"""ADR0037 one supported zero-velocity prior plus the verified nonlinear release."""

import argparse
import hashlib
import json
from collections.abc import Iterator
from concurrent.futures import ProcessPoolExecutor
from contextlib import contextmanager, nullcontext
from dataclasses import asdict, replace
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np

from experiments import supported_validation_protocol as protocol
from experiments.attitude_control_validation import source_sha256
from experiments.durable_release_evidence import save_history, save_json
from experiments.early_flight_diagnostic import authenticated
from experiments.estimated_feedback_evidence import load_history, pack_history, unpack_history
from experiments.navigation_feedback_oracle import noise_pairing
from experiments.nonlinear_release import nonlinear_release_prediction
from experiments.nonlinear_release_campaign import clean_comparison, modes
from experiments.nonlinear_release_campaign import summarize as boundary_summary
from experiments.nonlinear_release_validation import load_inputs
from experiments.residual_hover_diagnostic import INPUT_SHA, authenticate
from experiments.robustness_evidence import ensure, equal_json, load_json
from experiments.supported_start import PreparedStart, audit, configuration_for, plain_configuration
from experiments.supported_start_validation import score
from experiments.supported_velocity_prior import (
    SupportedVelocityConditioner,
    fixture_velocity_support,
)
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0037-combined-supported-velocity-comparison.md"
BOUNDARY_SHA = "54f848a0593cef3abe705bb713db97eb584299d3d74fedac91ab309df4ff9cf2"
MATH_SHA = "8daddf91b2bbfc446a98a431e974303f58837057684ac3a7e11118d378384c81"
PRECEDING_SOURCE_SHA = "cd190a70ccae59e9d437e9c44555902f0ce28912eae8dff8fa5c7d4a1a09b215"
PRECEDING_ADR_SHA = "2a98e254e1505ab19e0ec98731dfe6151417779a0169083628bbd9ff33e82265"


def condition_start(original: PreparedStart) -> PreparedStart:
    """Consume a local conditioning attempt before observations; preserve its owner."""
    ensure(original.release is not None, "supported release required")
    assert original.release is not None
    release = SupportedVelocityConditioner().condition(
        original.release,
        fixture_velocity_support(original),
        now_s=original.release.fresh_sample.time_s,
    )
    candidate = replace(original, release=release)
    validate_initialization(original, candidate)
    return candidate


def validate_initialization(original: PreparedStart, candidate: PreparedStart) -> None:
    """Only the independent initial velocity covariance may change."""
    ensure(original.release is not None and candidate.release is not None, "releases required")
    assert original.release is not None and candidate.release is not None
    expected = original.release.endpoint.joint_covariance.copy()
    expected[3:6, :] = expected[:, 3:6] = 0
    equal_json(
        plain_configuration(candidate.release),
        plain_configuration(
            replace(
                original.release,
                endpoint=replace(original.release.endpoint, joint_covariance=expected),
            )
        ),
        "conditioned release ownership",
    )
    first, second = (configuration_for(p, "aligned") for p in (original, candidate))
    expected_config = replace(
        first["estimator_configuration"], initial_covariance=expected[:15, :15]
    )
    equal_json(
        {k: plain_configuration(v) for k, v in second.items()},
        {
            k: plain_configuration(v)
            for k, v in {**first, "estimator_configuration": expected_config}.items()
        },
        "isolated prior configuration",
    )


@contextmanager
def combined_prediction(
    original: PreparedStart, candidate: PreparedStart, target: str
) -> Iterator[dict[str, Any]]:
    """Route one first interval; the original adapter authenticates sample ownership.

    The flight/replay caller uses candidate's initial configuration. The existing
    adapter validates the unconditioned release, then accepts the causal posterior
    supplied by that caller. This wrapper verifies that velocity was conditioned
    before that posterior. It never changes a prediction input or constrains a
    later state. Process-local patching must not be shared between threads.
    """
    validate_initialization(original, candidate)
    with nonlinear_release_prediction(original, target) as trace:
        yield trace
    covariance = np.asarray(trace["input"]["joint_covariance"])
    ensure(
        np.all(covariance[3:6] == 0)
        and np.all(covariance[:, 3:6] == 0)
        and np.all(np.asarray(trace["input"]["nominal_state"]["velocity_W"]) == 0),
        "conditioned prior did not reach first prediction",
    )
    trace["zero_velocity_conditioning"] = True


def combined_comparison(
    case: str, original: dict[str, Any], boundary: dict[str, Any], candidate: dict[str, Any]
) -> dict[str, Any]:
    comparisons = {
        name: clean_comparison(case, control, candidate)
        for name, control in (("original", original), ("boundary", boundary))
    }
    return dict(controls=comparisons, passed=all(x["passed"] for x in comparisons.values()))


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {**boundary_summary(rows), "zero_velocity_conditioning": True}


def preceding_source_sha256() -> str:
    """Bind unchanged execution files without relabelling old evidence as new."""
    digest = hashlib.sha256()
    paths = [
        *ROOT.glob("src/quadrotor_math/*.py"),
        *ROOT.glob("experiments/*.py"),
        ROOT / "pyproject.toml",
        ROOT / "uv.lock",
    ]
    for path in sorted(p for p in paths if p != Path(__file__).resolve()):
        data = path.read_bytes()
        digest.update(path.relative_to(ROOT).as_posix().encode() + b"\0")
        digest.update(str(len(data)).encode() + b"\0" + data)
    return digest.hexdigest()


def authenticate_prerequisites(
    campaign: Path, boundary: Path, uncertainty: Path
) -> tuple[dict[str, Any], dict[str, Any]]:
    ensure(preceding_source_sha256() == PRECEDING_SOURCE_SHA, "preceding execution source changed")
    authenticated(
        ROOT / "docs/decisions/0036-nonlinear-release-flight-comparison.md", PRECEDING_ADR_SHA
    )
    math = json.loads(authenticated(uncertainty / "report.json", MATH_SHA))
    equal_json(
        json.loads((uncertainty / "protocol.json").read_bytes()), math["protocol"], "math protocol"
    )
    ensure(
        math["summary"]["original_prior_passed"]
        and math["summary"]["exact_prior_mathematics_passed"],
        "joint uncertainty gate",
    )
    ensure([r["name"] for r in math["cases"]] == [j.name for j in protocol.jobs()], "math ledger")
    for row in math["cases"]:
        record = row["gaussian_payload"]
        authenticated(uncertainty / record["file"], record["sha256"])
        for arm in row["arms"].values():
            ensure(
                arm["rank"] == 21
                and all(
                    arm[k]["passed"] for k in ("accuracy", "calibration", "ballistic_accuracy")
                ),
                "full joint mathematical acceptance",
            )
    parent = authenticate(campaign)
    control = json.loads(authenticated(boundary / "report.json", BOUNDARY_SHA))
    equal_json(
        json.loads((boundary / "protocol.json").read_bytes()),
        control["protocol"],
        "boundary protocol",
    )
    equal_json(control["summary"], boundary_summary(control["cases"]), "boundary ledger/decision")
    ensure(
        control["protocol"]["source_sha256"] == PRECEDING_SOURCE_SHA, "boundary execution identity"
    )
    for row in control["cases"]:
        directory = boundary / row["name"]
        equal_json(json.loads((directory / "case.json").read_bytes()), row, "boundary case")
        for item in row["modes"].values():
            for record in [item["configuration"], item["diagnostic"], *item["history"]]:
                authenticated(directory / record["file"], record["sha256"])
    return parent, control


def control_data(
    prepared: PreparedStart,
    job: protocol.Job,
    mode: str,
    directory: Path,
    item: dict[str, Any],
    *,
    boundary: bool,
) -> tuple[dict[str, Any], dict[str, Any]]:
    index = (modes(job) if boundary else protocol.modes(job)).index(mode)
    arrays = load_history(directory, index, item["history"])
    result, _ = unpack_history(arrays)
    diagnostic = load_json(directory, f"diagnostic-{mode}.json", item["diagnostic"])
    base_diagnostic = {k: v for k, v in diagnostic.items() if k != "release_prediction"}
    context = nonlinear_release_prediction(prepared, "replay") if boundary else nullcontext()
    with context as trace:
        checks = audit(
            prepared,
            "aligned",
            result,
            base_diagnostic,
            supervision="off" if mode == "off" else "on",
        )
    if boundary:
        equal_json(trace, diagnostic["release_prediction"], "boundary prediction replay")
        ensure(
            not diagnostic["release_prediction"]["zero_velocity_conditioning"],
            "boundary prior changed",
        )
    equal_json(checks, item["audit"], "control full replay")
    metrics = {**score(arrays, job.case), "abort_reason": result.mission.abort_reason}
    equal_json(
        metrics, item["candidate_metrics" if boundary else "metrics"], "control complete score"
    )
    return arrays, metrics


def execute_job(
    job: protocol.Job,
    *,
    campaign: Path,
    boundary: Path,
    output: Path,
    parent_rows: dict[str, Any],
    boundary_rows: dict[str, Any],
    verify: bool = False,
) -> dict[str, Any]:
    parent, control = parent_rows[job.name], boundary_rows[job.name]
    original, original_args, _ = load_inputs(campaign, job, parent)
    candidate = condition_start(original)
    args = configuration_for(candidate, "aligned")
    target = output / job.name
    if verify:
        saved = json.loads((target / "case.json").read_bytes())
    else:
        target.mkdir()
    row: dict[str, Any] = dict(name=job.name, **asdict(job), modes={})
    candidate_results, candidate_diagnostics = {}, {}
    for index, mode in enumerate(modes(job)):
        controls, control_arrays = {}, {}
        for label, directory, item in (
            ("original", campaign / job.name, parent["modes"][mode]),
            ("boundary", boundary / job.name, control["modes"][mode]),
        ):
            arrays, metrics = control_data(
                original, job, mode, directory, item, boundary=label == "boundary"
            )
            control_arrays[label], controls[label] = arrays, metrics
            equal_json(
                load_json(directory, f"configuration-{mode}.json", item["configuration"]),
                {k: plain_configuration(v) for k, v in original_args.items()},
                "matched control configuration",
            )
        configuration = {k: plain_configuration(v) for k, v in args.items()}
        if verify:
            item = saved["modes"][mode]
            history, diagnostic_record, config_record = (
                item[k] for k in ("history", "diagnostic", "configuration")
            )
            equal_json(
                load_json(target, f"configuration-{mode}.json", config_record),
                configuration,
                "candidate configuration",
            )
        else:
            config_record = save_json(target, f"configuration-{mode}.json", configuration)
            # This is the only scientific execution. Save before auditing/scoring.
            with combined_prediction(original, candidate, "online") as trace:
                result, diagnostic = protocol.execute(candidate, mode)
            diagnostic["release_prediction"] = trace
            history = save_history(target, index, pack_history(result, result.mission))
            diagnostic_record = save_json(target, f"diagnostic-{mode}.json", diagnostic)
            save_json(
                target,
                f"flight-checkpoint-{mode}.json",
                dict(history=history, diagnostic=diagnostic_record, configuration=config_record),
            )
        arrays = load_history(target, index, history)
        restored, _ = unpack_history(arrays)
        diagnostic = load_json(target, f"diagnostic-{mode}.json", diagnostic_record)
        ordinary_diagnostic = {k: v for k, v in diagnostic.items() if k != "release_prediction"}
        with combined_prediction(original, candidate, "replay") as trace:
            checks = audit(
                candidate,
                "aligned",
                restored,
                ordinary_diagnostic,
                supervision="off" if mode == "off" else "on",
            )
        equal_json(
            trace, diagnostic["release_prediction"], "complete candidate first-prediction trace"
        )
        metrics = {**score(arrays, job.case), "abort_reason": restored.mission.abort_reason}
        fields = [
            "tracking_rmse_m",
            "final_position_error_m",
            "final_true_speed_m_s",
            "terminal_time_s",
        ]
        if job.case == "nominal_hover":
            fields.append("hover_peak_m")
        row["modes"][mode] = dict(
            configuration=config_record,
            history=history,
            diagnostic=diagnostic_record,
            audit=checks,
            controls=controls,
            candidate_metrics=metrics,
            noise_pairing={
                label: noise_pairing(data, arrays, args) for label, data in control_arrays.items()
            },
            metric_changes={
                label: {key: metrics[key] - values[key] for key in fields}
                for label, values in controls.items()
            },
        )
        candidate_results[mode], candidate_diagnostics[mode] = restored, ordinary_diagnostic
        print(json.dumps(dict(job=job.name, mode=mode, verify=verify, metrics=metrics)), flush=True)
    if job.case in protocol.CLEAN:
        item = row["modes"]["aligned"]
        row["comparison"] = combined_comparison(
            job.case,
            item["controls"]["original"],
            item["controls"]["boundary"],
            item["candidate_metrics"],
        )
    else:
        protocol.common_bias_prefix(candidate_results)
        response = protocol.response_metrics(candidate, candidate_results, candidate_diagnostics)
        row["response"] = response
        conditions = dict(response["response_conditions"])
        if response["flight_required"]:
            conditions["both_recovery_flights"] = response["flight_passed"]
        row["comparison"] = dict(conditions=conditions, passed=all(conditions.values()))
    if verify:
        equal_json(saved, row, "complete combined comparison reconstruction")
    else:
        save_json(target, "case.json", row)
    print(job.name, "comparison", row["comparison"]["passed"], flush=True)
    return row


def run(
    campaign: Path,
    boundary: Path,
    uncertainty: Path,
    output: Path,
    workers: int,
    *,
    verify: bool = False,
) -> dict[str, Any]:
    ensure(type(workers) is int and workers in (1, 2), "one or two worker processes")
    parent, control = authenticate_prerequisites(campaign, boundary, uncertainty)
    frozen = dict(
        design="ADR0037 combined supported velocity and nonlinear release comparison",
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        input_report_sha256=INPUT_SHA,
        boundary_report_sha256=BOUNDARY_SHA,
        uncertainty_report_sha256=MATH_SHA,
        jobs=[j.name for j in protocol.jobs()],
        planned_flights=25,
        zero_velocity_conditioning=True,
    )
    if verify:
        saved = json.loads((output / "report.json").read_bytes())
        equal_json(saved["protocol"], frozen, "combined protocol")
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
        boundary=boundary,
        output=output,
        parent_rows={r["name"]: r for r in parent["cases"]},
        boundary_rows={r["name"]: r for r in control["cases"]},
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
        ensure(report["summary"]["scientific_flights"] == 25, "exact combined flight count")
        if verify:
            equal_json(saved, report, "complete combined campaign report")
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
    for name in ("campaign", "boundary", "uncertainty"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--workers", type=int, default=2)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output", type=Path)
    group.add_argument("--verify", type=Path)
    args = parser.parse_args()
    report = run(
        args.campaign,
        args.boundary,
        args.uncertainty,
        args.verify or args.output,
        args.workers,
        verify=args.verify is not None,
    )
    print(json.dumps(report["summary"], indent=2), flush=True)
    return 0 if report["summary"]["frozen_comparison_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
