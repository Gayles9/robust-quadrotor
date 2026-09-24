"""Pole-placed cascade with moment-matched prior (ADR 0014); ADR 0013 preserved.

python -m experiments.feedback_bandwidth_validation --partition fixed --output NEW_DIR
Full measured-state and matched true-state closed loops, unchanged scoring rules.
"""

import argparse
import hashlib
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
from datetime import UTC, datetime
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
from experiments.estimated_feedback_validation import (
    audit_history,
    score_trial,
    summarize,
)
from experiments.estimated_feedback_validation import make_configuration as original_configuration
from experiments.estimated_feedback_validation import planned_jobs as original_jobs
from experiments.estimated_feedback_validation import validation_protocol as original_protocol
from experiments.position_control_validation import _unique_object
from quadrotor_math.cascade_analysis import HorizontalCascade
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.mission_simulation import simulate_mission
from quadrotor_math.run_manifest import capture_software_provenance

PARTITIONS = ("smoke", "fixed", "development", "validation")


def designed_horizontal_cascade() -> HorizontalCascade:
    """Coefficient matching, not a fitted seed-dependent gain search.

    tau*(s+3)^2*(s+6)*(s²+28s+392) fixes all four horizontal gains.
    Its s^4 coefficient is tau*(3+3+6+28)=1, satisfying the fixed
    motor-lag constraint. The initial covariance is matched separately.
    """
    coefficients = np.polymul(np.polymul([1.0, 6, 9], [1.0, 6]), [1.0, 28, 392])
    return HorizontalCascade(
        float(coefficients[5] / coefficients[3]),
        float(coefficients[4] / coefficients[3]),
        float(coefficients[3] / coefficients[2]),
        float(coefficients[2] / 40),
        9.81,
        0.025,
        0.01,
        2,
    )


def make_configuration(job: dict[str, Any]) -> dict[str, Any]:
    kwargs = original_configuration(job)
    design = designed_horizontal_cascade()
    inner, outer = kwargs["attitude_controller"], kwargs["position_controller"]
    kwargs["attitude_controller"] = replace(
        inner,
        attitude_gain_B=np.array([design.attitude_gain] * 2 + [inner.attitude_gain_B[2]]),
        rate_gain_B=np.array([design.rate_gain] * 2 + [inner.rate_gain_B[2]]),
    )
    kwargs["position_controller"] = replace(
        outer,
        position_gain_W=np.array([design.position_gain] * 2 + [outer.position_gain_W[2]]),
        velocity_gain_W=np.array([design.velocity_gain] * 2 + [outer.velocity_gain_W[2]]),
    )
    if not job["noiseless"]:
        # Fixed population assumptions, never the realized state or true biases.
        # Independent U[-a,a] errors have mean zero and covariance a²/3.
        half_width = np.array([0.02] * 6 + [np.deg2rad(2)] * 3 + [0.02] * 3 + [0.003] * 3)
        kwargs["estimator_configuration"] = replace(
            kwargs["estimator_configuration"], initial_covariance=np.diag(half_width**2 / 3)
        )
    return kwargs


def planned_jobs(partition: str) -> list[dict[str, Any]]:
    if partition == "fixed":
        return original_jobs("fixed")
    identities = {
        "smoke": [("smoke", seed) for seed in range(8500, 8502)],
        "development": [("hover", seed) for seed in range(8200, 8206)]
        + [("square", seed) for seed in range(8206, 8209)],
        "validation": [("hover", seed) for seed in range(91000, 91020)]
        + [("square", seed) for seed in range(92000, 92010)],
    }
    if partition not in identities:
        raise ValueError("unknown bandwidth validation partition")
    return [
        {"case": case, "seed": seed, "noiseless": False} for case, seed in identities[partition]
    ]


def validation_protocol(partition: str) -> dict[str, Any]:
    jobs = planned_jobs(partition)
    return {
        "name": "pole_placed_estimated_feedback",
        "version": 1,
        "partition": partition,
        "jobs": jobs,
        "configurations": [
            plain(
                {
                    key: asdict(value) if hasattr(value, "__dataclass_fields__") else value
                    for key, value in make_configuration(job).items()
                }
            )
            for job in jobs
        ],
        "inherited_contract": original_protocol("fixed"),
        "local_model": asdict(designed_horizontal_cascade()),
        "target_polynomial_factors": ".025*(s+3)^2*(s+6)*(s^2+28*s+392)",
        "initial_covariance": "Independent centered uniform errors: half-widths .02 m, .02 m/s, "
        "2 degrees, .02 m/s2, .003 rad/s; P0=diag(half_width^2/3). Fixed prior mean unchanged.",
        "semantics": "Horizontal/roll/pitch gains and stated initial covariance change. "
        "Matched true-feedback pairs use the same new gains. Unchanged plant, ESKF equations, "
        "initial truth distribution, prior mean, process/sample noise, clocks, bounds, "
        "references and scoring. Original fixed identities retained; complete "
        "60-second hover scored over 5..65 s. All failures stay in the ledger. "
        "Local controller poles are not a proof for nonlinear estimated feedback.",
    }


def _execute(job: dict[str, Any]) -> dict[str, Any]:
    try:
        kwargs = make_configuration(job)
        result = simulate_estimated_mission(**kwargs)
        baseline = simulate_mission(
            **{
                key: value
                for key, value in kwargs.items()
                if key not in ("sensors", "estimator_configuration")
            }
        )
        return {
            "job": job,
            "status": "ok",
            "metrics": score_trial(job, result, baseline),
            "history": pack_history(result, baseline),
        }
    except (ValueError, FloatingPointError, np.linalg.LinAlgError, RuntimeError) as error:
        return {"job": job, "status": "error", "error": str(error)}


def run_validation(partition: str, workers: int = 1) -> dict[str, Any]:
    if type(workers) is not int or workers < 1:
        raise ValueError("workers must be a positive integer")
    protocol = validation_protocol(partition)
    if workers == 1:
        rows = [_execute(job) for job in protocol["jobs"]]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(_execute, protocol["jobs"]))
    return {
        "protocol": protocol,
        "protocol_sha256": hashlib.sha256(canonical_json(protocol)).hexdigest(),
        "trials": rows,
        "summary": summarize(protocol, rows),
    }


def validate_report(report: dict[str, Any]) -> dict[str, Any]:
    protocol = validation_protocol(report["protocol"]["partition"])
    if canonical_json(protocol) != canonical_json(report["protocol"]) or (
        report["protocol_sha256"] != hashlib.sha256(canonical_json(protocol)).hexdigest()
    ):
        raise ValueError("protocol differs from its frozen bandwidth definition")
    summary = summarize(protocol, report["trials"])
    for row in report["trials"]:
        if row["status"] == "ok":
            result, baseline = unpack_history(row["history"])
            audit_history(
                row["job"], result, baseline, configuration=make_configuration(row["job"])
            )
            if canonical_json(row["metrics"]) != canonical_json(
                score_trial(row["job"], result, baseline)
            ):
                raise ValueError("metrics do not match complete histories")
    if canonical_json(summary) != canonical_json(report["summary"]):
        raise ValueError("summary does not match full trial ledger")
    return report


def save_report(report: dict[str, Any], output: Path) -> None:
    validate_report(report)
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    for index, row in enumerate(report["trials"]):
        saved = {key: value for key, value in row.items() if key != "history"}
        if row["status"] == "ok":
            saved["history_parts"] = save_history(output, index, row["history"])
        rows.append(saved)
    with (output / "report.json").open("xb") as stream:
        stream.write(canonical_json({**report, "format_version": 1, "trials": rows}) + b"\n")


def load_report(directory: Path) -> dict[str, Any]:
    report = json.loads((directory / "report.json").read_bytes(), object_pairs_hook=_unique_object)
    canonical_json(report)
    if type(report.get("format_version")) is not int or report["format_version"] != 1:
        raise ValueError("unsupported bandwidth evidence format")
    for index, row in enumerate(report["trials"]):
        if row["status"] == "ok":
            row["history"] = load_history(directory, index, row["history_parts"])
    return validate_report(report)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", choices=PARTITIONS, default="smoke")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    root = Path(__file__).resolve().parents[1]
    source = source_sha256(root)
    provenance = asdict(capture_software_provenance(root))
    print(
        "Protocol SHA256: "
        + hashlib.sha256(canonical_json(validation_protocol(args.partition))).hexdigest(),
        file=sys.stderr,
        flush=True,
    )
    report = run_validation(args.partition, args.workers)
    if source != source_sha256(root):
        raise RuntimeError("execution source changed during campaign")
    report.update(
        {
            "source_sha256": source,
            "software_provenance": provenance,
            "generated_at_utc": datetime.now(UTC).isoformat(),
            "workers": args.workers,
        }
    )
    save_report(report, args.output)
    print(json.dumps(report["summary"], sort_keys=True), file=sys.stderr)
    return int(not report["summary"]["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
