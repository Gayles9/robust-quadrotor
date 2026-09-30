"""ADR 0041: one fixed geometric/startup combination, matched evidence, then pause."""

import argparse
import hashlib
import json
from collections.abc import Iterator
from concurrent.futures import ProcessPoolExecutor
from contextlib import contextmanager, nullcontext
from dataclasses import asdict, dataclass, replace
from functools import partial
from pathlib import Path
from typing import Any
from unittest.mock import patch

import numpy as np

from experiments import robustness_evidence, supported_validation_protocol
from experiments.attitude_control_validation import source_sha256
from experiments.combined_supported_prior import combined_prediction, condition_start
from experiments.durable_release_evidence import save_arrays, save_history, save_json
from experiments.early_flight_diagnostic import authenticated
from experiments.estimated_feedback_evidence import load_history, pack_history, unpack_history
from experiments.geometric_reimplementation_validation import (
    configuration as geometric_configuration,
)
from experiments.geometric_transient_study import POLE_RAD_S, shaped_reference
from experiments.navigation_feedback_oracle import noise_pairing
from experiments.nonlinear_release import predict_nonlinear_release_endpoint
from experiments.nonlinear_release_validation import accuracy
from experiments.robustness_evidence import ensure, equal_json, load_json
from experiments.robustness_protocol import faults
from experiments.supported_start import (
    PreparedStart,
    audit,
    configuration_for,
    fly,
    plain_configuration,
    prepare_configured,
)
from experiments.trajectory_mission_validation import score_result
from quadrotor_math import eskf_replay, mission_simulation
from quadrotor_math.attitude_simulation import _wrench
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_endpoint import EskfEndpointState
from quadrotor_math.estimated_mission import EstimatedMissionResult
from quadrotor_math.geometric_control import GeometricControllerParameters
from quadrotor_math.mission_simulation import MissionResult
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0041-supported-geometric-comparison.md"
ADR_SHA = "7e3559843bbceecc47e4785e04967622f0f69c4c8e32660672266431573e26ef"
CLEAN = ("hover", "spline", "wind_spline")
ARMS = ("unaligned_geometric", "combined_geometric", "combined_cascade")


@dataclass(frozen=True)
class Job:
    case: str
    seed: int

    def __post_init__(self) -> None:
        if (
            self.case not in CLEAN + supported_validation_protocol.FAULTS
            or type(self.seed) is not int
            or self.seed < 0
        ):
            raise ValueError("invalid supported geometric job")

    @property
    def name(self) -> str:
        return f"{self.case}-{self.seed}"

    @property
    def fixture_case(self) -> str:
        return dict(
            hover="nominal_hover", spline="nominal_tracking", wind_spline="wind_tracking"
        ).get(self.case, self.case)


def jobs(stage: str) -> tuple[Job, ...]:
    clean: tuple[Job, ...]
    if stage == "development":
        clean = (Job("hover", 30), Job("hover", 93012), Job("spline", 30), Job("wind_spline", 31))
        fault_seed = 47004
    elif stage == "validation":
        clean = tuple(Job("hover", s) for s in range(95000, 95004)) + tuple(
            Job("spline" if s % 2 == 0 else "wind_spline", s) for s in range(96000, 96004)
        )
        fault_seed = 97000
    else:
        raise ValueError("stage must be development or validation")
    return clean + tuple(Job(case, fault_seed) for case in supported_validation_protocol.FAULTS)


def arms(job: Job) -> tuple[str, ...]:
    return ARMS if job.case in CLEAN else ("off", "on")


def configuration(job: Job, partition: str = "campaign") -> dict[str, Any]:
    """No scientific tuning arguments; smoke retains only a short software fixture."""
    if partition not in ("campaign", "smoke"):
        raise ValueError("unknown partition")
    if job.case in CLEAN and partition == "campaign":
        args = geometric_configuration(
            dict(
                mode="hover" if job.case == "hover" else "estimated",
                case="mild_wind" if job.case == "wind_spline" else "nominal",
                controller="geometric",
                seed=job.seed,
                repeat=False,
            )
        )
    else:
        args = supported_validation_protocol.configuration(
            supported_validation_protocol.Job(job.fixture_case, job.seed), partition
        )
    args["geometric_controller"] = GeometricControllerParameters(filter_pole_rad_s=POLE_RAD_S)
    return args


def prepare(job: Job, partition: str = "campaign") -> PreparedStart:
    return prepare_configured(job.fixture_case, configuration(job, partition), partition)


def arm_start(base: PreparedStart, arm: str) -> tuple[PreparedStart, PreparedStart, str]:
    ensure(arm in ARMS + ("off", "on"), "unknown arm")
    original = base
    if arm == "combined_cascade":
        args = base.original.copy()
        del args["geometric_controller"]
        args["allow_minimum_snap"] = True
        original = replace(base, original=args)
    if arm == "unaligned_geometric":
        return original, original, "unaligned"
    return original, condition_start(original), "aligned"


@contextmanager
def coherent_reference() -> Iterator[None]:
    """Use the existing shaping function in execution and independent saved replay."""
    with (
        patch.object(mission_simulation, "build_geometric_reference", shaped_reference),
        patch.object(robustness_evidence, "build_geometric_reference", shaped_reference),
    ):
        yield


def prediction(original: PreparedStart, selected: PreparedStart, mode: str, target: str) -> Any:
    return (
        combined_prediction(original, selected, target) if mode == "aligned" else nullcontext(None)
    )


def score(mission: MissionResult, *, hover: bool) -> dict[str, Any]:
    """Original geometric limits; allow only the specified initial motors-off sample."""
    metrics = score_result(mission)
    rotor_bound = bool(
        np.any((mission.actual_rotor_omega[1:] <= 0) | (mission.actual_rotor_omega[1:] >= 900))
    )
    metrics["actual_rotor_at_bound_after_release"] = rotor_bound
    metrics["moment_effort_N2_m2_s"] = float(
        np.trapezoid(np.sum(mission.actual_moment_B**2, axis=1), mission.time_s)
    )
    metrics["passed"] = metrics["passed"] and not rotor_bound
    if hover:
        mask = (mission.time_s >= 5) & (mission.time_s <= 65)
        covered = bool(mission.time_s[-1] >= 65 and np.any(mask))
        error = np.linalg.norm(mission.position_W - mission.reference_position_W, axis=1)
        peak = float(error[mask].max()) if covered else None
        metrics.update(full_hold_covered=covered, hold_peak_m=peak)
        metrics["passed"] = bool(metrics["passed"] and peak is not None and peak <= 0.08)
    return metrics


def endpoint(value: dict[str, Any]) -> EskfEndpointState:
    return EskfEndpointState(
        EskfNominalState(**{k: np.array(v) for k, v in value["nominal_state"].items()}),
        np.array(value["imu_noise_mean_B"]),
        np.array(value["joint_covariance"]),
    )


def audit_arm(
    original: PreparedStart,
    selected: PreparedStart,
    mode: str,
    arm: str,
    result: EstimatedMissionResult,
    diagnostic: dict[str, Any],
) -> dict[str, Any]:
    """Reconstruct all saved paths, including conditional sample memory and true wrench."""
    correct = eskf_replay._correct_eskf_epoch
    epochs = 0

    def correction(*args: Any) -> Any:
        nonlocal epochs
        output = correct(*args)
        ensure(args[4] == epochs, "replay epoch order")
        ensure(
            output[2] is not None
            and np.array_equal(output[2].imu_noise_mean_B, result.imu_noise_mean_B[epochs]),
            "conditional sample memory replay",
        )
        epochs += 1
        return output

    with (
        coherent_reference(),
        prediction(original, selected, mode, "replay") as trace,
        patch.object(eskf_replay, "_correct_eskf_epoch", correction),
    ):
        checks = audit(
            selected,
            mode,
            result,
            {k: v for k, v in diagnostic.items() if k != "release_prediction"},
            supervision="off" if arm == "off" else "on",
        )
    ensure(epochs == len(result.mission.time_s), "complete sample-memory replay")
    equal_json(trace, diagnostic["release_prediction"], "first-prediction replay")
    args = configuration_for(selected, mode)
    true_moment = np.array(
        [_wrench(w, args["truth_rotors"])[1] for w in result.mission.actual_rotor_omega]
    )
    ensure(
        np.array_equal(true_moment, result.mission.actual_moment_B), "actual moment reconstruction"
    )
    math = None
    if trace is not None:
        data = result.measurements
        reference = predict_nonlinear_release_endpoint(
            endpoint(trace["input"]),
            data.specific_force_measurements_B[0],
            data.angular_velocity_measurements_B[0],
            data.specific_force_measurements_B[1],
            data.angular_velocity_measurements_B[1],
            args["estimator_configuration"].gravity_acceleration,
            args["estimator_configuration"].sampled_imu_noise,
            args["numerics"].time_step_s,
            order=7,
        )
        math = accuracy(endpoint(trace["output"]), reference)
        ensure(math["passed"], "first prediction quadrature accuracy")
    return dict(**checks, sample_memory_epochs=epochs, release_accuracy=math, passed=True)


def clean_comparison(job: Job, values: dict[str, Any]) -> dict[str, Any]:
    ensure(job.case in CLEAN and set(values) == set(ARMS), "clean comparison arms")
    candidate, cascade = values["combined_geometric"], values["combined_cascade"]
    effort = cascade["moment_effort_N2_m2_s"]
    conditions = dict(
        candidate_physical_limits=bool(candidate["passed"]),
        effort=bool(effort > 0 and candidate["moment_effort_N2_m2_s"] <= 2 * effort),
    )
    rmse = cascade["position_rmse_m"]
    if job.case != "hover":
        conditions.update(
            cascade_physical_limits=bool(cascade["passed"]),
            trajectory_rmse=bool(rmse is not None and candidate["position_rmse_m"] <= rmse),
        )
    return dict(
        conditions=conditions,
        passed=all(conditions.values()),
        rmse_ratio=candidate["position_rmse_m"] / rmse if rmse else None,
        effort_ratio=candidate["moment_effort_N2_m2_s"] / effort if effort else None,
    )


def summarize(rows: list[dict[str, Any]], stage: str) -> dict[str, Any]:
    planned = jobs(stage)
    ledger = len(rows) == len(planned) and all(
        r.get("job") == asdict(j) and set(r.get("arms", {})) == set(arms(j))
        for r, j in zip(rows, planned, strict=False)
    )
    audited = ledger and all(
        item.get("audit", {}).get("passed", False) for row in rows for item in row["arms"].values()
    )
    hover = [r for r in rows if r.get("job", {}).get("case") == "hover"]
    worst: dict[str, float | None] = {}
    for arm in ARMS:
        peaks = [
            r.get("arms", {}).get(arm, {}).get("metrics", {}).get("hold_peak_m") for r in hover
        ]
        worst[arm] = max(peaks) if peaks and all(p is not None for p in peaks) else None
    improvement = bool(
        worst["combined_geometric"] is not None
        and worst["unaligned_geometric"] is not None
        and worst["combined_geometric"] < worst["unaligned_geometric"]
    )
    clean_rows = [r for r in rows if r.get("job", {}).get("case") in CLEAN]
    fault_rows = [
        r for r in rows if r.get("job", {}).get("case") in supported_validation_protocol.FAULTS
    ]
    accepted = bool(
        audited and improvement and all(r.get("comparison", {}).get("passed", False) for r in rows)
    )
    return dict(
        complete_ledger=ledger,
        planned_scientific_flights=sum(len(arms(j)) for j in planned),
        saved_scientific_flights=sum(
            "history" in a for r in rows for a in r.get("arms", {}).values()
        ),
        all_audits_passed=audited,
        clean_comparisons_passed=sum(
            r.get("comparison", {}).get("passed", False) for r in clean_rows
        ),
        fault_comparisons_passed=sum(
            r.get("comparison", {}).get("passed", False) for r in fault_rows
        ),
        worst_hover_peaks_m=worst,
        worst_hover_improved=improvement,
        accepted=accepted,
        fresh_validation_permitted=bool(stage == "development" and accepted),
        adopted_as_default=False,
    )


def execute_job(
    job: Job, *, output: Path, verify: bool = False, partition: str = "campaign"
) -> dict[str, Any]:
    directory = output / job.name
    saved = json.loads((directory / "case.json").read_bytes()) if verify else None
    if not verify:
        directory.mkdir()
    row: dict[str, Any] = dict(job=asdict(job), arms={})
    base = prepare(job, partition)
    ensure(base.release is not None, "prearm fixture rejected")
    assert base.release is not None
    if verify:
        assert saved is not None
        with np.load(directory / "support.npz", allow_pickle=False) as support:
            authenticated(directory / "support.npz", saved["support"]["sha256"])
            ensure(set(support.files) == set(base.evidence), "support schema")
            ensure(
                all(np.array_equal(support[k], v) for k, v in base.evidence.items()),
                "support replay",
            )
        row["support"], row["release"] = saved["support"], saved["release"]
        equal_json(
            load_json(directory, "release.json", row["release"]), asdict(base.release), "release"
        )
    else:
        save_arrays(directory / "support.npz", base.evidence)
        row["support"] = dict(
            file="support.npz",
            sha256=hashlib.sha256((directory / "support.npz").read_bytes()).hexdigest(),
        )
        row["release"] = save_json(directory, "release.json", asdict(base.release))
    histories, results, diagnostics = {}, {}, {}
    selected = base
    for index, arm in enumerate(arms(job)):
        item: dict[str, Any] = {}
        try:
            original, selected, mode = arm_start(base, arm)
            args = configuration_for(selected, mode)
            config = {k: plain_configuration(v) for k, v in args.items()}
            if verify:
                assert saved is not None
                source = saved["arms"][arm]
                item = {k: source[k] for k in ("configuration", "history", "diagnostic")}
                equal_json(
                    load_json(directory, f"configuration-{arm}.json", item["configuration"]),
                    config,
                    "arm configuration",
                )
            else:
                item["configuration"] = save_json(directory, f"configuration-{arm}.json", config)
                with coherent_reference(), prediction(original, selected, mode, "online") as trace:
                    result, diagnostic = fly(
                        selected,
                        mode,
                        faults=faults(job.fixture_case, partition, args),
                        supervised=arm != "off",
                    )
                diagnostic["release_prediction"] = trace
                item["history"] = save_history(
                    directory, index, pack_history(result, result.mission)
                )
                item["diagnostic"] = save_json(directory, f"diagnostic-{arm}.json", diagnostic)
                save_json(directory, f"checkpoint-{arm}.json", item)
            arrays = load_history(directory, index, item["history"])
            result, _ = unpack_history(arrays)
            diagnostic = load_json(directory, f"diagnostic-{arm}.json", item["diagnostic"])
            item["audit"] = audit_arm(original, selected, mode, arm, result, diagnostic)
            item["metrics"] = score(result.mission, hover=job.case == "hover")
            histories[arm], results[arm] = arrays, result
            diagnostics[arm] = {k: v for k, v in diagnostic.items() if k != "release_prediction"}
        except Exception as error:
            if verify:
                raise
            item["error"] = f"{type(error).__name__}: {error}"
        row["arms"][arm] = item
        print(
            json.dumps(
                dict(
                    job=job.name,
                    arm=arm,
                    verify=verify,
                    metrics=item.get("metrics"),
                    error=item.get("error"),
                )
            ),
            flush=True,
        )
    try:
        if len(results) != len(arms(job)):
            row["comparison"] = dict(passed=False, reason="missing or unaudited outcome")
        elif job.case in CLEAN:
            row["comparison"] = clean_comparison(
                job, {a: item["metrics"] for a, item in row["arms"].items()}
            )
            row["noise_pairing"] = {
                arm: noise_pairing(
                    histories["combined_geometric"],
                    histories[arm],
                    configuration_for(selected, "aligned"),
                )
                for arm in ("unaligned_geometric", "combined_cascade")
            }
        else:
            supported_validation_protocol.common_bias_prefix(results)
            response = supported_validation_protocol.response_metrics(
                selected, results, diagnostics
            )
            conditions = dict(response["response_conditions"])
            if response["flight_required"]:
                conditions["both_recovery_flights"] = response["flight_passed"]
            row["response"] = response
            row["comparison"] = dict(conditions=conditions, passed=all(conditions.values()))
    except Exception as error:
        if verify:
            raise
        row["comparison"] = dict(passed=False, error=f"{type(error).__name__}: {error}")
    if verify:
        equal_json(row, saved, "complete saved geometric case")
    else:
        save_json(directory, "case.json", row)
    print(job.name, "comparison", row["comparison"]["passed"], flush=True)
    return row


def definition(stage: str) -> dict[str, Any]:
    authenticated(ADR, ADR_SHA)
    return dict(
        design="ADR0041 one supported geometric combination",
        audited_commit="b855329d9d83f7c211883fcd24e390bdb2977218",
        source_sha256=source_sha256(ROOT),
        decision_sha256=ADR_SHA,
        stage=stage,
        jobs=[asdict(j) for j in jobs(stage)],
        arms={j.name: arms(j) for j in jobs(stage)},
        pole_rad_s=POLE_RAD_S,
        hover_window_s=[5.0, 65.0],
        hover_peak_limit_m=0.08,
        maximum_trajectory_rmse_ratio=1.0,
        maximum_effort_ratio=2.0,
        fresh_seeds_opened=stage == "validation",
    )


def authenticate_development(directory: Path) -> str:
    """No seed-dependent validation construction until the development gate passes."""
    report = json.loads((directory / "report.json").read_bytes())
    equal_json(
        report["protocol"],
        {**definition("development"), "development_report_sha256": None},
        "development source/protocol",
    )
    equal_json(
        json.loads((directory / "protocol.json").read_bytes()),
        report["protocol"],
        "development checkpoint",
    )
    equal_json(report["summary"], summarize(report["cases"], "development"), "development decision")
    ensure(
        report["summary"]["accepted"], "development failed: reserved validation remains unopened"
    )
    for row in report["cases"]:
        target = directory / Job(**row["job"]).name
        equal_json(json.loads((target / "case.json").read_bytes()), row, "development row")
        records = [row["support"], row["release"]]
        for item in row["arms"].values():
            records += [item["configuration"], item["diagnostic"], *item["history"]]
        for record in records:
            authenticated(target / record["file"], record["sha256"])
    return hashlib.sha256((directory / "report.json").read_bytes()).hexdigest()


def run(
    output: Path, stage: str, workers: int, *, verify: bool = False, development: Path | None = None
) -> dict[str, Any]:
    ensure(type(workers) is int and workers in (1, 2), "one or two worker processes")
    parent = None
    if stage == "validation":
        ensure(development is not None, "passing development evidence required")
        assert development is not None
        parent = authenticate_development(development)
    frozen = {**definition(stage), "development_report_sha256": parent}
    saved = json.loads((output / "report.json").read_bytes()) if verify else None
    if verify:
        assert saved is not None
        equal_json(saved["protocol"], frozen, "saved protocol identity")
        equal_json(
            json.loads((output / "protocol.json").read_bytes()), frozen, "protocol checkpoint"
        )
        software = saved["software"]
    else:
        output.mkdir(parents=True, exist_ok=False)
        save_json(output, "protocol.json", frozen)
        software = asdict(capture_software_provenance(ROOT))
    execute = partial(execute_job, output=output, verify=verify)
    if workers == 1:
        rows = [execute(j) for j in jobs(stage)]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(execute, jobs(stage)))
    ensure(source_sha256(ROOT) == frozen["source_sha256"], "source changed during campaign")
    report = dict(protocol=frozen, software=software, cases=rows, summary=summarize(rows, stage))
    if verify:
        equal_json(report, saved, "complete campaign reconstruction")
    else:
        save_json(output, "report.json", report)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("development", "validation"), default="development")
    parser.add_argument("--development", type=Path)
    parser.add_argument("--workers", type=int, choices=(1, 2), default=2)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output", type=Path)
    group.add_argument("--verify", type=Path)
    args = parser.parse_args()
    report = run(
        args.verify or args.output,
        args.stage,
        args.workers,
        verify=args.verify is not None,
        development=args.development,
    )
    print(json.dumps(report["summary"], indent=2), flush=True)
    return 0 if report["summary"]["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
