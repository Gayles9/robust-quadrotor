"""ADR 0042 fixed development selection, locked regressions and fresh comparison."""

import argparse
import hashlib
import json
from collections.abc import Iterator
from concurrent.futures import ProcessPoolExecutor
from contextlib import contextmanager
from dataclasses import asdict, dataclass, fields, replace
from functools import partial
from pathlib import Path
from typing import Any
from unittest.mock import patch

import numpy as np

from experiments import supported_geometric_comparison as previous
from experiments import supported_validation_protocol
from experiments.attitude_control_validation import source_sha256
from experiments.axis_shaped_geometric import (
    axis_shaped_reference,
    horizontal_transition,
    vertical_transition,
)
from experiments.combined_supported_prior import combined_prediction, condition_start
from experiments.durable_release_evidence import save_arrays, save_history, save_json
from experiments.early_flight_diagnostic import authenticated
from experiments.estimated_feedback_evidence import load_history, pack_history, unpack_history
from experiments.geometric_reimplementation_validation import TRUE_CASES
from experiments.geometric_reimplementation_validation import configuration as true_configuration
from experiments.geometric_transient_study import shaped_reference
from experiments.navigation_feedback_oracle import noise_pairing
from experiments.robustness_evidence import ensure, equal_json, load_json
from experiments.robustness_protocol import faults
from experiments.supported_start import (
    PreparedStart,
    configuration_for,
    fly,
    plain_configuration,
)
from quadrotor_math.estimated_mission import EstimatedMissionResult
from quadrotor_math.mission_simulation import MissionResult, simulate_mission
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0042-final-geometric-comparison.md"
ADR_SHA = "df78a8e97b863bf956bb42b3a50bf45f0dcb9e4e607790c49fcaa254e0cf9189"
CORRECTION = ROOT / "docs/decisions/0043-geometric-regression-adapter.md"
CORRECTION_SHA = "7031505e65738c826a5082c50d670e893ae903de63f2a84b3a2f17f0e73f1440"
LEGACY_REPORT_SHA = "b2dc09a00206fbc1f6092c78ed6504a15879a17e8c4546fa1b5504371c0ed86a"
LEGACY_SOURCE_SHA = "18af27137bd56d4328bf3227d901d84d119e77e341b72f3951d5a0352ce8fbe2"
ALGORITHM_SHA = "77a2c1b140111142412e316b74cd3b1a508ce375e04d9ec1d9226f35be28b75d"
STAGES = ("development", "regression", "validation")
BASE_CASCADE = "cascade-f1"
Job = previous.Job


@dataclass(frozen=True)
class Profile:
    name: str
    controller: str
    frequency: float
    horizontal_pole: float = 10.0
    vertical_pole: float = 30.0


PROFILES = (
    Profile("historical-geometric", "historical", 1.0, 10.0, 10.0),
    *(Profile(f"axis-h{h}-f{f:g}", "axis", f, float(h)) for f in (1.0, 1.25) for h in (10, 15, 20)),
    *(Profile(f"cascade-f{f:g}", "cascade", f) for f in (1.0, 1.25)),
)
BY_NAME = {p.name: p for p in PROFILES}


def clean_jobs(stage: str) -> tuple[Job, ...]:
    if stage == "development":
        return previous.jobs("development")[:4]
    if stage == "validation":
        return tuple(Job("hover", s) for s in range(95000, 95004)) + tuple(
            Job("spline" if s % 2 == 0 else "wind_spline", s) for s in range(96000, 96008)
        )
    raise ValueError("clean stage must be development or validation")


def selected_names(selection: dict[str, Any]) -> tuple[str, str]:
    names = selection["geometric"], selection["cascade"]
    ensure(BY_NAME[names[0]].controller == "axis", "selected geometric profile")
    ensure(BY_NAME[names[1]].controller == "cascade", "selected cascade profile")
    return names


def arms_for(stage: str, selection: dict[str, Any] | None) -> tuple[str, ...]:
    if stage == "development":
        return tuple(BY_NAME)
    ensure(selection is not None, "locked selection required")
    assert selection is not None
    names = selected_names(selection)
    return tuple(dict.fromkeys((*names, BASE_CASCADE))) if stage == "validation" else names


def configure(args: dict[str, Any], profile: Profile) -> dict[str, Any]:
    """Controller-only changes; physical fixtures, estimator and all clocks persist."""
    ensure(BY_NAME.get(profile.name) == profile, "undeclared profile")
    result = args.copy()
    outer = result["position_controller"]
    result["position_controller"] = replace(
        outer,
        position_gain_W=np.array([profile.frequency**2] * 2 + [outer.position_gain_W[2]]),
        velocity_gain_W=np.array([1.8 * profile.frequency] * 2 + [outer.velocity_gain_W[2]]),
    )
    if profile.controller == "cascade":
        result.pop("geometric_controller", None)
        if "estimator_configuration" in result:
            result["allow_minimum_snap"] = True
    else:
        result["geometric_controller"] = replace(
            result["geometric_controller"], filter_pole_rad_s=profile.horizontal_pole
        )
    return result


def start(base: PreparedStart, profile: Profile) -> tuple[PreparedStart, PreparedStart]:
    original = replace(base, original=configure(base.original, profile))
    return original, condition_start(original)


@contextmanager
def reference_context(profile: Profile) -> Iterator[None]:
    builder = (
        partial(axis_shaped_reference, vertical_pole_rad_s=profile.vertical_pole)
        if profile.controller == "axis"
        else shaped_reference
    )
    # audit_arm enters its own coherent_reference context; patch its source too.
    with patch.object(previous, "shaped_reference", builder), previous.coherent_reference():
        yield


def record(path: Path) -> dict[str, str]:
    return dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def execute_case(
    job: Job,
    *,
    output: Path,
    names: tuple[str, ...],
    fault: bool = False,
    verify: bool = False,
    partition: str = "campaign",
) -> dict[str, Any]:
    directory = output / job.name
    saved = json.loads((directory / "case.json").read_bytes()) if verify else None
    if not verify:
        directory.mkdir()
    base = previous.prepare(job, partition)
    ensure(base.release is not None, "supported acquisition must release")
    assert base.release is not None
    row: dict[str, Any] = dict(job=asdict(job), arms={})
    if verify:
        assert saved is not None
        row["support"], row["release"] = saved["support"], saved["release"]
        authenticated(directory / "support.npz", row["support"]["sha256"])
        with np.load(directory / "support.npz", allow_pickle=False) as data:
            ensure(set(data.files) == set(base.evidence), "support schema")
            ensure(all(np.array_equal(data[k], v) for k, v in base.evidence.items()), "support")
        equal_json(
            load_json(directory, "release.json", row["release"]), asdict(base.release), "release"
        )
    else:
        save_arrays(directory / "support.npz", base.evidence)
        row["support"] = record(directory / "support.npz")
        row["release"] = save_json(directory, "release.json", asdict(base.release))
    first_history: dict[str, Any] | None = None
    results: dict[str, EstimatedMissionResult] = {}
    diagnostics: dict[str, Any] = {}
    arm_names = ("off", "on") if fault else names
    for index, arm in enumerate(arm_names):
        profile = BY_NAME[names[0] if fault else arm]
        item: dict[str, Any] = dict(profile=asdict(profile))
        try:
            original, selected = start(base, profile)
            args = configuration_for(selected, "aligned")
            configuration = {k: plain_configuration(v) for k, v in args.items()}
            if verify:
                assert saved is not None
                source = saved["arms"][arm]
                item.update({k: source[k] for k in ("configuration", "history", "diagnostic")})
                equal_json(
                    load_json(directory, f"configuration-{arm}.json", item["configuration"]),
                    configuration,
                    "configuration",
                )
            else:
                item["configuration"] = save_json(
                    directory, f"configuration-{arm}.json", configuration
                )
                with (
                    reference_context(profile),
                    combined_prediction(original, selected, "online") as trace,
                ):
                    result, diagnostic = fly(
                        selected,
                        "aligned",
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
            with reference_context(profile):
                item["audit"] = previous.audit_arm(
                    original, selected, "aligned", arm, result, diagnostic
                )
            item["metrics"] = previous.score(result.mission, hover=job.case == "hover")
            if fault:
                results[arm] = result
                diagnostics[arm] = {
                    k: v for k, v in diagnostic.items() if k != "release_prediction"
                }
            elif first_history is None:
                first_history = {k: v for k, v in arrays.items() if k != "covariances"}
            else:
                item["noise_pairing"] = noise_pairing(first_history, arrays, args)
        except Exception as error:
            if verify:
                raise
            item["error"] = f"{type(error).__name__}: {error}"
        row["arms"][arm] = item
        print(
            json.dumps(
                dict(job=job.name, arm=arm, metrics=item.get("metrics"), error=item.get("error"))
            ),
            flush=True,
        )
    if fault:
        try:
            ensure(set(results) == {"off", "on"}, "both fault arms must be saved and audited")
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
        equal_json(row, saved, "complete reconstructed case")
    else:
        save_json(directory, "case.json", row)
    return row


def geometric_mean(values: list[float]) -> float:
    ensure(
        bool(values) and all(np.isfinite(v) and v > 0 for v in values), "positive finite metrics"
    )
    return float(np.exp(np.mean(np.log(values))))


def comparison(rows: list[dict[str, Any]], candidate: str, comparator: str) -> dict[str, Any]:
    pairs = []
    categories: dict[str, dict[str, float]] = {}
    metrics = ("position_rmse_m", "position_peak_m", "moment_effort_N2_m2_s")
    for row in rows:
        a, b = (row["arms"][name]["metrics"] for name in (candidate, comparator))
        ratios = {key: a[key] / b[key] for key in metrics}
        pairs.append(dict(job=row["job"], ratios=ratios))
    for case in previous.CLEAN:
        group = [p for p in pairs if p["job"]["case"] == case]
        categories[case] = {
            key: geometric_mean([p["ratios"][key] for p in group]) for key in metrics
        }
    balanced = {key: geometric_mean([v[key] for v in categories.values()]) for key in metrics}
    return dict(
        candidate=candidate,
        comparator=comparator,
        pairs=pairs,
        categories=categories,
        balanced_ratios=balanced,
        every_accuracy_peak_effort_pair_dominates=all(
            all(v <= 1 for v in p["ratios"].values()) for p in pairs
        ),
    )


def ledger_ok(rows: list[dict[str, Any]], stage: str, names: tuple[str, ...]) -> bool:
    expected = clean_jobs(stage)
    return bool(
        len(rows) == len(expected)
        and all(
            row.get("job") == asdict(job)
            and set(row.get("arms", {})) == set(names)
            and all(item.get("audit", {}).get("passed", False) for item in row["arms"].values())
            for row, job in zip(rows, expected, strict=False)
        )
    )


def eligible(rows: list[dict[str, Any]], name: str) -> bool:
    return all(row["arms"][name].get("metrics", {}).get("passed", False) for row in rows)


def select(rows: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = dict(complete_audited_ledger=False, accepted=False)
    if not ledger_ok(rows, "development", tuple(BY_NAME)):
        return summary
    summary["complete_audited_ledger"] = True
    scores = {
        p.name: comparison(rows, p.name, BASE_CASCADE)["balanced_ratios"]["position_rmse_m"]
        for p in PROFILES
    }
    summary["profiles"] = {
        p.name: dict(
            physical_passed=eligible(rows, p.name),
            balanced_rmse_ratio_to_original_cascade=scores[p.name],
        )
        for p in PROFILES
    }
    cascades = [p.name for p in PROFILES if p.controller == "cascade" and eligible(rows, p.name)]
    if not cascades:
        return summary
    cascade = min(cascades, key=scores.__getitem__)
    candidates = []
    for p in PROFILES:
        if p.controller != "axis" or not eligible(rows, p.name):
            continue
        paired = comparison(rows, p.name, cascade)
        effort_ok = all(r["ratios"]["moment_effort_N2_m2_s"] <= 2 for r in paired["pairs"])
        summary["profiles"][p.name]["effort_passed"] = effort_ok
        if effort_ok:
            candidates.append(p.name)
    summary["cascade"] = cascade
    if not candidates:
        return summary
    geometric = min(candidates, key=scores.__getitem__)
    primary = comparison(rows, geometric, cascade)
    summary.update(
        geometric=geometric,
        primary=primary,
        accepted=primary["balanced_ratios"]["position_rmse_m"] <= 0.95,
    )
    return summary


def bootstrap_interval(paired: dict[str, Any]) -> list[float]:
    rng = np.random.Generator(np.random.PCG64(424200))
    averaged = np.zeros(20000)
    for case in previous.CLEAN:
        values = np.log(
            [p["ratios"]["position_rmse_m"] for p in paired["pairs"] if p["job"]["case"] == case]
        )
        ensure(len(values) == 4, "bootstrap requires four fresh cases per category")
        averaged += (
            np.mean(values[rng.integers(0, len(values), size=(20000, len(values)))], axis=1) / 3
        )
    return [float(value) for value in np.exp(np.quantile(averaged, [0.025, 0.975]))]


def validation_summary(rows: list[dict[str, Any]], selection: dict[str, Any]) -> dict[str, Any]:
    names = arms_for("validation", selection)
    clean, fault_rows = rows[:12], rows[12:]
    fault_jobs = tuple(Job(case, 97000) for case in supported_validation_protocol.FAULTS)
    complete = (
        ledger_ok(clean, "validation", names)
        and len(fault_rows) == 8
        and all(
            row.get("job") == asdict(job)
            and set(row.get("arms", {})) == {"off", "on"}
            and all(item.get("audit", {}).get("passed", False) for item in row["arms"].values())
            for row, job in zip(fault_rows, fault_jobs, strict=False)
        )
    )
    summary: dict[str, Any] = dict(
        complete_audited_ledger=complete, accepted=False, adopted_as_default=False
    )
    if not complete:
        return summary
    geometric, cascade = selected_names(selection)
    primary = comparison(clean, geometric, cascade)
    conditions = dict(
        geometric_physical=eligible(clean, geometric),
        cascade_physical=eligible(clean, cascade),
        faults=all(row.get("comparison", {}).get("passed", False) for row in fault_rows),
        balanced_accuracy=primary["balanced_ratios"]["position_rmse_m"] <= 0.90,
        category_accuracy=all(v["position_rmse_m"] <= 1.05 for v in primary["categories"].values()),
        individual_accuracy=all(p["ratios"]["position_rmse_m"] <= 1.10 for p in primary["pairs"]),
        balanced_effort=primary["balanced_ratios"]["moment_effort_N2_m2_s"] <= 1.25,
        individual_effort=all(p["ratios"]["moment_effort_N2_m2_s"] <= 2 for p in primary["pairs"]),
    )
    summary.update(
        conditions=conditions,
        accepted=all(conditions.values()),
        primary=primary,
        secondary=comparison(clean, geometric, BASE_CASCADE),
        descriptive_balanced_rmse_interval=bootstrap_interval(primary),
        fault_comparisons_passed=sum(row["comparison"]["passed"] for row in fault_rows),
    )
    return summary


def execute_true(
    case: str, *, output: Path, names: tuple[str, ...], verify: bool = False
) -> dict[str, Any]:
    directory = output / case
    saved = json.loads((directory / "case.json").read_bytes()) if verify else None
    if not verify:
        directory.mkdir()
    row: dict[str, Any] = dict(case=case, arms={})
    for name in names:
        profile = BY_NAME[name]
        item: dict[str, Any] = dict(profile=asdict(profile))
        try:
            args = configure(
                true_configuration(
                    dict(mode="true", case=case, controller="geometric", seed=None, repeat=False)
                ),
                profile,
            )
            config = {k: plain_configuration(v) for k, v in args.items()}
            if verify:
                assert saved is not None
                item.update({key: saved["arms"][name][key] for key in ("configuration", "history")})
                equal_json(
                    load_json(directory, f"configuration-{name}.json", item["configuration"]),
                    config,
                    "true configuration",
                )
            else:
                item["configuration"] = save_json(directory, f"configuration-{name}.json", config)
                with reference_context(profile):
                    mission = simulate_mission(**args)
                payload = {
                    f.name: getattr(mission, f.name)
                    for f in fields(mission)
                    if f.name != "abort_reason"
                }
                payload["abort_reason"] = np.array(mission.abort_reason or "")
                save_arrays(directory / f"{name}.npz", payload)
                item["history"] = record(directory / f"{name}.npz")
            authenticated(directory / item["history"]["file"], item["history"]["sha256"])
            with np.load(directory / item["history"]["file"], allow_pickle=False) as data:
                mission = MissionResult(
                    **{k: data[k] for k in data.files if k != "abort_reason"},
                    abort_reason=str(data["abort_reason"]) or None,
                )
            item["metrics"] = previous.score(mission, hover=False)
            item["audit"] = dict(payload_authenticated=True, passed=True)
        except Exception as error:
            if verify:
                raise
            item["error"] = f"{type(error).__name__}: {error}"
        row["arms"][name] = item
        print(
            json.dumps(
                dict(case=case, arm=name, metrics=item.get("metrics"), error=item.get("error"))
            ),
            flush=True,
        )
    if verify:
        equal_json(row, saved, "true case reconstruction")
    else:
        save_json(directory, "case.json", row)
    return row


def regression_summary(
    rows: list[dict[str, Any]], selection: dict[str, Any], output: Path
) -> dict[str, Any]:
    names = arms_for("regression", selection)
    complete = len(rows) == len(TRUE_CASES) and all(
        row.get("case") == case
        and set(row.get("arms", {})) == set(names)
        and all(
            item.get("audit", {}).get("passed", False)
            and item.get("metrics", {}).get("passed", False)
            for item in row["arms"].values()
        )
        for row, case in zip(rows, TRUE_CASES, strict=False)
    )
    summary: dict[str, Any] = dict(complete_physical_ledger=complete, accepted=False)
    if not complete:
        return summary
    name = names[0]
    with (
        np.load(output / "nominal" / f"{name}.npz", allow_pickle=False) as coarse,
        np.load(output / "refined" / f"{name}.npz", allow_pickle=False) as fine,
    ):
        same_clock = np.array_equal(coarse["time_s"], fine["time_s"][::2])
        ensure(same_clock, "refinement must preserve common epochs")
        position = float(
            np.max(np.linalg.norm(coarse["position_W"] - fine["position_W"][::2], axis=1))
        )
        dot = np.abs(np.sum(coarse["q_WB"] * fine["q_WB"][::2], axis=1))
        angle = float(np.rad2deg(np.max(2 * np.arccos(np.clip(dot, 0, 1)))))
    summary.update(
        refinement_position_m=position,
        refinement_attitude_deg=angle,
        accepted=position <= 0.005 and angle <= 0.05,
    )
    return summary


def definition(
    stage: str, selection: dict[str, Any] | None, parents: dict[str, str]
) -> dict[str, Any]:
    ensure(stage in STAGES, "declared study stage")
    authenticated(ADR, ADR_SHA)
    authenticated(CORRECTION, CORRECTION_SHA)
    local: dict[str, Any] = dict(
        horizontal={
            p.name: [
                float(
                    np.max(
                        np.abs(
                            np.linalg.eigvals(
                                horizontal_transition(p.horizontal_pole, p.frequency, j)
                            )
                        )
                    )
                )
                for j in (0.02, 0.025)
            ]
            for p in PROFILES
            if p.controller == "axis"
        },
        vertical=float(np.max(np.abs(np.linalg.eigvals(vertical_transition())))),
    )
    ensure(
        all(v < 1 for values in local["horizontal"].values() for v in values)
        and local["vertical"] < 1,
        "local sampled maps must be stable",
    )
    return dict(
        decision="ADR0042 final bounded geometric comparison",
        decision_sha256=ADR_SHA,
        adapter_correction_sha256=CORRECTION_SHA,
        audited_commit="5eb932766db6fc3ef5f465bf551e390633561d43",
        source_sha256=source_sha256(ROOT),
        stage=stage,
        profiles=[asdict(p) for p in PROFILES],
        arms=list(arms_for(stage, selection)),
        selection=selection,
        parent_report_sha256=parents,
        local_spectral_radii=local,
        jobs=list(TRUE_CASES)
        if stage == "regression"
        else [asdict(j) for j in clean_jobs(stage)]
        + (
            [asdict(Job(case, 97000)) for case in supported_validation_protocol.FAULTS]
            if stage == "validation"
            else []
        ),
        fresh_seeds_opened=stage == "validation",
    )


def algorithm_sha256() -> str:
    """Bind all execution algorithms/dependencies except this corrected driver."""
    paths = sorted(
        [
            *ROOT.glob("src/**/*.py"),
            *ROOT.glob("experiments/**/*.py"),
            ROOT / "pyproject.toml",
            ROOT / "uv.lock",
        ]
    )
    manifest = [
        (p.relative_to(ROOT).as_posix(), hashlib.sha256(p.read_bytes()).hexdigest())
        for p in paths
        if p != Path(__file__).resolve()
    ]
    return hashlib.sha256(json.dumps(manifest, separators=(",", ":")).encode()).hexdigest()


def parent_protocol(
    raw: bytes, stage: str, selection: dict[str, Any] | None, parents: dict[str, str]
) -> tuple[dict[str, Any], bool]:
    """ADR 0043 admits one exact unaffected historical report, never arbitrary data."""
    expected = definition(stage, selection, parents)
    legacy = stage == "development" and hashlib.sha256(raw).hexdigest() == LEGACY_REPORT_SHA
    if legacy:
        ensure(algorithm_sha256() == ALGORITHM_SHA, "historical execution algorithms changed")
        expected["source_sha256"] = LEGACY_SOURCE_SHA
        del expected["adapter_correction_sha256"]
    return expected, legacy


def authenticate_report(
    output: Path, stage: str, selection: dict[str, Any] | None, parents: dict[str, str]
) -> dict[str, Any]:
    raw = (output / "report.json").read_bytes()
    report: dict[str, Any] = json.loads(raw)
    expected, legacy = parent_protocol(raw, stage, selection, parents)
    equal_json(report["protocol"], expected, "parent source/protocol")
    equal_json(
        json.loads((output / "protocol.json").read_bytes()), report["protocol"], "parent checkpoint"
    )
    recomputed = (
        select(report["cases"])
        if stage == "development"
        else regression_summary(report["cases"], selection or {}, output)
    )
    equal_json(report["summary"], recomputed, "parent decision reconstruction")
    ensure(recomputed["accepted"], f"{stage} failed: later scientific stages remain unopened")

    def payloads(value: Any, directory: Path) -> None:
        if isinstance(value, dict):
            if set(value) == {"file", "sha256"}:
                ensure(Path(value["file"]).name == value["file"], "payload basename only")
                authenticated(directory / value["file"], value["sha256"])
            else:
                for child in value.values():
                    payloads(child, directory)
        elif isinstance(value, list):
            for child in value:
                payloads(child, directory)

    for row in report["cases"]:
        name = Job(**row["job"]).name if stage == "development" else row["case"]
        directory = output / name
        equal_json(json.loads((directory / "case.json").read_bytes()), row, "parent case")
        payloads(row, directory)
        if legacy:
            base = previous.prepare(Job(**row["job"]))
            for arm, item in row["arms"].items():
                _, selected = start(base, BY_NAME[arm])
                configuration = {
                    k: plain_configuration(v)
                    for k, v in configuration_for(selected, "aligned").items()
                }
                equal_json(
                    load_json(directory, f"configuration-{arm}.json", item["configuration"]),
                    configuration,
                    "historical estimated configuration under corrected adapter",
                )
    return report


def run(
    output: Path,
    stage: str,
    workers: int,
    *,
    development: Path | None = None,
    regression: Path | None = None,
    verify: bool = False,
) -> dict[str, Any]:
    ensure(type(workers) is int and workers in (1, 2), "one or two workers")
    parents: dict[str, str] = {}
    selection = None
    if stage != "development":
        ensure(development is not None, "authenticated development required")
        assert development is not None
        parent = authenticate_report(development, "development", None, {})
        selection = {key: parent["summary"][key] for key in ("geometric", "cascade")}
        parents["development"] = record(development / "report.json")["sha256"]
    if stage == "validation":
        ensure(regression is not None, "authenticated true-state regression required")
        assert regression is not None
        authenticate_report(regression, "regression", selection, parents)
        parents["regression"] = record(regression / "report.json")["sha256"]
    frozen = definition(stage, selection, parents)
    saved = json.loads((output / "report.json").read_bytes()) if verify else None
    if verify:
        assert saved is not None
        equal_json(saved["protocol"], frozen, "saved protocol")
        equal_json(json.loads((output / "protocol.json").read_bytes()), frozen, "saved checkpoint")
        software = saved["software"]
    else:
        output.mkdir(parents=True, exist_ok=False)
        save_json(output, "protocol.json", frozen)
        software = asdict(capture_software_provenance(ROOT))
    names = arms_for(stage, selection)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        if stage == "regression":
            rows = list(
                pool.map(
                    partial(execute_true, output=output, names=names, verify=verify), TRUE_CASES
                )
            )
        else:
            rows = list(
                pool.map(
                    partial(execute_case, output=output, names=names, verify=verify),
                    clean_jobs(stage),
                )
            )
            if stage == "validation":
                rows += list(
                    pool.map(
                        partial(
                            execute_case,
                            output=output,
                            names=(names[0],),
                            fault=True,
                            verify=verify,
                        ),
                        [Job(case, 97000) for case in supported_validation_protocol.FAULTS],
                    )
                )
    if stage == "development":
        summary = select(rows)
    elif stage == "regression":
        summary = regression_summary(rows, selection or {}, output)
    else:
        summary = validation_summary(rows, selection or {})
    ensure(source_sha256(ROOT) == frozen["source_sha256"], "source changed during stage")
    report = dict(protocol=frozen, software=software, cases=rows, summary=summary)
    if verify:
        equal_json(report, saved, "complete report reconstruction")
    else:
        save_json(output, "report.json", report)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=STAGES, required=True)
    parser.add_argument("--development", type=Path)
    parser.add_argument("--regression", type=Path)
    parser.add_argument("--workers", type=int, choices=(1, 2), default=2)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output", type=Path)
    group.add_argument("--verify", type=Path)
    args = parser.parse_args()
    report = run(
        args.verify or args.output,
        args.stage,
        args.workers,
        development=args.development,
        regression=args.regression,
        verify=args.verify is not None,
    )
    print(json.dumps(report["summary"], indent=2), flush=True)
    return 0 if report["summary"]["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
