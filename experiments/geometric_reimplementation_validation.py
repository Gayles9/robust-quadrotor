"""Fresh fixed reconstruction campaign (ADR 0017); failures remain in the ledger."""

import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, fields, replace
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.estimated_feedback_evidence import pack_history, plain, save_history
from experiments.estimated_feedback_validation import make_configuration
from experiments.feedback_bandwidth_validation import make_configuration as v2_configuration
from experiments.trajectory_mission_validation import TARGETS, build_mission, score_result
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.geometric_control import GeometricControllerParameters
from quadrotor_math.mission_simulation import MissionResult, simulate_mission

TRUE_CASES = (
    "nominal",
    "refined",
    "offset_positive",
    "offset_negative",
    "mild_wind",
    "reversed_wind",
    "wind_offset",
)
CONTROLLERS = ("cascade", "geometric")


def jobs() -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = [
        dict(mode="true", case=case, controller=controller, seed=None, repeat=False)
        for case in TRUE_CASES
        for controller in CONTROLLERS
    ]
    result += [
        dict(mode="estimated", case=case, controller=controller, seed=seed, repeat=False)
        for case in ("nominal", "mild_wind")
        for seed in (30, 31)
        for controller in CONTROLLERS
    ]
    result += [
        dict(mode="hover", case="hover", controller=controller, seed=seed, repeat=False)
        for seed in (30, 93012)
        for controller in CONTROLLERS
    ]
    result += [
        dict(mode="true", case="nominal", controller="geometric", seed=None, repeat=True),
        dict(mode="estimated", case="nominal", controller="geometric", seed=30, repeat=True),
    ]
    return result


def name_of(job: dict[str, Any]) -> str:
    return "-".join(str(job[key]) for key in ("mode", "case", "controller", "seed", "repeat"))


def configuration(job: dict[str, Any]) -> dict[str, Any]:
    """Exact independent initial/sensor/estimator fixtures, recorded in protocol."""
    case = (
        "hover" if job["mode"] == "hover" else ("mild_wind" if "wind" in job["case"] else "square")
    )
    source = dict(case=case, seed=job["seed"], noiseless=job["mode"] == "true")
    config = make_configuration(source)
    if job["mode"] == "hover" and job["controller"] == "cascade":
        config = v2_configuration(source, design_version=2)
    if job["mode"] != "hover":
        config["plan"] = build_mission()[0]
    if job["case"] == "refined":
        n = config["numerics"]
        config["numerics"] = replace(
            n,
            time_step_s=n.time_step_s / 2,
            attitude_stride=n.attitude_stride * 2,
            position_stride=n.position_stride * 2,
        )
    if "offset" in job["case"]:
        sign = -1 if job["case"] == "offset_negative" else 1
        angle = sign * np.deg2rad(1.0)
        config["initial_state"] = replace(
            config["initial_state"],
            position_W=sign * np.array([0.02, -0.01, 0.015]),
            velocity_W=sign * np.array([0.01, 0, -0.01]),
            q_WB=np.array([np.cos(angle / 2), np.sin(angle / 2), 0.0, 0.0]),
            omega_B=sign * np.array([0.01, -0.01, 0.005]),
        )
    if job["case"] == "reversed_wind":
        config["truth_world"] = replace(
            config["truth_world"], wind_velocity_W=-config["truth_world"].wind_velocity_W
        )
    if job["mode"] == "true":
        del config["sensors"], config["estimator_configuration"]
    if job["controller"] == "geometric":
        config["geometric_controller"] = GeometricControllerParameters()
    elif job["mode"] == "estimated":
        config["allow_minimum_snap"] = True
    return config


def metrics(result: MissionResult, hover: bool = False) -> dict[str, Any]:
    score = score_result(result)
    actual_bound = bool(
        np.any((result.actual_rotor_omega <= 0) | (result.actual_rotor_omega >= 900))
    )
    score["actual_rotor_at_bound"] = actual_bound
    score["passed"] = score["passed"] and not actual_bound
    score["moment_effort_N2_m2_s"] = float(
        np.trapezoid(np.sum(result.actual_moment_B**2, axis=1), result.time_s)
    )
    score["requested_moment_variation_Nm"] = float(
        np.sum(np.linalg.norm(np.diff(result.moment_requested_B, axis=0), axis=1))
    )
    if hover:
        hold = (result.time_s >= 5) & (result.time_s <= 65)
        covered = bool(result.time_s[-1] >= 65 and np.any(hold))
        peak = (
            float(
                np.max(
                    np.linalg.norm(
                        result.position_W[hold] - result.reference_position_W[hold], axis=1
                    )
                )
            )
            if covered
            else None
        )
        score["full_hold_covered"] = covered
        score["hold_peak_m"] = peak
        score["passed"] = score["passed"] and peak is not None and peak <= 0.08
    return score


def execute(
    item: tuple[int, dict[str, Any], str],
    *,
    config_override: dict[str, Any] | None = None,
) -> dict[str, Any]:
    index, job, output = item
    directory = Path(output) / name_of(job)
    directory.mkdir()
    try:
        config = configuration(job) if config_override is None else config_override
        estimated = None
        if job["mode"] == "true":
            mission = simulate_mission(**config)
        else:
            estimated = simulate_estimated_mission(**config)
            mission = estimated.mission
        arrays = {
            f.name: getattr(mission, f.name) for f in fields(mission) if f.name != "abort_reason"
        }
        path = directory / "mission.npz"
        with path.open("xb") as stream:
            np.savez_compressed(stream, **arrays)
        files = [dict(file="mission.npz", sha256=hashlib.sha256(path.read_bytes()).hexdigest())]
        if estimated is not None:
            files += save_history(directory, index, pack_history(estimated, mission))
        row = dict(
            index=index,
            job=job,
            directory=directory.name,
            files=files,
            metrics=metrics(mission, job["mode"] == "hover"),
        )
    except Exception as error:
        row = dict(
            index=index, job=job, directory=directory.name, error=f"{type(error).__name__}: {error}"
        )
    (directory / "result.json").write_bytes(canonical_json(row) + b"\n")
    print(canonical_json(row).decode(), flush=True)
    return row


def summarize(rows: list[dict[str, Any]], directory: Path) -> dict[str, Any]:
    """Fail closed on a missing/duplicate case, incomplete flight, or failed pair."""
    expected = jobs()
    complete_ledger = len(rows) == len(expected) and all(
        r.get("index") == i and r.get("job") == j
        for i, (r, j) in enumerate(zip(rows, expected, strict=False))
    )
    by_name = {name_of(r["job"]): r for r in rows}
    comparisons = []
    for row in rows:
        j = row["job"]
        if j["repeat"] or j["controller"] != "geometric" or j["mode"] == "hover":
            continue
        pair = by_name.get(name_of({**j, "controller": "cascade"}))
        a, b = row.get("metrics", {}), (pair or {}).get("metrics", {})
        ratio = effort = None
        if a.get("position_rmse_m") is not None and b.get("position_rmse_m", 0):
            ratio = a["position_rmse_m"] / b["position_rmse_m"]
        if a.get("moment_effort_N2_m2_s") is not None and b.get("moment_effort_N2_m2_s", 0):
            effort = a["moment_effort_N2_m2_s"] / b["moment_effort_N2_m2_s"]
        passed = bool(
            a.get("passed")
            and b.get("passed")
            and ratio is not None
            and ratio <= 1
            and (j["mode"] == "true" or (effort is not None and effort <= 2))
        )
        comparisons.append(dict(job=j, rmse_ratio=ratio, effort_ratio=effort, passed=passed))
    repeats = []
    for row in rows:
        if not row["job"]["repeat"]:
            continue
        original = by_name.get(name_of({**row["job"], "repeat": False}))
        equal = False
        if original and "files" in original and "files" in row:
            with (
                np.load(directory / original["directory"] / "mission.npz", allow_pickle=False) as a,
                np.load(directory / row["directory"] / "mission.npz", allow_pickle=False) as b,
            ):
                equal = a.files == b.files and all(np.array_equal(a[k], b[k]) for k in a.files)
        repeats.append(dict(job=row["job"], mission_arrays_identical=equal))
    ratios = [
        r["rmse_ratio"]
        for r in comparisons
        if r["job"]["mode"] == "true" and r["job"]["case"] in TRUE_CASES[:5]
    ]
    mean = (
        float(np.mean(ratios)) if len(ratios) == 5 and all(r is not None for r in ratios) else None
    )
    refinement = []
    for controller in CONTROLLERS:
        ref = dict(mode="true", case="nominal", controller=controller, seed=None, repeat=False)
        a, b = by_name.get(name_of(ref)), by_name.get(name_of({**ref, "case": "refined"}))
        result: dict[str, Any] = dict(controller=controller, passed=False)
        if a and b and "files" in a and "files" in b:
            with (
                np.load(directory / a["directory"] / "mission.npz", allow_pickle=False) as coarse,
                np.load(directory / b["directory"] / "mission.npz", allow_pickle=False) as fine,
            ):
                same_clock = np.array_equal(coarse["time_s"], fine["time_s"][::2])
                if same_clock:
                    difference = float(
                        np.max(
                            np.linalg.norm(coarse["position_W"] - fine["position_W"][::2], axis=1)
                        )
                    )
                    dot = np.abs(np.sum(coarse["q_WB"] * fine["q_WB"][::2], axis=1))
                    angle = float(np.rad2deg(np.max(2 * np.arccos(np.clip(dot, 0, 1)))))
                    result.update(
                        position_difference_m=difference,
                        attitude_difference_deg=angle,
                        passed=difference <= 0.005 and angle <= 0.05,
                    )
        refinement.append(result)
    true_pass = (
        complete_ledger
        and all(r["passed"] for r in comparisons if r["job"]["mode"] == "true")
        and mean is not None
        and mean <= 0.8
        and all(r["passed"] for r in refinement)
    )
    return dict(
        complete_ledger=complete_ledger,
        comparisons=comparisons,
        first_five_mean_ratio=mean,
        repeats=repeats,
        refinement=refinement,
        true_state_passed=true_pass,
        all_performance_passed=bool(
            true_pass
            and all(r.get("metrics", {}).get("passed", False) for r in rows)
            and all(r["passed"] for r in comparisons)
            and all(r["mission_arrays_identical"] for r in repeats)
        ),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, choices=range(1, 5), default=3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    fingerprint = source_sha256(root)
    planned = jobs()
    configurations = [
        {
            k: plain(asdict(v)) if hasattr(v, "__dataclass_fields__") else plain(v)
            for k, v in configuration(j).items()
        }
        for j in planned
    ]
    protocol = dict(
        version=1,
        scope="ADR 0017 new implementation; no inherited qualification",
        jobs=planned,
        configurations=configurations,
        physical_targets=TARGETS,
        first_five_mean_ratio=0.8,
        maximum_paired_rmse_ratio=1.0,
        maximum_estimated_effort_ratio=2.0,
        full_hover_peak_m=0.08,
        hover_note=(
            "Geometric original gains versus established v2 cascade: context, "
            "not isolated control-law comparison."
        ),
    )
    data = canonical_json(protocol)
    (args.output / "protocol.json").write_bytes(data + b"\n")
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(execute, [(i, j, str(args.output)) for i, j in enumerate(planned)]))
    report: dict[str, Any] = dict(
        source_sha256=fingerprint,
        source_unchanged=fingerprint == source_sha256(root),
        protocol_sha256=hashlib.sha256(data).hexdigest(),
        rows=rows,
        summary=summarize(rows, args.output),
    )
    (args.output / "report.json").write_bytes(canonical_json(report) + b"\n")
    print(json.dumps(report["summary"], indent=2), flush=True)
    return 0 if report["source_unchanged"] and report["summary"]["all_performance_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
