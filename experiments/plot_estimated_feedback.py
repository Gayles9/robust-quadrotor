"""Headless full-data plots from a validated estimated-feedback evidence bundle."""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from experiments.attitude_control_validation import canonical_json  # noqa: E402
from experiments.estimated_feedback_evidence import unpack_history  # noqa: E402
from experiments.estimated_feedback_validation import load_report, validate_report  # noqa: E402
from quadrotor_math.attitude_control import attitude_error_body  # noqa: E402


def _render(report: dict[str, Any], output: Path) -> list[str]:
    output.mkdir(parents=True, exist_ok=False)
    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.25})
    names = []
    if report["protocol"]["partition"] == "fixed":
        for index, row in enumerate(report["trials"]):
            if row["status"] != "ok":
                continue
            result, baseline = unpack_history(row["history"])
            r = result.mission
            time = r.time_s
            estimated_position = np.array([s.position_W for s in result.estimates.states])
            fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
            ax = axes[0, 0]
            ax.plot(time, r.position_W[:, 2], label="Actual z, estimated feedback")
            ax.plot(time, estimated_position[:, 2], label="Estimated z", alpha=0.8)
            ax.plot(time, r.reference_position_W[:, 2], "--", label="Reference z")
            ax.set(xlabel="Time [s]", ylabel="NED down position z [m]")
            ax.legend(fontsize=8)
            ax = axes[0, 1]
            ax.plot(
                time,
                np.linalg.norm(r.position_W - r.reference_position_W, axis=1),
                label="Estimated-feedback run",
            )
            ax.plot(
                baseline.time_s,
                np.linalg.norm(baseline.position_W - baseline.reference_position_W, axis=1),
                label="Paired true-feedback run",
            )
            ax.set(xlabel="Time [s]", ylabel="True tracking error norm [m]")
            ax.legend(fontsize=8)
            ax = axes[1, 0]
            ax.plot(time, np.linalg.norm(estimated_position - r.position_W, axis=1))
            ax.set(xlabel="Time [s]", ylabel="Position estimation error norm [m]")
            ax = axes[1, 1]
            angle = np.rad2deg(
                [
                    np.linalg.norm(attitude_error_body(s.q_WB, q_WB))
                    for s, q_WB in zip(result.estimates.states, r.q_WB, strict=True)
                ]
            )
            ax.plot(time, angle)
            ax.set(xlabel="Time [s]", ylabel="Attitude estimation error [deg]")
            label = "noiseless" if row["job"]["noiseless"] else f"seed {row['job']['seed']}"
            fig.suptitle(
                f"{row['job']['case'].replace('_', ' ').title()} — {label}\n"
                "Matched-model numerical mission; "
                f"completed={row['metrics']['estimated_feedback']['completed']}"
            )
            name = f"feedback_{index:02d}_{row['job']['case']}.png"
            fig.savefig(output / name, dpi=160)
            plt.close(fig)
            names.append(name)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), layout="constrained")
    rows = report["trials"]
    x = np.arange(len(rows))
    labels = ["noiseless" if r["job"]["seed"] is None else str(r["job"]["seed"]) for r in rows]
    actual = [
        r["metrics"]["estimated_feedback"]["position_rmse_m"] if r["status"] == "ok" else np.nan
        for r in rows
    ]
    true = [
        r["metrics"]["true_feedback"]["position_rmse_m"] if r["status"] == "ok" else np.nan
        for r in rows
    ]
    estimation = [
        r["metrics"]["estimation_position_rmse_m"] if r["status"] == "ok" else np.nan for r in rows
    ]
    axes[0].bar(x - 0.18, actual, width=0.36, label="Estimated feedback")
    axes[0].bar(x + 0.18, true, width=0.36, label="True feedback")
    axes[0].axhline(0.15, color="black", linestyle="--", label="Mission RMSE target")
    axes[0].set(ylabel="Full-mission true position RMSE [m]")
    axes[0].legend(fontsize=8)
    axes[1].bar(x, estimation)
    axes[1].axhline(0.10, color="black", linestyle="--")
    axes[1].set(ylabel="Position estimation RMSE [m]")
    axes[2].bar(
        x,
        [
            r["metrics"]["estimated_feedback"]["longest_actuator_limiting_s"]
            if r["status"] == "ok"
            else np.nan
            for r in rows
        ],
    )
    axes[2].axhline(0.5, color="black", linestyle="--")
    axes[2].set(ylabel="Longest actuator limiting [s]")
    for ax in axes:
        ax.set_xticks(x, labels, rotation=60, ha="right")
        ax.set_xlabel("Planned seed / deterministic case")
        for k, row in enumerate(rows):
            if row["status"] != "ok" or not row["metrics"]["passed"]:
                ax.text(
                    k,
                    0.02,
                    "FAIL",
                    transform=ax.get_xaxis_transform(),
                    color="red",
                    rotation=90,
                    va="bottom",
                    ha="center",
                )
    fig.suptitle(
        f"{report['protocol']['partition'].title()} — all {len(rows)} planned trials\n"
        "Separate paired closed loops; no sensor-noise or hardware generalization"
    )
    fig.savefig(output / "feedback_ensemble.png", dpi=160)
    plt.close(fig)
    names.append("feedback_ensemble.png")
    inputs = {
        "protocol_sha256": report["protocol_sha256"],
        "summary": report["summary"],
        "trials": [
            {
                "job": r["job"],
                "status": r["status"],
                "metrics": r.get("metrics"),
                "history_sha256": {
                    name: hashlib.sha256(value.tobytes()).hexdigest()
                    for name, value in r.get("history", {}).items()
                },
            }
            for r in rows
        ],
    }
    manifest = {
        "input_sha256": hashlib.sha256(canonical_json(inputs)).hexdigest(),
        "plotter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "source_sha256": report.get("source_sha256"),
        "files": names,
    }
    (output / "plot_manifest.json").write_bytes(canonical_json(manifest) + b"\n")
    return names


def render_report(report: dict[str, Any], output: Path) -> list[str]:
    validate_report(report)
    return _render(report, output)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    report = load_report(args.input)
    print(json.dumps(_render(report, args.output)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
