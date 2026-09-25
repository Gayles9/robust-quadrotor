"""Validated full-history plots and explicitly local design diagnostics (ADR 0014)."""

import argparse
import hashlib
import json
from dataclasses import replace
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from experiments.attitude_control_validation import canonical_json  # noqa: E402
from experiments.feedback_bandwidth_validation import (  # noqa: E402
    designed_horizontal_cascade,
    load_report,
    validate_report,
)
from experiments.plot_estimated_feedback import _render as render_histories  # noqa: E402


def _render(report: dict[str, Any], output: Path) -> list[str]:
    names = render_histories(report, output)
    design = designed_horizontal_cascade(report["protocol"]["version"])
    original = replace(
        design, position_gain=1.0, velocity_gain=1.8, attitude_gain=3.0, rate_gain=12.0
    )
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")
    for label, model in (("Original", original), ("Pole-placed", design)):
        transition = model.lifted_transition()
        poles = np.log(np.linalg.eigvals(transition).astype(complex)) / 0.02
        axes[0].scatter(poles.real, poles.imag, label=label, s=60)
        times = np.arange(501) * 0.02
        states = np.empty((len(times), 5))
        states[0] = [0, 0.15, 0, 0, 0]
        for i in range(1, len(times)):
            states[i] = transition @ states[i - 1]
        axes[1].plot(times, states[:, 0], label=label)
    axes[0].axvline(0, color="black", linewidth=0.8)
    axes[0].set(
        xlabel="Real part [1/s]",
        ylabel="Imaginary part [rad/s]",
        title="Equivalent poles: log(discrete pole)/0.02 s",
    )
    axes[1].set(
        xlabel="Time [s]",
        ylabel="Horizontal position error [m]",
        title="Local response to initial velocity 0.15 m/s",
    )
    for ax in axes:
        ax.legend()
    fig.suptitle("Linearized, matched hover design — not nonlinear flight results")
    fig.savefig(output / "feedback_local_design.png", dpi=160)
    plt.close(fig)
    names.append("feedback_local_design.png")
    rows = report["trials"]
    hover = [row for row in rows if row["job"]["case"] == "hover"]
    if hover:
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
        for ax, key, limit, label in (
            (axes[0], "hover_segment_peak_error_m", 0.08, "True hold peak error [m], 5–65 s"),
            (
                axes[1],
                "peak_attitude_estimation_deg",
                15.0,
                "Full-mission attitude estimation peak [deg]",
            ),
        ):
            values = []
            for row in hover:
                if row["status"] != "ok":
                    values.append(np.nan)
                else:
                    metrics = row["metrics"]
                    values.append(
                        metrics["estimated_feedback"][key] if ax is axes[0] else metrics[key]
                    )
            x = np.arange(len(hover))
            ax.bar(x, values)
            ax.axhline(limit, color="black", linestyle="--", label="Frozen limit")
            ax.set_xticks(x, [str(row["job"]["seed"]) for row in hover], rotation=60, ha="right")
            ax.set(ylabel=label, xlabel="Every planned hover seed")
            ax.legend()
        fig.suptitle("Noisy 60-second hover — tracking and attitude are separate requirements")
        fig.savefig(output / "feedback_hover_acceptance.png", dpi=160)
        plt.close(fig)
        names.append("feedback_hover_acceptance.png")
    path = output / "plot_manifest.json"
    manifest = json.loads(path.read_bytes())
    manifest["extension_plotter_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    manifest["files"] = names
    path.write_bytes(canonical_json(manifest) + b"\n")
    return names


def render_report(report: dict[str, Any], output: Path) -> list[str]:
    validate_report(report)
    return _render(report, output)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    report = load_report(args.input, workers=args.workers)
    print(json.dumps(_render(report, args.output)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
