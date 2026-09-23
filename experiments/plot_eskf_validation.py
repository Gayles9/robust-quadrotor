"""Render stored estimator evidence without rerunning or retuning the estimator.

python -m experiments.plot_eskf_validation --input /tmp/result.json --output /tmp/new-plots
All figures use the headless Agg backend and record their input's SHA-256.
"""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Patch

from experiments.eskf_validation import validate_validation_report


def _figure(rows: int, columns: int, width: float, height: float) -> tuple[Figure, Any]:
    figure = Figure(figsize=(width, height), layout="constrained")
    FigureCanvasAgg(figure)
    return figure, figure.subplots(rows, columns, squeeze=False)


def render_validation_report(report: dict[str, Any], output: Path) -> list[str]:
    """Write new plots only; incomplete groups are visibly omitted, never replaced by zeros."""
    # Validate the complete trial ledger and recompute the stored summaries before
    # creating output. Legacy v1 evidence gets its verified fixed clock restored.
    report = validate_validation_report(report)
    output.mkdir(parents=True, exist_ok=False)
    names: list[str] = []
    groups = report["groups"]
    status = (
        f"{report['protocol']['partition']}; "
        f"numerical failures {report['numerical_failure_count']}; "
        f"nominal/gated divergence {report['nominal_or_gated_divergence_count']}"
    )

    def save(figure: Figure, name: str) -> None:
        figure.savefig(output / name, dpi=150)
        names.append(name)

    nominal = groups.get("excited/nominal", {}).get("ensemble")
    if nominal is not None:
        figure, axes = _figure(2, 3, 13, 7)
        for col, (key, expected, title) in enumerate(
            (
                ("nees", 15, "Full-state NEES"),
                ("local_position_nis", 3, "Position NIS"),
                ("barometric_altitude_nis", 1, "Altitude NIS"),
            )
        ):
            scores = nominal[key]
            if scores is None:
                continue
            times = scores["time_s"]
            lower, upper = scores["pointwise_mean_interval_95"]
            axes[0, col].plot(times, scores["mean_by_epoch"], linewidth=1, label="Seed mean")
            axes[0, col].axhspan(
                lower, upper, color="green", alpha=0.15, label="Pointwise 95% reference"
            )
            axes[0, col].axhline(expected, color="black", linestyle=":", linewidth=0.8)
            axes[0, col].set(title=title, ylabel="Dimensionless score", xlabel="Time [s]")
            axes[1, col].plot(times, scores["coverage_by_epoch"], linewidth=0.8)
            axes[1, col].axhspan(0.90, 0.98, color="orange", alpha=0.15)
            axes[1, col].axhline(0.95, color="black", linestyle=":", linewidth=0.8)
            axes[1, col].set(
                ylim=(0, 1.02), xlabel="Time [s]", ylabel="Individual central95 coverage"
            )
        axes[0, 0].legend(fontsize=8)
        figure.suptitle("Nominal consistency: temporal summaries are descriptive\n" + status)
        save(figure, "consistency.png")

        figure, axes = _figure(1, 2, 11, 4)
        for col, (block, label) in enumerate(
            ((3, "Accelerometer bias error [m/s²]"), (4, "Gyroscope bias error [rad/s]"))
        ):
            for group_name, group in groups.items():
                if group_name.endswith("/nominal") and group["ensemble"] is not None:
                    ensemble = group["ensemble"]
                    axes[0, col].plot(
                        ensemble["sample_time_s"],
                        np.array(ensemble["mean_error_block_norms"])[:, block],
                        label=group_name.split("/")[0],
                    )
            axes[0, col].set(xlabel="Time [s]", ylabel=label, title="Mean three-axis error norm")
            axes[0, col].legend(fontsize=8)
        figure.suptitle("Bias convergence depends on excitation\n" + status)
        save(figure, "bias_convergence.png")

        trial = next(
            t
            for t in report["trials"]
            if t["family"] == "excited" and "representative" in t["variants"]["nominal"]
        )
        variant = trial["variants"]["nominal"]
        representative = variant["representative"]
        times = np.array(variant["sample_time_s"])
        errors = np.array(representative["error_states"])
        std = np.array(representative["standard_deviations"])
        figure, axes = _figure(3, 3, 12, 9)
        for row, (start, label) in enumerate(
            ((0, "Position [m]"), (9, "Accel. bias [m/s²]"), (12, "Gyro bias [rad/s]"))
        ):
            for col in range(3):
                index = start + col
                axes[row, col].plot(times, errors[:, index], label="Truth − estimate")
                axes[row, col].fill_between(
                    times,
                    -1.96 * std[:, index],
                    1.96 * std[:, index],
                    alpha=0.2,
                    label="Marginal ±1.96σ",
                )
                axes[row, col].set(
                    xlabel="Time [s]",
                    ylabel=label,
                    title=("NED " if start == 0 else "FRD ") + str(col + 1),
                )
        axes[0, 0].legend(fontsize=8)
        figure.suptitle(f"Representative nominal components, seed {trial['seed']}\n" + status)
        save(figure, "component_errors.png")

        completed = [
            t
            for t in report["trials"]
            if t["family"] == "excited"
            and all(
                t["variants"][v]["status"] == "completed" for v in ("nominal", "dead_reckoning")
            )
        ]
        # A complete paired ensemble is necessary for the RMSE distribution figure.
        planned = sum(t["family"] == "excited" for t in report["trials"])
        if len(completed) == planned:
            figure, axes = _figure(1, 2, 11, 4)
            for col, (unit, label) in enumerate(
                (("position_m", "Position RMSE [m]"), ("attitude_rad", "Attitude RMSE [rad]"))
            ):
                axes[0, col].boxplot(
                    [
                        [t["variants"][v]["metrics"]["rmse"][unit] for t in completed]
                        for v in ("nominal", "dead_reckoning")
                    ],
                    tick_labels=["Fused", "Dead reckoning"],
                    showfliers=True,
                )
                axes[0, col].set(
                    ylabel=label, yscale="log", title="All paired seeds (including divergence)"
                )
            figure.suptitle(status)
            save(figure, "rmse_distributions.png")

    gated = groups.get("excited/fault_gated", {}).get("ensemble")
    ungated = groups.get("excited/fault_ungated", {}).get("ensemble")
    if gated is not None and ungated is not None:
        figure, axes = _figure(2, 2, 12, 8)
        for name, ensemble in (("Gated", gated), ("Ungated", ungated)):
            times = ensemble["sample_time_s"]
            axes[0, 0].plot(times, ensemble["mean_horizontal_position_variance_m2"], label=name)
            axes[0, 1].plot(times, np.array(ensemble["mean_error_block_norms"])[:, 0], label=name)
        for axis in axes[0]:
            for interval, color, label in (
                ("position_dropout_s", "orange", "Position dropout"),
                ("delay_window_s", "gray", "Delayed acquisition window"),
            ):
                start, end = report["protocol"]["faults"][interval]
                # Do not stretch a short smoke plot out to absent fault windows.
                if times[0] < end and times[-1] > start:
                    axis.axvspan(
                        max(start, times[0]),
                        min(end, times[-1]),
                        alpha=0.12,
                        color=color,
                        label=label,
                    )
            axis.set(xlabel="Time [s]")
            axis.legend(fontsize=8)
        axes[0, 0].set(ylabel="P_NN + P_EE [m²]", title="Mean horizontal uncertainty")
        axes[0, 1].set(ylabel="Position error norm [m]", title="Identical fault realizations")
        sensors = ("local_position", "barometric_altitude")
        for index, key in enumerate(("precision", "recall")):
            values = [gated["fault_detection"][sensor][key] for sensor in sensors]
            available = [i for i, value in enumerate(values) if value is not None]
            axes[1, 0].bar(
                np.array(available) + index * 0.3,
                [values[i] for i in available],
                width=0.3,
                label=key,
                color=("tab:blue", "tab:orange")[index],
            )
            for sensor, value in enumerate(values):
                if value is None:
                    axes[1, 0].text(sensor + index * 0.3, 0.04, "N/A", ha="center", fontsize=8)
        axes[1, 0].set(
            xticks=[0.15, 1.15],
            xticklabels=["Position", "Altitude"],
            ylim=(0, 1.05),
            xlim=(-0.35, 1.65),
            ylabel="Fraction",
            title="Eligible fresh observations only (N/A: undefined)",
        )
        # Empty BarContainers otherwise get backend-default legend colors.
        axes[1, 0].legend(
            handles=[
                Patch(facecolor="tab:blue", label="precision"),
                Patch(facecolor="tab:orange", label="recall"),
            ],
            fontsize=8,
        )
        trial = next(t for t in report["trials"] if "fault_gated" in t["variants"])
        event_times = trial["variants"]["fault_gated"]["time_s"]
        events = [
            e
            for e in trial["variants"]["fault_gated"]["fault_events"]
            if e["kind"] == "local_position" and e["nis"] is not None
        ]
        for faulted, label in ((False, "Clean"), (True, "Injected outlier")):
            chosen = [
                e for e in events if (e["offset"] is not None and any(e["offset"])) == faulted
            ]
            axes[1, 1].scatter(
                [event_times[e["acquisition_index"]] for e in chosen],
                [e["nis"] for e in chosen],
                s=10,
                label=label,
            )
        axes[1, 1].axhline(
            report["protocol"]["gate"]["local_position_nis_threshold"],
            color="black",
            linestyle="--",
            label="Fixed gate",
        )
        axes[1, 1].set(
            xlabel="Acquisition time [s]",
            ylabel="Position NIS",
            yscale="log",
            title=f"Representative seed {trial['seed']}; rejected scores retained",
        )
        axes[1, 1].legend(fontsize=8)
        figure.suptitle("Fault behavior and detection\n" + status)
        save(figure, "faults.png")

    sensitivity = report["assessment"]["paired_sensitivity"]
    available = [name for name, data in sensitivity.items() if data is not None]
    if available:
        figure, axes = _figure(1, 2, 11, 4)
        for col, key, label in (
            (0, "position_rmse_means", "Position RMSE [m]"),
            (1, "nees_means", "Mean NEES"),
        ):
            axes[0, col].bar(
                np.arange(len(available)) - 0.15,
                [sensitivity[name][key]["nominal"] for name in available],
                width=0.3,
                label="Matched paired seeds",
            )
            axes[0, col].bar(
                np.arange(len(available)) + 0.15,
                [sensitivity[name][key][name] for name in available],
                width=0.3,
                label="Changed covariance",
            )
            axes[0, col].set(xticks=np.arange(len(available)), xticklabels=available, ylabel=label)
            axes[0, col].legend(fontsize=8)
        figure.suptitle("Q/R sensitivity: covariance multipliers .25 and 4\n" + status)
        save(figure, "sensitivity.png")
    return names


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    content = args.input.read_bytes()
    report = json.loads(content)
    names = render_validation_report(report, args.output)
    (args.output / "figures.json").write_text(
        json.dumps(
            {
                "input_sha256": hashlib.sha256(content).hexdigest(),
                "protocol_sha256": report["protocol_sha256"],
                "matplotlib_version": matplotlib.__version__,
                "figures": names,
                "omitted_incomplete_groups": [
                    name for name, group in report["groups"].items() if group["ensemble"] is None
                ],
            },
            indent=2,
        )
        + "\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
