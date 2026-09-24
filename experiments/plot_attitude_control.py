"""Headless plots of validated stored attitude evidence; never reruns the controller."""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from experiments.attitude_control_validation import (
    IDENTITY,
    _result,
    attitude_errors,
    canonical_json,
    make_case,
    validate_report,
)
from quadrotor_math.attitude_control import attitude_error_body


def _figure(rows: int, cols: int, title: str) -> tuple[Figure, Any]:
    figure = Figure(figsize=(12, 3.1 * rows), layout="constrained")
    FigureCanvasAgg(figure)
    figure.suptitle(title)
    return figure, figure.subplots(rows, cols, squeeze=False)


def render_report(report: dict[str, Any], output: Path) -> list[str]:
    """Validate ledger, metrics and clocks before creating a new plot directory."""
    validate_report(report)
    output.mkdir(parents=True, exist_ok=False)
    names = []
    status = "PASS" if report["summary"]["passed"] else "INCOMPLETE / FAIL"
    partition = report["protocol"]["partition"]
    trials = report["trials"]
    successful = [row for row in trials if row["status"] == "ok"]

    def save(figure: Figure, name: str) -> None:
        for axis in figure.axes:
            axis.grid(alpha=0.2)
            axis.set_xlabel("Time (s)")
        figure.savefig(output / name, dpi=150)
        names.append(name)

    if partition == "fixed":
        by_case = {row["job"]["case"]: row for row in successful if row["job"]["refinement"] == 1}
        figure, axes = _figure(3, 2, f"Signed reference steps — {status}")
        for axis_index, axis_name in enumerate(("roll", "pitch", "yaw")):
            for column, sign in enumerate(("positive", "negative")):
                axis = axes[axis_index, column]
                row = by_case.get(f"{axis_name}_{sign}")
                axis.set_title(f"{axis_name.capitalize()} {sign}")
                axis.set_ylabel("Rotation about body axis (deg)")
                if row is None:
                    axis.text(
                        0.5,
                        0.5,
                        "Trial failed / unavailable",
                        transform=axis.transAxes,
                        ha="center",
                    )
                    continue
                result = _result(row["history"])
                for values, label, style in (
                    (result.q_WB, "Actual", "-"),
                    (result.reference_q_WB, "Reference", "--"),
                ):
                    angle = [attitude_error_body(IDENTITY, value)[axis_index] for value in values]
                    axis.plot(result.time_s, np.rad2deg(angle), style, label=label)
                axis.legend(loc="best")
        save(figure, "attitude_steps.png")
        figure, axes = _figure(3, 2, f"Recovery, finite pulse and limiting — {status}")
        for index, case in enumerate(("coupled", "pulse", "saturation")):
            row = by_case.get(case)
            left, right = axes[index]
            left.set_title(case.capitalize())
            left.set_ylabel("Attitude error (deg)")
            right.set_ylabel("Moment (N m)")
            right.set_title("Dotted: requested; solid: actual rotor moment")
            if row is None:
                left.text(
                    0.5, 0.5, "Trial failed / unavailable", transform=left.transAxes, ha="center"
                )
                continue
            result = _result(row["history"])
            errors = np.rad2deg(attitude_errors(result))
            for j, name in enumerate(("x / roll", "y / pitch", "z / yaw")):
                color = f"C{j}"
                left.plot(result.time_s, errors[:, j], label=name, color=color)
                right.plot(
                    result.time_s, result.actual_moment_B[:, j], color=color, label=f"Actual {name}"
                )
                right.step(
                    result.control_time_s,
                    result.moment_requested_B[:, j],
                    where="post",
                    color=color,
                    linestyle=":",
                    alpha=0.6,
                )
            _, model, schedule = make_case(row["job"])
            if case == "pulse":
                right.step(
                    result.time_s[:-1],
                    schedule.disturbance_moment_B[:, 0],
                    where="post",
                    color="black",
                    linestyle="--",
                    label="External x",
                )
            if case == "saturation":
                for polarity in (-1, 1):
                    right.axhline(
                        polarity * model.maximum_moment_B[0],
                        color="black",
                        linestyle="--",
                        label="x/y clip bound" if polarity == 1 else None,
                    )
            left.legend(fontsize=8)
            right.legend(fontsize=7)
        save(figure, "attitude_recovery.png")
        if "saturation" in by_case:
            result = _result(by_case["saturation"]["history"])
            figure, axes = _figure(2, 1, f"Bounded commands and motor response — {status}")
            for j in range(4):
                axes[0, 0].plot(
                    result.time_s, result.actual_rotor_omega[:, j], label=f"Rotor {j + 1}"
                )
                axes[0, 0].step(
                    result.control_time_s,
                    result.commanded_rotor_omega[:, j],
                    where="post",
                    color=f"C{j}",
                    linestyle=":",
                    alpha=0.7,
                )
            axes[0, 0].set_ylabel("Rotor speed (rad/s)")
            axes[0, 0].set_title("Solid: actual; dotted: command (allowed interval 0–900 rad/s)")
            axes[0, 0].legend()
            for j, label in enumerate(("Rate demand", "Moment clipping", "Allocation scaling")):
                axes[1, 0].step(
                    result.control_time_s,
                    result.limit_flags[:, j].astype(float),
                    where="post",
                    label=label,
                    linestyle=("-", "--", ":")[j],
                )
            axes[1, 0].set_ylabel("Limit active (0 or 1)")
            axes[1, 0].legend()
            save(figure, "attitude_actuation.png")
        if "persistent" in by_case:
            row = by_case["persistent"]
            result = _result(row["history"])
            figure, axes = _figure(1, 2, f"Persistent torque diagnostic — {status}")
            angles = [attitude_error_body(IDENTITY, value)[0] for value in result.q_WB]
            axes[0, 0].plot(result.time_s, np.rad2deg(angles), label="Actual roll")
            axes[0, 0].axhline(
                row["metrics"]["predicted_roll_offset_deg"],
                linestyle="--",
                color="black",
                label="Predicted equilibrium",
            )
            axes[0, 0].set_ylabel("Roll (deg)")
            axes[0, 0].legend()
            axes[0, 1].plot(result.time_s, result.omega_B[:, 0])
            axes[0, 1].set_ylabel("Body roll rate (rad/s)")
            save(figure, "attitude_persistent_torque.png")
    else:
        figure, axes = _figure(
            1, 2, f"{partition.capitalize()} initial-attitude recovery — {status}"
        )
        seeds = [row["job"]["seed"] for row in successful]
        for axis, field, threshold, unit in (
            (axes[0, 0], "final_error_deg", 0.5, "Final attitude error (deg)"),
            (axes[0, 1], "final_rate_rad_s", 0.02, "Final rate norm (rad/s)"),
        ):
            axis.scatter(
                seeds, [row["metrics"][field] for row in successful], label="Completed trial"
            )
            axis.axhline(threshold, color="black", linestyle="--", label="Acceptance bound")
            axis.set_ylabel(unit)
            axis.legend()
        # This plot's horizontal coordinate is seed, not time.
        for axis in figure.axes:
            axis.set_xlabel("Independent seed")
            axis.grid(alpha=0.2)
        figure.savefig(output / "attitude_ensemble.png", dpi=150)
        names.append("attitude_ensemble.png")
    return names


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    data = args.input.read_bytes()
    names = render_report(json.loads(data), args.output)
    (args.output / "plot_manifest.json").write_bytes(
        canonical_json(
            {
                "input_sha256": hashlib.sha256(data).hexdigest(),
                "plots": names,
                "plotter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            }
        )
        + b"\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
