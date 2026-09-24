"""Headless plots from validated mission evidence; no rerunning or selected seeds."""

import argparse
import hashlib
from pathlib import Path
from typing import Any

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from experiments.attitude_control_validation import canonical_json
from experiments.position_control_validation import (
    load_report,
    result_from_history,
    validate_report,
)
from quadrotor_math.attitude_control import attitude_error_body


def render_report(report: dict[str, Any], output: Path) -> list[str]:
    validate_report(report)
    output.mkdir(parents=True, exist_ok=False)
    passed = "PASS" if report["summary"]["passed"] else "FAIL / INCOMPLETE"
    names: list[str] = []
    rows = [row for row in report["trials"] if row["status"] == "ok"]

    def save(figure: Figure, name: str) -> None:
        figure.savefig(output / name, dpi=140)
        names.append(name)

    if report["protocol"]["partition"] == "fixed":
        for row in rows:
            if row["job"]["refinement"] != 1:
                continue
            result = result_from_history(row["history"])
            figure = Figure(figsize=(12, 10), layout="constrained")
            FigureCanvasAgg(figure)
            case = row["job"]["case"]
            figure.suptitle(
                f"True-state {case.replace('_', ' ')} — {passed}\n"
                f"Position RMSE {row['metrics']['position_rmse_m']:.4f} m; "
                f"{row['metrics']['transitions'][-1]['phase']}"
            )
            axes = figure.subplots(3, 2)
            for col, quantity, label in (
                (0, "position_W", "NED position (m)"),
                (1, "velocity_W", "NED velocity (m/s)"),
            ):
                for coordinate, name in enumerate(("North", "East", "Down")):
                    axes[0, col].plot(
                        result.time_s,
                        getattr(result, quantity)[:, coordinate],
                        color=f"C{coordinate}",
                        label=f"Actual {name}",
                    )
                    axes[0, col].plot(
                        result.time_s,
                        getattr(result, "reference_" + quantity)[:, coordinate],
                        "--",
                        color=f"C{coordinate}",
                        alpha=0.75,
                        label=f"Reference {name}",
                    )
                axes[0, col].set_ylabel(label)
                axes[0, col].legend(fontsize=7, ncol=2)
            errors = np.linalg.norm(result.position_W - result.reference_position_W, axis=1)
            axes[1, 0].plot(result.time_s, errors, label="Position error norm")
            axes[1, 0].set_ylabel("Position error (m)")
            if len(result.control_time_s):
                held = np.searchsorted(result.control_time_s, result.time_s, side="right") - 1
                error_angles = np.rad2deg(
                    [
                        np.linalg.norm(attitude_error_body(q_WB, result.q_reference_WB[j]))
                        for q_WB, j in zip(result.q_WB, held, strict=True)
                    ]
                )
                axes[1, 1].plot(result.time_s, error_angles, label="Attitude error norm")
            tilt = np.rad2deg(
                np.arccos(np.clip(1 - 2 * np.sum(result.q_WB[:, 1:3] ** 2, axis=1), -1, 1))
            )
            axes[1, 1].plot(result.time_s, tilt, label="Actual tilt")
            axes[1, 1].axhline(35, linestyle="--", color="black", label="Actual tilt guard")
            axes[1, 1].set_ylabel("Angle (deg)")
            axes[1, 1].legend(fontsize=8)
            axes[2, 0].step(
                result.control_time_s,
                result.collective_thrust,
                where="post",
                label="Commanded collective",
            )
            for bound in (2.0, 18.0):
                axes[2, 0].axhline(bound, linestyle="--", color="black", alpha=0.6)
            axes[2, 0].set_ylabel("Collective thrust (N); bounds 2–18")
            axes[2, 1].step(result.time_s, result.phase, where="post")
            axes[2, 1].set_yticks(
                range(6), ["Initialize", "Takeoff", "Track", "Land", "Complete", "Abort"]
            )
            axes[2, 1].set_ylim(-0.2, 5.2)
            axes[2, 1].set_ylabel("Sampled mission phase")
            for axis in figure.axes:
                axis.set_xlabel("Time (s)")
                axis.grid(alpha=0.2)
            save(figure, f"mission_{case}.png")
        square = next(
            (
                row
                for row in rows
                if row["job"]["case"] == "square" and row["job"]["refinement"] == 1
            ),
            None,
        )
        if square is not None:
            result = result_from_history(square["history"])
            figure = Figure(figsize=(12, 8), layout="constrained")
            FigureCanvasAgg(figure)
            figure.suptitle(f"Square mission path and actuator evidence — {passed}")
            axes = figure.subplots(2, 2)
            axes[0, 0].plot(result.position_W[:, 1], result.position_W[:, 0], label="Actual")
            axes[0, 0].plot(
                result.reference_position_W[:, 1],
                result.reference_position_W[:, 0],
                "--",
                label="Reference",
            )
            axes[0, 0].set(xlabel="East (m)", ylabel="North (m)", aspect="equal")
            axes[0, 0].legend()
            for rotor in range(4):
                axes[0, 1].plot(
                    result.time_s,
                    result.actual_rotor_omega[:, rotor],
                    label=f"Actual {rotor + 1}",
                    color=f"C{rotor}",
                )
                axes[0, 1].step(
                    result.control_time_s,
                    result.commanded_rotor_omega[:, rotor],
                    where="post",
                    linestyle=":",
                    color=f"C{rotor}",
                )
            axes[0, 1].set(
                xlabel="Time (s)",
                ylabel="Rotor speed (rad/s)",
                title="Solid actual; dotted command; allowed 0–900",
            )
            axes[0, 1].legend(fontsize=8)
            for col, clock, flags, labels in (
                (
                    0,
                    result.control_time_s,
                    result.inner_limit_flags,
                    ("Rate", "Moment", "Allocation"),
                ),
                (
                    1,
                    result.position_control_time_s,
                    result.outer_limit_flags,
                    ("Acceleration", "Tilt", "Thrust"),
                ),
            ):
                for j, label in enumerate(labels):
                    axes[1, col].step(
                        clock,
                        flags[:, j].astype(float),
                        where="post",
                        label=label,
                        linestyle=("-", "--", ":")[j],
                    )
                axes[1, col].set(
                    xlabel="Time (s)", ylabel="Limit active (0 or 1)", ylim=(-0.1, 1.1)
                )
                axes[1, col].legend()
            for axis in figure.axes:
                axis.grid(alpha=0.2)
            save(figure, "mission_path_and_actuation.png")
    else:
        figure = Figure(figsize=(10, 4), layout="constrained")
        FigureCanvasAgg(figure)
        figure.suptitle(
            f"{report['protocol']['partition'].capitalize()} mission ensemble — {passed}"
        )
        axes = figure.subplots(1, 2)
        for axis, field, label, bound in (
            (axes[0], "position_rmse_m", "Full-mission position RMSE (m)", 0.15),
            (axes[1], "longest_actuator_limiting_s", "Longest actuator limiting (s)", 0.5),
        ):
            axis.scatter(
                [row["job"]["seed"] for row in rows],
                [row["metrics"][field] for row in rows],
                label="Completed numerical trial",
            )
            axis.axhline(bound, linestyle="--", color="black", label="Nominal acceptance bound")
            axis.set(xlabel="Deterministic initial-condition seed", ylabel=label)
            axis.ticklabel_format(useOffset=False, style="plain")
            axis.legend(fontsize=8)
            axis.grid(alpha=0.2)
        save(figure, "mission_ensemble.png")
    return names


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    report = load_report(args.input)
    names = render_report(report, args.output)
    (args.output / "plot_manifest.json").write_bytes(
        canonical_json(
            {
                "report_sha256": hashlib.sha256(
                    (args.input / "report.json").read_bytes()
                ).hexdigest(),
                "plotter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "plots": names,
            }
        )
        + b"\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
