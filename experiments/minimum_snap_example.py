"""Reproducible fixed-time spline example; no flight or feasibility claim."""

import argparse
import hashlib
from pathlib import Path

import numpy as np

from experiments.attitude_control_validation import canonical_json, source_sha256
from quadrotor_math.minimum_snap import minimum_snap_trajectory


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    points = np.array([[0.0, 0, 0], [1.0, 0, -1], [1.5, 1, -1.5], [2.0, 1, 0]])
    durations = np.array([2.0, 1.5, 2.5])
    trajectory = minimum_snap_trajectory(points, durations)
    times = np.linspace(0.0, float(trajectory.knot_times_s[-1]), 401)
    derivatives = np.array([[trajectory.evaluate(float(t), r) for t in times] for r in range(5)])
    # This feasible comparison stops with v=a=j=0 at each intermediate waypoint.
    stopped_cost = float(
        np.sum(100800 * np.sum(np.diff(points, axis=0) ** 2, axis=1) / durations**7)
    )
    nodes, weights = np.polynomial.legendre.leggauss(8)
    quadrature_cost = 0.0
    continuity = np.zeros(4)
    for i, duration in enumerate(durations):
        for axis in range(3):
            c = trajectory.coefficients_W[i, :, axis]
            d4 = np.polynomial.polynomial.polyder(c, 4) / duration**4
            samples = np.polynomial.polynomial.polyval((nodes + 1) / 2, d4)
            quadrature_cost += float(duration / 2 * (weights @ samples**2))
            if i:
                for order in range(4):
                    left = (
                        np.polynomial.polynomial.polyval(
                            1.0,
                            np.polynomial.polynomial.polyder(
                                trajectory.coefficients_W[i - 1, :, axis], order
                            ),
                        )
                        / durations[i - 1] ** order
                    )
                    right = (
                        np.polynomial.polynomial.polyval(
                            0.0, np.polynomial.polynomial.polyder(c, order)
                        )
                        / duration**order
                    )
                    continuity[order] = max(continuity[order], abs(left - right))
    waypoint_error = max(
        float(np.max(np.abs(trajectory.evaluate(float(t)) - point)))
        for t, point in zip(trajectory.knot_times_s, points, strict=True)
    )
    if (
        waypoint_error > 1e-9
        or np.max(continuity) > 1e-8
        or not np.isclose(quadrature_cost, trajectory.snap_cost, rtol=2e-10)
    ):
        raise RuntimeError("independent example verification failed")
    args.output.mkdir(parents=True, exist_ok=False)
    archive = args.output / "trajectory.npz"
    with archive.open("xb") as stream:
        np.savez_compressed(
            stream,
            waypoints_W=points,
            durations_s=durations,
            coefficients_W=trajectory.coefficients_W,
            origin_W=trajectory.position_origin_W,
            time_s=times,
            derivatives_W=derivatives,
        )
    report = {
        "semantics": "Fixed-duration polynomial optimization only; "
        "no actuator, obstacle or closed-loop validation.",
        "minimum_snap_cost_m2_s7": trajectory.snap_cost,
        "stopped_reference_cost_m2_s7": stopped_cost,
        "cost_reduction_percent": 100 * (1 - trajectory.snap_cost / stopped_cost),
        "quadrature_cost_m2_s7": quadrature_cost,
        "maximum_waypoint_residual_m": waypoint_error,
        "maximum_knot_discontinuity_by_order": continuity.tolist(),
        "scaled_system_condition": trajectory.scaled_system_condition,
        "relative_stationarity_residual": trajectory.relative_stationarity_residual,
        "source_sha256": source_sha256(Path(__file__).resolve().parents[1]),
        "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
    }
    (args.output / "report.json").write_bytes(canonical_json(report) + b"\n")
    # Matplotlib is an existing development-only dependency; the solver is NumPy-only.
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(11, 5), layout="constrained")
    ax = fig.add_subplot(121, projection="3d")
    p = derivatives[0]
    ax.plot(p[:, 0], p[:, 1], -p[:, 2], color="#176B87", linewidth=2)
    ax.scatter(points[:, 0], points[:, 1], -points[:, 2], color="#D16A38", s=35)
    ax.set(
        xlabel="North [m]",
        ylabel="East [m]",
        zlabel="Altitude [m]",
        title="Optimized path through fixed waypoints",
    )
    bx = fig.add_subplot(122)
    for axis, label in enumerate(("North", "East", "Down")):
        bx.plot(times, derivatives[4, :, axis], label=label)
    for time in trajectory.knot_times_s[1:-1]:
        bx.axvline(time, color="#b0b0b0", linewidth=0.7)
    bx.set(xlabel="Time [s]", ylabel="Snap [m/s⁴]", title="Fourth time derivative")
    bx.grid(alpha=0.2)
    bx.legend(frameon=False)
    fig.suptitle(
        f"Minimum snap: {report['cost_reduction_percent']:.1f}% lower cost "
        "than stopping at every waypoint"
    )
    fig.savefig(args.output / "trajectory.png", dpi=180)
    plt.close(fig)
    print(canonical_json(report).decode())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
