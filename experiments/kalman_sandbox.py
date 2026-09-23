"""Small scalar and position/velocity Kalman examples, separate from flight code.

Run with python -m experiments.kalman_sandbox --output /tmp/new-sandbox-directory.
The fixed-seed examples illustrate covariance and incorrect Q/R, not ESKF performance.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from quadrotor_math.eskf import _validated_float64_matrix
from quadrotor_math.eskf_replay import _owned_array, _scalar


def scalar_kalman_update(
    mean: float, variance: float, measurement: float, noise_variance: float
) -> tuple[float, float, float]:
    """K=P/(P+R), x+=K(z-x), Joseph P+=(1-K)^2 P+K^2 R; independent noise."""
    mean = _scalar("mean", mean)
    measurement = _scalar("measurement", measurement)
    variance = _scalar("variance", variance, nonnegative=True)
    noise_variance = _scalar("noise_variance", noise_variance, nonnegative=True)
    total = variance + noise_variance
    if not np.isfinite(total) or total <= 0:
        raise ValueError("innovation variance must be positive and finite")
    gain = variance / total
    result = (
        mean + gain * (measurement - mean),
        (1 - gain) ** 2 * variance + gain**2 * noise_variance,
        gain,
    )
    if not np.all(np.isfinite(result)):
        raise ValueError("scalar correction must remain finite")
    return result


def position_velocity_prediction(
    state: NDArray[np.float64],
    covariance: NDArray[np.float64],
    acceleration: float,
    acceleration_variance: float,
    dt: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """1D sampled-acceleration model: x+=F x+B a, P+=F P F.T+B sigma_a² B.T.

    x=[position metres, velocity metres/second]. Acceleration in m/s² is held
    over dt seconds; its independent per-sample variance is (m/s²)². This exact
    linear example differs from the ESKF's continuous-noise discretization.
    """
    state = _owned_array("state", state, (2,))
    covariance = _validated_float64_matrix(
        "covariance",
        _owned_array("covariance", covariance, (2, 2)),
        (2, 2),
        symmetric_positive_semidefinite=True,
    )
    acceleration = _scalar("acceleration", acceleration)
    acceleration_variance = _scalar(
        "acceleration_variance", acceleration_variance, nonnegative=True
    )
    dt = _scalar("dt", dt, nonnegative=True)
    try:
        with np.errstate(over="raise", invalid="raise"):
            F = np.array([[1.0, dt], [0.0, 1.0]])
            B = np.array([0.5 * dt * dt, dt])
            predicted = F @ state + B * acceleration
            P = F @ covariance @ F.T + np.outer(B, B) * acceleration_variance
            if not np.all(np.isfinite(predicted)) or not np.all(np.isfinite(P)):
                raise FloatingPointError
    except FloatingPointError:
        raise ValueError("linear prediction must remain finite") from None
    return predicted, P


def sandbox_examples() -> dict[str, object]:
    """Fixed example realization for plots; incorrect covariances never change truth/noise."""
    rng = np.random.Generator(np.random.PCG64(901))
    measurements = 2.0 + rng.standard_normal(101)
    mean, variance = 0.0, 4.0
    scalar = []
    for observed in measurements:
        mean, variance, gain = scalar_kalman_update(mean, variance, float(observed), 1.0)
        scalar.append([mean, variance, gain])
    dt = 0.1
    times = np.arange(301) * dt
    truth = np.column_stack((0.5 * 0.2 * times**2, 0.2 * times))
    measured_acc = 0.2 + 0.1 * rng.standard_normal(len(times))
    measured_pos = truth[:, 0] + 0.3 * rng.standard_normal(len(times))
    runs = {}
    for name, q_scale, r_scale in (
        ("matched", 1, 1),
        ("Q_low", 0.1, 1),
        ("Q_high", 10, 1),
        ("R_low", 1, 0.1),
        ("R_high", 1, 10),
    ):
        state = np.array([0.2, -0.1])
        P = np.diag([0.3**2, 0.2**2])
        records = []
        for k in range(len(times)):
            if k:
                state, P = position_velocity_prediction(
                    state, P, float(measured_acc[k - 1]), 0.1**2 * q_scale, dt
                )
            if k % 5 == 0:
                R = 0.3**2 * r_scale
                gain = P[:, 0] / (P[0, 0] + R)
                state = state + gain * (measured_pos[k] - state[0])
                A = np.eye(2) - np.outer(gain, [1, 0])
                P = A @ P @ A.T + np.outer(gain, gain) * R
            records.append([*state, *np.diag(P)])
        runs[name] = np.array(records).tolist()
    return {
        "seed": 901,
        "scalar_truth": 2.0,
        "scalar": scalar,
        "time_s": times.tolist(),
        "truth": truth.tolist(),
        "position_velocity": runs,
    }


def main(argv: list[str] | None = None) -> int:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    report = sandbox_examples()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "data.json").write_text(json.dumps(report, allow_nan=False) + "\n")
    figure = Figure(figsize=(10, 7), layout="constrained")
    FigureCanvasAgg(figure)
    axes = figure.subplots(2, 1)
    scalar = np.array(report["scalar"])
    axes[0].plot(scalar[:, 0], label="Estimate")
    axes[0].fill_between(
        np.arange(len(scalar)),
        scalar[:, 0] - 2 * np.sqrt(scalar[:, 1]),
        scalar[:, 0] + 2 * np.sqrt(scalar[:, 1]),
        alpha=0.2,
        label="Marginal ±2σ",
    )
    axes[0].axhline(2, color="black", linestyle="--", label="Truth")
    axes[0].set(
        xlabel="Measurement number", ylabel="Scalar value", title="Constant-state Kalman filter"
    )
    axes[0].legend()
    times = np.array(report["time_s"])
    truth = np.array(report["truth"])
    runs = report["position_velocity"]
    assert isinstance(runs, dict)
    for name, values in runs.items():
        history = np.array(values)
        axes[1].plot(times, history[:, 0] - truth[:, 0], label=name)
    history = np.array(runs["matched"])
    axes[1].fill_between(
        times,
        -2 * np.sqrt(history[:, 2]),
        2 * np.sqrt(history[:, 2]),
        alpha=0.15,
        label="Matched ±2σ",
    )
    axes[1].set(
        xlabel="Time [s]",
        ylabel="Position estimate − truth [m]",
        title="Sampled acceleration: fixed data, changed Q/R",
    )
    axes[1].legend(ncols=3, fontsize=8)
    figure.savefig(args.output / "kalman_examples.png", dpi=150)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
