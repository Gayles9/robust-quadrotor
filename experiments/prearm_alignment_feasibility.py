"""ADR 0027 frozen offline feasibility; not a production arming interface."""

import argparse
import hashlib
import json
from dataclasses import asdict, dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any, TypedDict

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import source_sha256
from quadrotor_math.run_manifest import capture_software_provenance

Array = NDArray[np.float64]
ROOT = Path(__file__).resolve().parents[1]
DT, G = 0.0025, 9.81
SA, SG, WA, WG = 0.04, 0.002, 0.0002, 0.00002
BA, BG, HEADING = 0.03, 0.005, np.deg2rad(3.0)
TRIALS = 5000


def walk_factors(n: int) -> tuple[float, float, float]:
    """Mean variance, mean/terminal covariance and terminal variance per walk PSD."""
    if not isinstance(n, int) or n < 2:
        raise ValueError("at least two samples")
    return DT * (n - 1) * (2 * n - 1) / (6 * n), DT * (n - 1) / 2, DT * (n - 1)


def allocation() -> dict[str, float | int]:
    """Solve white-accel and white-plus-walk gyro inequalities before block rounding."""
    accel_n = int(np.ceil((SA / (G * np.deg2rad(0.02))) ** 2))
    a, b, c = WG**2 * DT / 3, np.deg2rad(0.01) ** 2 + WG**2 * DT / 2, SG**2 + WG**2 * DT / 6
    lower_root = 2 * c / (b + np.sqrt(b * b - 4 * a * c))
    gyro_n = int(np.ceil(lower_root))
    minimum = (max(accel_n, gyro_n) - 1) * DT
    duration = float(np.ceil(minimum / 0.5) * 0.5)
    n = int(round(duration / DT)) + 1
    mean_walk, _, _ = walk_factors(n)
    gyro_sigma = float(np.rad2deg(np.sqrt(SG**2 / n + WG**2 * mean_walk)))
    accel_sigma = float(np.rad2deg(SA / np.sqrt(n) / G))
    if accel_sigma > 0.02 or 3 * gyro_sigma > 0.03:
        raise ValueError("rounded acquisition fails frozen allocation")
    return dict(
        minimum_accel_samples=accel_n,
        minimum_gyro_samples=gyro_n,
        minimum_span_s=minimum,
        acquisition_span_s=duration,
        samples=n,
        accel_white_sigma_deg=accel_sigma,
        terminal_gyro_three_sigma_deg_s=3 * gyro_sigma,
    )


def euler_rotation(roll: float, pitch: float, heading: float = 0.0) -> Array:
    """ZYX body-to-world matrix, independently explicit NED/FRD convention."""
    cr, sr, cp, sp, cy, sy = (
        np.cos(roll),
        np.sin(roll),
        np.cos(pitch),
        np.sin(pitch),
        np.cos(heading),
        np.sin(heading),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def rotation_vector_matrix(vector: Array) -> Array:
    angle = float(np.linalg.norm(vector))
    x, y, z = vector
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.asarray(
        np.eye(3) + np.sinc(angle / np.pi) * K + (0.5 * np.sinc(angle / (2 * np.pi)) ** 2 * K @ K),
        dtype=np.float64,
    )


def rotation_log(R: Array) -> Array:
    """Principal SO(3) logarithm for this near-identity feasibility population."""
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2
    sine = float(np.linalg.norm(v))
    angle = np.arctan2(sine, (np.trace(R) - 1) / 2)
    return v if sine < 1e-15 else v * (angle / sine)


def inclination(s: Array) -> tuple[Array, Array, Array]:
    """Return R_WB at retained zero heading, local dtheta/ds and heading direction."""
    s = np.asarray(s, dtype=float)
    if s.shape != (3,) or not np.all(np.isfinite(s)):
        raise ValueError("finite three-vector")
    x, y, z = s
    r2, norm2 = y * y + z * z, float(s @ s)
    if norm2 < 1.0 or r2 < 0.25 * norm2:
        raise ValueError("ill-conditioned gravity direction")
    r = np.sqrt(r2)
    roll, pitch = np.arctan2(-y, -z), np.arctan2(x, r)
    E = np.array(
        [
            [1, 0, -np.sin(pitch)],
            [0, np.cos(roll), np.sin(roll) * np.cos(pitch)],
            [0, -np.sin(roll), np.cos(roll) * np.cos(pitch)],
        ]
    )
    D = np.array([[0, z / r2, -y / r2], [r / norm2, -x * y / (r * norm2), -x * z / (r * norm2)]])
    return euler_rotation(float(roll), float(pitch)), E[:, :2] @ D, E[:, 2]


def covariance(s: Array, n: int) -> Array:
    """First-order joint covariance in [right-local theta, terminal ba, terminal bg]."""
    _, J, u = inclination(s)
    A, B, T = walk_factors(n)
    P = np.zeros((9, 9))
    P[:3, :3] = (BA**2 + WA**2 * A + SA**2 / n) * J @ J.T + HEADING**2 * np.outer(u, u)
    P[:3, 3:6] = -(BA**2 + WA**2 * B) * J
    P[3:6, :3] = P[:3, 3:6].T
    P[3:6, 3:6] = (BA**2 + WA**2 * T) * np.eye(3)
    P[6:, 6:] = (SG**2 / n + WG**2 * A) * np.eye(3)
    return P


def release_covariance(P: Array, delay_s: float) -> Array:
    """Supported hold for exactly one fresh endpoint interval; no flight propagation."""
    if P.shape != (9, 9) or not np.all(np.isfinite(P)):
        raise ValueError("finite nine-state covariance")
    if not np.isfinite(delay_s) or abs(delay_s - DT) > 1e-12:
        raise ValueError("one fresh endpoint interval required")
    np.linalg.cholesky(P)
    result = P.copy()
    result[3:6, 3:6] += WA**2 * delay_s * np.eye(3)
    result[6:, 6:] += WG**2 * delay_s * np.eye(3)
    return result


def tail_bound(d: int, alpha: float = 0.001) -> float:
    t = -np.log(alpha)
    return float(d + 2 * np.sqrt(d * t) + 2 * t)


@lru_cache(maxsize=1)
def contrast_whiteners(n: int) -> tuple[Array, Array]:
    """Helmert contrasts remove an arbitrary constant without estimating it twice."""
    Q = np.zeros((n, n - 1))
    for j in range(n - 1):
        Q[: j + 1, j] = 1 / np.sqrt((j + 1) * (j + 2))
        Q[j + 1, j] = -(j + 1) / np.sqrt((j + 1) * (j + 2))
    times = np.arange(n) * DT
    K = np.minimum.outer(times, times)
    outputs = []
    for sigma, walk in ((SA, WA), (SG, WG)):
        C = sigma**2 * np.eye(n - 1) + walk**2 * Q.T @ K @ Q
        outputs.append(np.linalg.solve(np.linalg.cholesky(C), Q.T))
    return outputs[0], outputs[1]


@dataclass(frozen=True)
class Candidate:
    """Diagnostic values only; never an authorization to arm."""

    R_WB: Array
    gyro_bias_B: Array
    joint_covariance: Array
    statistics: Array
    radius_99_deg: float
    reasons: tuple[str, ...]


def evaluate(time_s: Array, accel: Array, gyro: Array, *, supported: bool) -> Candidate:
    """Offline reference for a zero-bias/zero-heading prior and frozen noise profile."""
    n = int(allocation()["samples"])
    if not supported:
        raise ValueError("external supported-stationary assertion required")
    if time_s.shape != (n,) or accel.shape != (n, 3) or gyro.shape != (n, 3):
        raise ValueError("complete paired sample window required")
    if not all(np.all(np.isfinite(x)) for x in (time_s, accel, gyro)):
        raise ValueError("finite data required")
    if np.max(np.abs(time_s - time_s[0] - np.arange(n) * DT)) > 1e-12:
        raise ValueError("400 Hz acquisition clocks required")
    s, mean_g = accel.mean(axis=0), gyro.mean(axis=0)
    R, _, _ = inclination(s)
    P = covariance(s, n)
    A, _, _ = walk_factors(n)
    Va, Vg = BA**2 + WA**2 * A + SA**2 / n, BG**2 + WG**2 * A + SG**2 / n
    wa, wg = contrast_whiteners(n)
    statistics = np.array(
        [
            mean_g @ mean_g / Vg,
            (np.linalg.norm(s) - G) ** 2 / Va,
            np.sum((wa @ accel) ** 2) + np.sum((wg @ gyro) ** 2),
        ]
    )
    reasons = []
    for name, value, d in zip(
        ("mean_gyro", "gravity_magnitude", "imu_variation"),
        statistics,
        (3, 3, 6 * (n - 1)),
        strict=True,
    ):
        if value > tail_bound(d):
            reasons.append(name)
    tilt = np.rad2deg(np.arccos(np.clip(R[2, 2], -1, 1)))
    if tilt > 15:
        reasons.append("inclination_domain")
    radius = float(np.rad2deg(np.sqrt(-2 * np.log(0.01) * np.linalg.eigvalsh(P[:2, :2])[-1])))
    if radius > 0.75:
        reasons.append("axis_uncertainty")
    return Candidate(R, mean_g, P, statistics, radius, tuple(reasons))


class Trial(TypedDict):
    R_WB: Array
    heading: float
    h: Array
    eta_a: Array
    ba_terminal: Array
    bg_terminal: Array
    accel: Array
    gyro: Array


def draw_trial(partition: int, trial: int, n: int) -> Trial:
    """Truth is isolated in the harness and never supplied to evaluate()."""
    rng = np.random.Generator(
        np.random.PCG64(np.random.SeedSequence([0x50524541, 1, partition, trial]))
    )
    if partition == 0:
        R = rotation_vector_matrix(rng.uniform(-np.deg2rad(2), np.deg2rad(2), 3))
        ba0, bg0 = rng.uniform(-0.02, 0.02, 3), rng.uniform(-0.003, 0.003, 3)
        heading = float(np.arctan2(R[1, 0], R[0, 0]))
    elif partition == 1:
        heading = float(rng.normal(0, HEADING))
        R = euler_rotation(np.deg2rad(1), np.deg2rad(-1), heading)
        ba0, bg0 = rng.normal(0, BA, 3), rng.normal(0, BG, 3)
    else:
        raise ValueError("frozen population partition")
    aw = np.vstack((np.zeros(3), np.cumsum(rng.normal(0, WA * np.sqrt(DT), (n - 1, 3)), axis=0)))
    gw = np.vstack((np.zeros(3), np.cumsum(rng.normal(0, WG * np.sqrt(DT), (n - 1, 3)), axis=0)))
    an, gn = rng.normal(0, SA, (n, 3)), rng.normal(0, SG, (n, 3))
    h = -G * R[2]
    return dict(
        R_WB=R,
        heading=heading,
        h=h,
        eta_a=ba0 + aw.mean(axis=0) + an.mean(axis=0),
        ba_terminal=ba0 + aw[-1],
        bg_terminal=bg0 + gw[-1],
        accel=h + ba0 + aw + an,
        gyro=bg0 + gw + gn,
    )


def population(partition: int) -> tuple[dict[str, Any], dict[str, Array]]:
    n = int(allocation()["samples"])
    time = np.arange(n, dtype=np.float64) * DT
    errors, whitened, linear_whitened, stats, radii, failures, axis_errors, covariances = (
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
    )
    reasons_count: dict[str, int] = {}
    for trial in range(TRIALS):
        truth = draw_trial(partition, trial, n)
        result = evaluate(time, truth["accel"], truth["gyro"], supported=True)
        Rtrue = truth["R_WB"]
        err = np.concatenate(
            (
                rotation_log(result.R_WB.T @ Rtrue),
                truth["ba_terminal"],
                truth["bg_terminal"] - result.gyro_bias_B,
            )
        )
        _, J, u = inclination(truth["h"])
        linear = np.concatenate((-J @ truth["eta_a"] + u * truth["heading"], err[3:]))
        linear_P = covariance(truth["h"], n)
        errors.append(err)
        whitened.append(np.linalg.solve(np.linalg.cholesky(result.joint_covariance), err))
        linear_whitened.append(np.linalg.solve(np.linalg.cholesky(linear_P), linear))
        stats.append(result.statistics)
        radii.append(result.radius_99_deg)
        failures.append(bool(result.reasons))
        axis_errors.append(
            np.rad2deg(
                np.arctan2(
                    np.linalg.norm(np.cross(result.R_WB[:, 2], Rtrue[:, 2])),
                    result.R_WB[:, 2] @ Rtrue[:, 2],
                )
            )
        )
        covariances.append(result.joint_covariance)
        for reason in result.reasons:
            reasons_count[reason] = reasons_count.get(reason, 0) + 1
    arrays = {
        "errors": np.array(errors),
        "whitened": np.array(whitened),
        "linear_whitened": np.array(linear_whitened),
        "statistics": np.array(stats),
        "radius_99_deg": np.array(radii),
        "rejected": np.array(failures, dtype=float),
        "axis_error_deg": np.array(axis_errors),
        "reported_covariances": np.array(covariances),
    }
    summary = {
        "trials": TRIALS,
        "rejected": int(np.sum(failures)),
        "rejection_fraction": float(np.mean(failures)),
        "reasons": reasons_count,
        "axis_error_deg_quantiles_50_95_99_max": np.quantile(
            axis_errors, [0.5, 0.95, 0.99, 1]
        ).tolist(),
        "radius_deg_min_max": [float(np.min(radii)), float(np.max(radii))],
        "empirical_radius_coverage": float(np.mean(np.array(axis_errors) <= radii)),
        "whitened_covariance_eigenvalues": np.linalg.eigvalsh(
            np.cov(np.array(whitened), rowvar=False)
        ).tolist(),
        "linear_whitened_covariance_eigenvalues": np.linalg.eigvalsh(
            np.cov(np.array(linear_whitened), rowvar=False)
        ).tolist(),
        "whitened_mean": np.mean(whitened, axis=0).tolist(),
    }
    return summary, arrays


def fixtures() -> dict[str, Any]:
    n = int(allocation()["samples"])
    time = np.arange(n, dtype=np.float64) * DT
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([0x50524541, 1, 2, 0])))
    accel = np.tile([0, 0, -G], (n, 1)) + rng.normal(0, SA, (n, 3))
    gyro = rng.normal(0, SG, (n, 3))
    output: dict[str, Any] = {}
    for name in ("impulse", "vibration", "changing_gravity", "mean_rate", "ambiguous_acceleration"):
        a, w = accel.copy(), gyro.copy()
        if name == "impulse":
            a[n // 2, 0] += 1
        elif name == "vibration":
            a[:, 0] += 0.2 * np.sin(2 * np.pi * 40 * time)
        elif name == "changing_gravity":
            a += np.array(
                [-G * euler_rotation(r, 0)[2] + [0, 0, G] for r in np.linspace(0, np.deg2rad(3), n)]
            )
        elif name == "mean_rate":
            w[:, 2] += 0.05
        else:
            acceleration = -G * euler_rotation(0, np.deg2rad(1))[2] + [0, 0, G]
            a += acceleration
            output["indistinguishable_world_acceleration_m_s2"] = acceleration.tolist()
        result = evaluate(time, a, w, supported=True)
        output[name] = {"reasons": result.reasons, "statistics": result.statistics.tolist()}
    for name, t, a, support in (
        ("missing_support", time, accel, False),
        ("bad_clock", time + np.linspace(0, 1e-5, n), accel, True),
        ("missing_sample", time[:-1], accel[:-1], True),
        ("nonfinite", time, np.full_like(accel, np.nan), True),
    ):
        try:
            evaluate(t, a, gyro, supported=support)
        except ValueError as exc:
            output[name] = str(exc)
        else:
            raise AssertionError(f"required rejection missing: {name}")
    return output


def run(output: Path) -> dict[str, Any]:
    if output.exists():
        raise ValueError("new output directory required; preserve previous results")
    output.mkdir(parents=True)
    nominal, nominal_arrays = population(0)
    gaussian, gaussian_arrays = population(1)
    stress = fixtures()
    checks = {
        "nominal_rejection": nominal["rejection_fraction"] <= 0.01,
        "gaussian_covariance": gaussian["whitened_covariance_eigenvalues"][-1] <= 1.10,
        "motion_fixtures": all(
            stress[k]["reasons"] for k in ("impulse", "vibration", "changing_gravity", "mean_rate")
        ),
        "ambiguity_preserved": not stress["ambiguous_acceleration"]["reasons"],
    }
    report = {
        "design": "ADR 0027; first-order covariance; no flight or hardware qualification",
        "source_sha256": source_sha256(ROOT),
        "protocol_sha256": hashlib.sha256(
            (ROOT / "docs/decisions/0027-stationary-prearm-alignment-design.md").read_bytes()
        ).hexdigest(),
        "software": asdict(capture_software_provenance(ROOT)),
        "allocation": allocation(),
        "nominal": nominal,
        "gaussian": gaussian,
        "fixtures": stress,
        "acceptance": checks,
        "standalone_implementation_go": all(checks.values()),
        "flight_qualified": False,
    }
    np.savez_compressed(
        output / "trials.npz",
        allow_pickle=False,
        **{f"nominal_{k}": v for k, v in nominal_arrays.items()},
        **{f"gaussian_{k}": v for k, v in gaussian_arrays.items()},
    )
    (output / "report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.output), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
