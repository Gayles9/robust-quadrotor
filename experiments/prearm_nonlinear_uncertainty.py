"""ADR 0028: one Gaussian-pushforward uncertainty study, never a flight initializer."""

import argparse
import hashlib
import json
from dataclasses import asdict, dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from experiments import prearm_alignment_feasibility as base
from experiments.attitude_control_validation import source_sha256
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.run_manifest import capture_software_provenance

Array = NDArray[np.float64]
ROOT = Path(__file__).resolve().parents[1]
TRIALS = 5000
PRIOR_REPORT = "260532b86c36cb7cd31c3d2fd079f6ff9abd4bb975b350522984cc1dafb4dfba"
PRIOR_TRIALS = "953adb6dfa156abffaa726fab58fc4a803efb3c13acba6c960a2dba2c7c90451"


@lru_cache(maxsize=2)
def normal_rule(order: int) -> tuple[Array, Array]:
    """Positive tensor Gauss-Hermite rule for four independent standard normals."""
    if order not in (5, 7):
        raise ValueError("frozen candidate order five or accuracy-reference order seven")
    x, w = np.polynomial.hermite.hermgauss(order)
    indices = np.stack(np.meshgrid(*([np.arange(order)] * 4), indexing="ij"), axis=-1).reshape(
        -1, 4
    )
    nodes = np.asarray(np.sqrt(2) * x[indices], dtype=np.float64)
    weights = np.asarray(np.prod(w[indices] / np.sqrt(np.pi), axis=1), dtype=np.float64)
    nodes.setflags(write=False)
    weights.setflags(write=False)
    return nodes, weights


def quaternion_product(left: Array, right: Array) -> Array:
    """Hamilton product with NumPy broadcasting and scalar-first convention."""
    scalar = left[..., :1] * right[..., :1] - np.sum(
        left[..., 1:] * right[..., 1:], axis=-1, keepdims=True
    )
    vector = (
        left[..., :1] * right[..., 1:]
        + right[..., :1] * left[..., 1:]
        + np.cross(left[..., 1:], right[..., 1:])
    )
    return np.concatenate((scalar, vector), axis=-1)


def quaternion_exp(vector: Array) -> Array:
    angle = float(np.linalg.norm(vector))
    return np.asarray(
        np.r_[np.cos(angle / 2), 0.5 * np.sinc(angle / (2 * np.pi)) * vector], dtype=np.float64
    )


def local_logs(center_q_WB: Array, samples_q_WB: Array) -> Array:
    """Right-local principal logs; quaternion sign independent, near-mean domain."""
    relative = quaternion_product(center_q_WB * [1, -1, -1, -1], samples_q_WB)
    relative /= np.linalg.norm(relative, axis=-1, keepdims=True)
    relative = np.where(relative[..., :1] < 0, -relative, relative)
    v = relative[..., 1:]
    norm = np.linalg.norm(v, axis=-1)
    scale = np.full_like(norm, 2.0)
    np.divide(2 * np.arctan2(norm, relative[..., 0]), norm, out=scale, where=norm > 1e-15)
    return np.asarray(v * scale[..., None], dtype=np.float64)


def gravity_quaternions(s: Array, heading: Array) -> Array:
    """Inverse gravity direction, ZYX heading supplied independently of the IMU."""
    s = np.asarray(s, dtype=np.float64)
    heading = np.asarray(heading, dtype=np.float64)
    if s.ndim != 2 or s.shape[1] != 3 or heading.shape != (len(s),):
        raise ValueError("paired gravity three-vectors and headings")
    if not np.all(np.isfinite(s)) or not np.all(np.isfinite(heading)):
        raise ValueError("finite gravity and headings")
    r2 = np.sum(s[:, 1:] ** 2, axis=1)
    norm2 = np.sum(s**2, axis=1)
    if np.any(norm2 < 1) or np.any(r2 < 0.25 * norm2):
        raise ValueError("ill-conditioned quadrature gravity direction")
    roll, pitch = np.arctan2(-s[:, 1], -s[:, 2]), np.arctan2(s[:, 0], np.sqrt(r2))
    cr, sr, cp, sp, cy, sy = (
        np.cos(roll / 2),
        np.sin(roll / 2),
        np.cos(pitch / 2),
        np.sin(pitch / 2),
        np.cos(heading / 2),
        np.sin(heading / 2),
    )
    return np.stack(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ),
        axis=-1,
    )


def bias_conditioning(n: int) -> tuple[float, float, float, float]:
    """Va, gain C/Va, conditional terminal accel-bias variance, terminal gyro variance."""
    A, B, T = base.walk_factors(n)
    Va = base.BA**2 + base.WA**2 * A + base.SA**2 / n
    C = base.BA**2 + base.WA**2 * B
    Pa = base.BA**2 + base.WA**2 * T
    residual = Pa - C * C / Va
    if residual <= 0:
        raise ValueError("positive conditional bias covariance required")
    return Va, C / Va, residual, base.SG**2 / n + base.WG**2 * A


@dataclass(frozen=True)
class Moments:
    q_WB: Array
    joint_covariance: Array
    shift_B: Array
    centered_mean_B: Array
    corrections: int


def nonlinear_moments(s: Array, n: int, *, order: int = 5) -> Moments:
    """Local Gaussian pushforward; not an exact gravity-constrained Bayesian posterior."""
    s = np.asarray(s, dtype=np.float64)
    base.inclination(s)
    nodes, weights = normal_rule(order)
    Va, gain, residual, Pg = bias_conditioning(n)
    eta = np.sqrt(Va) * nodes[:, :3]
    samples_q_WB = gravity_quaternions(s - eta, base.HEADING * nodes[:, 3])
    initial_q_WB = gravity_quaternions(s[None, :], np.zeros(1))[0]
    center_q_WB = initial_q_WB.copy()
    for corrections in range(9):
        delta = local_logs(center_q_WB, samples_q_WB)
        mean = weights @ delta
        if np.linalg.norm(mean) <= 1e-13:
            break
        if corrections == 8:
            raise ValueError("rotation mean did not converge within eight corrections")
        center_q_WB = quaternion_product(center_q_WB, quaternion_exp(mean))
        center_q_WB /= np.linalg.norm(center_q_WB)
    centered = delta - mean
    bias_mean_nodes = gain * eta
    P = np.zeros((9, 9))
    P[:3, :3] = centered.T @ (weights[:, None] * centered)
    P[:3, 3:6] = centered.T @ (weights[:, None] * bias_mean_nodes)
    P[3:6, :3] = P[:3, 3:6].T
    P[3:6, 3:6] = bias_mean_nodes.T @ (weights[:, None] * bias_mean_nodes) + residual * np.eye(3)
    P[6:, 6:] = Pg * np.eye(3)
    P = (P + P.T) / 2
    np.linalg.cholesky(P)
    shift = local_logs(initial_q_WB, center_q_WB[None, :])[0]
    return Moments(center_q_WB, P, shift, mean, corrections)


def candidate(s: Array, original: base.Candidate) -> tuple[base.Candidate, Moments]:
    moments = nonlinear_moments(s, int(base.allocation()["samples"]))
    P = moments.joint_covariance
    R = rotation_matrix_body_to_world(moments.q_WB)
    radius = float(np.rad2deg(np.sqrt(-2 * np.log(0.01) * np.linalg.eigvalsh(P[:2, :2])[-1])))
    reasons = [r for r in original.reasons if r != "axis_uncertainty"]
    if radius > 0.75:
        reasons.append("axis_uncertainty")
    return base.Candidate(
        R, original.gyro_bias_B, P, original.statistics, radius, tuple(reasons)
    ), moments


def draw_trial(partition: int, trial: int, n: int) -> base.Trial:
    """Fresh version-two streams, same frozen distributions and draw construction."""
    rng = np.random.Generator(
        np.random.PCG64(np.random.SeedSequence([0x50524541, 2, partition, trial]))
    )
    if partition == 0:
        R = base.rotation_vector_matrix(rng.uniform(-np.deg2rad(2), np.deg2rad(2), 3))
        ba0, bg0 = rng.uniform(-0.02, 0.02, 3), rng.uniform(-0.003, 0.003, 3)
        heading = float(np.arctan2(R[1, 0], R[0, 0]))
    elif partition == 1:
        heading = float(rng.normal(0, base.HEADING))
        R = base.euler_rotation(np.deg2rad(1), np.deg2rad(-1), heading)
        ba0, bg0 = rng.normal(0, base.BA, 3), rng.normal(0, base.BG, 3)
    else:
        raise ValueError("frozen fresh population")
    aw = np.vstack(
        (np.zeros(3), np.cumsum(rng.normal(0, base.WA * np.sqrt(base.DT), (n - 1, 3)), axis=0))
    )
    gw = np.vstack(
        (np.zeros(3), np.cumsum(rng.normal(0, base.WG * np.sqrt(base.DT), (n - 1, 3)), axis=0))
    )
    an, gn = rng.normal(0, base.SA, (n, 3)), rng.normal(0, base.SG, (n, 3))
    h = -base.G * R[2]
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


def whitened_error(result: base.Candidate, truth: base.Trial) -> tuple[Array, Array]:
    error = np.concatenate(
        (
            base.rotation_log(result.R_WB.T @ truth["R_WB"]),
            truth["ba_terminal"],
            truth["bg_terminal"] - result.gyro_bias_B,
        )
    )
    return error, np.linalg.solve(np.linalg.cholesky(result.joint_covariance), error)


def summarize(arrays: dict[str, Array]) -> dict[str, Any]:
    z = arrays["whitened"]
    axis, radius = arrays["axis_error_deg"], arrays["radius_99_deg"]
    return dict(
        trials=len(z),
        rejected=int(np.sum(arrays["rejected"])),
        rejection_fraction=float(np.mean(arrays["rejected"])),
        whitened_covariance_eigenvalues=np.linalg.eigvalsh(np.cov(z, rowvar=False)).tolist(),
        whitened_mean=np.mean(z, axis=0).tolist(),
        axis_error_deg_quantiles_50_95_99_max=np.quantile(axis, [0.5, 0.95, 0.99, 1]).tolist(),
        empirical_radius_coverage=float(np.mean(axis <= radius)),
        radius_deg_min_max=[float(radius.min()), float(radius.max())],
        maximum_mean_shift_rad=float(np.linalg.norm(arrays["mean_shift_B"], axis=1).max()),
        maximum_centered_mean_rad=float(np.linalg.norm(arrays["modeled_mean_B"], axis=1).max()),
    )


def population(
    partition: int, prior: dict[str, Array] | None = None
) -> tuple[dict[str, Any], dict[str, Array]]:
    n = int(base.allocation()["samples"])
    time_s = np.arange(n, dtype=np.float64) * base.DT
    fields = (
        "errors",
        "whitened",
        "covariance",
        "first_order_whitened",
        "first_order_errors",
        "first_order_covariance",
        "axis_error_deg",
        "radius_99_deg",
        "rejected",
        "statistics",
        "mean_shift_B",
        "modeled_mean_B",
        "mean_accel",
    )
    saved: dict[str, list[Any]] = {name: [] for name in fields}
    for trial in range(TRIALS):
        truth = (
            base.draw_trial(partition, trial, n)
            if prior is not None
            else draw_trial(partition, trial, n)
        )
        original = base.evaluate(time_s, truth["accel"], truth["gyro"], supported=True)
        old_error, old_z = whitened_error(original, truth)
        if prior is not None:
            for key, value in (
                ("errors", old_error),
                ("whitened", old_z),
                ("reported_covariances", original.joint_covariance),
                ("statistics", original.statistics),
            ):
                if not np.array_equal(value, prior["gaussian_" + key][trial]):
                    raise ValueError("original Gaussian history does not reconstruct exactly")
        s = truth["accel"].mean(axis=0)
        result, moments = candidate(s, original)
        error, z = whitened_error(result, truth)
        a, b = result.R_WB[:, 2], truth["R_WB"][:, 2]
        axis_error = float(np.rad2deg(np.arctan2(np.linalg.norm(np.cross(a, b)), a @ b)))
        values = (
            error,
            z,
            result.joint_covariance,
            old_z,
            old_error,
            original.joint_covariance,
            axis_error,
            result.radius_99_deg,
            float(bool(result.reasons)),
            result.statistics,
            moments.shift_B,
            moments.centered_mean_B,
            s,
        )
        for key, saved_value in zip(fields, values, strict=True):
            saved[key].append(saved_value)
    arrays = {k: np.array(v) for k, v in saved.items()}
    summary = summarize(arrays)
    summary["first_order_covariance_eigenvalues"] = np.linalg.eigvalsh(
        np.cov(arrays["first_order_whitened"], rowvar=False)
    ).tolist()
    return summary, arrays


def accuracy_reference() -> dict[str, Any]:
    records = []
    for roll, pitch in ((0, 0), (1, -1), (-2, 2), (5, -3)):
        for norm in (base.G - 0.06, base.G, base.G + 0.06):
            s = -norm * base.euler_rotation(np.deg2rad(roll), np.deg2rad(pitch))[2]
            a, b = nonlinear_moments(s, 201), nonlinear_moments(s, 201, order=7)
            shift = local_logs(a.q_WB, b.q_WB[None, :])[0]
            L = np.linalg.cholesky(b.joint_covariance)
            delta = a.joint_covariance - b.joint_covariance
            scaled = np.linalg.solve(L, np.linalg.solve(L, delta).T).T
            records.append(
                dict(
                    roll_deg=roll,
                    pitch_deg=pitch,
                    norm_m_s2=norm,
                    mean_difference_rad=float(np.linalg.norm(shift)),
                    covariance_relative_error=float(np.max(np.abs(np.linalg.eigvalsh(scaled)))),
                )
            )
    return dict(
        records=records,
        passed=all(
            x["mean_difference_rad"] <= 1e-8 and x["covariance_relative_error"] <= 0.001
            for x in records
        ),
    )


def authenticate_prior(path: Path) -> dict[str, Array]:
    for name, expected in (("report.json", PRIOR_REPORT), ("trials.npz", PRIOR_TRIALS)):
        if hashlib.sha256((path / name).read_bytes()).hexdigest() != expected:
            raise ValueError("prior evidence digest mismatch")
    with np.load(path / "trials.npz", allow_pickle=False) as values:
        return {k: values[k] for k in values.files}


def run(prior_path: Path, output: Path) -> dict[str, Any]:
    prior = authenticate_prior(prior_path)
    if output.exists():
        raise ValueError("new output directory required; preserve earlier evidence")
    output.mkdir(parents=True)
    accuracy = accuracy_reference()
    regression, regression_arrays = population(1, prior)
    nominal, nominal_arrays = population(0)
    gaussian, gaussian_arrays = population(1)
    checks = dict(
        numerical_integration=accuracy["passed"],
        original_regression=regression["whitened_covariance_eigenvalues"][-1] <= 1.10,
        fresh_gaussian_covariance=gaussian["whitened_covariance_eigenvalues"][-1] <= 1.10,
        fresh_gaussian_mean=max(abs(x) for x in gaussian["whitened_mean"]) <= 0.05,
        nominal_rejection=nominal["rejection_fraction"] <= 0.01,
        modeled_mean=max(x["maximum_centered_mean_rad"] for x in (regression, nominal, gaussian))
        <= 1e-13,
    )
    arrays: dict[str, Any] = {}
    for label, values in (
        ("regression", regression_arrays),
        ("nominal", nominal_arrays),
        ("gaussian", gaussian_arrays),
    ):
        arrays.update({label + "_" + k: v for k, v in values.items()})
    np.savez_compressed(output / "trials.npz", allow_pickle=False, **arrays)
    report = dict(
        design="ADR 0028: order-five local Gaussian pushforward, no flight integration",
        software=asdict(capture_software_provenance(ROOT)),
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(
            (ROOT / "docs/decisions/0028-nonlinear-prearm-uncertainty.md").read_bytes()
        ).hexdigest(),
        prior_report_sha256=PRIOR_REPORT,
        prior_trials_sha256=PRIOR_TRIALS,
        accuracy_reference=accuracy,
        regression=regression,
        nominal=nominal,
        gaussian=gaussian,
        acceptance=checks,
        standalone_implementation_go=all(checks.values()),
        flight_qualified=False,
    )
    (output / "report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prior-evidence", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.prior_evidence, args.output), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
