"""ADR0040 conditional measurement algebra only; no sensor or ESKF integration."""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import source_sha256
from experiments.durable_release_evidence import save_arrays, save_json
from experiments.early_flight_diagnostic import authenticated
from experiments.robustness_evidence import ensure
from experiments.whole_flight_error_budget import information_structure
from quadrotor_math.rotations import rotation_matrix_body_to_world, skew_symmetric

Array = NDArray[np.float64]
ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0040-independent-inclination-feasibility.md"
PRECEDING_SOURCE = "d83889b7189ab898aadb87bfa258259ec30e381b14f0d550d207343835216f95"
INPUT_SHA = "b93b1fb050bab01b06b790b9bac7d795469cb99942572ea476ecfe47775798cb"
DOWN = np.array([0.0, 0.0, 1.0])


def rotation(phi: Array) -> Array:
    """Finite rotation for algebraic fixtures, in the declared axis coordinates."""
    ensure(phi.shape == (3,) and np.all(np.isfinite(phi)), "finite rotation vector")
    angle = float(np.linalg.norm(phi))
    q_WB = np.r_[np.cos(angle / 2), 0.5 * np.sinc(angle / (2 * np.pi)) * phi]
    return rotation_matrix_body_to_world(q_WB)


def validate_rotation(R: Array) -> None:
    ensure(R.shape == (3, 3) and np.all(np.isfinite(R)), "rotation shape and finiteness")
    ensure(
        np.max(np.abs(R.T @ R - np.eye(3))) <= 1e-12 and abs(np.linalg.det(R) - 1) <= 1e-12,
        "proper rotation",
    )


def validate_direction(direction: Array) -> None:
    ensure(
        direction.shape == (3,) and np.all(np.isfinite(direction)), "direction shape and finiteness"
    )
    ensure(abs(float(direction @ direction) - 1) <= 1e-12, "unit direction")


def registered_direction(R_WE: Array, R_EC: Array, R_CB: Array) -> Array:
    """External target C in reference E, body mounting B->C and E->world registration."""
    for R in (R_WE, R_EC, R_CB):
        validate_rotation(R)
    return np.asarray(R_WE @ R_EC @ R_CB @ DOWN, dtype=np.float64)


def tangent_basis(direction_W: Array) -> Array:
    """Deterministic local chart; never differentiated through an axis-selection switch."""
    validate_direction(direction_W)
    axis = np.eye(3)[int(np.argmin(np.abs(direction_W)))]
    first = np.cross(direction_W, axis)
    first /= np.linalg.norm(first)
    return np.column_stack((first, np.cross(direction_W, first)))


def local_model(R_WB: Array, observed_down_W: Array, basis_W: Array) -> tuple[Array, Array, Array]:
    """Chord residual, right-local 21-state H and world-angular-noise J.

    This is a derivation helper, not an estimator correction. Its chart is fixed
    at the nominal direction. Hemisphere validation prevents an antipodal zero
    residual; it does not establish a small-error Gaussian likelihood.
    """
    validate_rotation(R_WB)
    validate_direction(observed_down_W)
    predicted = R_WB @ DOWN
    ensure(basis_W.shape == (3, 2) and np.all(np.isfinite(basis_W)), "tangent basis shape")
    ensure(
        np.max(np.abs(basis_W.T @ basis_W - np.eye(2))) <= 1e-12
        and np.max(np.abs(basis_W.T @ predicted)) <= 1e-12,
        "orthonormal tangent basis",
    )
    ensure(float(predicted @ observed_down_W) > 0, "local hemisphere")
    H = np.zeros((2, 21))
    H[:, 6:9] = -basis_W.T @ R_WB @ skew_symmetric(DOWN)
    J = -basis_W.T @ skew_symmetric(predicted)
    return basis_W.T @ (observed_down_W - predicted), H, J


def covariance(matrix: Array) -> None:
    """Finite symmetric PSD joint covariance, with scale-relative roundoff allowance."""
    ensure(matrix.ndim == 2 and matrix.shape[0] == matrix.shape[1] > 0, "covariance shape")
    ensure(np.all(np.isfinite(matrix)), "finite covariance")
    scale = float(np.max(np.abs(matrix)))
    tolerance = 64 * len(matrix) * np.finfo(float).eps * scale
    ensure(np.max(np.abs(matrix - matrix.T)) <= tolerance, "symmetric covariance")
    ensure(np.all(np.diag(matrix) >= 0), "nonnegative variance")
    ensure(float(np.linalg.eigvalsh((matrix + matrix.T) / 2).min()) >= -tolerance, "PSD covariance")


def condition_joint(
    P: Array,
    H: Array,
    R: Array,
    residual: Array,
    U: Array | None = None,
) -> tuple[Array, Array, float]:
    """Linear Gaussian algebra with U=Cov(state error, measurement noise).

    The returned covariance is in the original linear coordinates, before any
    attitude injection/reset. No online endpoint, state, gate or sample is changed.
    """
    covariance(P)
    covariance(R)
    n, m = len(P), len(R)
    ensure(H.shape == (m, n) and residual.shape == (m,), "conditioning dimensions")
    U = np.zeros((n, m)) if U is None else U
    ensure(U.shape == (n, m), "state-noise covariance shape")
    ensure(all(np.all(np.isfinite(x)) for x in (H, residual, U)), "finite conditioning inputs")
    covariance(np.block([[P, U], [U.T, R]]))
    S = H @ P @ H.T + H @ U + U.T @ H.T + R
    ensure(float(np.linalg.eigvalsh((S + S.T) / 2).min()) > 0, "positive innovation covariance")
    cross = P @ H.T + U
    gain = np.linalg.solve(S, cross.T).T
    posterior = P - gain @ cross.T
    posterior = (posterior + posterior.T) / 2
    covariance(posterior)
    return gain @ residual, posterior, float(residual @ np.linalg.solve(S, residual))


def condition_decorrelated(
    P: Array, H: Array, R: Array, residual: Array, split: int
) -> tuple[Array, Array]:
    """Correct sequential equivalence when measurement noise is independent of state.

    Conditional rows remove cross-sensor noise covariance. They must not be
    reused for state-correlated measurements, nor with an intervening rotation
    reset without transforming every remaining row/cross covariance.
    """
    ensure(type(split) is int and 0 < split < len(R), "nonempty observation blocks")
    condition_joint(P, H, R, residual)  # Validate the complete covariance before splitting.
    Rpp, Rtp = R[:split, :split], R[split:, :split]
    ensure(float(np.linalg.eigvalsh(Rpp).min()) > 0, "invertible first noise block")
    L = np.linalg.solve(Rpp, Rtp.T).T
    Ht = H[split:] - L @ H[:split]
    Rt = R[split:, split:] - L @ Rtp.T
    mean, posterior, _ = condition_joint(P, H[:split], Rpp, residual[:split])
    delta, posterior, _ = condition_joint(
        posterior, Ht, Rt, residual[split:] - L @ residual[:split] - Ht @ mean
    )
    return mean + delta, posterior


def age_angle_bound(maximum_true_rate_rad_s: float, age_s: float) -> float:
    """Geodesic direction error bound; the supplied true-rate bound must be justified."""
    ensure(
        all(np.isfinite(x) and x >= 0 for x in (maximum_true_rate_rad_s, age_s)),
        "nonnegative finite rate and age",
    )
    return min(float(np.pi), maximum_true_rate_rad_s * age_s)


def gaussian_sequence_radius(
    sigma_max_rad: float, samples: int, failure_probability: float
) -> float:
    """Two-dimensional Gaussian union bound; temporal independence is unnecessary.

    Each zero-mean tangent marginal must have covariance <= sigma_max² I.
    Bias, calibration, nonlinear projection and estimator error are not covered.
    """
    ensure(np.isfinite(sigma_max_rad) and sigma_max_rad >= 0, "finite nonnegative angular sigma")
    ensure(type(samples) is int and samples > 0, "positive sample count")
    ensure(np.isfinite(failure_probability) and 0 < failure_probability < 1, "failure probability")
    return float(sigma_max_rad * np.sqrt(2 * (np.log(samples) - np.log(failure_probability))))


def bias_information(gravity: float) -> dict[str, Any]:
    """Two unknown constant inclination offsets restore the absolute tilt ambiguity."""
    previous = information_structure(gravity)
    F = np.zeros((17, 17))
    F[:15, :15] = previous["F"]
    H = np.zeros((5, 17))
    H[:3, :3], H[3:, 6:8], H[3:, 15:] = np.eye(3), np.eye(2), np.eye(2)
    observability = np.vstack([H @ np.linalg.matrix_power(F, k) for k in range(17)])
    null = np.zeros((17, 4))
    null[:15] = previous["null_vectors"]
    null[15:, :2] = -np.eye(2)
    ensure(np.max(np.abs(observability @ null)) <= 1e-12, "unknown-bias nullspace")
    return dict(
        calibrated_state_dimension=15,
        calibrated_rank=previous["cases"]["added_inclination"]["rank"],
        unknown_bias_state_dimension=17,
        unknown_bias_rank=int(np.linalg.matrix_rank(observability)),
        F=F.tolist(),
        H=H.tolist(),
        null_vectors=null.tolist(),
        null_residual=float(np.max(np.abs(observability @ null))),
    )


def preceding_source_sha256() -> str:
    """Bind all prior execution source, excluding exactly this new derivation module."""
    digest = hashlib.sha256()
    for path in sorted(
        [
            *ROOT.glob("src/quadrotor_math/*.py"),
            *ROOT.glob("experiments/*.py"),
            ROOT / "pyproject.toml",
            ROOT / "uv.lock",
        ]
    ):
        if path == Path(__file__).resolve():
            continue
        data = path.read_bytes()
        digest.update(path.relative_to(ROOT).as_posix().encode() + b"\0")
        digest.update(str(len(data)).encode() + b"\0" + data)
    return digest.hexdigest()


def run(budget_report: Path, output: Path) -> dict[str, Any]:
    ensure(preceding_source_sha256() == PRECEDING_SOURCE, "preceding execution identity")
    budget = json.loads(authenticated(budget_report, INPUT_SHA))
    output.mkdir(parents=True, exist_ok=False)
    protocol = dict(
        audited_commit="0129d658e905a644f06da1c51731ffe85c2655a9",
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        input_report_sha256=INPUT_SHA,
        new_scientific_flights=0,
    )
    save_json(output, "protocol.json", protocol)
    matrices, rows = {}, []
    step = 1e-6
    poses = ((0.0, 0.0, 0.0), (0.2, -0.3, 0.4), (-0.7, 0.8, -1.1), (1.2, -0.4, 2.0))
    for index, phi in enumerate(poses):
        R_WB = rotation(np.array(phi))
        direction = R_WB @ DOWN
        E = tangent_basis(direction)
        _, H, J = local_model(R_WB, direction, E)
        right_columns, world_columns = [], []
        for axis in np.eye(3):
            right_columns.append(
                E.T
                @ (R_WB @ rotation(step * axis) @ DOWN - R_WB @ rotation(-step * axis) @ DOWN)
                / (2 * step)
            )
            world_columns.append(
                E.T
                @ (rotation(step * axis) @ direction - rotation(-step * axis) @ direction)
                / (2 * step)
            )
        right_fd, world_fd = np.array(right_columns).T, np.array(world_columns).T
        maximum = max(
            float(np.max(np.abs(right_fd - H[:, 6:9]))), float(np.max(np.abs(world_fd - J)))
        )
        ensure(maximum <= 1e-8, "finite-difference Jacobians")
        twist = R_WB @ rotation(0.7 * DOWN) @ DOWN - direction
        ensure(np.max(np.abs(twist)) <= 1e-12, "body-axis twist invariant")
        rows.append(
            dict(
                pose=phi,
                jacobian_max_error=maximum,
                right_twist_error=float(np.linalg.norm(twist)),
                world_yaw_direction_change=float(
                    np.linalg.norm(rotation(0.7 * DOWN) @ direction - direction)
                ),
            )
        )
        for key, value in dict(
            R_WB=R_WB,
            direction_W=direction,
            basis_W=E,
            H=H,
            J=J,
            right_fd=right_fd,
            world_fd=world_fd,
        ).items():
            matrices[f"pose{index}_{key}"] = value
    # Fixed dimensionless linear fixture, not a proposed physical sensor covariance.
    P = np.array([[1.0, 0.2], [0.2, 0.7]])
    H = np.array([[1.0, 0.3], [-0.2, 1.0]])
    R = np.array([[0.4, 0.18], [0.18, 0.3]])
    residual = np.array([0.25, -0.1])
    mean, posterior, nis = condition_joint(P, H, R, residual)
    seq_mean, seq_cov = condition_decorrelated(P, H, R, residual, 1)
    difference = max(
        float(np.max(np.abs(mean - seq_mean))), float(np.max(np.abs(posterior - seq_cov)))
    )
    ensure(difference <= 1e-12, "decorrelated sequential identity")
    wrong_mean, wrong_cov, _ = condition_joint(P, H, np.diag(np.diag(R)), residual)
    U = np.array([[0.05, -0.02], [0.01, 0.04]])
    dependent_mean, dependent_cov, dependent_nis = condition_joint(P, H, R, residual, U)
    matrices.update(
        P=P,
        H=H,
        R=R,
        U=U,
        residual=residual,
        mean=mean,
        posterior=posterior,
        sequential_mean=seq_mean,
        sequential_covariance=seq_cov,
        falsely_independent_mean=wrong_mean,
        falsely_independent_covariance=wrong_cov,
        state_correlated_mean=dependent_mean,
        state_correlated_covariance=dependent_cov,
    )
    save_arrays(output / "algebra-fixtures.npz", matrices)
    coefficients = budget["sensitivity"]["finite_hover_bounds"]
    missing = [
        "justified_independent_source_or_simulation_model",
        "frame_and_mounting_calibration",
        "angular_bias_and_noise_bounds",
        "acquisition_latency_and_rate",
        "outage_policy",
        "position_orientation_and_state_noise_correlations",
        "posterior_joint_error_bounds",
        "vertical_initial_and_physical_residual_allowance",
    ]
    result = dict(
        protocol=protocol,
        geometry=rows,
        information=bias_information(9.81),
        conditioning=dict(
            sequential_max_difference=difference,
            nis=nis,
            state_correlated_nis=dependent_nis,
            false_independence_mean_difference=float(np.linalg.norm(wrong_mean - mean)),
            false_independence_covariance_difference=float(np.linalg.norm(wrong_cov - posterior)),
        ),
        screening=dict(
            coefficients=coefficients,
            single_input_angle_ceiling_rad=0.08 / coefficients[2],
            illustrative_age_at_1_rad_s_s=0.08 / coefficients[2],
            unit_sigma_1000_sample_1percent_radius=gaussian_sequence_radius(1.0, 1000, 0.01),
            conditional_two_dof_99percent_nis=float(-2 * np.log(0.01)),
            simultaneous_allocation=None,
        ),
        source_inventory=dict(
            existing=["IMU", "local_position", "barometric_altitude"],
            independent_orientation_model=None,
            calibrated_external_data=None,
            simulator_truth_use="scoring and diagnostic oracles only",
            vision_and_hardware="deferred scope",
        ),
        summary=dict(
            mathematical_contract_complete=True,
            component_go=False,
            scientific_flights=0,
            synthetic_sensor_campaigns=0,
            production_changes=False,
            flight_qualification=False,
            missing_requirements=missing,
            decision="Close extension feasibility; no supported source or joint allocation.",
        ),
    )
    ensure(source_sha256(ROOT) == protocol["source_sha256"], "execution source changed")
    save_json(output, "report.json", result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--budget-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.budget_report, args.output)["summary"], indent=2))


if __name__ == "__main__":
    main()
