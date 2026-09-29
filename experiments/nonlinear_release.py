"""ADR0036 conditional Gaussian moments for the first release interval only."""

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, replace
from functools import lru_cache
from typing import Any
from unittest.mock import patch

import numpy as np
from numpy.typing import NDArray

from experiments import release_prediction
from experiments.prearm_nonlinear_uncertainty import local_logs, quaternion_exp, quaternion_product
from experiments.release_prediction import release_endpoint_map
from experiments.release_uncertainty import exp_batch
from experiments.robustness_evidence import ensure
from experiments.supported_start import PreparedStart, plain_configuration
from quadrotor_math.eskf_endpoint import (
    EskfEndpointState,
    EskfSampledImuNoise,
    _array,
    _nonnegative_scalar,
)
from quadrotor_math.prearm_uncertainty import Array
from quadrotor_math.rotations import rotation_matrix_body_to_world


@lru_cache(maxsize=14)
def rotation_rule(order: int, dimensions: int) -> tuple[Array, Array]:
    """Positive standard-normal tensor rule; five is candidate, seven is reference."""
    if type(order) is not int or order not in (5, 7):
        raise ValueError("frozen order five or seven required")
    if type(dimensions) is not int or not 0 <= dimensions <= 6:
        raise ValueError("zero to six rotation dimensions required")
    if dimensions == 0:
        nodes, weights = np.zeros((1, 0)), np.ones(1)
    else:
        x, w = np.polynomial.hermite.hermgauss(order)
        indices = np.stack(
            np.meshgrid(*([np.arange(order)] * dimensions), indexing="ij"), axis=-1
        ).reshape(-1, dimensions)
        nodes = np.sqrt(2) * x[indices]
        weights = np.prod(w[indices] / np.sqrt(np.pi), axis=1)
    nodes.flags.writeable = weights.flags.writeable = False
    return nodes, weights


def structural_factor(C: Array) -> tuple[Array, NDArray[np.intp]]:
    """Remove only exact zero rows; reject remaining singularity, never clip eigenvalues."""
    active = np.flatnonzero(np.diag(C) > 0)
    zero = np.flatnonzero(np.diag(C) == 0)
    ensure(np.all(np.diag(C) >= 0) and np.all(C[zero] == 0), "invalid deterministic covariance")
    factor = np.zeros((len(C), len(active)))
    if len(active):
        scale = np.sqrt(np.diag(C)[active])
        correlation = C[np.ix_(active, active)] / scale[:, None] / scale[None, :]
        try:
            factor[active] = scale[:, None] * np.linalg.cholesky(correlation)
        except np.linalg.LinAlgError:
            raise ValueError("unsupported non-structural covariance singularity") from None
    return factor, active


@dataclass(frozen=True)
class ConditionalInputs:
    rotation_selector: Array
    linear_selector: Array
    rotation_factor: Array
    gain: Array
    residual_covariance: Array


def conditional_inputs(
    current: EskfEndpointState, noise: EskfSampledImuNoise, h: float
) -> ConditionalInputs:
    """Reduce 33 original latents to six rotation and 18 affine output variables."""
    if not isinstance(current, EskfEndpointState) or not isinstance(noise, EskfSampledImuNoise):
        raise TypeError("typed endpoint and sampled noise required")
    h = _nonnegative_scalar("time_step_s", h, positive=True)
    covariance = np.zeros((33, 33))
    covariance[:21, :21] = current.joint_covariance
    covariance[21:27, 21:27] = noise.sample_covariance_B
    covariance[27:, 27:] = h * noise.bias_walk_spectral_density_B
    T = np.zeros((6, 33))
    T[:3, 6:9] = np.eye(3)
    T[3:, 12:15] = -h * np.eye(3)
    for start in (18, 24, 30):
        T[3:, start : start + 3] = -h / 2 * np.eye(3)
    U = np.zeros((18, 33))
    U[:6, :6] = np.eye(6)
    U[6:12, 9:15] = U[6:12, 27:33] = np.eye(6)
    U[12:18, 21:27] = np.eye(6)
    V, X, Z = T @ covariance @ T.T, U @ covariance @ T.T, U @ covariance @ U.T
    factor, active = structural_factor(V)
    gain = np.zeros((18, 6))
    if len(active):
        scale = np.sqrt(np.diag(V)[active])
        normalized_factor = factor[active] / scale[:, None]
        gain[:, active] = (
            np.linalg.solve(
                normalized_factor.T,
                np.linalg.solve(normalized_factor, X[:, active].T / scale[:, None]),
            )
            / scale[:, None]
        ).T
    residual = Z - gain @ X.T
    residual = (residual + residual.T) / 2
    structural_factor(residual)
    return ConditionalInputs(
        _array("rotation_selector", T, (6, 33)),
        _array("linear_selector", U, (18, 33)),
        _array("rotation_factor", factor, factor.shape),
        _array("conditional_gain", gain, (18, 6)),
        _array("conditional_residual", residual, (18, 18), covariance=True),
    )


def rotation_matrices(q_WB: Array) -> Array:
    """Batched scalar-first unit quaternion rotations for quadrature only."""
    w, x, y, z = q_WB.T
    return np.stack(
        (
            1 - 2 * (y * y + z * z),
            2 * (x * y - z * w),
            2 * (x * z + y * w),
            2 * (x * y + z * w),
            1 - 2 * (x * x + z * z),
            2 * (y * z - x * w),
            2 * (x * z - y * w),
            2 * (y * z + x * w),
            1 - 2 * (x * x + y * y),
        ),
        axis=1,
    ).reshape(-1, 3, 3)


def conditional_output_covariance(
    residual: Array, matrices: Array, weights: Array, F: Array, G: Array, E: Array
) -> Array:
    """Exactly rearranged positive sum of (F-G Ri E) D (F-G Ri E)^T."""
    mean_R = np.einsum("n,nij->ij", weights, matrices)
    force_covariance = E @ residual @ E.T
    rotated = np.einsum("nij,jk,nlk->nil", matrices, force_covariance, matrices)
    mean_rotated = np.einsum("n,nij->ij", weights, rotated)
    cross = F @ residual @ E.T
    first = G @ mean_R @ cross.T
    return np.asarray(
        F @ residual @ F.T - first - first.T + G @ mean_rotated @ G.T, dtype=np.float64
    )


def predict_nonlinear_release_endpoint(
    current: EskfEndpointState,
    specific_force_start_B: Array,
    angular_velocity_start_B: Array,
    specific_force_end_B: Array,
    angular_velocity_end_B: Array,
    gravity_acceleration: float,
    noise: EskfSampledImuNoise,
    time_step_s: float,
    *,
    order: int = 5,
) -> EskfEndpointState:
    """Moment-match the complete nonlinear first-interval conditional Gaussian map.

    No velocity constraint is imposed here. The returned mean/covariance retain
    bias and fresh-noise correlations; all later predictions remain ordinary ESKF.
    """
    if not isinstance(current, EskfEndpointState) or not isinstance(noise, EskfSampledImuNoise):
        raise TypeError("typed endpoint and sampled noise required")
    nominal, _, _ = release_endpoint_map(
        current.nominal_state,
        specific_force_start_B,
        angular_velocity_start_B,
        specific_force_end_B,
        angular_velocity_end_B,
        current.imu_noise_mean_B,
        gravity_acceleration,
        time_step_s,
    )
    h = float(time_step_s)
    model = conditional_inputs(current, noise, h)
    nodes, weights = rotation_rule(order, model.rotation_factor.shape[1])
    state = current.nominal_state
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            eta = nodes @ model.rotation_factor.T
            Z = eta @ model.gain.T
            phi = (
                h
                / 2
                * (
                    angular_velocity_start_B
                    + angular_velocity_end_B
                    - 2 * state.gyroscope_bias_B
                    - current.imu_noise_mean_B[3:]
                )
            )
            samples_q_WB = quaternion_product(
                quaternion_product(state.q_WB, exp_batch(eta[:, :3])),
                exp_batch(phi + eta[:, 3:]),
            )
            samples_q_WB /= np.linalg.norm(samples_q_WB, axis=1, keepdims=True)
            center_q_WB = nominal.q_WB.copy()
            for iteration in range(9):
                theta = local_logs(center_q_WB, samples_q_WB)
                mean_theta = weights @ theta
                if np.linalg.norm(mean_theta) <= 1e-13:
                    break
                ensure(iteration < 8, "release rotation mean did not converge")
                center_q_WB = quaternion_product(center_q_WB, quaternion_exp(mean_theta))
                center_q_WB /= np.linalg.norm(center_q_WB)
            theta -= mean_theta
            matrices = rotation_matrices(samples_q_WB)
            F = np.eye(18)
            F[:3, 3:6] = h * np.eye(3)
            G = np.zeros((18, 3))
            G[:3] = h * h / 2 * np.eye(3)
            G[3:6] = h * np.eye(3)
            E = np.zeros((3, 18))
            E[:, 6:9] = E[:, 12:15] = np.eye(3)
            force = specific_force_end_B - state.accelerometer_bias_B
            conditional_force = force - Z @ E.T
            acceleration_difference = (
                np.einsum("nij,nj->ni", matrices, conditional_force)
                - rotation_matrix_body_to_world(nominal.q_WB) @ force
            )
            Y = Z @ F.T + acceleration_difference @ G.T
            mean_Y = weights @ Y
            # These means are exactly zero by the Gaussian input construction.
            mean_Y[6:] = 0
            Y -= mean_Y
            YY = Y.T @ (weights[:, None] * Y) + conditional_output_covariance(
                model.residual_covariance, matrices, weights, F, G, E
            )
            Ytheta = Y.T @ (weights[:, None] * theta)
            C = np.zeros((21, 21))
            indices = np.r_[0:6, 9:21]
            C[np.ix_(indices, indices)] = YY
            C[6:9, 6:9] = theta.T @ (weights[:, None] * theta)
            C[indices, 6:9], C[6:9, indices] = Ytheta, Ytheta.T
            endpoint_mean = replace(
                nominal,
                position_W=nominal.position_W + mean_Y[:3],
                velocity_W=nominal.velocity_W + mean_Y[3:6],
                q_WB=center_q_WB,
            )
    except (FloatingPointError, OverflowError):
        raise ValueError("nonlinear release moments must remain finite") from None
    return EskfEndpointState(endpoint_mean, np.zeros(6), (C + C.T) / 2)


@contextmanager
def nonlinear_release_prediction(prepared: PreparedStart, target: str) -> Iterator[dict[str, Any]]:
    """Opt in for one flight/replay; keep the original prior and first-only checks."""
    adapter = release_prediction.ReleasePrediction(prepared)
    trace: dict[str, Any] = {}

    def predict(*args: Any) -> EskfEndpointState:
        ensure(not trace, "nonlinear release prediction already consumed")
        result = predict_nonlinear_release_endpoint(*args)
        trace.update(
            input=plain_configuration(args[0]),
            output=plain_configuration(result),
            order=5,
            zero_velocity_conditioning=False,
        )
        return result

    with patch.object(release_prediction, "predict_release_endpoint", predict):
        with adapter.installed(target) as routing:
            yield trace
    trace["routing"] = dict(routing)
