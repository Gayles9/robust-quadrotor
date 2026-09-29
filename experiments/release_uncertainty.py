"""Independent finite perturbations and nonlinear uncertainty checks for ADR0035."""

from typing import Any

import numpy as np

from experiments.prearm_nonlinear_uncertainty import local_logs, quaternion_product
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.prearm_uncertainty import Array


def exp_batch(vectors: Array) -> Array:
    angle = np.linalg.norm(vectors, axis=1, keepdims=True)
    return np.concatenate((np.cos(angle / 2), 0.5 * np.sinc(angle / (2 * np.pi)) * vectors), axis=1)


def rotate_batch(q_WB: Array, vectors: Array) -> Array:
    twice = 2 * np.cross(q_WB[:, 1:], vectors)
    return vectors + q_WB[:, :1] * twice + np.cross(q_WB[:, 1:], twice)


def finite_errors(
    state: EskfNominalState,
    w0: Array,
    f1: Array,
    w1: Array,
    mean: Array,
    g: float,
    h: float,
    perturbations: Array,
) -> Array:
    """Independent batched exact map differences for all 33 latent inputs.

    Uses quaternion-vector products, not candidate matrices or its Jacobians.
    Noise variables represent actual minus conditional-mean sensor error.
    Returns [right-local physical error, new sample noise] around its own nominal.
    """
    d = np.vstack((np.zeros((1, 33)), perturbations))
    start_q_WB = quaternion_product(state.q_WB, exp_batch(d[:, 6:9]))
    ba = state.accelerometer_bias_B + d[:, 9:12]
    bg = state.gyroscope_bias_B + d[:, 12:15]
    old_rate = w0 - bg - mean[3:] - d[:, 18:21]
    new_rate = w1 - bg - d[:, 24:27] - d[:, 30:33]
    q_WB = quaternion_product(start_q_WB, exp_batch(h * (old_rate + new_rate) / 2))
    force = f1 - ba - d[:, 21:24] - d[:, 27:30]
    acceleration = rotate_batch(q_WB, force) + [0, 0, g]
    velocity = state.velocity_W + d[:, 3:6] + h * acceleration
    position = (
        state.position_W + d[:, :3] + h * (state.velocity_W + d[:, 3:6]) + h * h / 2 * acceleration
    )
    out = np.c_[
        position - position[0],
        velocity - velocity[0],
        local_logs(q_WB[0], q_WB),
        d[:, 9:15] + d[:, 27:33],
        d[:, 21:27],
    ]
    return np.asarray(out[1:], dtype=np.float64)


def numerical_derivative(
    state: EskfNominalState, w0: Array, f1: Array, w1: Array, mean: Array, g: float, h: float
) -> Array:
    step = 1e-6
    perturbations = np.vstack((np.eye(33), -np.eye(33))) * step
    values = finite_errors(state, w0, f1, w1, mean, g, h, perturbations)
    return np.asarray(((values[:33] - values[33:]) / (2 * step)).T, dtype=np.float64)


def yaw_noise_counterexample() -> dict[str, Any]:
    """Exact missing tangent-normal variance for independent Gaussian yaw/noise.

    With fixed zero initial velocity and mean f1=0, v=-h*Rz(psi)*[z,0,0].
    The linear map claims r=[vx+h*z,vy] has zero covariance. Both nonlinear
    variances are positive; this calculation requires no empirical trial fit.
    """
    h, sigma_yaw, sigma_force = 0.0025, np.deg2rad(3.0), 0.035
    s2 = sigma_yaw**2
    exact = (
        h
        * h
        * sigma_force**2
        * np.array([1.5 - 2 * np.exp(-s2 / 2) + 0.5 * np.exp(-2 * s2), (1 - np.exp(-2 * s2)) / 2])
    )
    x, w = np.polynomial.hermite.hermgauss(9)
    yaw = np.sqrt(2) * sigma_yaw * x
    quadrature = (
        h
        * h
        * sigma_force**2
        * np.array([w @ (1 - np.cos(yaw)) ** 2, w @ np.sin(yaw) ** 2])
        / np.sqrt(np.pi)
    )
    return dict(
        h_s=h,
        yaw_sigma_rad=float(sigma_yaw),
        force_sigma_m_s2=sigma_force,
        linear_variance=[0.0, 0.0],
        exact_variance=exact.tolist(),
        quadrature_variance=quadrature.tolist(),
        missing_standard_deviation_m_s=np.sqrt(exact).tolist(),
        maximum_quadrature_error=float(np.max(np.abs(exact - quadrature))),
        exact_zero_variance_claim_valid=bool(np.all(exact == 0)),
    )
