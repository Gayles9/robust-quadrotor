"""Fixed ADR 0028 numerical model, used only after externally supported acquisition."""

from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from numpy.typing import NDArray

from .eskf_endpoint import EskfSampledImuNoise, _array
from .rotations import rotation_matrix_body_to_world

Array = NDArray[np.float64]
PROFILE_ID = "adr0028-400hz-ned-frd-v1"
SAMPLE_PERIOD_S, SAMPLE_COUNT, GRAVITY = 0.0025, 201, 9.81
WINDOW_TIME_S, RELEASE_TIME_S, CLOCK_TOLERANCE_S = 0.5, 0.5025, 1e-12
ACCEL_SIGMA, GYRO_SIGMA = 0.04, 0.002
ACCEL_WALK_SIGMA, GYRO_WALK_SIGMA = 0.0002, 0.00002
ACCEL_BIAS_SIGMA, GYRO_BIAS_SIGMA, HEADING_SIGMA = 0.03, 0.005, float(np.deg2rad(3))


def prearm_imu_noise() -> EskfSampledImuNoise:
    """Return owned FRD/SI sample and bias-walk matrices for the approved profile."""
    return EskfSampledImuNoise(
        np.diag([ACCEL_SIGMA**2] * 3 + [GYRO_SIGMA**2] * 3),
        np.diag([ACCEL_WALK_SIGMA**2] * 3 + [GYRO_WALK_SIGMA**2] * 3),
    )


def _walk_factors() -> tuple[float, float, float]:
    n = SAMPLE_COUNT
    return (
        SAMPLE_PERIOD_S * (n - 1) * (2 * n - 1) / (6 * n),
        SAMPLE_PERIOD_S * (n - 1) / 2,
        SAMPLE_PERIOD_S * (n - 1),
    )


class _GravityDirectionError(ValueError):
    """The local gravity-direction chart cannot represent this input."""


def _gravity_quaternions(s: Array, heading: Array) -> Array:
    r2 = np.sum(s[:, 1:] ** 2, axis=1)
    norm2 = np.sum(s**2, axis=1)
    if np.any(norm2 < 1) or np.any(r2 < 0.25 * norm2):
        raise _GravityDirectionError("ill-conditioned gravity direction")
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


def _quaternion_product(left: Array, right: Array) -> Array:
    scalar = left[..., :1] * right[..., :1] - np.sum(
        left[..., 1:] * right[..., 1:], axis=-1, keepdims=True
    )
    vector = (
        left[..., :1] * right[..., 1:]
        + right[..., :1] * left[..., 1:]
        + np.cross(left[..., 1:], right[..., 1:])
    )
    return np.concatenate((scalar, vector), axis=-1)


def _quaternion_exp(vector: Array) -> Array:
    angle = float(np.linalg.norm(vector))
    return np.asarray(
        np.r_[np.cos(angle / 2), 0.5 * np.sinc(angle / (2 * np.pi)) * vector], dtype=np.float64
    )


def _local_logs(center_q_WB: Array, samples_q_WB: Array) -> Array:
    relative = _quaternion_product(center_q_WB * [1, -1, -1, -1], samples_q_WB)
    relative /= np.linalg.norm(relative, axis=-1, keepdims=True)
    relative = np.where(relative[..., :1] < 0, -relative, relative)
    v = relative[..., 1:]
    norm = np.linalg.norm(v, axis=-1)
    scale = np.full_like(norm, 2.0)
    np.divide(2 * np.arctan2(norm, relative[..., 0]), norm, out=scale, where=norm > 1e-15)
    return np.asarray(v * scale[..., None], dtype=np.float64)


@lru_cache(maxsize=1)
def _normal_rule() -> tuple[Array, Array]:
    x, w = np.polynomial.hermite.hermgauss(5)
    indices = np.stack(np.meshgrid(*([np.arange(5)] * 4), indexing="ij"), axis=-1).reshape(-1, 4)
    nodes = np.asarray(np.sqrt(2) * x[indices], dtype=np.float64)
    weights = np.asarray(np.prod(w[indices] / np.sqrt(np.pi), axis=1), dtype=np.float64)
    nodes.flags.writeable = weights.flags.writeable = False
    return nodes, weights


@dataclass(frozen=True, slots=True, eq=False)
class PrearmMoments:
    """Owned covariance about q_WB in [right-local theta, terminal ba, terminal bg]."""

    q_WB: Array
    joint_covariance: Array
    mean_shift_B: Array
    centered_mean_B: Array

    def __post_init__(self) -> None:
        for name, shape in (
            ("q_WB", (4,)),
            ("joint_covariance", (9, 9)),
            ("mean_shift_B", (3,)),
            ("centered_mean_B", (3,)),
        ):
            object.__setattr__(
                self,
                name,
                _array(name, getattr(self, name), shape, covariance=name == "joint_covariance"),
            )
        np.linalg.cholesky(self.joint_covariance)


def _moments(mean_specific_force_B: Array) -> PrearmMoments:
    """Approved 625-node local Gaussian pushforward; no physical-support inference."""
    s = _array("mean_specific_force_B", mean_specific_force_B, (3,))
    nodes, weights = _normal_rule()
    A, B, T = _walk_factors()
    Va = ACCEL_BIAS_SIGMA**2 + ACCEL_WALK_SIGMA**2 * A + ACCEL_SIGMA**2 / SAMPLE_COUNT
    C = ACCEL_BIAS_SIGMA**2 + ACCEL_WALK_SIGMA**2 * B
    Pa = ACCEL_BIAS_SIGMA**2 + ACCEL_WALK_SIGMA**2 * T
    gain, residual = C / Va, Pa - C * C / Va
    if residual <= 0:
        raise ValueError("positive conditional bias covariance required")
    eta = np.sqrt(Va) * nodes[:, :3]
    samples_q_WB = _gravity_quaternions(s - eta, HEADING_SIGMA * nodes[:, 3])
    initial_q_WB = _gravity_quaternions(s[None, :], np.zeros(1))[0]
    center_q_WB = initial_q_WB.copy()
    for corrections in range(9):
        delta = _local_logs(center_q_WB, samples_q_WB)
        mean = weights @ delta
        if np.linalg.norm(mean) <= 1e-13:
            break
        if corrections == 8:
            raise ValueError("rotation mean did not converge within eight corrections")
        center_q_WB = _quaternion_product(center_q_WB, _quaternion_exp(mean))
        center_q_WB /= np.linalg.norm(center_q_WB)
    centered = delta - mean
    bias_mean_nodes = gain * eta
    P = np.zeros((9, 9))
    P[:3, :3] = centered.T @ (weights[:, None] * centered)
    P[:3, 3:6] = centered.T @ (weights[:, None] * bias_mean_nodes)
    P[3:6, :3] = P[:3, 3:6].T
    P[3:6, 3:6] = bias_mean_nodes.T @ (weights[:, None] * bias_mean_nodes) + residual * np.eye(3)
    P[6:, 6:] = (GYRO_SIGMA**2 / SAMPLE_COUNT + GYRO_WALK_SIGMA**2 * A) * np.eye(3)
    P = (P + P.T) / 2
    return PrearmMoments(center_q_WB, P, _local_logs(initial_q_WB, center_q_WB[None, :])[0], mean)


@lru_cache(maxsize=1)
def _contrast_whiteners() -> tuple[Array, Array]:
    n = SAMPLE_COUNT
    Q = np.zeros((n, n - 1))
    for j in range(n - 1):
        Q[: j + 1, j] = 1 / np.sqrt((j + 1) * (j + 2))
        Q[j + 1, j] = -(j + 1) / np.sqrt((j + 1) * (j + 2))
    times = np.arange(n) * SAMPLE_PERIOD_S
    K = np.minimum.outer(times, times)
    result = []
    for sigma, walk in ((ACCEL_SIGMA, ACCEL_WALK_SIGMA), (GYRO_SIGMA, GYRO_WALK_SIGMA)):
        C = sigma**2 * np.eye(n - 1) + walk**2 * Q.T @ K @ Q
        W = np.linalg.solve(np.linalg.cholesky(C), Q.T)
        W.flags.writeable = False
        result.append(W)
    return result[0], result[1]


@dataclass(frozen=True, slots=True, eq=False)
class PrearmEstimate:
    """Window result with all diagnostics; readiness is a separate session state."""

    q_WB: Array
    accelerometer_bias_B: Array
    gyroscope_bias_B: Array
    joint_covariance: Array
    mean_shift_B: Array
    centered_mean_B: Array
    statistics: Array
    thresholds: Array
    radius_99_deg: float
    inclination_deg: float

    def __post_init__(self) -> None:
        for name, shape in (
            ("q_WB", (4,)),
            ("accelerometer_bias_B", (3,)),
            ("gyroscope_bias_B", (3,)),
            ("joint_covariance", (9, 9)),
            ("mean_shift_B", (3,)),
            ("centered_mean_B", (3,)),
            ("statistics", (3,)),
            ("thresholds", (3,)),
        ):
            object.__setattr__(
                self,
                name,
                _array(name, getattr(self, name), shape, covariance=name == "joint_covariance"),
            )


def _estimate_window(accel: Array, gyro: Array) -> PrearmEstimate:
    accel = _array("accelerometer window", accel, (SAMPLE_COUNT, 3))
    gyro = _array("gyroscope window", gyro, (SAMPLE_COUNT, 3))
    with np.errstate(over="raise", invalid="raise", divide="raise"):
        s, mean_g = accel.mean(axis=0), gyro.mean(axis=0)
        moments = _moments(s)
        A, _, _ = _walk_factors()
        Va = ACCEL_BIAS_SIGMA**2 + ACCEL_WALK_SIGMA**2 * A + ACCEL_SIGMA**2 / SAMPLE_COUNT
        Vg = GYRO_BIAS_SIGMA**2 + GYRO_WALK_SIGMA**2 * A + GYRO_SIGMA**2 / SAMPLE_COUNT
        wa, wg = _contrast_whiteners()
        statistics = np.array(
            [
                mean_g @ mean_g / Vg,
                (np.linalg.norm(s) - GRAVITY) ** 2 / Va,
                np.sum((wa @ accel) ** 2) + np.sum((wg @ gyro) ** 2),
            ]
        )
        d = np.array([3, 3, 6 * (SAMPLE_COUNT - 1)])
        thresholds = d + 2 * np.sqrt(d * (-np.log(0.001))) + 2 * (-np.log(0.001))
        P = moments.joint_covariance
        radius = float(np.rad2deg(np.sqrt(-2 * np.log(0.01) * np.linalg.eigvalsh(P[:2, :2])[-1])))
        # Retain the original gravity-map domain gate, before the mean correction.
        R = rotation_matrix_body_to_world(_gravity_quaternions(s[None, :], np.zeros(1))[0])
        inclination = float(np.rad2deg(np.arccos(np.clip(R[2, 2], -1, 1))))
    return PrearmEstimate(
        moments.q_WB,
        np.zeros(3),
        mean_g,
        P,
        moments.mean_shift_B,
        moments.centered_mean_B,
        statistics,
        thresholds,
        radius,
        inclination,
    )
