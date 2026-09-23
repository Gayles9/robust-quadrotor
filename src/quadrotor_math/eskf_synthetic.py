"""Independent analytic motion and seeded measurements for estimator verification.

These kinematic fixtures do not use ESKF propagation to generate truth and do not
claim rotor feasibility. SI units, NED world, FRD body, scalar-first q_WB. See ADR
0009 for the fixed distributions, noise model and known-prior experiment boundary.
"""

from dataclasses import dataclass
from math import pi

import numpy as np
from numpy.typing import NDArray

from .eskf import EskfNominalState, inject_eskf_error_state
from .eskf_consistency import EskfReferenceHistory, sampled_imu_continuous_noise_covariance
from .eskf_innovation import EskfInnovationPolicy
from .eskf_replay import (
    EskfObservationKind,
    EskfReplayConfiguration,
    EskfReplayInput,
    EskfReplayObservation,
    _owned_array,
    _time_vector,
)
from .rotations import rotation_matrix_body_to_world
from .run_configuration import ImuParameters

MOTIONS = ("stationary", "translating_yaw", "excited")
PRIOR_STD = (0.2, 0.1, 0.015, 0.08, 0.008)
GRAVITY = 9.81
ACCEL_STD, GYRO_STD, POSITION_STD, ALTITUDE_STD = 0.08, 0.004, 0.10, 0.08
ACCEL_WALK, GYRO_WALK = 0.0005, 0.00005
ALTITUDE_DATUM = 100.0


def synthetic_eskf_rng(seed: int, stream_id: int) -> np.random.Generator:
    """Independent PCG64 stream; stable identity never depends on call ordering."""
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError("seed must be a non-Boolean integer in [0,2**32)")
    if type(stream_id) is not int or not 0 <= stream_id <= 9:
        raise ValueError("stream_id must be an integer in [0,9]")
    return np.random.Generator(
        np.random.PCG64(np.random.SeedSequence([0x45534B43, 1, stream_id, seed], pool_size=4))
    )


@dataclass(frozen=True, slots=True, eq=False)
class EskfAnalyticMotion:
    """p=A*sin(w*t+phase)+v*t; Euler angles=B*sin(nu*t+phase)+rate*t.

    Euler order is roll, pitch, yaw and R_WB=Rz(yaw) Ry(pitch) Rx(roll).
    Translation vectors are NED; angles are radians and rates radians/second.
    Arrays have shape (3,), are finite, copied and read-only. This is a forward
    analytic parameterization; no inverse Euler-rate singularity is evaluated.
    """

    position_amplitude_W: NDArray[np.float64]
    position_frequency: NDArray[np.float64]
    position_phase: NDArray[np.float64]
    velocity_W: NDArray[np.float64]
    euler_amplitude: NDArray[np.float64]
    euler_frequency: NDArray[np.float64]
    euler_phase: NDArray[np.float64]
    euler_rate: NDArray[np.float64]

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, _owned_array(name, getattr(self, name), (3,)))


def randomized_eskf_motion(seed: int, family: str) -> EskfAnalyticMotion:
    """Declare motion independently of all prior, sensor and fault streams."""
    rng = synthetic_eskf_rng(seed, 0)
    if family not in MOTIONS:
        raise ValueError(f"unknown motion family: {family}")
    if family == "excited":
        return EskfAnalyticMotion(
            rng.uniform(1.0, 2.0, 3),
            rng.uniform(0.5, 1.0, 3),
            rng.uniform(-pi, pi, 3),
            np.zeros(3),
            rng.uniform([0.15, 0.15, 0.3], [0.3, 0.3, 0.6]),
            rng.uniform(0.4, 0.8, 3),
            rng.uniform(-pi, pi, 3),
            np.zeros(3),
        )
    return EskfAnalyticMotion(
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.array([0.5, -0.25, 0.1]) if family == "translating_yaw" else np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.array([0.0, 0.0, 0.2]) if family == "translating_yaw" else np.zeros(3),
    )


@dataclass(frozen=True, slots=True, eq=False)
class EskfKinematicHistory:
    """Exact continuous kinematics evaluated at explicit epochs, with owned arrays."""

    time_s: NDArray[np.float64]
    position_W: NDArray[np.float64]
    velocity_W: NDArray[np.float64]
    acceleration_W: NDArray[np.float64]
    q_WB: NDArray[np.float64]
    angular_velocity_B: NDArray[np.float64]
    specific_force_B: NDArray[np.float64]

    def __post_init__(self) -> None:
        times = _time_vector(self.time_s)
        object.__setattr__(self, "time_s", times)
        for name in self.__dataclass_fields__:
            if name != "time_s":
                width = 4 if name == "q_WB" else 3
                object.__setattr__(
                    self, name, _owned_array(name, getattr(self, name), (len(times), width))
                )
        for quaternion in self.q_WB:
            rotation_matrix_body_to_world(quaternion)


def sample_eskf_analytic_motion(
    motion: EskfAnalyticMotion, time_s: NDArray[np.float64]
) -> EskfKinematicHistory:
    """Analytically differentiate position and Rz Ry Rx, then form f_B=R_WB.T(a_W-g_W).

    Body angular velocity is [roll_dot-yaw_dot*sin(pitch),
    pitch_dot*cos(roll)+yaw_dot*sin(roll)*cos(pitch),
    -pitch_dot*sin(roll)+yaw_dot*cos(roll)*cos(pitch)]. No ESKF step generates truth.
    """
    if not isinstance(motion, EskfAnalyticMotion):
        raise TypeError("motion must be an EskfAnalyticMotion")
    times = _time_vector(time_s)
    try:
        with np.errstate(over="raise", invalid="raise"):
            phase = times[:, None] * motion.position_frequency + motion.position_phase
            p_W = motion.position_amplitude_W * np.sin(phase) + times[:, None] * motion.velocity_W
            v_W = (
                motion.position_amplitude_W * motion.position_frequency * np.cos(phase)
                + motion.velocity_W
            )
            a_W = -motion.position_amplitude_W * motion.position_frequency**2 * np.sin(phase)
            phase_euler = times[:, None] * motion.euler_frequency + motion.euler_phase
            angles = (
                motion.euler_amplitude * np.sin(phase_euler) + times[:, None] * motion.euler_rate
            )
            rates = (
                motion.euler_amplitude * motion.euler_frequency * np.cos(phase_euler)
                + motion.euler_rate
            )
            roll, pitch, yaw = angles.T
            cr, cp, cy = np.cos(angles / 2).T
            sr, sp, sy = np.sin(angles / 2).T
            q_WB = np.column_stack(
                (
                    cr * cp * cy + sr * sp * sy,
                    sr * cp * cy - cr * sp * sy,
                    cr * sp * cy + sr * cp * sy,
                    cr * cp * sy - sr * sp * cy,
                )
            )
            roll_dot, pitch_dot, yaw_dot = rates.T
            omega_B = np.column_stack(
                (
                    roll_dot - yaw_dot * np.sin(pitch),
                    pitch_dot * np.cos(roll) + yaw_dot * np.sin(roll) * np.cos(pitch),
                    -pitch_dot * np.sin(roll) + yaw_dot * np.cos(roll) * np.cos(pitch),
                )
            )
            gravity_W = np.array([0.0, 0.0, GRAVITY])
            f_B = np.array(
                [
                    rotation_matrix_body_to_world(quaternion).T @ (acceleration - gravity_W)
                    for quaternion, acceleration in zip(q_WB, a_W, strict=True)
                ]
            )
    except FloatingPointError:
        raise ValueError("analytic kinematics must remain finite") from None
    return EskfKinematicHistory(times, p_W, v_W, a_W, q_WB, omega_B, f_B)


@dataclass(frozen=True, slots=True, eq=False)
class EskfSyntheticCase:
    """Separate measured input, known prior, evaluation-only truth and motion metadata."""

    measurements: EskfReplayInput
    configuration: EskfReplayConfiguration
    reference: EskfReferenceHistory
    motion: EskfAnalyticMotion


def make_eskf_synthetic_case(
    seed: int,
    *,
    family: str = "excited",
    number_of_steps: int = 3000,
    time_step_s: float = 0.01,
    stochastic: bool = True,
) -> EskfSyntheticCase:
    """Sample a declared motion at t=0..N*dt; observe position/altitude every .1/.2 s.

    dt must resolve both observation periods by integer steps. Noise-free mode has
    exact initial state and zero biases/noise, but keeps positive estimator P/Q/R
    for diagnostic use. Stochastic prior pose errors are independent of sensor
    noise; true initial biases are sampled separately, nominal biases start at zero.
    Bias walks start *after* the t=0 measurement, with variance density²*dt per step.
    """
    motion = randomized_eskf_motion(seed, family)
    if type(number_of_steps) is not int or not 1 <= number_of_steps <= 100_000:
        raise ValueError("number_of_steps must be an integer in [1,100000]")
    if (
        type(time_step_s) not in (float, int)
        or not np.isfinite(time_step_s)
        or not 0 < time_step_s <= 0.1
    ):
        raise ValueError("time_step_s must be positive, finite and <=.1 s")
    if type(stochastic) is not bool:
        raise TypeError("stochastic must be a bool")
    ratios = [period / time_step_s for period in (0.1, 0.2)]
    if not all(np.isfinite(ratio) for ratio in ratios):
        raise ValueError("time_step_s must resolve finite observation-period ratios")
    strides = [round(ratio) for ratio in ratios]
    if any(
        abs(stride * time_step_s - period) > 1e-12 * period
        for stride, period in zip(strides, (0.1, 0.2), strict=True)
    ):
        raise ValueError("time_step_s must resolve observation periods by integer steps")
    times = np.arange(number_of_steps + 1, dtype=np.float64) * time_step_s
    exact = sample_eskf_analytic_motion(motion, times)
    n = len(times)
    prior_error = synthetic_eskf_rng(seed, 1).standard_normal(15) * np.repeat(PRIOR_STD, 3)
    prior_error[9:] = 0.0
    bias_initial = synthetic_eskf_rng(seed, 2).standard_normal(6) * np.repeat(PRIOR_STD[3:], 3)
    if not stochastic:
        prior_error[:] = 0.0
        bias_initial[:] = 0.0
    bias_histories = []
    for stream, density, initial in (
        (5, ACCEL_WALK, bias_initial[:3]),
        (6, GYRO_WALK, bias_initial[3:]),
    ):
        increments = (
            synthetic_eskf_rng(seed, stream).standard_normal((number_of_steps, 3))
            * density
            * np.sqrt(time_step_s)
        )
        bias_histories.append(
            initial + np.vstack((np.zeros(3), np.cumsum(increments, axis=0))) * stochastic
        )
    ba_B, bg_B = bias_histories
    specific_force = (
        exact.specific_force_B
        + ba_B
        + stochastic * ACCEL_STD * synthetic_eskf_rng(seed, 3).standard_normal((n, 3))
    )
    angular_velocity = (
        exact.angular_velocity_B
        + bg_B
        + stochastic * GYRO_STD * synthetic_eskf_rng(seed, 4).standard_normal((n, 3))
    )
    observations = []
    for kind, stride, stream, sigma in (
        (EskfObservationKind.LOCAL_POSITION, strides[0], 7, POSITION_STD),
        (EskfObservationKind.BAROMETRIC_ALTITUDE, strides[1], 8, ALTITUDE_STD),
    ):
        epochs = list(range(0, n, stride))
        width = 3 if kind is EskfObservationKind.LOCAL_POSITION else 1
        noise = (
            synthetic_eskf_rng(seed, stream).standard_normal((len(epochs), width))
            * sigma
            * stochastic
        )
        for index, epoch in enumerate(epochs):
            value = (
                exact.position_W[epoch]
                if width == 3
                else np.array([ALTITUDE_DATUM - exact.position_W[epoch, 2]])
            )
            observations.append(
                EskfReplayObservation(kind, index, epoch, epoch, value + noise[index])
            )
    imu = ImuParameters(
        np.zeros(3),
        np.full(3, ACCEL_STD),
        np.full(3, ACCEL_WALK),
        np.zeros(3),
        np.full(3, GYRO_STD),
        np.full(3, GYRO_WALK),
    )
    anchor = EskfNominalState(
        exact.position_W[0], exact.velocity_W[0], exact.q_WB[0], np.zeros(3), np.zeros(3)
    )
    configuration = EskfReplayConfiguration(
        0.0,
        inject_eskf_error_state(anchor, -prior_error),
        np.diag(np.repeat(PRIOR_STD, 3) ** 2),
        GRAVITY,
        sampled_imu_continuous_noise_covariance(imu, time_step_s),
        np.zeros(3),
        np.eye(3) * POSITION_STD**2,
        ALTITUDE_DATUM,
        0.0,
        ALTITUDE_STD**2,
        innovation_policy=EskfInnovationPolicy(),
    )
    reference = EskfReferenceHistory(
        times,
        tuple(
            EskfNominalState(
                exact.position_W[k], exact.velocity_W[k], exact.q_WB[k], ba_B[k], bg_B[k]
            )
            for k in range(n)
        ),
    )
    return EskfSyntheticCase(
        EskfReplayInput(times, specific_force, angular_velocity, tuple(observations)),
        configuration,
        reference,
        motion,
    )
