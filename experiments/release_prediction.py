"""ADR0035 experiment-only one-sided first prediction and explicit one-shot routing."""

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import replace
from typing import Any
from unittest.mock import patch

import numpy as np

from experiments.robustness_evidence import ensure
from experiments.supported_start import PreparedStart, configuration_for
from experiments.supported_velocity_prior import (
    SupportedVelocityConditioner,
    fixture_velocity_support,
)
from quadrotor_math import eskf_online, eskf_replay
from quadrotor_math.eskf import (
    EskfNominalState,
    _quaternion_product,
    _rotation_vector_quaternion,
    _symmetrized_float64_matrix,
    eskf_reset_jacobian,
)
from quadrotor_math.eskf_endpoint import (
    EskfEndpointState,
    EskfSampledImuNoise,
    _array,
    _nonnegative_scalar,
    predict_eskf_endpoint,
)
from quadrotor_math.prearm_uncertainty import CLOCK_TOLERANCE_S, SAMPLE_PERIOD_S, Array
from quadrotor_math.rotations import (
    normalize_quaternion_body_to_world,
    rotation_matrix_body_to_world,
    skew_symmetric,
)


def release_endpoint_map(
    nominal_state: EskfNominalState,
    specific_force_start_B: Array,
    angular_velocity_start_B: Array,
    specific_force_end_B: Array,
    angular_velocity_end_B: Array,
    imu_noise_mean_B: Array,
    gravity_acceleration: float,
    time_step_s: float,
) -> tuple[EskfNominalState, Array, Array]:
    """First release map, using only the post-release force and both gyro samples.

    A21x21 and B21x12 use the ordinary endpoint error/noise ordering. The old
    accelerometer input is validated but never used as open-interval force.
    Covariance derivatives are a local first-order approximation.
    """
    if not isinstance(nominal_state, EskfNominalState):
        raise TypeError("nominal_state must be an EskfNominalState")
    state = replace(nominal_state)
    _array("specific_force_start_B", specific_force_start_B, (3,))
    w0 = _array("angular_velocity_start_B", angular_velocity_start_B, (3,))
    f1 = _array("specific_force_end_B", specific_force_end_B, (3,))
    w1 = _array("angular_velocity_end_B", angular_velocity_end_B, (3,))
    mean = _array("imu_noise_mean_B", imu_noise_mean_B, (6,))
    g = _nonnegative_scalar("gravity_acceleration", gravity_acceleration)
    h = _nonnegative_scalar("time_step_s", time_step_s, positive=True)
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            w0 = w0 - state.gyroscope_bias_B - mean[3:]
            w1 = w1 - state.gyroscope_bias_B
            f1 = f1 - state.accelerometer_bias_B
            phi = h * (0.5 * w0 + 0.5 * w1)
            delta_q_BB = normalize_quaternion_body_to_world(_rotation_vector_quaternion(phi))
            q_WB = normalize_quaternion_body_to_world(_quaternion_product(state.q_WB, delta_q_BB))
            E = rotation_matrix_body_to_world(delta_q_BB)
            R1 = rotation_matrix_body_to_world(q_WB)
            a1 = np.array([0.0, 0.0, g]) + R1 @ f1
            nominal = EskfNominalState(
                state.position_W + h * state.velocity_W + h * h / 2 * a1,
                state.velocity_W + h * a1,
                q_WB,
                state.accelerometer_bias_B,
                state.gyroscope_bias_B,
            )
            correction = np.zeros(15)
            correction[6:9] = phi
            Jr = eskf_reset_jacobian(correction)[6:9, 6:9]
            theta = np.zeros((3, 33))
            theta[:, 6:9] = E.T
            theta[:, 12:15] = -h * Jr
            for block in (18, 24, 30):
                theta[:, block : block + 3] = -h / 2 * Jr
            acceleration = -R1 @ skew_symmetric(f1) @ theta
            for block in (9, 21, 27):
                acceleration[:, block : block + 3] -= R1
            derivative = np.zeros((21, 33))
            derivative[:3] = h * h / 2 * acceleration
            derivative[:3, :3] += np.eye(3)
            derivative[:3, 3:6] += h * np.eye(3)
            derivative[3:6] = h * acceleration
            derivative[3:6, 3:6] += np.eye(3)
            derivative[6:9] = theta
            derivative[9:15, 9:15] = np.eye(6)
            derivative[9:15, 27:33] = np.eye(6)
            derivative[15:21, 21:27] = np.eye(6)
    except (FloatingPointError, OverflowError):
        raise ValueError("release endpoint map must remain finite") from None
    return (
        nominal,
        _array("transition_matrix", derivative[:, :21], (21, 21)),
        _array("noise_input_matrix", derivative[:, 21:], (21, 12)),
    )


def predict_release_endpoint(
    current: EskfEndpointState,
    specific_force_start_B: Array,
    angular_velocity_start_B: Array,
    specific_force_end_B: Array,
    angular_velocity_end_B: Array,
    gravity_acceleration: float,
    noise: EskfSampledImuNoise,
    time_step_s: float,
) -> EskfEndpointState:
    """Propagate the full joint tangent covariance without a floor or lost memory."""
    if not isinstance(current, EskfEndpointState) or not isinstance(noise, EskfSampledImuNoise):
        raise TypeError("prediction requires EskfEndpointState and EskfSampledImuNoise")
    h = _nonnegative_scalar("time_step_s", time_step_s, positive=True)
    nominal, A, B = release_endpoint_map(
        current.nominal_state,
        specific_force_start_B,
        angular_velocity_start_B,
        specific_force_end_B,
        angular_velocity_end_B,
        current.imu_noise_mean_B,
        gravity_acceleration,
        h,
    )
    try:
        with np.errstate(over="raise", invalid="raise"):
            D = np.zeros((12, 12))
            D[:6, :6] = noise.sample_covariance_B
            D[6:, 6:] = h * noise.bias_walk_spectral_density_B
            C = _symmetrized_float64_matrix(A @ current.joint_covariance @ A.T + B @ D @ B.T)
    except FloatingPointError:
        raise ValueError("release endpoint covariance must remain finite") from None
    return EskfEndpointState(nominal, np.zeros(6), C)


class ReleasePrediction:
    """One experiment context for one online flight OR one saved replay.

    Caller supplies an authenticated explicit fixture; it is not discovered from
    quiet IMU data. Process-local patching is not safe for concurrent threads.
    A rejected or incomplete context is consumed. No arming authorization results.
    """

    def __init__(self, prepared: PreparedStart) -> None:
        ensure(prepared.release is not None, "supported release required")
        assert prepared.release is not None
        # Validate support, ownership, freshness and the unchanged supported profile;
        # discard the diagnostic conditioned prior. This adapter never changes it.
        SupportedVelocityConditioner().condition(
            prepared.release,
            fixture_velocity_support(prepared),
            now_s=prepared.release.fresh_sample.time_s,
        )
        self._release = prepared.release
        self._noise = configuration_for(prepared, "aligned")[
            "estimator_configuration"
        ].sampled_imu_noise
        self._used = False
        self._failed = False
        self.trace = {"predictions": 0, "release_predictions": 0}

    @contextmanager
    def installed(self, target: str) -> Iterator[dict[str, int]]:
        ensure(not self._used, "release adapter already consumed")
        self._used = True
        ensure(target in ("online", "replay"), "explicit online or replay target required")
        module = eskf_online if target == "online" else eskf_replay
        with patch.object(module, "predict_eskf_endpoint", self._predict):
            yield self.trace
        ensure(self.trace["release_predictions"] == 1 and not self._failed, "incomplete release")

    def _predict(self, *args: Any, **kwargs: Any) -> EskfEndpointState:
        ensure(not self._failed, "release adapter rejected")
        self._failed = True
        ensure(not kwargs and len(args) == 8, "explicit endpoint argument contract")
        if self.trace["predictions"]:
            result = predict_eskf_endpoint(*args)
        else:
            fresh = self._release.fresh_sample
            ensure(
                np.array_equal(args[1], fresh.specific_force_B)
                and np.array_equal(args[2], fresh.angular_velocity_B),
                "first prediction must own the fresh supported sample",
            )
            ensure(
                abs(args[7] - SAMPLE_PERIOD_S) <= CLOCK_TOLERANCE_S,
                "first release interval must match the frozen sample clock",
            )
            ensure(
                isinstance(args[6], EskfSampledImuNoise)
                and np.array_equal(args[6].sample_covariance_B, self._noise.sample_covariance_B)
                and np.array_equal(
                    args[6].bias_walk_spectral_density_B,
                    self._noise.bias_walk_spectral_density_B,
                ),
                "release noise profile mismatch",
            )
            result = predict_release_endpoint(*args)
            self.trace["release_predictions"] += 1
        self._failed = False
        self.trace["predictions"] += 1
        return result
