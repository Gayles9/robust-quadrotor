"""Transactional measurement-only ESKF epochs, sharing the offline fusion law."""

from dataclasses import dataclass, replace

import numpy as np
from numpy.typing import NDArray

from .eskf import EskfNominalState, predict_eskf
from .eskf_endpoint import initialize_eskf_endpoint, predict_eskf_endpoint
from .eskf_replay import (
    EskfReplayConfiguration,
    EskfReplayEvent,
    EskfReplayObservation,
    _correct_eskf_epoch,
    _owned_array,
    _owned_covariance,
    _scalar,
)


@dataclass(frozen=True, slots=True, eq=False)
class EskfOnlineEstimate:
    """Owned posterior and rate feedback at one delivered IMU epoch.

    Covariance is the physical (15,15) marginal, not continuation storage.
    angular_velocity_estimate_B is FRD rad/s: measured gyro minus posterior
    bias and, in endpoint mode, conditional gyro sample-noise mean. Events are
    in position/altitude order and contain only this epoch's deliveries.
    """

    epoch_index: int
    time_s: float
    nominal_state: EskfNominalState
    covariance: NDArray[np.float64]
    angular_velocity_estimate_B: NDArray[np.float64]
    imu_noise_mean_B: NDArray[np.float64]
    events: tuple[EskfReplayEvent, ...]

    def __post_init__(self) -> None:
        if type(self.epoch_index) is not int or self.epoch_index < 0:
            raise ValueError("epoch_index must be a nonnegative integer")
        object.__setattr__(self, "time_s", _scalar("time_s", self.time_s, nonnegative=True))
        if not isinstance(self.nominal_state, EskfNominalState):
            raise TypeError("nominal_state must be EskfNominalState")
        object.__setattr__(self, "nominal_state", replace(self.nominal_state))
        object.__setattr__(self, "covariance", _owned_covariance("covariance", self.covariance, 15))
        object.__setattr__(
            self,
            "angular_velocity_estimate_B",
            _owned_array("angular_velocity_estimate_B", self.angular_velocity_estimate_B, (3,)),
        )
        object.__setattr__(
            self, "imu_noise_mean_B", _owned_array("imu_noise_mean_B", self.imu_noise_mean_B, (6,))
        )
        if not isinstance(self.events, tuple) or not all(
            isinstance(e, EskfReplayEvent) and e.observation.delivery_index == self.epoch_index
            for e in self.events
        ):
            raise ValueError("events must contain only this epoch's delivered observations")
        object.__setattr__(self, "events", tuple(replace(e) for e in self.events))


class EskfOnlineEstimator:
    """Caller-driven stream; no truth, scheduler, RNG or command input.

    A call contains one paired, zero-delay IMU sample and all slow observations
    delivered at that epoch. No prediction precedes the explicit prior epoch.
    Later epochs may be nonuniform but must strictly increase. Observation IDs
    and per-sensor acquisition epochs cannot be reused; gaps/out-of-order slow
    delivery are allowed, and older data is stale, never rewound. Pending data
    belongs to the caller's queue, not a step. Not a concurrent/thread-safe API.

    A failed call is atomic: no state, sample memory, epoch or identity is consumed.
    Every returned value owns its storage separately from continuation memory.
    """

    def __init__(self, configuration: EskfReplayConfiguration) -> None:
        if not isinstance(configuration, EskfReplayConfiguration):
            raise TypeError("configuration must be EskfReplayConfiguration")
        self._configuration = replace(configuration)
        self._state = replace(self._configuration.initial_state)
        self._covariance = self._configuration.initial_covariance
        noise = self._configuration.sampled_imu_noise
        self._endpoint = (
            initialize_eskf_endpoint(self._state, self._covariance, noise)
            if noise is not None
            else None
        )
        self._index = -1
        self._time: float | None = None
        self._force = np.zeros(3)
        self._rate = np.zeros(3)
        self._rate_estimate = np.zeros(3)
        self._events: tuple[EskfReplayEvent, ...] = ()
        self._identities: set[tuple[int, int]] = set()
        self._acquisitions: set[tuple[int, int]] = set()

    @property
    def latest(self) -> EskfOnlineEstimate | None:
        """Independent snapshot, or None until the first successful sample."""
        if self._time is None:
            return None
        return EskfOnlineEstimate(
            self._index,
            self._time,
            self._state,
            self._covariance,
            self._rate_estimate,
            self._endpoint.imu_noise_mean_B if self._endpoint is not None else np.zeros(6),
            self._events,
        )

    def step(
        self,
        time_s: float,
        specific_force_measurement_B: NDArray[np.float64],
        angular_velocity_measurement_B: NDArray[np.float64],
        observations: tuple[EskfReplayObservation, ...] = (),
    ) -> EskfOnlineEstimate:
        """Predict, then correct with delivered data; commit only on full success."""
        time = _scalar("time_s", time_s, nonnegative=True)
        config = self._configuration
        if (self._time is None and time != config.initial_time_s) or (
            self._time is not None and time <= self._time
        ):
            raise ValueError("time must start at the explicit prior epoch and strictly increase")
        force = _owned_array("specific_force_measurement_B", specific_force_measurement_B, (3,))
        rate = _owned_array("angular_velocity_measurement_B", angular_velocity_measurement_B, (3,))
        if not isinstance(observations, tuple) or not all(
            isinstance(o, EskfReplayObservation) for o in observations
        ):
            raise TypeError("observations must be a tuple of EskfReplayObservation")
        index = self._index + 1
        owned = tuple(
            sorted(
                (replace(o) for o in observations),
                key=lambda o: (o.kind.value, o.observation_index),
            )
        )
        identities = {(o.kind.value, o.observation_index) for o in owned}
        acquisitions = {(o.kind.value, o.acquisition_index) for o in owned}
        if (
            len(identities) != len(owned)
            or len(acquisitions) != len(owned)
            or identities & self._identities
            or acquisitions & self._acquisitions
        ):
            raise ValueError("duplicate observation identity or acquisition epoch")
        if any(o.delivery_index != index or o.acquisition_index > index for o in owned):
            raise ValueError("observations must be delivered at the current epoch, never pending")
        state, covariance, endpoint = self._state, self._covariance, self._endpoint
        if self._time is not None:
            dt = time - self._time
            if endpoint is not None and config.sampled_imu_noise is not None:
                endpoint = predict_eskf_endpoint(
                    endpoint,
                    self._force,
                    self._rate,
                    force,
                    rate,
                    config.gravity_acceleration,
                    config.sampled_imu_noise,
                    dt,
                )
                state, covariance = endpoint.nominal_state, endpoint.joint_covariance[:15, :15]
            else:
                state, covariance = predict_eskf(
                    state,
                    covariance,
                    self._force,
                    self._rate,
                    config.gravity_acceleration,
                    config.continuous_noise_covariance,
                    dt,
                )
        state, covariance, endpoint, events = _correct_eskf_epoch(
            state, covariance, endpoint, owned, index, config
        )
        with np.errstate(over="ignore", invalid="ignore"):
            rate_estimate = rate - state.gyroscope_bias_B
            if endpoint is not None:
                rate_estimate = rate_estimate - endpoint.imu_noise_mean_B[3:]
        noise_mean = endpoint.imu_noise_mean_B if endpoint is not None else np.zeros(6)
        result = EskfOnlineEstimate(
            index, time, state, covariance, rate_estimate, noise_mean, events
        )
        # All validation/arithmetic above succeeded. No caller-owned aliases below.
        self._state, self._covariance, self._endpoint = state, covariance, endpoint
        self._force, self._rate = force, rate
        self._rate_estimate, self._events = rate_estimate, events
        self._time, self._index = time, index
        self._identities.update(identities)
        self._acquisitions.update(acquisitions)
        return result
