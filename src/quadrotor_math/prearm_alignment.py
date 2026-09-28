"""One-shot externally supported alignment; no arming or flight integration.

The caller owns physical support authentication, unique acquisition IDs, disjoint
sample allocation and invocation of time checks. READY is an observed snapshot,
not a continuing permission. See ADR 0029 for the fixed numerical contract.
"""

from dataclasses import dataclass, replace
from enum import StrEnum

import numpy as np

from .eskf import EskfNominalState
from .eskf_endpoint import (
    EskfEndpointState,
    EskfSampledImuNoise,
    _array,
    _nonnegative_scalar,
    initialize_eskf_endpoint,
)
from .prearm_uncertainty import (
    ACCEL_BIAS_SIGMA,
    CLOCK_TOLERANCE_S,
    GRAVITY,
    GYRO_BIAS_SIGMA,
    HEADING_SIGMA,
    PROFILE_ID,
    SAMPLE_COUNT,
    SAMPLE_PERIOD_S,
    Array,
    PrearmEstimate,
    _estimate_window,
)
from .prearm_uncertainty import (
    prearm_imu_noise as prearm_imu_noise,
)


def _identifier(name: str, value: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")


def _sample_id(value: int) -> None:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError("sample ID must be a nonnegative integer")


@dataclass(frozen=True, slots=True)
class PrearmSessionIdentity:
    """Bindings supplied by the acquisition owner; not a global ID registry."""

    acquisition_id: str
    stream_id: str
    clock_id: str
    support_id: str
    source_id: str
    first_sample_id: int
    profile_id: str = PROFILE_ID

    def __post_init__(self) -> None:
        for name in (
            "acquisition_id",
            "stream_id",
            "clock_id",
            "support_id",
            "source_id",
            "profile_id",
        ):
            _identifier(name, getattr(self, name))
        _sample_id(self.first_sample_id)
        object.__setattr__(self, "first_sample_id", int(self.first_sample_id))


@dataclass(frozen=True, slots=True)
class StationarySupportEvidence:
    """External assertion over an observed interval, never inferred from IMU data."""

    acquisition_id: str
    support_id: str
    source_id: str
    clock_id: str
    covered_from_s: float
    observed_through_s: float
    mechanically_supported: bool
    motors_off: bool
    zero_world_acceleration: bool
    zero_angular_velocity: bool
    revoked: bool

    def __post_init__(self) -> None:
        for name in ("acquisition_id", "support_id", "source_id", "clock_id"):
            _identifier(name, getattr(self, name))
        for name in ("covered_from_s", "observed_through_s"):
            object.__setattr__(self, name, _nonnegative_scalar(name, getattr(self, name)))
        if self.observed_through_s < self.covered_from_s:
            raise ValueError("support interval is reversed")
        for name in (
            "mechanically_supported",
            "motors_off",
            "zero_world_acceleration",
            "zero_angular_velocity",
            "revoked",
        ):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be an explicit boolean")


@dataclass(frozen=True, slots=True, eq=False)
class PrearmImuSample:
    """Owned paired FRD specific force (m/s²) and angular rate (rad/s)."""

    stream_id: str
    clock_id: str
    sample_id: int
    time_s: float
    specific_force_B: Array
    angular_velocity_B: Array
    frame: str = "FRD"
    units: str = "SI"
    profile_id: str = PROFILE_ID

    def __post_init__(self) -> None:
        for name in ("stream_id", "clock_id", "frame", "units", "profile_id"):
            _identifier(name, getattr(self, name))
        _sample_id(self.sample_id)
        object.__setattr__(self, "sample_id", int(self.sample_id))
        object.__setattr__(self, "time_s", _nonnegative_scalar("time_s", self.time_s))
        for name in ("specific_force_B", "angular_velocity_B"):
            object.__setattr__(self, name, _array(name, getattr(self, name), (3,)))


@dataclass(frozen=True, slots=True, eq=False)
class PrearmPrior:
    """Independent navigation prior and the unmodified ADR 0028 alignment prior."""

    nominal_state: EskfNominalState
    covariance: Array
    navigation_independent: bool

    def __post_init__(self) -> None:
        if self.navigation_independent is not True:
            raise ValueError("explicit navigation/alignment independence required")
        if not isinstance(self.nominal_state, EskfNominalState):
            raise ValueError("nominal_state must be an EskfNominalState")
        state = replace(self.nominal_state)
        P = _array("covariance", self.covariance, (15, 15), covariance=True)
        expected = np.diag(
            [HEADING_SIGMA**2] * 3 + [ACCEL_BIAS_SIGMA**2] * 3 + [GYRO_BIAS_SIGMA**2] * 3
        )
        if (
            not np.array_equal(P[6:, 6:], expected)
            or np.any(P[:6, 6:] != 0)
            or np.any(P[6:, :6] != 0)
        ):
            raise ValueError("unsupported alignment covariance or navigation cross covariance")
        if (
            np.any(state.q_WB[1:] != 0)
            or abs(state.q_WB[0]) != 1
            or np.any(state.accelerometer_bias_B != 0)
            or np.any(state.gyroscope_bias_B != 0)
        ):
            raise ValueError("identity attitude and zero bias prior means required")
        object.__setattr__(self, "nominal_state", state)
        object.__setattr__(self, "covariance", P)


class PrearmStatus(StrEnum):
    COLLECTING = "collecting"
    READY = "ready"
    REJECTED = "rejected"
    RELEASED = "released"


class PrearmRejection(StrEnum):
    MISSING_SUPPORT = "missing_support"
    INVALID_SUPPORT = "invalid_support"
    INVALID_CLOCK = "invalid_clock"
    DEADLINE_EXPIRED = "deadline_expired"
    INVALID_SAMPLE = "invalid_sample"
    UNEXPECTED_SAMPLE = "unexpected_sample"
    INCOMPLETE_WINDOW = "incomplete_window"
    NUMERICAL_FAILURE = "numerical_failure"
    RATE_COMPATIBILITY = "rate_compatibility"
    GRAVITY_COMPATIBILITY = "gravity_compatibility"
    TEMPORAL_COMPATIBILITY = "temporal_compatibility"
    AXIS_UNCERTAINTY = "axis_uncertainty"
    INCLINATION_DOMAIN = "inclination_domain"


@dataclass(frozen=True, slots=True, eq=False)
class PrearmSnapshot:
    """Owned diagnostics; status applies only at observed_through_s."""

    status: PrearmStatus
    accepted_samples: int
    last_sample_id: int | None
    last_sample_time_s: float | None
    observed_through_s: float
    reasons: tuple[PrearmRejection, ...]
    estimate: PrearmEstimate | None

    def __post_init__(self) -> None:
        if self.estimate is not None:
            object.__setattr__(self, "estimate", replace(self.estimate))


@dataclass(frozen=True, slots=True, eq=False)
class PrearmRelease:
    """One-time estimator handoff; contains no permission to arm."""

    identity: PrearmSessionIdentity
    aligned_sample_ids: tuple[int, int]
    endpoint: EskfEndpointState
    fresh_sample: PrearmImuSample
    support: StationarySupportEvidence

    def __post_init__(self) -> None:
        object.__setattr__(self, "endpoint", replace(self.endpoint))
        object.__setattr__(self, "fresh_sample", replace(self.fresh_sample))


class PrearmAlignmentSession:
    """Fixed 201-sample acquisition and fresh-sample release on a relative clock.

    Call check when waiting without samples. No background clock, automatic
    retry, physical authentication, cross-session registry or motor API exists.
    """

    def __init__(
        self,
        identity: PrearmSessionIdentity,
        prior: PrearmPrior,
        *,
        noise: EskfSampledImuNoise | None = None,
        gravity_acceleration: float = GRAVITY,
        sample_period_s: float = SAMPLE_PERIOD_S,
    ) -> None:
        if not isinstance(identity, PrearmSessionIdentity) or not isinstance(prior, PrearmPrior):
            raise ValueError("typed identity and prior required")
        self._identity, self._prior = replace(identity), replace(prior)
        approved = prearm_imu_noise()
        if noise is not None:
            if not isinstance(noise, EskfSampledImuNoise):
                raise ValueError("typed noise required")
            noise = replace(noise)
            if not np.array_equal(
                noise.sample_covariance_B, approved.sample_covariance_B
            ) or not np.array_equal(
                noise.bias_walk_spectral_density_B, approved.bias_walk_spectral_density_B
            ):
                raise ValueError("unsupported noise profile")
        if (
            identity.profile_id != PROFILE_ID
            or _nonnegative_scalar("gravity", gravity_acceleration) != GRAVITY
            or _nonnegative_scalar("sample period", sample_period_s) != SAMPLE_PERIOD_S
        ):
            raise ValueError("unsupported alignment profile")
        self._noise = approved
        self._status = PrearmStatus.COLLECTING
        self._samples: list[PrearmImuSample] = []
        self._observed_s = 0.0
        self._reasons: tuple[PrearmRejection, ...] = ()
        self._estimate: PrearmEstimate | None = None

    @property
    def status(self) -> PrearmStatus:
        return self._status

    @property
    def snapshot(self) -> PrearmSnapshot:
        last = self._samples[-1] if self._samples else None
        return PrearmSnapshot(
            self._status,
            len(self._samples),
            last.sample_id if last else None,
            last.time_s if last else None,
            self._observed_s,
            self._reasons,
            self._estimate,
        )

    def _reject(self, *reasons: PrearmRejection) -> None:
        self._status, self._reasons = PrearmStatus.REJECTED, reasons

    def _active(self) -> None:
        if self._status in (PrearmStatus.REJECTED, PrearmStatus.RELEASED):
            raise ValueError("terminal pre-arm session cannot be reused")

    def _check(self, support: StationarySupportEvidence | None, now_s: float) -> bool:
        self._active()
        try:
            now = _nonnegative_scalar("now_s", now_s)
        except ValueError:
            self._reject(PrearmRejection.INVALID_CLOCK)
            return False
        if now < self._observed_s - CLOCK_TOLERANCE_S:
            self._reject(PrearmRejection.INVALID_CLOCK)
            return False
        if support is None:
            self._reject(PrearmRejection.MISSING_SUPPORT)
            return False
        try:
            if not isinstance(support, StationarySupportEvidence):
                raise ValueError("typed support required")
            support = replace(support)
            if (
                any(
                    getattr(support, key) != getattr(self._identity, key)
                    for key in ("acquisition_id", "support_id", "source_id", "clock_id")
                )
                or abs(support.covered_from_s) > CLOCK_TOLERANCE_S
                or abs(support.observed_through_s - now) > CLOCK_TOLERANCE_S
                or support.revoked
                or not support.mechanically_supported
                or not support.motors_off
                or not support.zero_world_acceleration
                or not support.zero_angular_velocity
            ):
                raise ValueError("unsupported interval")
        except ValueError:
            self._reject(PrearmRejection.INVALID_SUPPORT)
            return False
        deadline = len(self._samples) * SAMPLE_PERIOD_S
        if now > deadline + CLOCK_TOLERANCE_S:
            self._reject(PrearmRejection.DEADLINE_EXPIRED)
            return False
        self._observed_s = max(now, self._observed_s)
        return True

    def check(self, support: StationarySupportEvidence | None, *, now_s: float) -> PrearmSnapshot:
        """Revalidate support and detect an overdue sample or release."""
        self._check(support, now_s)
        return self.snapshot

    def _sample(self, sample: PrearmImuSample, now_s: float) -> PrearmImuSample | None:
        try:
            if not isinstance(sample, PrearmImuSample):
                raise ValueError("typed sample required")
            sample = replace(sample)
            k = len(self._samples)
            if (
                sample.stream_id != self._identity.stream_id
                or sample.clock_id != self._identity.clock_id
                or sample.sample_id != self._identity.first_sample_id + k
                or sample.frame != "FRD"
                or sample.units != "SI"
                or sample.profile_id != PROFILE_ID
                or abs(sample.time_s - k * SAMPLE_PERIOD_S) > CLOCK_TOLERANCE_S
                or abs(sample.time_s - now_s) > CLOCK_TOLERANCE_S
            ):
                raise ValueError("sample binding, sequence or epoch mismatch")
            return sample
        except ValueError:
            self._reject(PrearmRejection.INVALID_SAMPLE)
            return None

    def consume(
        self, sample: PrearmImuSample, support: StationarySupportEvidence | None, *, now_s: float
    ) -> PrearmSnapshot:
        """Consume one paired sample; evaluate all numeric gates on the full window."""
        if not self._check(support, now_s):
            return self.snapshot
        if self._status is not PrearmStatus.COLLECTING:
            self._reject(PrearmRejection.UNEXPECTED_SAMPLE)
            return self.snapshot
        owned = self._sample(sample, now_s)
        if owned is None:
            return self.snapshot
        self._samples.append(owned)
        if len(self._samples) == SAMPLE_COUNT:
            try:
                estimate = _estimate_window(
                    np.stack([x.specific_force_B for x in self._samples]),
                    np.stack([x.angular_velocity_B for x in self._samples]),
                )
            except (ValueError, FloatingPointError, np.linalg.LinAlgError):
                self._reject(PrearmRejection.NUMERICAL_FAILURE)
                return self.snapshot
            self._estimate = estimate
            reasons = [
                reason
                for value, threshold, reason in zip(
                    estimate.statistics,
                    estimate.thresholds,
                    (
                        PrearmRejection.RATE_COMPATIBILITY,
                        PrearmRejection.GRAVITY_COMPATIBILITY,
                        PrearmRejection.TEMPORAL_COMPATIBILITY,
                    ),
                    strict=True,
                )
                if value > threshold
            ]
            if estimate.radius_99_deg > 0.75:
                reasons.append(PrearmRejection.AXIS_UNCERTAINTY)
            if estimate.inclination_deg > 15:
                reasons.append(PrearmRejection.INCLINATION_DOMAIN)
            if reasons:
                self._reject(*reasons)
            else:
                self._status = PrearmStatus.READY
        return self.snapshot

    def release(
        self, sample: PrearmImuSample, support: StationarySupportEvidence | None, *, now_s: float
    ) -> PrearmRelease | None:
        """Revalidate and return one independent endpoint at 0.5025 seconds.

        The fresh measurement is not added to the alignment window or used to
        condition the estimate. Its white noise remains independent at handoff.
        """
        if not self._check(support, now_s):
            return None
        if self._status is not PrearmStatus.READY:
            self._reject(PrearmRejection.INCOMPLETE_WINDOW)
            return None
        fresh = self._sample(sample, now_s)
        if fresh is None:
            return None
        estimate = self._estimate
        assert estimate is not None and support is not None
        try:
            covariance = self._prior.covariance.copy()
            covariance[6:, 6:] = estimate.joint_covariance
            covariance[9:15, 9:15] += self._noise.bias_walk_spectral_density_B * SAMPLE_PERIOD_S
            state = replace(
                self._prior.nominal_state,
                q_WB=estimate.q_WB,
                accelerometer_bias_B=estimate.accelerometer_bias_B,
                gyroscope_bias_B=estimate.gyroscope_bias_B,
            )
            result = PrearmRelease(
                self._identity,
                (self._identity.first_sample_id, self._identity.first_sample_id + SAMPLE_COUNT - 1),
                initialize_eskf_endpoint(state, covariance, self._noise),
                fresh,
                replace(support),
            )
        except (ValueError, FloatingPointError, np.linalg.LinAlgError):
            self._reject(PrearmRejection.NUMERICAL_FAILURE)
            return None
        self._status = PrearmStatus.RELEASED
        return result
