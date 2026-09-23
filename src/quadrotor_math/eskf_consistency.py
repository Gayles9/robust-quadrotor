"""Evaluation-only ESKF errors and consistency scores; never an estimator input.

See ADR 0008 for epoch alignment, right-local errors and the Gaussian domain.
"""

from dataclasses import dataclass, replace
from math import atan2, hypot, isfinite, sqrt

import numpy as np
from numpy.typing import NDArray

from .eskf import EskfNominalState, _quaternion_product, _scaled_innovation_cholesky
from .eskf_replay import EskfReplayResult, _owned_array, _owned_covariance, _scalar, _time_vector
from .run_artifact import RunArtifactData
from .run_configuration import ImuParameters
from .sensor_scheduling import _time_comparison_tolerance_s


def eskf_right_local_error(
    nominal: EskfNominalState, reference: EskfNominalState
) -> NDArray[np.float64]:
    """Return owned read-only (15,) reference-minus-nominal error, in ADR 0004 order.

    Additive blocks: p_W [m], v_W [m/s], b_a_B [m/s²], b_g_B [rad/s]. Attitude
    is Log(R_nominal_WB.T R_reference_WB) [rad], in the nominal body's local
    coordinates. Its principal angle lies in [0, pi]. Quaternion signs do not
    matter; exactly at pi the first nonzero axis component is chosen positive.
    The log's pi discontinuity is intrinsic; such errors are not locally Gaussian.
    """
    if not isinstance(nominal, EskfNominalState) or not isinstance(reference, EskfNominalState):
        raise TypeError("nominal and reference must be EskfNominalState values")
    nominal, reference = replace(nominal), replace(reference)
    inverse_q_BW = nominal.q_WB * np.array([1.0, -1.0, -1.0, -1.0])
    relative_q_BB = _quaternion_product(inverse_q_BW, reference.q_WB)
    relative_q_BB /= hypot(*relative_q_BB)
    first_nonzero = next(float(v) for v in relative_q_BB if v != 0.0)
    if first_nonzero < 0.0:
        relative_q_BB *= -1.0
    vector = relative_q_BB[1:]
    sine_half = hypot(*vector)
    # Use the limiting multiplier 2 near zero, avoiding subnormal angle ratios.
    scale = 2.0 if sine_half < 1e-8 else 2.0 * atan2(sine_half, relative_q_BB[0]) / sine_half
    try:
        with np.errstate(over="raise", invalid="raise"):
            error = np.concatenate(
                (
                    reference.position_W - nominal.position_W,
                    reference.velocity_W - nominal.velocity_W,
                    scale * vector,
                    reference.accelerometer_bias_B - nominal.accelerometer_bias_B,
                    reference.gyroscope_bias_B - nominal.gyroscope_bias_B,
                )
            )
    except FloatingPointError:
        raise ValueError("ESKF reference error must remain finite") from None
    return _owned_array("error_state", error, (15,))


def normalized_estimation_error_squared(
    error_state: NDArray[np.float64], covariance: NDArray[np.float64]
) -> float:
    """Compute dimensionless e.T P^-1 e for finite (15,) e and SPD (15,15) P.

    Uses a diagonally scaled Cholesky solve, never an inverse, pseudoinverse or
    regularization. Singular PSD P is valid for propagation but cannot define
    this full-state, 15-degree-of-freedom statistic, even when e is zero.
    """
    error = _owned_array("error_state", error_state, (15,))
    matrix = _owned_covariance("covariance", covariance, 15)
    try:
        scales, factor = _scaled_innovation_cholesky(matrix)
        with np.errstate(over="raise", divide="raise", invalid="raise"):
            whitened = np.linalg.solve(factor, error / scales)
        norm = hypot(*whitened)
        nees = norm * norm
        if not isfinite(nees):
            raise FloatingPointError
    except (ValueError, FloatingPointError, np.linalg.LinAlgError):
        raise ValueError(
            "NEES requires positive-definite covariance and finite solvable whitening"
        ) from None
    return nees


@dataclass(frozen=True, slots=True, eq=False)
class EskfReferenceHistory:
    """Owned evaluation reference states on explicit nonnegative increasing epochs."""

    time_s: NDArray[np.float64]
    states: tuple[EskfNominalState, ...]

    def __post_init__(self) -> None:
        times = _time_vector(self.time_s)
        if len(self.states) != times.size or not all(
            isinstance(s, EskfNominalState) for s in self.states
        ):
            raise ValueError("states must contain one EskfNominalState per reference epoch")
        object.__setattr__(self, "time_s", times)
        object.__setattr__(self, "states", tuple(replace(s) for s in self.states))


@dataclass(frozen=True, slots=True, eq=False)
class EskfConsistencyHistory:
    """Owned post-update errors (n,15) and dimensionless NEES (n,) at n epochs."""

    time_s: NDArray[np.float64]
    error_states: NDArray[np.float64]
    nees: NDArray[np.float64]

    def __post_init__(self) -> None:
        times = _time_vector(self.time_s)
        errors = _owned_array("error_states", self.error_states, (times.size, 15))
        nees = _owned_array("nees", self.nees, (times.size,))
        if np.any(nees < 0.0):
            raise ValueError("nees must be nonnegative")
        for name, value in (("time_s", times), ("error_states", errors), ("nees", nees)):
            object.__setattr__(self, name, value)


def eskf_reference_from_run_artifact(data: RunArtifactData) -> EskfReferenceHistory:
    """Extract truth/bias rows 1..N for evaluation of completed-row IMU replay.

    Reads only clock, truth p/v/q, and true bias histories. It does not construct
    estimator priors or consume measured values. Requires the replay adapter's
    exact canonical zero-origin fixed grid; no interpolation or time tolerance
    joins. All consumed truth rows, including the unused origin, are validated.
    """
    clock = _time_vector(data.truth_time_s)
    if clock.size < 2 or clock[0] != 0.0:
        raise ValueError("truth clock must contain origin and completed rows")
    dt = float(clock[1])
    with np.errstate(over="ignore", invalid="ignore"):
        canonical = np.arange(clock.size, dtype=np.float64) * dt
    if not np.array_equal(clock, canonical):
        raise ValueError("truth clock must equal the canonical fixed grid")
    if dt <= 2.0 * _time_comparison_tolerance_s(float(clock[-1]), float(clock[-1])):
        raise ValueError("truth clock spacing must resolve scheduler comparison tolerance")
    names = (
        "truth_position_history_W",
        "truth_velocity_history_W",
        "truth_q_history_WB",
        "accelerometer_bias_history_B",
        "gyroscope_bias_history_B",
    )
    arrays = [
        _owned_array(name, getattr(data, name), (clock.size, 4 if i == 2 else 3))
        for i, name in enumerate(names)
    ]
    states = tuple(EskfNominalState(*(a[i] for a in arrays)) for i in range(clock.size))
    return EskfReferenceHistory(clock[1:], states[1:])


def evaluate_eskf_replay(
    result: EskfReplayResult, reference: EskfReferenceHistory
) -> EskfConsistencyHistory:
    """Pair exact epochs, then evaluate post-update states and reset covariances.

    Result and reference constructors enforce their contracts; revalidation here
    also protects against callers deliberately re-enabling array writes.
    """
    if not isinstance(result, EskfReplayResult) or not isinstance(reference, EskfReferenceHistory):
        raise TypeError("evaluation requires EskfReplayResult and EskfReferenceHistory")
    result, reference = replace(result), replace(reference)
    if not np.array_equal(result.time_s, reference.time_s):
        raise ValueError("replay and reference epochs must match exactly")
    errors = np.stack(
        [eskf_right_local_error(n, r) for n, r in zip(result.states, reference.states, strict=True)]
    )
    nees = np.array(
        [
            normalized_estimation_error_squared(e, P)
            for e, P in zip(errors, result.covariances, strict=True)
        ]
    )
    return EskfConsistencyHistory(result.time_s, errors, nees)


def sampled_imu_continuous_noise_covariance(
    parameters: ImuParameters, sample_interval_s: float
) -> NDArray[np.float64]:
    """Explicit Q_c (12,12) for independent full-rate left-held IMU samples.

    White sample stddevs sigma_a [m/s²], sigma_g [rad/s] map to spectral
    densities sigma² dt. Bias random-walk densities already have bias/sqrt(s)
    units and are simply squared. Noise order is [n_a,n_g,n_wa,n_wg]. This
    matches leading velocity/attitude increment variance; the existing first-
    order discretization omits higher-order position noise and cross terms.
    """
    if not isinstance(parameters, ImuParameters):
        raise TypeError("parameters must be ImuParameters")
    parameters = replace(parameters)
    dt = _scalar("sample_interval_s", sample_interval_s, nonnegative=True)
    if dt == 0.0:
        raise ValueError("sample_interval_s must be positive")
    source = np.concatenate(
        (
            parameters.accelerometer_noise_standard_deviation_B,
            parameters.gyroscope_noise_standard_deviation_B,
            parameters.accelerometer_bias_random_walk_density_B,
            parameters.gyroscope_bias_random_walk_density_B,
        )
    )
    try:
        with np.errstate(over="raise", invalid="raise", under="ignore"):
            scaled = source * np.concatenate((np.full(6, sqrt(dt)), np.ones(6)))
            diagonal = scaled * scaled
    except FloatingPointError:
        raise ValueError("continuous noise variances must be finite and representable") from None
    if np.any((source > 0.0) & (diagonal == 0.0)):
        raise ValueError("positive continuous noise variances must be representable")
    return _owned_array("continuous_noise_covariance", np.diag(diagonal), (12, 12))
