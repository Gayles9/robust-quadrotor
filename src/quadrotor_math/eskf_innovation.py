"""Pre-correction innovation diagnostics and explicit, optional ESKF gate policy.

No state injection, covariance correction, timestamp handling, truth access or RNG
is performed here. See ADR 0007 for the statistical assumptions and finite domain.
"""

from dataclasses import dataclass, field
from math import hypot, isfinite

import numpy as np
from numpy.typing import NDArray

from .eskf import (
    _scaled_innovation_cholesky,
    _symmetrized_float64_matrix,
    _validated_float64_matrix,
)


def _real_array(
    name: str, values: NDArray[np.float64], shape: tuple[int, ...]
) -> NDArray[np.float64]:
    if not isinstance(values, np.ndarray) or values.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    if values.dtype.kind not in "fiu":
        raise ValueError(f"{name} must contain real numeric values")
    with np.errstate(over="ignore", invalid="ignore"):
        owned = np.array(values, dtype=np.float64, order="C", copy=True)
    if not np.all(np.isfinite(owned)):
        raise ValueError(f"{name} must contain only finite values")
    return owned


def _positive_nis_threshold(name: str, value: float) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError(f"{name} must be a positive finite real scalar")
    try:
        threshold = float(value)
    except OverflowError:
        raise ValueError(f"{name} must be positive and finite") from None
    if not isfinite(threshold) or threshold <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return threshold


@dataclass(frozen=True, slots=True, eq=False)
class EskfInnovationPolicy:
    """Dimensionless per-observation upper NIS thresholds, fixed by the caller.

    A None threshold records diagnostics without rejecting that sensor's values.
    Reject strictly above the threshold; equality is accepted. Neither field is
    a probability or a measurement standard deviation. Replay defaults to no
    policy (no scoring); explicitly constructing this class opts into scoring.
    """

    local_position_nis_threshold: float | None = None
    barometric_altitude_nis_threshold: float | None = None

    def __post_init__(self) -> None:
        for name in ("local_position_nis_threshold", "barometric_altitude_nis_threshold"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _positive_nis_threshold(name, value))


def chi_square_99_percent_eskf_innovation_policy() -> EskfInnovationPolicy:
    """Explicit 99% upper chi-square preset: position 3 DOF, altitude 1 DOF.

    Uses NIST's three-decimal table entries, not exact computed quantiles. The
    approximately 1% marginal false-rejection interpretation requires a correct
    zero-mean Gaussian innovation model; it is not a sequence-wide guarantee or
    a demonstrated ESKF consistency result. No preset is enabled implicitly.
    """
    return EskfInnovationPolicy(11.345, 6.635)


@dataclass(frozen=True, slots=True, eq=False)
class EskfInnovation:
    """Own pre-update residual r (m,), covariance S (m,m), whitened r and NIS.

    r is in observation units; S uses their pairwise products. Whitened r and
    normalized_innovation_squared are dimensionless and derived, not supplied.
    S must be finite symmetric and numerically positive definite. Arrays are
    independent C-contiguous read-only float64 copies, with m strictly positive.
    """

    innovation: NDArray[np.float64]
    innovation_covariance: NDArray[np.float64]
    whitened_innovation: NDArray[np.float64] = field(init=False)
    normalized_innovation_squared: float = field(init=False)

    def __post_init__(self) -> None:
        if (
            not isinstance(self.innovation, np.ndarray)
            or self.innovation.ndim != 1
            or not self.innovation.size
        ):
            raise ValueError("innovation must have shape (m,) with m positive")
        size = self.innovation.size
        residual = _real_array("innovation", self.innovation, (size,))
        covariance = _validated_float64_matrix(
            "innovation_covariance",
            _real_array("innovation_covariance", self.innovation_covariance, (size, size)),
            (size, size),
            symmetric_positive_semidefinite=True,
        )
        scales, factor = _scaled_innovation_cholesky(covariance)
        try:
            with np.errstate(over="raise", divide="raise", invalid="raise"):
                whitened = np.linalg.solve(factor, residual / scales)
            # Scaled norm avoids premature underflow of individual squares.
            norm = hypot(*whitened)
            nis = norm * norm
            if not np.all(np.isfinite(whitened)) or not isfinite(nis):
                raise FloatingPointError
        except (FloatingPointError, np.linalg.LinAlgError):
            raise ValueError(
                "ESKF innovation whitening and NIS must remain finite and solvable"
            ) from None
        for name, values in (
            ("innovation", residual),
            ("innovation_covariance", covariance),
            ("whitened_innovation", np.array(whitened, dtype=np.float64, order="C", copy=True)),
        ):
            values.flags.writeable = False
            object.__setattr__(self, name, values)
        object.__setattr__(self, "normalized_innovation_squared", nis)


def compute_eskf_linear_innovation(
    covariance: NDArray[np.float64],
    measurement: NDArray[np.float64],
    predicted_measurement: NDArray[np.float64],
    measurement_jacobian: NDArray[np.float64],
    measurement_noise_covariance: NDArray[np.float64],
) -> EskfInnovation:
    """Score one independent same-epoch observation, without computing a correction.

    Inputs P (15,15), z/h (m,), H (m,15), R (m,m) follow ADR 0005: H differentiates
    the model with respect to the right-local error; P and discrete R are PSD.
    Compute r=z-h and S=H P H.T+R using the update core's arithmetic order, then
    whiten with a diagonally scaled Cholesky solve. No inverse, jitter or hidden
    gate is used. Singular S, invalid data or nonfinite results raise ValueError.
    """
    if not isinstance(measurement, np.ndarray) or measurement.ndim != 1 or not measurement.size:
        raise ValueError("measurement must have shape (m,) with m positive")
    size = measurement.size
    measured = _real_array("measurement", measurement, (size,))
    predicted = _real_array("predicted_measurement", predicted_measurement, (size,))
    P = _validated_float64_matrix(
        "covariance",
        _real_array("covariance", covariance, (15, 15)),
        (15, 15),
        symmetric_positive_semidefinite=True,
    )
    H = _real_array("measurement_jacobian", measurement_jacobian, (size, 15))
    R = _validated_float64_matrix(
        "measurement_noise_covariance",
        _real_array("measurement_noise_covariance", measurement_noise_covariance, (size, size)),
        (size, size),
        symmetric_positive_semidefinite=True,
    )
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            residual = measured - predicted
            cross_covariance = P @ H.T
            S = _symmetrized_float64_matrix(H @ cross_covariance + R)
            if not all(np.all(np.isfinite(a)) for a in (residual, cross_covariance, S)):
                raise FloatingPointError
    except FloatingPointError:
        raise ValueError("ESKF innovation and covariance must remain finite") from None
    return EskfInnovation(residual, S)
