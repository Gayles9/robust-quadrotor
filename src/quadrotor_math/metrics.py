import numpy as np
from numpy.typing import NDArray


def euclidean_vector_trajectory_errors(
    vector_history: NDArray[np.float64],
    reference_vector_history: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return row-wise Euclidean errors between corresponding vector samples.

    Both histories are expected to have shape ``(N, D)``, with each row
    containing one vector sample. Corresponding rows are subtracted and their
    Euclidean norms are returned independently.

    Args:
        vector_history: Vector samples with shape ``(N, D)``.
        reference_vector_history: Corresponding reference vector samples with
            shape ``(N, D)``.

    Returns:
        A float64 array with shape ``(N,)`` containing one Euclidean error per
        sample. Each error has the same units as the supplied vector quantity.

    Raises:
        ValueError: If either history is not two-dimensional, the history
            shapes do not match, the histories contain no samples or no vector
            components, or either history contains a non-finite value.

    This metric is intended to be applied independently to position, velocity,
    or angular-velocity histories. Differently dimensioned physical quantities
    are not combined because their numerical magnitudes have incompatible
    units and meanings.
    """
    if vector_history.ndim != 2:
        raise ValueError("vector_history must be two-dimensional")

    if reference_vector_history.ndim != 2:
        raise ValueError("reference_vector_history must be two-dimensional")

    if vector_history.shape != reference_vector_history.shape:
        raise ValueError("vector_history and reference_vector_history must have matching shapes")

    if vector_history.shape[0] == 0:
        raise ValueError("vector histories must contain at least one sample")

    if vector_history.shape[1] == 0:
        raise ValueError("vector histories must contain at least one component")

    if not np.all(np.isfinite(vector_history)):
        raise ValueError("vector_history must contain only finite values")

    if not np.all(np.isfinite(reference_vector_history)):
        raise ValueError("reference_vector_history must contain only finite values")

    trajectory_errors = np.linalg.norm(
        vector_history - reference_vector_history,
        axis=1,
    )
    return np.asarray(trajectory_errors, dtype=np.float64)


def observed_convergence_orders(
    time_steps: NDArray[np.float64],
    errors: NDArray[np.float64],
    *,
    error_floor: float,
) -> NDArray[np.float64]:
    """Return observed convergence orders for adjacent resolution pairs.

    ``time_steps`` is expected to be a one-dimensional array of decreasing
    time steps, and ``errors`` is expected to contain the corresponding
    nonnegative errors. Each adjacent-pair order uses the pair's actual
    refinement ratio, so refinement ratios need not be two. ``error_floor`` is
    keyword-only and identifies errors too small for a meaningful order
    estimate. Differences of logarithms avoid overflow or underflow from
    directly forming extreme finite ratios. A ``log1p`` fallback preserves
    distinguishable adjacent values when separately rounded logarithms cancel.

    Args:
        time_steps: Decreasing time steps with shape ``(M,)``.
        errors: Corresponding nonnegative errors with shape ``(M,)``.
        error_floor: Numerical error floor. An adjacent-pair order is computed
            only when both errors are strictly above this value.

    Returns:
        A float64 array with shape ``(M - 1,)`` containing adjacent-pair order
        estimates. Entries are ``np.nan`` when either adjacent error is at or
        below ``error_floor``. Negative orders remain valid measurements and
        are not clipped.

    Raises:
        ValueError: If either input array is not one-dimensional, their shapes
            do not match, fewer than two resolutions are supplied, time steps
            are non-finite, non-positive, or not strictly decreasing, errors
            are non-finite or negative, or ``error_floor`` is non-finite or
            negative.
    """
    if time_steps.ndim != 1:
        raise ValueError("time_steps must be one-dimensional")

    if errors.ndim != 1:
        raise ValueError("errors must be one-dimensional")

    if time_steps.shape != errors.shape:
        raise ValueError("time_steps and errors must have matching shapes")

    if time_steps.size < 2:
        raise ValueError("at least two time steps and errors are required")

    if not np.all(np.isfinite(time_steps)):
        raise ValueError("time_steps must contain only finite values")

    if not np.all(time_steps > 0.0):
        raise ValueError("time_steps must be positive")

    if not np.all(np.diff(time_steps) < 0.0):
        raise ValueError("time_steps must be strictly decreasing")

    if not np.all(np.isfinite(errors)):
        raise ValueError("errors must contain only finite values")

    if not np.all(errors >= 0.0):
        raise ValueError("errors must be nonnegative")

    if not np.isfinite(error_floor):
        raise ValueError("error_floor must be finite")

    if error_floor < 0.0:
        raise ValueError("error_floor must be nonnegative")

    orders = np.full(time_steps.shape[0] - 1, np.nan, dtype=np.float64)
    valid_pairs = (errors[:-1] > error_floor) & (errors[1:] > error_floor)
    current_errors = errors[:-1][valid_pairs]
    next_errors = errors[1:][valid_pairs]
    current_time_steps = time_steps[:-1][valid_pairs]
    next_time_steps = time_steps[1:][valid_pairs]
    log_error_change = np.log(current_errors) - np.log(next_errors)
    cancelled_error_changes = (log_error_change == 0.0) & (current_errors != next_errors)
    log_error_change[cancelled_error_changes] = np.log1p(
        (current_errors[cancelled_error_changes] - next_errors[cancelled_error_changes])
        / next_errors[cancelled_error_changes]
    )
    log_time_step_change = np.log(current_time_steps) - np.log(next_time_steps)
    cancelled_time_step_changes = (log_time_step_change == 0.0) & (
        current_time_steps != next_time_steps
    )
    log_time_step_change[cancelled_time_step_changes] = np.log1p(
        (
            current_time_steps[cancelled_time_step_changes]
            - next_time_steps[cancelled_time_step_changes]
        )
        / next_time_steps[cancelled_time_step_changes]
    )
    orders[valid_pairs] = log_error_change / log_time_step_change
    return orders


def quaternion_attitude_trajectory_errors_body_to_world(
    q_history_WB: NDArray[np.float64],
    reference_q_history_WB: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return sign-invariant body-to-world attitude errors over a trajectory.

    Both inputs are expected to have shape ``(N, 4)`` and contain unit
    quaternions using Hamilton scalar-first ``[w, x, y, z]`` ordering. Each
    quaternion maps body-coordinate vectors into world coordinates.

    Corresponding quaternion signs are aligned before measuring their
    Euclidean chord. Because opposite signs represent the same physical
    attitude, this selects the shorter chord between equivalent unit-
    quaternion representations. The chord is converted to the physical
    geodesic attitude error.

    Args:
        q_history_WB: Body-to-world unit-quaternion history with shape
            ``(N, 4)``.
        reference_q_history_WB: Reference body-to-world unit-quaternion
            history with shape ``(N, 4)``.

    Returns:
        A float64 array with shape ``(N,)`` containing attitude errors in
        radians.

    Raises:
        ValueError: If either quaternion history does not have shape
            ``(N, 4)``, the complete history shapes do not match, or the
            histories contain no samples, a non-finite value, or a non-unit
            quaternion.
    """
    if q_history_WB.ndim != 2 or q_history_WB.shape[1] != 4:
        raise ValueError(f"q_history_WB must have shape (N, 4), got {q_history_WB.shape}")

    if reference_q_history_WB.ndim != 2 or reference_q_history_WB.shape[1] != 4:
        raise ValueError(
            f"reference_q_history_WB must have shape (N, 4), got {reference_q_history_WB.shape}"
        )

    if q_history_WB.shape != reference_q_history_WB.shape:
        raise ValueError("q_history_WB and reference_q_history_WB must have matching shapes")

    if q_history_WB.shape[0] == 0:
        raise ValueError("quaternion histories must contain at least one sample")

    if not np.all(np.isfinite(q_history_WB)):
        raise ValueError("q_history_WB must contain only finite values")

    if not np.all(np.isfinite(reference_q_history_WB)):
        raise ValueError("reference_q_history_WB must contain only finite values")

    q_norms = np.linalg.norm(q_history_WB, axis=1)
    if not np.allclose(q_norms, 1.0, rtol=1e-12, atol=1e-12):
        raise ValueError("q_history_WB must contain only unit quaternions")

    reference_q_norms = np.linalg.norm(reference_q_history_WB, axis=1)
    if not np.allclose(reference_q_norms, 1.0, rtol=1e-12, atol=1e-12):
        raise ValueError("reference_q_history_WB must contain only unit quaternions")

    dot_products = np.sum(q_history_WB * reference_q_history_WB, axis=1)
    alignment_factors = np.where(dot_products < 0.0, -1.0, 1.0)
    aligned_q_history_WB = q_history_WB * alignment_factors[:, np.newaxis]
    half_chords = 0.5 * np.linalg.norm(
        aligned_q_history_WB - reference_q_history_WB,
        axis=1,
    )
    attitude_errors = 4.0 * np.arcsin(np.clip(half_chords, 0.0, 1.0))
    return np.asarray(attitude_errors, dtype=np.float64)
