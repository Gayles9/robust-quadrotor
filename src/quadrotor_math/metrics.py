import numpy as np
from numpy.typing import NDArray


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
