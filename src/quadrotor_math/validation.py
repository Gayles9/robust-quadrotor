import numpy as np
from numpy.typing import NDArray


def validate_rigid_body_state_history(
    time_s: NDArray[np.float64],
    position_history_W: NDArray[np.float64],
    velocity_history_W: NDArray[np.float64],
    q_history_WB: NDArray[np.float64],
    omega_history_B: NDArray[np.float64],
) -> None:
    """Validate implemented rigid-body history structural requirements.

    Args:
        time_s: Sample times expected to have shape ``(N,)`` in seconds.
        position_history_W: Position history expected to have shape ``(N, 3)``,
            expressed in the north-east-down (NED) world frame in metres.
        velocity_history_W: Translational velocity history expected to have
            shape ``(N, 3)``, expressed in the NED world frame in m/s.
        q_history_WB: Dimensionless Hamilton scalar-first body-to-world
            quaternion history expected to have shape ``(N, 4)``.
        omega_history_B: Angular-velocity history expected to have shape
            ``(N, 3)``, expressed in the forward-right-down (FRD) body frame
            in rad/s.

    Raises:
        ValueError: If ``time_s`` is not one-dimensional or contains fewer
            than two samples, if ``position_history_W`` or
            ``velocity_history_W`` or ``omega_history_B`` does not have shape
            ``(N, 3)``, if ``q_history_WB`` does not have shape ``(N, 4)``, or
            if a state history does not contain the same number of samples as
            ``time_s``, or if the time or state histories contain a non-finite
            value, or if ``time_s`` is not strictly increasing or uniformly
            spaced, or if ``q_history_WB`` contains a nonunit quaternion.
    """
    if time_s.ndim != 1:
        raise ValueError("time_s must have shape (N,)")

    if time_s.size < 2:
        raise ValueError("state histories must contain at least two samples")

    if position_history_W.ndim != 2 or position_history_W.shape[1] != 3:
        raise ValueError("position_history_W must have shape (N, 3)")

    if velocity_history_W.ndim != 2 or velocity_history_W.shape[1] != 3:
        raise ValueError("velocity_history_W must have shape (N, 3)")

    if q_history_WB.ndim != 2 or q_history_WB.shape[1] != 4:
        raise ValueError("q_history_WB must have shape (N, 4)")

    if omega_history_B.ndim != 2 or omega_history_B.shape[1] != 3:
        raise ValueError("omega_history_B must have shape (N, 3)")

    number_of_samples = time_s.size
    if (
        position_history_W.shape[0] != number_of_samples
        or velocity_history_W.shape[0] != number_of_samples
        or q_history_WB.shape[0] != number_of_samples
        or omega_history_B.shape[0] != number_of_samples
    ):
        raise ValueError("state histories must have the same number of samples as time_s")

    if not np.all(np.isfinite(time_s)):
        raise ValueError("time_s must contain only finite values")

    if not np.all(np.isfinite(position_history_W)):
        raise ValueError("position_history_W must contain only finite values")

    if not np.all(np.isfinite(velocity_history_W)):
        raise ValueError("velocity_history_W must contain only finite values")

    if not np.all(np.isfinite(q_history_WB)):
        raise ValueError("q_history_WB must contain only finite values")

    if not np.all(np.isfinite(omega_history_B)):
        raise ValueError("omega_history_B must contain only finite values")

    adjacent_time_differences_s = np.diff(time_s)
    if not np.all(adjacent_time_differences_s > 0.0):
        raise ValueError("time_s must be strictly increasing")

    if not np.allclose(
        adjacent_time_differences_s,
        adjacent_time_differences_s[0],
        rtol=1e-12,
        atol=0.0,
    ):
        raise ValueError("time_s must be uniformly spaced")

    quaternion_norms = np.linalg.norm(q_history_WB, axis=1)
    if not np.allclose(quaternion_norms, 1.0, rtol=1e-12, atol=1e-12):
        raise ValueError("q_history_WB must contain only unit quaternions")
