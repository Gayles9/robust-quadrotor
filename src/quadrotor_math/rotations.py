import numpy as np
from numpy.typing import NDArray


def skew_symmetric(vector: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the matrix satisfying ``skew_symmetric(a) @ b == np.cross(a, b)``.

    Args:
        vector: Three-element vector ``[x, y, z]``.

    Raises:
        ValueError: If ``vector`` does not have shape ``(3,)``.
    """
    if vector.shape != (3,):
        raise ValueError(f"vector must have shape (3,), got {vector.shape}")

    x, y, z = vector
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )


def normalize_quaternion_body_to_world(
    q_WB: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return a new unit-norm body-to-world quaternion.

    Args:
        q_WB: Hamilton scalar-first body-to-world quaternion.

    Returns:
        A new unit-norm quaternion.

    Raises:
        ValueError: If ``q_WB`` does not have shape ``(4,)``, contains a non-finite
            component, or has zero norm.
    """
    if q_WB.shape != (4,):
        raise ValueError(f"q_WB must have shape (4,), got {q_WB.shape}")

    if not np.all(np.isfinite(q_WB)):
        raise ValueError("q_WB must contain only finite values")

    norm = float(np.linalg.norm(q_WB))
    if norm == 0.0:
        raise ValueError("q_WB must have nonzero norm")

    return q_WB / norm


def rotation_matrix_body_to_world(
    q_WB: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return the active body-to-world rotation matrix from ``q_WB``.

    Args:
        q_WB: Hamilton scalar-first ``[w, x, y, z]`` quaternion representing
            body-to-world orientation.

    Returns:
        ``R_WB``, which actively maps body-coordinate vectors into world coordinates.

    Raises:
        ValueError: If ``q_WB`` does not have shape ``(4,)`` or unit norm.
    """
    if q_WB.shape != (4,):
        raise ValueError(f"q_WB must have shape (4,), got {q_WB.shape}")

    squared_norm = float(np.dot(q_WB, q_WB))
    if not np.isclose(
        squared_norm,
        1.0,
        rtol=1e-12,
        atol=1e-12,
    ):
        raise ValueError(f"q_WB must have unit norm; squared norm is {squared_norm}")

    w, x, y, z = q_WB
    return np.array(
        [
            [1.0 - 2.0 * (y**2 + z**2), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x**2 + z**2), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x**2 + y**2)],
        ],
        dtype=np.float64,
    )
