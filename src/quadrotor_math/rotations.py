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
