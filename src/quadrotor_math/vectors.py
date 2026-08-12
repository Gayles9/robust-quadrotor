import numpy as np
from numpy.typing import NDArray


def squared_norm(vector: NDArray[np.float64]) -> float:
    """Return the squared Euclidean norm vᵀv.

    Args:
        vector: One-dimensional float64 vector.

    Returns:
        The squared norm as a built-in Python float.
    """
    return float(np.dot(vector, vector))
