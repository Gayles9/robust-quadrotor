import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.rotations import skew_symmetric


def test_skew_symmetric_represents_cross_product() -> None:
    vector_a = np.array([1.0, -2.0, 3.0], dtype=np.float64)

    skew_matrix = skew_symmetric(vector_a)

    expected = np.array(
        [
            [0.0, -3.0, -2.0],
            [3.0, 0.0, -1.0],
            [2.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    np.testing.assert_allclose(skew_matrix, expected)

    vector_b = np.array([4.0, 5.0, -6.0], dtype=np.float64)

    np.testing.assert_allclose(skew_matrix @ vector_b, np.cross(vector_a, vector_b))


@pytest.mark.parametrize(
    "vector",
    [
        np.zeros(2, dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        np.zeros(4, dtype=np.float64),
    ],
)
def test_skew_symmetric_rejects_invalid_shape(vector: NDArray[np.float64]) -> None:
    with pytest.raises(ValueError, match=r"vector must have shape \(3,\)"):
        skew_symmetric(vector)
