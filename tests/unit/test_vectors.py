import numpy as np
import pytest

from quadrotor_math.vectors import squared_norm


def test_squared_norm_returns_sum_of_squared_components() -> None:
    vector = np.array([1.0, -2.0, 3.0], dtype=np.float64)

    assert squared_norm(vector) == pytest.approx(14.0)
