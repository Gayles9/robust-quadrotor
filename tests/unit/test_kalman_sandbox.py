"""Hand-computed linear cases and the reproducible learning example boundary."""

import numpy as np
import pytest

from experiments.kalman_sandbox import (
    main,
    position_velocity_prediction,
    sandbox_examples,
    scalar_kalman_update,
)


def test_scalar_hand_calculation_and_contraction():
    mean, variance, gain = scalar_kalman_update(0, 4, 2, 1)
    assert mean == pytest.approx(1.6)
    assert variance == pytest.approx(0.8)
    assert gain == 0.8
    mean, variance, gain = scalar_kalman_update(mean, variance, 2, 1)
    assert mean == pytest.approx(16 / 9)
    assert variance == pytest.approx(4 / 9)
    assert variance < 0.8


def test_linear_prediction_hand_calculated_cross_covariance_and_noise():
    state, P = position_velocity_prediction(
        np.array([1.0, 2.0]), np.diag([4.0, 1.0]), 3.0, 2.0, 0.5
    )
    np.testing.assert_allclose(state, [2.375, 3.5])
    np.testing.assert_allclose(P, [[4.28125, 0.625], [0.625, 1.5]])
    assert np.linalg.eigvalsh(P).min() > 0


@pytest.mark.parametrize(
    "args",
    [
        (0, -1, 2, 1),
        (0, 1, 2, -1),
        (0, 0, 2, 0),
        (np.inf, 1, 2, 1),
        (0, 1, np.nan, 1),
        (True, 1, 2, 1),
    ],
)
def test_scalar_invalid_inputs(args):
    with pytest.raises(ValueError):
        scalar_kalman_update(*args)


def test_examples_are_repeatable_and_plot_without_a_display(tmp_path):
    report = sandbox_examples()
    assert report == sandbox_examples()
    assert len(report["position_velocity"]) == 5
    assert main(["--output", str(tmp_path / "sandbox")]) == 0
    assert (tmp_path / "sandbox" / "kalman_examples.png").read_bytes().startswith(b"\x89PNG")
    with pytest.raises(FileExistsError):
        main(["--output", str(tmp_path / "sandbox")])


def test_zero_interval_returns_independent_values_and_bad_covariance_fails():
    state = np.array([1.0, 2.0])
    covariance = np.eye(2)
    actual, P = position_velocity_prediction(state, covariance, 1.0, 1.0, 0.0)
    np.testing.assert_array_equal(actual, state)
    np.testing.assert_array_equal(P, covariance)
    assert not np.shares_memory(actual, state)
    assert not np.shares_memory(P, covariance)
    with pytest.raises(ValueError):
        position_velocity_prediction(state, np.array([[1.0, 2.0], [2.0, 1.0]]), 0.0, 0.0, 1.0)
