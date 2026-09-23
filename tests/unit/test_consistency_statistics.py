"""Statistical bounds checked against analytic and published critical values."""

from math import log

import numpy as np
import pytest

from quadrotor_math.consistency_statistics import (
    NormalizedErrorEnsemble,
    chi_square_central_95_interval,
)


def test_two_dof_quantiles_match_exact_exponential_distribution():
    np.testing.assert_allclose(
        chi_square_central_95_interval(2), [-2 * log(0.975), -2 * log(0.025)], rtol=3e-14
    )


@pytest.mark.parametrize(
    "degrees,expected",
    [
        (1500, [1394.5550305348847, 1609.23321785498]),
        (15000, [14662.42253867985, 15341.366036759584]),
    ],
)
def test_large_ensemble_bounds_match_independent_scipy_audit(degrees, expected):
    # SciPy 1.17.0 chi2.ppf([.025,.975], dof), independently evaluated in the
    # audit environment. SciPy is not a project/runtime/test dependency.
    np.testing.assert_allclose(chi_square_central_95_interval(degrees), expected, rtol=2e-11)


@pytest.mark.parametrize(
    "dof,lower,upper",
    [
        (1, 0.001, 5.024),
        (3, 0.216, 9.348),
        (15, 6.262, 27.488),
        (30, 16.791, 46.979),
        (100, 74.222, 129.561),
    ],
)
def test_intervals_match_nist_three_decimal_table(dof, lower, upper):
    # NIST/SEMATECH handbook, eda3674.htm, 0.025/0.975 columns.
    np.testing.assert_allclose(
        chi_square_central_95_interval(dof), [lower, upper], rtol=0, atol=0.00051
    )


@pytest.mark.parametrize("dof", [True, 0, -1, 1.0, np.int64(15), 15001, "15"])
def test_interval_rejects_out_of_domain_degrees_of_freedom(dof):
    with pytest.raises(ValueError):
        chi_square_central_95_interval(dof)


def test_ensemble_averages_across_seeds_not_correlated_epochs():
    samples = np.array([[0.0, 1.0, 100.0], [2.0, 3.0, 10.0]])
    result = NormalizedErrorEnsemble(samples, 1)
    np.testing.assert_allclose(result.mean_by_epoch, [1.0, 2.0, 55.0], rtol=3e-16)
    np.testing.assert_array_equal(result.coverage_by_epoch, [0.5, 1.0, 0.0])
    np.testing.assert_array_equal(
        result.mean_interval_95, np.array(chi_square_central_95_interval(2)) / 2
    )
    assert result.descriptive_coverage == 0.5
    assert result.descriptive_mean == pytest.approx(116 / 6)
    repeated = NormalizedErrorEnsemble(np.tile(samples, (1, 50)), 1)
    assert repeated.mean_interval_95 == result.mean_interval_95
    assert repeated.descriptive_coverage == result.descriptive_coverage
    assert repeated.descriptive_mean == pytest.approx(result.descriptive_mean, rel=3e-16)
    samples[:] = -1
    assert not np.any(result.samples < 0)
    for a in (result.samples, result.mean_by_epoch, result.coverage_by_epoch):
        assert a.flags.owndata and not a.flags.writeable


def test_boundary_coverage_is_inclusive_and_mean_avoids_overflow():
    lower, upper = chi_square_central_95_interval(3)
    result = NormalizedErrorEnsemble(np.array([[lower, upper], [lower, upper]]), 3)
    assert result.descriptive_coverage == 1
    huge = NormalizedErrorEnsemble(np.full((2, 2), 1e308), 15)
    assert huge.descriptive_mean == 1e308
    np.testing.assert_array_equal(huge.mean_by_epoch, [1e308, 1e308])


@pytest.mark.parametrize(
    "bad",
    [
        np.zeros(3),
        np.zeros((0, 2)),
        np.zeros((2, 0)),
        np.array([[-1.0]]),
        np.array([[np.nan]]),
        np.array([[np.inf]]),
        [[1.0]],
        np.ones((2, 3), dtype=bool),
        np.ones((2, 3), dtype=complex),
    ],
)
def test_ensemble_requires_nonempty_finite_nonnegative_real_seed_epoch_matrix(bad):
    with pytest.raises(ValueError):
        NormalizedErrorEnsemble(bad, 15)


def test_ensemble_rejects_unsupported_product_before_computing_bounds():
    with pytest.raises(ValueError):
        NormalizedErrorEnsemble(np.ones((1001, 1)), 15)


def test_seeded_ideal_gaussian_coverage_and_ensemble_mean():
    rng = np.random.default_rng(718)
    samples = np.sum(rng.normal(size=(100, 100, 3)) ** 2, axis=2)
    result = NormalizedErrorEnsemble(samples, 3)
    assert 0.94 < result.descriptive_coverage < 0.96
    lower, upper = result.mean_interval_95
    assert np.mean((result.mean_by_epoch >= lower) & (result.mean_by_epoch <= upper)) > 0.9
