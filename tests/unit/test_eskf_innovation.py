"""Independent NIS mathematics, statistical reference and boundary contracts."""

from dataclasses import FrozenInstanceError, replace
from math import erf, exp, pi, sqrt

import numpy as np
import pytest

from quadrotor_math.eskf import EskfNominalState, update_eskf_linear_measurement
from quadrotor_math.eskf_innovation import (
    EskfInnovation,
    EskfInnovationPolicy,
    chi_square_99_percent_eskf_innovation_policy,
    compute_eskf_linear_innovation,
)


def _arguments(m: int = 3) -> dict:
    return {
        "covariance": np.eye(15),
        "measurement": np.arange(1, m + 1, dtype=float),
        "predicted_measurement": np.zeros(m),
        "measurement_jacobian": np.eye(m, 15),
        "measurement_noise_covariance": np.eye(m),
    }


@pytest.mark.parametrize("m", [1, 3, 5, 17])
def test_diagonal_analytic_innovation(m: int) -> None:
    arguments = _arguments(m)
    result = compute_eskf_linear_innovation(**arguments)
    diagonal = np.ones(m)
    diagonal[: min(m, 15)] += 1
    residual = arguments["measurement"]
    np.testing.assert_array_equal(result.innovation, residual)
    np.testing.assert_array_equal(result.innovation_covariance, np.diag(diagonal))
    np.testing.assert_allclose(result.whitened_innovation, residual / np.sqrt(diagonal))
    assert result.normalized_innovation_squared == pytest.approx(np.sum(residual**2 / diagonal))


@pytest.mark.parametrize("seed", range(12))
@pytest.mark.parametrize("m", [1, 3, 7])
def test_correlated_innovation_matches_independent_solve_and_update(seed: int, m: int) -> None:
    rng = np.random.default_rng(seed)
    A, B = rng.normal(size=(15, 15)), rng.normal(size=(m, m))
    P, R = A @ A.T, B @ B.T + np.eye(m) * 0.3
    H, z, h = rng.normal(size=(m, 15)), rng.normal(size=m), rng.normal(size=m)
    result = compute_eskf_linear_innovation(P, z, h, H, R)
    S, r = H @ P @ H.T + R, z - h
    assert result.normalized_innovation_squared == pytest.approx(r @ np.linalg.solve(S, r))
    np.testing.assert_allclose(np.linalg.cholesky(S) @ result.whitened_innovation, r)
    state = EskfNominalState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )
    update = update_eskf_linear_measurement(state, P, z, h, H, R)
    assert result.innovation.tobytes() == update.innovation.tobytes()
    assert result.innovation_covariance.tobytes() == update.innovation_covariance.tobytes()


@pytest.mark.parametrize(
    "transform",
    [
        np.diag([1e-100, 1e100, -3]),
        np.eye(3)[[2, 0, 1]],
        np.array([[0.6, -0.8, 0], [0.8, 0.6, 0], [0, 0, 1]]),
    ],
)
def test_nis_invariant_under_invertible_measurement_coordinates(transform) -> None:
    r = np.array([1.0, -2.0, 3.0])
    S = np.array([[3.0, 1.0, 0.2], [1.0, 4.0, -0.3], [0.2, -0.3, 2.0]])
    reference = EskfInnovation(r, S)
    actual = EskfInnovation(transform @ r, transform @ S @ transform.T)
    assert actual.normalized_innovation_squared == pytest.approx(
        reference.normalized_innovation_squared
    )


@pytest.mark.parametrize("scale", [np.nextafter(0.0, 1.0), 1e-300, 1.0, 1e300, 1e308])
def test_extreme_variance_is_scaled_before_whitening(scale: float) -> None:
    result = EskfInnovation(np.array([sqrt(scale)]), np.array([[scale]]))
    assert result.normalized_innovation_squared == pytest.approx(1.0)


def test_squared_norm_retains_representable_sum_of_underflowing_squares() -> None:
    result = EskfInnovation(np.full(4, 1e-162), np.eye(4))
    assert result.normalized_innovation_squared == np.nextafter(0.0, 1.0)


def test_zero_innovation_and_zero_prior_are_valid_with_positive_noise() -> None:
    arguments = _arguments()
    arguments.update(covariance=np.zeros((15, 15)), measurement=np.zeros(3))
    result = compute_eskf_linear_innovation(**arguments)
    assert result.normalized_innovation_squared == 0.0
    np.testing.assert_array_equal(result.whitened_innovation, np.zeros(3))


def test_output_is_frozen_owned_read_only_and_derived_fields_cannot_be_supplied() -> None:
    r, S = np.arange(3, dtype=float), np.eye(3)
    result = EskfInnovation(r, S)
    duplicate = replace(result)
    r[:] = 100
    S[:] = 100
    np.testing.assert_array_equal(result.innovation, [0, 1, 2])
    assert result.normalized_innovation_squared == pytest.approx(5)
    arrays = (result.innovation, result.innovation_covariance, result.whitened_innovation)
    for name, values in zip(
        ("innovation", "innovation_covariance", "whitened_innovation"), arrays, strict=True
    ):
        assert values.dtype == np.float64 and values.flags.c_contiguous and values.flags.owndata
        assert not values.flags.writeable
        assert not np.shares_memory(values, getattr(duplicate, name))
        with pytest.raises(ValueError):
            values.flat[0] = 9
    assert not np.shares_memory(result.innovation, result.whitened_innovation)
    with pytest.raises(FrozenInstanceError):
        result.normalized_innovation_squared = 4
    with pytest.raises(TypeError):
        EskfInnovation(np.ones(1), np.eye(1), normalized_innovation_squared=0)


@pytest.mark.parametrize("name", list(_arguments()))
@pytest.mark.parametrize("bad", [None, [], np.array(1.0), np.array([]), np.ones((2, 2, 1))])
def test_input_shape_contract(name: str, bad) -> None:
    arguments = _arguments()
    arguments[name] = bad
    with pytest.raises(ValueError, match=name):
        compute_eskf_linear_innovation(**arguments)


@pytest.mark.parametrize("name", list(_arguments()))
@pytest.mark.parametrize("dtype", [bool, complex, str, object])
def test_only_real_numeric_arrays_are_accepted(name: str, dtype) -> None:
    arguments = _arguments()
    arguments[name] = arguments[name].astype(dtype)
    with pytest.raises(ValueError, match=name):
        compute_eskf_linear_innovation(**arguments)


@pytest.mark.parametrize("name", list(_arguments()))
@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_inputs_must_be_finite(name: str, bad: float) -> None:
    arguments = _arguments()
    arguments[name].flat[0] = bad
    with pytest.raises(ValueError, match=name):
        compute_eskf_linear_innovation(**arguments)


@pytest.mark.parametrize("name", ["covariance", "measurement_noise_covariance"])
@pytest.mark.parametrize("defect", ["negative", "asymmetric", "indefinite", "zero_coupling"])
def test_covariance_domain_is_checked_even_for_zero_residual(name: str, defect: str) -> None:
    arguments = _arguments()
    arguments["measurement"][:] = 0
    matrix = arguments[name]
    if defect == "negative":
        matrix[0, 0] = -np.nextafter(0.0, 1.0)
    elif defect == "asymmetric":
        matrix[0, 1] = 1e-20
    elif defect == "indefinite":
        matrix[0, 1] = matrix[1, 0] = 2
    else:
        matrix[0, 0] = 0
        matrix[0, 1] = matrix[1, 0] = 1e-20
    with pytest.raises(ValueError, match=name):
        compute_eskf_linear_innovation(**arguments)


@pytest.mark.parametrize("residual", [0.0, 1.0, 1e200])
def test_singular_innovation_is_an_error_regardless_of_residual(residual: float) -> None:
    with pytest.raises(ValueError, match="positive definite"):
        EskfInnovation(np.full(2, residual), np.ones((2, 2)))


@pytest.mark.parametrize(
    "r,S",
    [
        (np.array([1e308]), np.array([[1e-308]])),
        (np.array([1e200]), np.eye(1)),
        (np.array([np.inf]), np.eye(1)),
        (np.ones(1), np.array([[np.inf]])),
    ],
)
def test_nonfinite_whitening_or_nis_is_an_error(r, S) -> None:
    with pytest.raises(ValueError):
        EskfInnovation(r, S)


@pytest.mark.parametrize("defect", ["residual", "cross", "innovation_covariance"])
def test_finite_inputs_that_overflow_raise_without_mutation(defect: str) -> None:
    arguments = _arguments()
    if defect == "residual":
        arguments["measurement"][:] = 1e308
        arguments["predicted_measurement"][:] = -1e308
    elif defect == "cross":
        arguments["covariance"] *= 1e308
        arguments["measurement_jacobian"] *= 2
    else:
        arguments["measurement_jacobian"] *= 1e200
    before = {name: value.tobytes() for name, value in arguments.items()}
    with pytest.raises(ValueError, match="finite"):
        compute_eskf_linear_innovation(**arguments)
    assert before == {name: value.tobytes() for name, value in arguments.items()}


@pytest.mark.parametrize(
    "name", ["local_position_nis_threshold", "barometric_altitude_nis_threshold"]
)
@pytest.mark.parametrize(
    "bad",
    [True, np.bool_(False), "4", 2j, [], np.array(3), 0, -1, np.nan, np.inf, -np.inf, 10**400],
)
def test_policy_requires_positive_finite_non_boolean_threshold(name: str, bad) -> None:
    with pytest.raises(ValueError, match=name):
        EskfInnovationPolicy(**{name: bad})


@pytest.mark.parametrize("threshold", [None, 1, np.float64(3), np.nextafter(0.0, 1.0), 1e308])
def test_explicit_policy_accepts_none_or_positive_scalar(threshold) -> None:
    policy = EskfInnovationPolicy(threshold, threshold)
    assert policy.local_position_nis_threshold == threshold
    assert policy.barometric_altitude_nis_threshold == threshold
    if threshold is not None:
        assert type(policy.local_position_nis_threshold) is float
    with pytest.raises(FrozenInstanceError):
        policy.local_position_nis_threshold = 5


def test_rounded_nist_preset_has_99_percent_marginal_probability() -> None:
    policy = chi_square_99_percent_eskf_innovation_policy()
    assert policy.local_position_nis_threshold == 11.345
    assert policy.barometric_altitude_nis_threshold == 6.635
    for dimensions, critical in [(1, 6.635), (3, 11.345)]:
        cdf = erf(sqrt(critical / 2))
        if dimensions == 3:
            cdf -= sqrt(2 * critical / pi) * exp(-critical / 2)
        assert abs(cdf - 0.99) < 3e-6  # Three-decimal table rounding, not exact quantiles.


@pytest.mark.parametrize("m,seed,threshold", [(1, 6021, 6.635), (3, 6023, 11.345)])
def test_seeded_gaussian_innovations_match_predeclared_statistical_bounds(
    m, seed, threshold
) -> None:
    # Statistic-only reference. This does not test nonlinear ESKF consistency.
    rng = np.random.default_rng(seed)
    factor = np.tril(np.full((m, m), 0.2)) + np.diag(np.arange(1, m + 1))
    covariance = factor @ factor.T
    standard = rng.normal(size=(4096, m))
    results = [EskfInnovation(factor @ sample, covariance) for sample in standard]
    whitened = np.array([result.whitened_innovation for result in results])
    nis = np.array([result.normalized_innovation_squared for result in results])
    np.testing.assert_allclose(whitened, standard, rtol=1e-11, atol=2e-15)
    assert np.max(np.abs(whitened.mean(axis=0))) < 0.06
    assert abs(nis.mean() - m) < 0.18
    assert abs(nis.var() - 2 * m) < 0.9
    assert 0.98 < np.mean(nis <= threshold) < 0.998
