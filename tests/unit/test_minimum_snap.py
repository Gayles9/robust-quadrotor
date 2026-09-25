"""Independent analytic, coefficient-space and invariance checks for ADR 0015."""

from math import factorial

import numpy as np
import pytest

from quadrotor_math.minimum_snap import minimum_snap_trajectory


def basis(s, derivative=0):
    return np.array(
        [
            0.0
            if k < derivative
            else factorial(k) / factorial(k - derivative) * s ** (k - derivative)
            for k in range(8)
        ]
    )


def coefficient_oracle(points, durations, start, end):
    """Independent full coefficient-space nullspace solve; no Hermite elimination."""
    n = len(durations)
    constraints, targets = [], []

    def row(i, side, derivative):
        value = np.zeros(8 * n)
        value[8 * i : 8 * i + 8] = basis(side, derivative) / durations[i] ** derivative
        return value

    for i in range(n):
        for side in (0, 1):
            constraints.append(row(i, side, 0))
            targets.append(points[i + side])
    for derivative in range(1, 4):
        constraints.extend((row(0, 0, derivative), row(n - 1, 1, derivative)))
        targets.extend((start[derivative - 1], end[derivative - 1]))
        for i in range(n - 1):
            constraints.append(row(i, 1, derivative) - row(i + 1, 0, derivative))
            targets.append(np.zeros(3))
    a, b = np.array(constraints), np.array(targets)
    particular = np.linalg.lstsq(a, b, rcond=None)[0]
    _, singular, vt = np.linalg.svd(a, full_matrices=True)
    assert singular[-1] > 1e-7
    null = vt[len(a) :].T
    hessian = np.zeros((8 * n, 8 * n))
    for i, duration in enumerate(durations):
        for k in range(4, 8):
            for power in range(4, 8):
                hessian[8 * i + k, 8 * i + power] = (
                    factorial(k)
                    / factorial(k - 4)
                    * factorial(power)
                    / factorial(power - 4)
                    / (k + power - 7)
                    / duration**7
                )
    if null.shape[1]:
        particular += null @ np.linalg.solve(
            null.T @ hessian @ null, -null.T @ hessian @ particular
        )
    return particular.reshape(n, 8, 3), null, hessian


def test_single_segment_exact_polynomial_and_cost():
    start = np.array([2.0, -1, 3])
    delta = np.array([1.0, 2, -3])
    trajectory = minimum_snap_trajectory(np.array([start, start + delta]), np.array([2.0]))
    smooth = np.array([0.0, 0, 0, 0, 35, -84, 70, -20])
    for t in (0.0, 0.25, 0.7, 1.0, 1.75, 2.0):
        for r in range(5):
            expected = (basis(t / 2, r) @ smooth) * delta / 2**r
            if r == 0:
                expected += start
            np.testing.assert_allclose(trajectory.evaluate(t, r), expected, atol=2e-10)
    assert trajectory.snap_cost == pytest.approx(100800 * (delta @ delta) / 2**7, rel=2e-12)


def test_nonuniform_constraints_oracle_optimality_and_quadrature():
    points = np.array([[0.0, 0, 0], [1.0, -2, -1], [2.0, 1, -2], [4.0, 0, -1]])
    durations = np.array([1.0, 1.7, 0.9])
    start = np.array([[0.2, 0.1, -0.1], [0.3, 0, 0.2], [0, -0.2, 0.1]])
    end = np.array([[-0.1, 0, 0.2], [0.1, 0.2, 0], [0.2, 0, -0.1]])
    trajectory = minimum_snap_trajectory(points, durations, start, end)
    coefficients, null, hessian = coefficient_oracle(points, durations, start, end)
    physical_coefficients = trajectory.coefficients_W.copy()
    physical_coefficients[:, 0] += trajectory.position_origin_W
    np.testing.assert_allclose(physical_coefficients, coefficients, atol=2e-8, rtol=2e-9)
    for i, t in enumerate(trajectory.knot_times_s):
        np.testing.assert_allclose(trajectory.evaluate(float(t)), points[i], atol=2e-10)
    for r in range(1, 4):
        np.testing.assert_allclose(trajectory.evaluate(0.0, r), start[r - 1], atol=2e-10)
        np.testing.assert_allclose(
            trajectory.evaluate(float(sum(durations)), r), end[r - 1], atol=2e-9
        )
        for i in range(1, len(durations)):
            left = basis(1.0, r) @ trajectory.coefficients_W[i - 1] / durations[i - 1] ** r
            right = basis(0.0, r) @ trajectory.coefficients_W[i] / durations[i] ** r
            np.testing.assert_allclose(left, right, atol=3e-9)
    nodes, weights = np.polynomial.legendre.leggauss(8)
    cost = 0.0
    for i, duration in enumerate(durations):
        snap = np.array(
            [basis(float(s), 4) @ physical_coefficients[i] / duration**4 for s in (nodes + 1) / 2]
        )
        cost += duration / 2 * float(weights @ np.sum(snap**2, axis=1))
    assert trajectory.snap_cost == pytest.approx(cost, rel=3e-11)
    flat = physical_coefficients.reshape(-1, 3)
    direction = null @ np.random.default_rng(27).normal(size=(null.shape[1], 3))
    optimum = float(np.sum(flat * (hessian @ flat)))
    for scale in (-1.0, -0.1, 0.1, 1.0):
        candidate = flat + scale * direction
        candidate_cost = float(np.sum(candidate * (hessian @ candidate)))
        assert candidate_cost > optimum


def test_improves_on_feasible_stop_at_every_waypoint():
    points = np.array([[0.0, 0, 0], [1.0, 1, -1], [3.0, 0, -1], [4.0, -1, 0]])
    durations = np.array([1.0, 2.0, 1.5])
    result = minimum_snap_trajectory(points, durations)
    stopped_cost = sum(
        100800 * np.sum(d**2) / t**7
        for d, t in zip(np.diff(points, axis=0), durations, strict=True)
    )
    assert 0 < result.snap_cost < 0.8 * stopped_cost


def test_time_rotation_translation_and_length_scaling():
    points = np.array([[0.0, 0, 0], [1.0, 2, -1], [3.0, -1, 2]])
    durations = np.array([0.8, 1.4])
    start = np.array([[0.2, -0.1, 0], [0, 0.2, 0.1], [0.1, 0, 0.2]])
    end = -start
    original = minimum_snap_trajectory(points, durations, start, end)
    rotation = np.array([[0.0, -1, 0], [1.0, 0, 0], [0.0, 0, 1]])
    shifted = minimum_snap_trajectory(
        points @ rotation.T + [100, 200, -50], durations, start @ rotation.T, end @ rotation.T
    )
    stretched = minimum_snap_trajectory(
        points,
        durations * 3,
        start / 3.0 ** np.arange(1, 4)[:, None],
        end / 3.0 ** np.arange(1, 4)[:, None],
    )
    doubled = minimum_snap_trajectory(points * 2, durations, start * 2, end * 2)
    for time in np.linspace(0, sum(durations), 13):
        stretched_time = (
            float(stretched.knot_times_s[-1])
            if time == original.knot_times_s[-1]
            else float(time * 3)
        )
        for r in range(5):
            np.testing.assert_allclose(
                stretched.evaluate(stretched_time, r),
                original.evaluate(float(time), r) / 3**r,
                atol=2e-9,
            )
            expected = original.evaluate(float(time), r) @ rotation.T
            if r == 0:
                expected += [100, 200, -50]
            np.testing.assert_allclose(shifted.evaluate(float(time), r), expected, atol=2e-9)
    assert stretched.snap_cost == pytest.approx(original.snap_cost / 3**7, rel=2e-11)
    assert shifted.snap_cost == pytest.approx(original.snap_cost, rel=2e-11)
    assert doubled.snap_cost == pytest.approx(4 * original.snap_cost, rel=2e-11)


def test_constant_curve_and_owned_outputs_and_reference():
    points = np.tile([3.0, -2.0, -1.0], (4, 1))
    durations = np.array([1.0, 2.0, 1.0])
    result = minimum_snap_trajectory(points, durations)
    points[:] = 0
    durations[:] = 100
    assert result.snap_cost == 0.0
    np.testing.assert_array_equal(result.evaluate(2.0), [3.0, -2.0, -1.0])
    for r in range(1, 5):
        np.testing.assert_array_equal(result.evaluate(2.0, r), 0)
    for value in (
        result.coefficients_W,
        result.position_origin_W,
        result.segment_durations_s,
        result.knot_times_s,
        result.evaluate(2.0),
    ):
        assert not value.flags.writeable
    ref = result.position_reference(2.0, 0.3)
    np.testing.assert_array_equal(ref.position_W, [3.0, -2.0, -1.0])
    assert ref.yaw_rad == 0.3


@pytest.mark.parametrize(
    "durations",
    [
        np.array([0.0]),
        np.array([-1.0]),
        np.array([np.nan]),
        np.array([np.inf]),
        np.array([True]),
        np.array([1.0 + 1j]),
        np.array([[1.0]]),
        np.array([1.0, 2.0]),
    ],
)
def test_invalid_durations(durations):
    with pytest.raises(ValueError):
        minimum_snap_trajectory(np.zeros((2, 3)), durations)


@pytest.mark.parametrize(
    "points",
    [
        np.zeros((1, 3)),
        np.zeros((2, 2)),
        np.ones((2, 3), dtype=bool),
        np.full((2, 3), np.inf),
        np.full((2, 3), 1j),
    ],
)
def test_invalid_waypoints(points):
    with pytest.raises(ValueError):
        minimum_snap_trajectory(points, np.ones(1))


@pytest.mark.parametrize(
    "boundary", [np.zeros(3), np.full((3, 3), np.nan), np.ones((3, 3), dtype=bool)]
)
def test_invalid_boundary_derivatives(boundary):
    with pytest.raises(ValueError):
        minimum_snap_trajectory(np.zeros((2, 3)), np.ones(1), boundary)


@pytest.mark.parametrize("time", [-1.0, 1.001, np.nan, np.inf, True, "0"])
def test_no_silent_extrapolation_or_invalid_time(time):
    result = minimum_snap_trajectory(np.zeros((2, 3)), np.ones(1))
    with pytest.raises(ValueError):
        result.evaluate(time)


@pytest.mark.parametrize("order", [-1, 5, True, 1.5])
def test_invalid_derivative_order(order):
    result = minimum_snap_trajectory(np.zeros((2, 3)), np.ones(1))
    with pytest.raises(ValueError):
        result.evaluate(0.5, order)


def test_unresolved_problem_and_overflow_fail_explicitly():
    with pytest.raises(ValueError, match="duration|condition|resolv"):
        minimum_snap_trajectory(
            np.array([[0.0, 0, 0], [1.0, 0, 0], [2.0, 0, 0]]), np.array([1e-12, 1.0])
        )
    with pytest.raises(ValueError):
        minimum_snap_trajectory(np.array([[1e308, 0, 0], [-1e308, 0, 0]]), np.ones(1))
    with pytest.raises(ValueError):
        minimum_snap_trajectory(np.array([[0.0, 0, 0], [1.0, 0, 0]]), np.array([1e-100]))


def test_rejects_endpoint_cancellation_despite_well_scaled_reduced_system():
    # The reduced system condition is only ~17, but normalized coefficients
    # approach 1e19 and endpoint subtraction loses metres of precision.
    with pytest.raises(ValueError, match="boundary residual"):
        minimum_snap_trajectory(
            np.array([[0.0, 0, 0], [1.0, -1, 0], [2.0, 2, 1]]), np.array([1.0, 1e6])
        )


def test_recovers_global_cubic_with_zero_snap_and_nonzero_endpoint_derivatives():
    coefficients = np.array([[1.0, 2, -1], [0.3, -0.2, 0.1], [0.2, 0.1, -0.3], [-0.1, 0.2, 0.1]])
    knots = np.array([0.0, 0.8, 2.0, 3.0])

    def derivative(t, order):
        return sum(
            factorial(k) / factorial(k - order) * coefficients[k] * t ** (k - order)
            for k in range(order, 4)
        )

    points = np.array([derivative(t, 0) for t in knots])
    start = np.array([derivative(0.0, r) for r in range(1, 4)])
    end = np.array([derivative(3.0, r) for r in range(1, 4)])
    result = minimum_snap_trajectory(points, np.diff(knots), start, end)
    for t in np.linspace(0, 3, 17):
        for r in range(4):
            np.testing.assert_allclose(result.evaluate(float(t), r), derivative(t, r), atol=1e-10)
    assert result.snap_cost < 1e-19


def test_maximum_segment_budget_and_natural_continuity():
    points = np.column_stack((np.linspace(0, 5, 65), np.sin(np.linspace(0, 3, 65)), np.zeros(65)))
    result = minimum_snap_trajectory(points, np.ones(64))
    assert result.scaled_system_condition < 100
    for i in range(1, 64):
        # Free interior v/a/j imply continuous snap and its first two
        # derivatives at the optimum, independent of the assembled constraints.
        for r in range(4, 7):
            np.testing.assert_allclose(
                basis(1.0, r) @ result.coefficients_W[i - 1],
                basis(0.0, r) @ result.coefficients_W[i],
                atol=2e-8,
            )
    with pytest.raises(ValueError, match="64"):
        minimum_snap_trajectory(np.zeros((66, 3)), np.ones(65))
