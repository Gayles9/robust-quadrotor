"""Independent bounds, time scaling and failure contracts for reference planning."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.position_control_validation import position_parameters
from quadrotor_math.minimum_snap import MinimumSnapTrajectory, minimum_snap_trajectory
from quadrotor_math.position_control import compute_position_control
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.trajectory_feasibility import (
    TrajectoryLimits,
    check_trajectory_feasibility,
    retime_trajectory,
    validate_trajectory_continuity,
)


def limits(**changes):
    return replace(
        TrajectoryLimits(
            np.full(3, -3.0),
            np.full(3, 3.0),
            0.6,
            np.full(3, 0.6),
            1.0,
            9.81,
            7.0,
            13.0,
            np.deg2rad(10),
            0.4,
        ),
        **changes,
    )


def trajectory():
    return minimum_snap_trajectory(
        np.array([[0.0, 0, -1], [1.0, 0, -1.2], [1.0, 1, -0.8], [0.0, 1, -1], [0.0, 0, -1]]),
        np.ones(4),
    )


def test_exact_hover_and_analytic_single_segment_speed():
    hold = minimum_snap_trajectory(np.zeros((2, 3)), np.ones(1))
    report = check_trajectory_feasibility(hold, limits())
    assert report.accepted and report.violations == ()
    assert report.minimum_thrust_N <= 9.81 <= report.maximum_thrust_N
    assert report.maximum_speed_m_s < 1e-10
    single = minimum_snap_trajectory(np.array([[0.0, 0, 0], [1.0, 0, 0]]), np.array([5.0]))
    report = check_trajectory_feasibility(single, limits())
    # Derivative of 35s^4-84s^5+70s^6-20s^7 peaks at s=1/2: 35/16.
    assert 35 / 16 / 5 <= report.maximum_speed_m_s < 0.44
    assert report.accepted


def test_between_endpoint_excursion_is_rejected_without_sampling():
    coefficients = np.zeros((1, 8, 3))
    coefficients[0, :, 0] = [0, 0, 0, 64, -192, 192, -64, 0]
    curve = MinimumSnapTrajectory(np.array([20.0]), coefficients, np.zeros(3), 1.0, 0.0)
    np.testing.assert_allclose(curve.evaluate(0), curve.evaluate(20), atol=1e-12)
    assert curve.evaluate(10)[0] == 1.0
    report = check_trajectory_feasibility(curve, limits(maximum_position_W=np.full(3, 0.9)))
    assert not report.accepted and "geofence" in report.violations
    assert report.maximum_position_W[0] >= 1


def test_retiming_records_attempts_preserves_geometry_and_derivative_laws():
    curve = trajectory()
    result = retime_trajectory(curve, limits())
    assert result.trajectory is not None and result.attempts[-1].feasibility.accepted
    assert not result.attempts[0].feasibility.accepted
    assert result.attempts[0].scale == 1
    assert all(
        b.scale > a.scale for a, b in zip(result.attempts[:-1], result.attempts[1:], strict=True)
    )
    assert 1 < result.scale <= 8 and len(result.attempts) <= 12
    np.testing.assert_array_equal(curve.coefficients_W, result.trajectory.coefficients_W)
    for t in np.linspace(0, 3.99, 25):
        for r in range(5):
            np.testing.assert_allclose(
                result.trajectory.evaluate(float(t * result.scale), r),
                curve.evaluate(float(t), r) / result.scale**r,
                atol=2e-10,
            )
    assert result.trajectory.snap_cost == pytest.approx(curve.snap_cost / result.scale**7)
    assert not np.shares_memory(curve.coefficients_W, result.trajectory.coefficients_W)


def test_dense_independent_values_and_attitude_rates_fit_whole_curve_bounds():
    result = retime_trajectory(trajectory(), limits())
    curve = result.trajectory
    assert curve is not None
    report = result.attempts[-1].feasibility
    params = position_parameters()
    dt = 1e-5
    for t in np.linspace(dt, curve.knot_times_s[-1] - dt, 161):
        values = [curve.evaluate(float(t), r) for r in range(4)]
        p, v, a, _ = values
        assert np.all(p >= report.minimum_position_W) and np.all(p <= report.maximum_position_W)
        assert np.linalg.norm(v) <= report.maximum_speed_m_s
        assert np.all(np.abs(a) <= report.maximum_acceleration_W)
        lift = np.array([0.0, 0, 9.81]) - a
        assert report.minimum_thrust_N <= np.linalg.norm(lift) <= report.maximum_thrust_N
        assert np.arctan2(np.linalg.norm(lift[:2]), lift[2]) <= report.maximum_tilt_rad
        for yaw in (0.0, 0.7, 2.0):
            rotations = []
            for time in (t - dt, t + dt):
                ref = curve.position_reference(float(time), yaw)
                command = compute_position_control(ref.position_W, ref.velocity_W, ref, params)
                rotations.append(rotation_matrix_body_to_world(command.q_reference_WB))
            # ||Rdot||_F / sqrt(2) = ||omega||; independently differentiate rotations.
            rate = np.linalg.norm((rotations[1] - rotations[0]) / (2 * dt), ord="fro") / np.sqrt(2)
            assert rate <= report.maximum_reference_rate_rad_s + 1e-8


def test_budget_failure_and_geofence_do_not_claim_feasible_output():
    failed = retime_trajectory(trajectory(), limits(), maximum_attempts=1)
    assert failed.trajectory is None and len(failed.attempts) == 1
    assert failed.reason == "search_budget_exhausted"
    failed = retime_trajectory(trajectory(), limits(), maximum_scale=1.1)
    assert failed.trajectory is None and failed.attempts[-1].scale == 1.1
    failed = retime_trajectory(trajectory(), limits(maximum_position_W=np.full(3, 0.1)))
    assert failed.trajectory is None and len(failed.attempts) == 1
    assert failed.reason == "geofence_not_certified"


def test_nonrest_and_discontinuous_constructed_curves_are_not_silently_accepted():
    curve = trajectory()
    altered = curve.coefficients_W.copy()
    altered[1, 0, 0] += 0.1
    broken = replace(curve, coefficients_W=altered)
    with pytest.raises(ValueError, match="C3"):
        check_trajectory_feasibility(broken, limits())
    d = np.zeros((3, 3))
    d[0, 0] = 0.1
    moving = minimum_snap_trajectory(np.zeros((2, 3)), np.ones(1), d)
    with pytest.raises(ValueError, match="rest"):
        retime_trajectory(moving, limits())
    validate_trajectory_continuity(moving)


@pytest.mark.parametrize(
    "changes",
    [
        {"maximum_speed_m_s": 0},
        {"maximum_speed_m_s": True},
        {"maximum_acceleration_W": np.zeros(3)},
        {"nominal_mass": np.inf},
        {"nominal_gravity_m_s2": 0},
        {"minimum_thrust_N": 12},
        {"maximum_thrust_N": 8},
        {"maximum_tilt_rad": np.pi / 2},
        {"maximum_reference_rate_rad_s": -1},
        {"minimum_position_W": np.full(3, 4.0)},
    ],
)
def test_limit_validation(changes):
    with pytest.raises(ValueError):
        limits(**changes)


@pytest.mark.parametrize(
    "changes",
    [
        {"maximum_scale": 0.9},
        {"maximum_scale": np.inf},
        {"scale_step": 1},
        {"scale_step": True},
        {"maximum_attempts": 0},
        {"maximum_attempts": True},
        {"subdivision_depth": -1},
        {"subdivision_depth": 9},
    ],
)
def test_search_validation(changes):
    with pytest.raises(ValueError):
        retime_trajectory(trajectory(), limits(), **changes)


def test_result_and_limit_arrays_own_readonly_memory():
    box = np.full(3, -3.0)
    model = limits(minimum_position_W=box)
    box[:] = 0
    report = check_trajectory_feasibility(trajectory(), model)
    for value in (
        model.minimum_position_W,
        model.maximum_position_W,
        model.maximum_acceleration_W,
        report.minimum_position_W,
        report.maximum_position_W,
        report.maximum_acceleration_W,
    ):
        assert value.flags.owndata and not value.flags.writeable
    np.testing.assert_array_equal(model.minimum_position_W, -3)


def test_nonpositive_vertical_lift_has_explicit_undefined_rate_bound():
    c = np.zeros((1, 8, 3))
    c[0, 2, 2] = 10.0
    curve = MinimumSnapTrajectory(np.ones(1), c, np.zeros(3), 1.0, 0.0)
    report = check_trajectory_feasibility(curve, limits())
    assert not report.accepted and "positive_down_thrust" in report.violations
    assert report.maximum_reference_rate_rad_s is None
