"""Independent physical and boundary oracles for ADR 0011."""

from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from quadrotor_math.actuation import (
    commanded_rotor_speeds_from_collective_thrust_and_body_moment,
    force_and_moment_body_from_rotor_speeds,
)
from quadrotor_math.attitude_control import (
    AttitudeControllerParameters,
    allocate_limited_body_moment,
    attitude_error_body,
    body_rate_moment,
    compute_attitude_control,
)
from quadrotor_math.dynamics import angular_acceleration_body_from_moment
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.run_configuration import RotorParameters


def rotors():
    return RotorParameters(
        np.array([[0.15, 0.15, 0], [0.15, -0.15, 0], [-0.15, -0.15, 0], [-0.15, 0.15, 0]]),
        np.array([1.0, -1, 1, -1]),
        1e-5,
        2e-7,
        0.0,
        900.0,
        0.025,
    )


def parameters():
    return AttitudeControllerParameters(
        np.array([3.0, 3, 2]),
        np.array([12.0, 12, 8]),
        np.array([2.0, 2, 1.5]),
        np.array([0.8, 0.8, 0.3]),
        np.pi / 2,
        np.diag([0.02, 0.025, 0.04]),
        rotors(),
    )


def quaternion(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.r_[np.cos(angle / 2), axis * np.sin(angle / 2)]


def forward(speeds, model):
    return force_and_moment_body_from_rotor_speeds(
        speeds,
        model.rotor_positions_B,
        model.rotor_spin_directions,
        model.thrust_coefficient,
        model.moment_coefficient,
    )


@pytest.mark.parametrize("axis", np.eye(3))
@pytest.mark.parametrize("sign", [-1.0, 1.0])
@pytest.mark.parametrize("q_signs", [(1, 1), (-1, 1), (1, -1), (-1, -1)])
def test_signed_axis_error_and_command(axis, sign, q_signs):
    reference = quaternion(axis, sign * 0.2)
    error = attitude_error_body(q_signs[0] * np.array([1.0, 0, 0, 0]), q_signs[1] * reference)
    np.testing.assert_allclose(error, axis * sign * 0.2, atol=1e-15)
    command = compute_attitude_control(
        np.array([1.0, 0, 0, 0]), np.zeros(3), reference, 9.81, parameters()
    )
    np.testing.assert_allclose(
        command.moment_requested_B,
        parameters().nominal_inertia_B
        @ (parameters().rate_gain_B * parameters().attitude_gain_B * error),
        atol=1e-15,
    )
    force, moment = forward(command.allocation.commanded_rotor_omega, rotors())
    np.testing.assert_allclose(force, [0, 0, -9.81], atol=5e-15)
    np.testing.assert_allclose(moment, command.moment_limited_B, atol=5e-16)


def test_nonidentity_error_is_in_current_body_axes():
    current = quaternion([1, 2, -1], 0.6)
    target = quaternion([-2, 1, 3], 0.3)
    relative = rotation_matrix_body_to_world(current).T @ rotation_matrix_body_to_world(target)
    angle = np.arccos((np.trace(relative) - 1) / 2)
    oracle = (
        angle
        / (2 * np.sin(angle))
        * np.array(
            [
                relative[2, 1] - relative[1, 2],
                relative[0, 2] - relative[2, 0],
                relative[1, 0] - relative[0, 1],
            ]
        )
    )
    np.testing.assert_allclose(attitude_error_body(current, target), oracle, atol=3e-16)


@pytest.mark.parametrize("angle", [0.0, 1e-14, 1e-8, 0.1, np.pi])
def test_zero_small_and_pi_rotations(angle):
    expected = np.array([angle, 0, 0])
    np.testing.assert_allclose(
        attitude_error_body(np.array([1.0, 0, 0, 0]), quaternion([1, 0, 0], angle)),
        expected,
        atol=1e-15,
    )


def test_full_inertia_gyroscopic_cancellation():
    inertia = np.array([[0.03, 0.002, -0.001], [0.002, 0.04, 0.003], [-0.001, 0.003, 0.05]])
    rate = np.array([0.4, -0.2, 0.3])
    desired = np.array([-0.1, 0.5, 0.8])
    gains = np.array([9.0, 10, 11])
    moment = body_rate_moment(rate, desired, inertia, gains)
    actual = angular_acceleration_body_from_moment(moment, rate, inertia)
    np.testing.assert_allclose(actual, gains * (desired - rate), atol=2e-15)


@pytest.mark.parametrize("moment", [np.zeros(3), np.array([0.1, -0.15, 0.03])])
def test_feasible_allocation_preserves_existing_allocator(moment):
    model = rotors()
    result = allocate_limited_body_moment(9.81, moment, model)
    expected = commanded_rotor_speeds_from_collective_thrust_and_body_moment(
        9.81,
        moment,
        model.rotor_positions_B,
        model.rotor_spin_directions,
        1e-5,
        2e-7,
        0,
        900,
    )
    assert result.moment_scale == 1
    np.testing.assert_array_equal(result.commanded_rotor_omega, expected)


@pytest.mark.parametrize("moment", [np.array([5.0, 0, 0]), np.array([-4.0, 3, -0.8])])
def test_saturated_allocation_preserves_collective_and_direction(moment):
    model = rotors()
    result = allocate_limited_body_moment(9.81, moment, model)
    A = np.array(
        [
            [1e-5] * 4,
            [-1.5e-6, 1.5e-6, 1.5e-6, -1.5e-6],
            [1.5e-6, 1.5e-6, -1.5e-6, -1.5e-6],
            [-2e-7, 2e-7, -2e-7, 2e-7],
        ]
    )
    base = np.linalg.solve(A, [9.81, 0, 0, 0])
    direction = np.linalg.solve(A, np.r_[0, moment])
    bounds = [
        (900**2 - b) / d if d > 0 else -b / d
        for b, d in zip(base, direction, strict=True)
        if d != 0
    ]
    assert result.moment_scale == pytest.approx(min(1, *bounds), abs=2e-14)
    assert 0 < result.moment_scale < 1
    force, actual = forward(result.commanded_rotor_omega, model)
    np.testing.assert_allclose(force, [0, 0, -9.81], atol=1e-14)
    np.testing.assert_allclose(actual, result.moment_scale * moment, atol=2e-15)
    assert np.all((result.commanded_rotor_omega >= 0) & (result.commanded_rotor_omega <= 900))


def test_rate_and_moment_clips_are_explicit():
    model = replace(
        parameters(), maximum_body_rate_B=np.full(3, 0.1), maximum_moment_B=np.full(3, 0.005)
    )
    output = compute_attitude_control(
        np.array([1.0, 0, 0, 0]),
        np.zeros(3),
        quaternion([1, 2, 3], 0.5),
        9.81,
        model,
    )
    assert output.rate_limited and output.moment_limited
    np.testing.assert_array_equal(output.desired_omega_B, [0.1, 0.1, 0.1])
    np.testing.assert_array_equal(output.moment_limited_B, [0.005, 0.005, 0.005])


def test_declared_local_domain_and_quaternion_validation():
    model = parameters()
    with pytest.raises(ValueError, match="domain"):
        compute_attitude_control(
            np.array([1.0, 0, 0, 0]), np.zeros(3), quaternion([1, 0, 0], 1.6), 9.81, model
        )
    with pytest.raises(ValueError, match="unit"):
        attitude_error_body(np.ones(4), np.array([1.0, 0, 0, 0]))


def test_ownership_and_frozen_parameters():
    model = parameters()
    array = np.ones(3)
    other = replace(model, attitude_gain_B=array)
    array[:] = 99
    np.testing.assert_array_equal(other.attitude_gain_B, np.ones(3))
    assert not other.attitude_gain_B.flags.writeable
    assert not np.shares_memory(other.nominal_inertia_B, model.nominal_inertia_B)
    with pytest.raises(FrozenInstanceError):
        other.maximum_attitude_error_rad = 1


@pytest.mark.parametrize(
    "field", ["attitude_gain_B", "rate_gain_B", "maximum_body_rate_B", "maximum_moment_B"]
)
@pytest.mark.parametrize(
    "bad",
    [
        np.zeros(3),
        -np.ones(3),
        np.full(3, np.nan),
        np.ones(2),
        np.ones(3, dtype=complex),
        np.ones(3, dtype=bool),
    ],
)
def test_parameter_vector_rejection(field, bad):
    with pytest.raises((ValueError, TypeError)):
        replace(parameters(), **{field: bad})


@pytest.mark.parametrize("bad", [0, -1, np.nan, np.inf, True, np.pi, 4])
def test_angle_domain_rejection(bad):
    with pytest.raises((ValueError, TypeError)):
        replace(parameters(), maximum_attitude_error_rad=bad)


@pytest.mark.parametrize("bad", [-1.0, np.nan, np.inf, True, 40.0])
def test_collective_is_never_silently_changed(bad):
    with pytest.raises((ValueError, TypeError)):
        allocate_limited_body_moment(bad, np.zeros(3), rotors())


def test_finite_arithmetic_overflow_rejects():
    with pytest.raises(ValueError):
        body_rate_moment(np.full(3, 1e308), -np.ones(3), np.eye(3), np.ones(3))


@pytest.mark.parametrize(
    "inertia",
    [
        np.zeros((3, 3)),
        np.diag([-1.0, 1, 1]),
        np.array([[1.0, 1e-9, 0], [0, 1, 0], [0, 0, 1]]),
        np.full((3, 3), np.inf),
        np.eye(2),
        np.eye(3, dtype=complex),
    ],
)
def test_nominal_inertia_execution_contract(inertia):
    with pytest.raises(ValueError):
        replace(parameters(), nominal_inertia_B=inertia)


@pytest.mark.parametrize("speed", [0.0, 100.0, 900.0])
def test_zero_moment_anchor_on_bounds(speed):
    model = replace(rotors(), minimum_rotor_omega=min(speed, 100.0))
    thrust = 4e-5 * speed**2
    result = allocate_limited_body_moment(thrust, np.array([1.0, -2, 3]), model)
    assert result.moment_scale == 0
    np.testing.assert_array_equal(result.commanded_rotor_omega, np.full(4, speed))
    np.testing.assert_allclose(result.allocated_moment_B, 0, atol=1e-15)


def test_randomized_allocation_feasibility_and_maximal_ray_scale():
    rng = np.random.Generator(np.random.PCG64(34059))
    model = replace(rotors(), minimum_rotor_omega=100.0)
    for _ in range(80):
        thrust = rng.uniform(1.0, 32.0)
        moment = rng.uniform(-10, 10, 3)
        result = allocate_limited_body_moment(thrust, moment, model)
        force, actual = forward(result.commanded_rotor_omega, model)
        np.testing.assert_allclose(force, [0, 0, -thrust], atol=2e-14)
        np.testing.assert_allclose(actual, moment * result.moment_scale, atol=3e-15)
        assert np.all((result.commanded_rotor_omega >= 100) & (result.commanded_rotor_omega <= 900))
        if result.moment_scale < 1:
            with pytest.raises(ValueError, match="infeasible"):
                commanded_rotor_speeds_from_collective_thrust_and_body_moment(
                    thrust,
                    moment * (result.moment_scale + 1e-7),
                    model.rotor_positions_B,
                    model.rotor_spin_directions,
                    1e-5,
                    2e-7,
                    100,
                    900,
                )


def test_local_principal_axis_linearization():
    # Independent finite differences of the composed physical angular acceleration.
    model = parameters()
    eps = 1e-6
    for axis in range(3):

        def acceleration(theta, rate, axis=axis):
            omega = np.eye(3)[axis] * rate
            output = compute_attitude_control(
                quaternion(np.eye(3)[axis], theta), omega, np.array([1.0, 0, 0, 0]), 9.81, model
            )
            return angular_acceleration_body_from_moment(
                output.moment_requested_B, omega, model.nominal_inertia_B
            )[axis]

        dtheta = (acceleration(eps, 0) - acceleration(-eps, 0)) / (2 * eps)
        drate = (acceleration(0, eps) - acceleration(0, -eps)) / (2 * eps)
        assert dtheta == pytest.approx(-[36, 36, 16][axis])
        assert drate == pytest.approx(-[12, 12, 8][axis])


def test_huge_integer_and_singular_geometry_reject():
    with pytest.raises(ValueError, match="finite"):
        allocate_limited_body_moment(10**1000, np.zeros(3), rotors())
    with pytest.raises(ValueError, match="nonsingular"):
        allocate_limited_body_moment(
            9.81, np.zeros(3), replace(rotors(), rotor_positions_B=np.zeros((4, 3)))
        )


def test_exact_pi_tie_and_local_boundary():
    for sign in [-1, 1]:
        np.testing.assert_array_equal(
            attitude_error_body(np.array([1.0, 0, 0, 0]), sign * np.array([0.0, -1, 0, 0])),
            [np.pi, 0, 0],
        )
    model = replace(parameters(), maximum_attitude_error_rad=0.5)
    compute_attitude_control(
        np.array([1.0, 0, 0, 0]), np.zeros(3), quaternion([1, 0, 0], 0.5), 9.81, model
    )
    with pytest.raises(ValueError, match="domain"):
        compute_attitude_control(
            np.array([1.0, 0, 0, 0]), np.zeros(3), quaternion([1, 0, 0], 0.5 + 1e-12), 9.81, model
        )
