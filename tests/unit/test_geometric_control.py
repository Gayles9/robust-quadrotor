"""Independent physical and differential checks for the replacement controller."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.attitude_control_validation import baseline_parameters
from quadrotor_math.geometric_control import (
    GeometricControllerParameters,
    GeometricDomainError,
    RotationReference,
    compute_geometric_control,
    force_rotation_reference,
)
from quadrotor_math.rotations import rotation_matrix_body_to_world


def hat(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


def quaternion(axis, angle):
    return np.r_[np.cos(angle / 2), np.sin(angle / 2) * axis / np.linalg.norm(axis)]


def command(q, omega, target, inner=None, thrust=9.81):
    return compute_geometric_control(
        q,
        omega,
        target,
        thrust,
        baseline_parameters() if inner is None else inner,
        GeometricControllerParameters(),
    )


def test_hover_sign_and_allocation():
    target = force_rotation_reference(np.array([0.0, 0, 9.81]), np.zeros(3), np.zeros(3), 0.0)
    result = command(np.array([1.0, 0, 0, 0]), np.zeros(3), target)
    np.testing.assert_array_equal(target.q_reference_WB, [1, 0, 0, 0])
    np.testing.assert_array_equal(result.moment_requested_B, 0)
    np.testing.assert_allclose(result.allocation.commanded_rotor_omega, np.sqrt(9.81 / 4e-5))


@pytest.mark.parametrize("seed", range(8))
def test_force_jets_against_independent_finite_differences(seed):
    rng = np.random.default_rng(seed)
    u = rng.normal(size=3) + [0, 0, 9.81]
    d, dd = rng.normal(size=(2, 3))
    yaw = 0.7
    jet = force_rotation_reference(u, d, dd, yaw)
    R = rotation_matrix_body_to_world(jet.q_reference_WB)

    def rotation(t):
        down = u + t * d + t * t * dd / 2
        down /= np.linalg.norm(down)
        forward = np.cross([-np.sin(yaw), np.cos(yaw), 0], down)
        forward /= np.linalg.norm(forward)
        return np.column_stack((forward, np.cross(down, forward), down))

    h = 2e-4
    first = (rotation(-2 * h) - 8 * rotation(-h) + 8 * rotation(h) - rotation(2 * h)) / (12 * h)
    second = (
        -rotation(2 * h)
        + 16 * rotation(h)
        - 30 * rotation(0)
        + 16 * rotation(-h)
        - rotation(-2 * h)
    ) / (12 * h * h)
    np.testing.assert_allclose(R, rotation(0), atol=1e-14)
    np.testing.assert_allclose(first, R @ hat(jet.omega_reference_D), atol=2e-11)
    np.testing.assert_allclose(
        second,
        R @ (hat(jet.alpha_reference_D) + hat(jet.omega_reference_D) @ hat(jet.omega_reference_D)),
        atol=3e-8,
    )


def test_moving_reference_full_inertia_energy_identity_and_quaternion_sign():
    J = np.array([[0.022, 0.002, -0.001], [0.002, 0.026, 0.003], [-0.001, 0.003, 0.041]])
    inner = replace(baseline_parameters(), nominal_inertia_B=J)
    q = quaternion(np.array([1.0, 2, 3]), 0.17)
    qd = quaternion(np.array([2.0, -1, 1]), -0.13)
    omega, wd, ad = (
        np.array([0.12, -0.08, 0.04]),
        np.array([0.03, 0.06, -0.02]),
        np.array([0.07, -0.09, 0.05]),
    )
    ref = RotationReference(qd, wd, ad)
    result = command(q, omega, ref, inner)
    R, Rd = rotation_matrix_body_to_world(q), rotation_matrix_body_to_world(qd)
    A = R.T @ Rd
    E = Rd.T @ R - R.T @ Rd
    er = np.array([E[2, 1], E[0, 2], E[1, 0]]) / 2
    ew = omega - A @ wd
    omega_dot = np.linalg.solve(J, result.moment_requested_B - np.cross(omega, J @ omega))
    ew_dot = omega_dot + np.cross(omega, A @ wd) - A @ ad
    np.testing.assert_allclose(J @ ew_dot, -0.64 * er - 0.32 * ew, atol=1e-15)
    assert 0.64 * er @ ew + ew @ J @ ew_dot == pytest.approx(-0.32 * ew @ ew, abs=1e-15)
    negative = command(-q, omega, RotationReference(-qd, wd, ad), inner)
    np.testing.assert_array_equal(result.moment_requested_B, negative.moment_requested_B)


@pytest.mark.parametrize("field", ["attitude_stiffness", "rate_damping", "filter_pole_rad_s"])
@pytest.mark.parametrize("bad", [0.0, -1.0, np.nan, np.inf, True])
def test_gain_validation(field, bad):
    with pytest.raises(ValueError):
        replace(GeometricControllerParameters(), **{field: bad})


def test_owned_reference_and_domain_limits():
    q = np.array([1.0, 0, 0, 0])
    rate = np.zeros(3)
    ref = RotationReference(q, rate, rate)
    q[:] = 0
    assert not ref.q_reference_WB.flags.writeable
    np.testing.assert_array_equal(ref.q_reference_WB, [1, 0, 0, 0])
    with pytest.raises(GeometricDomainError, match="force"):
        force_rotation_reference(np.zeros(3), rate, rate, 0.0)
    with pytest.raises(GeometricDomainError, match="heading"):
        force_rotation_reference(np.array([0.0, 1, 0]), rate, rate, 0.0)
    with pytest.raises(GeometricDomainError, match="attitude"):
        command(quaternion(np.array([1.0, 0, 0]), np.pi), rate, ref)
    with pytest.raises(GeometricDomainError, match="rate"):
        command(
            ref.q_reference_WB, rate, RotationReference(ref.q_reference_WB, np.ones(3) * 100, rate)
        )


def test_moment_limits_are_visible():
    ref = RotationReference(np.array([1.0, 0, 0, 0]), np.zeros(3), np.ones(3) * 100)
    result = command(ref.q_reference_WB, np.zeros(3), ref)
    assert result.moment_limited
    assert np.all(np.abs(result.moment_limited_B) <= baseline_parameters().maximum_moment_B)


@pytest.mark.parametrize("bad", [np.ones(2), np.array([0.0, np.nan, 1]), np.ones(3, dtype=complex)])
def test_force_validation(bad):
    with pytest.raises(ValueError):
        force_rotation_reference(bad, np.zeros(3), np.zeros(3), 0.0)
