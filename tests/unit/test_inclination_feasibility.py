"""Independent finite-difference, Gaussian, frame and impossibility checks for ADR0040."""

import numpy as np
import pytest

from experiments.inclination_feasibility import (
    DOWN,
    age_angle_bound,
    bias_information,
    condition_decorrelated,
    condition_joint,
    gaussian_sequence_radius,
    local_model,
    registered_direction,
    rotation,
    tangent_basis,
)


def axis_rotation(axis, angle):
    """Rodrigues formula, independent of the fixture's quaternion construction."""
    x, y, z = axis
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


@pytest.mark.parametrize("phi", [[0.0, 0.0, 0.0], [0.4, -0.3, 0.7], [-1.2, 0.5, 0.2]])
def test_right_local_and_world_noise_jacobians_at_fixed_chart(phi):
    R_WB = rotation(np.array(phi))
    direction = R_WB @ DOWN
    E = tangent_basis(direction)
    residual, H, J = local_model(R_WB, direction, E)
    np.testing.assert_array_equal(residual, [0, 0])
    assert np.linalg.matrix_rank(H) == 2
    np.testing.assert_array_equal(H[:, 8], [0, 0])
    for i, axis in enumerate(np.eye(3)):
        plus, minus = axis_rotation(axis, 1e-6), axis_rotation(axis, -1e-6)
        right_fd = E.T @ (R_WB @ (plus - minus) @ DOWN) / 2e-6
        world_fd = E.T @ ((plus - minus) @ direction) / 2e-6
        np.testing.assert_allclose(right_fd, H[:, 6 + i], atol=1e-8, rtol=0)
        np.testing.assert_allclose(world_fd, J[:, i], atol=1e-8, rtol=0)


def test_registration_uses_world_reference_target_and_body_mount_in_order():
    R_WE = axis_rotation([0, 0, 1], np.pi / 2)
    R_EC = axis_rotation([0, 1, 0], np.pi / 2)
    R_CB = axis_rotation([1, 0, 0], np.pi / 2)
    np.testing.assert_allclose(registered_direction(R_WE, R_EC, R_CB), [1, 0, 0], atol=1e-15)
    assert np.linalg.norm(registered_direction(R_CB, R_WE, R_EC) - [1, 0, 0]) > 1


def test_unobserved_body_axis_twist_is_not_world_yaw_when_tilted():
    R_WB = axis_rotation([0, 1, 0], 0.5)
    spin = axis_rotation([0, 0, 1], 0.7)
    np.testing.assert_array_equal(R_WB @ spin @ DOWN, R_WB @ DOWN)
    assert np.linalg.norm(spin @ R_WB @ DOWN - R_WB @ DOWN) > 0.3


@pytest.mark.parametrize("direction", [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
def test_antipodal_and_equatorial_directions_are_not_local_measurements(direction):
    # The antipode has zero tangent projection; hemisphere rejection is necessary.
    with pytest.raises(ValueError, match="hemisphere"):
        local_model(np.eye(3), np.array(direction), tangent_basis(DOWN))


@pytest.mark.parametrize("bad", ["nonunit", "reflection", "basis", "nan"])
def test_invalid_measurement_geometry_is_rejected(bad):
    R, d, E = np.eye(3), DOWN.copy(), tangent_basis(DOWN)
    if bad == "nonunit":
        d *= 2
    elif bad == "reflection":
        R[0, 0] = -1
    elif bad == "basis":
        E[:, 0] = DOWN
    else:
        d[0] = np.nan
    with pytest.raises(ValueError):
        local_model(R, d, E)


def test_tangent_chart_rotation_preserves_innovation_and_linear_posterior():
    R = rotation(np.array([0.2, -0.3, 0.4]))
    d = R @ DOWN
    E = tangent_basis(d)
    measurement = axis_rotation([1, 0, 0], 0.015) @ d
    angle = 0.6
    Q = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    a, H, J = local_model(R, measurement, E)
    b, G, K = local_model(R, measurement, E @ Q)
    P = np.eye(21) * 0.01
    angular = np.diag([0.0001, 0.0002, 0.0003])
    mean0, cov0, nis0 = condition_joint(P, H, J @ angular @ J.T, a)
    mean1, cov1, nis1 = condition_joint(P, G, K @ angular @ K.T, b)
    np.testing.assert_allclose(mean0, mean1, atol=1e-12, rtol=0)
    np.testing.assert_allclose(cov0, cov1, atol=1e-12, rtol=0)
    assert nis0 == pytest.approx(nis1, abs=1e-12)


def test_correlated_two_sensor_fixture_matches_independent_batch_conditioning():
    P = np.array([[1.0, 0.2], [0.2, 0.7]])
    H = np.array([[1.0, 0.3], [-0.2, 1.0]])
    R = np.array([[0.4, 0.18], [0.18, 0.3]])
    y = np.array([0.25, -0.1])
    mean, posterior, _ = condition_joint(P, H, R, y)
    # Independent information-form solution, valid here because U=0 and P,R are PD.
    invP, invR = np.linalg.inv(P), np.linalg.inv(R)
    reference_cov = np.linalg.inv(invP + H.T @ invR @ H)
    reference_mean = reference_cov @ H.T @ invR @ y
    np.testing.assert_allclose(mean, reference_mean, atol=1e-12, rtol=0)
    np.testing.assert_allclose(posterior, reference_cov, atol=1e-12, rtol=0)
    sequential = condition_decorrelated(P, H, R, y, 1)
    np.testing.assert_allclose(sequential[0], mean, atol=1e-12, rtol=0)
    np.testing.assert_allclose(sequential[1], posterior, atol=1e-12, rtol=0)
    wrong, wrong_cov, _ = condition_joint(P, H, np.diag(np.diag(R)), y)
    assert np.linalg.norm(wrong - mean) > 0.01
    assert np.linalg.norm(wrong_cov - posterior) > 0.01


def test_state_noise_correlation_matches_joint_gaussian_block_marginal():
    P = np.diag([0.8, 0.6])
    R = np.diag([0.5, 0.4])
    U = np.array([[0.1, -0.04], [0.05, 0.03]])
    H = np.array([[1.0, 0.2], [0.3, 1.0]])
    y = np.array([0.2, -0.1])
    joint = np.block([[P, U], [U.T, R]])
    transform = np.block([[np.eye(2), np.zeros((2, 2))], [H, np.eye(2)]])
    distribution = transform @ joint @ transform.T
    gain = distribution[:2, 2:] @ np.linalg.inv(distribution[2:, 2:])
    expected_cov = distribution[:2, :2] - gain @ distribution[2:, :2]
    mean, cov, nis = condition_joint(P, H, R, y, U)
    np.testing.assert_allclose(mean, gain @ y, atol=1e-12, rtol=0)
    np.testing.assert_allclose(cov, expected_cov, atol=1e-12, rtol=0)
    assert nis == pytest.approx(float(y @ np.linalg.solve(distribution[2:, 2:], y)))
    wrong, _, _ = condition_joint(P, H, R, y)
    assert np.linalg.norm(wrong - mean) > 0.001


@pytest.mark.parametrize("bad", ["cross", "negative", "asymmetric", "singular_innovation", "nan"])
def test_invalid_joint_uncertainty_rejected(bad):
    P, R, H, U, y = np.eye(2), np.eye(2), np.eye(2), np.zeros((2, 2)), np.ones(2)
    if bad == "cross":
        U *= 0
        U[0, 0] = 2
    elif bad == "negative":
        P[0, 0] = -1
    elif bad == "asymmetric":
        R[0, 1] = 0.2
    elif bad == "singular_innovation":
        P *= 0
        R *= 0
    else:
        U[0, 0] = np.nan
    with pytest.raises(ValueError):
        condition_joint(P, H, R, y, U)


def test_age_bound_attains_constant_axis_direction_motion():
    omega, age = 0.7, 0.2
    d = axis_rotation([1, 0, 0], omega * age) @ DOWN
    exact = np.arctan2(np.linalg.norm(np.cross(DOWN, d)), DOWN @ d)
    assert exact == pytest.approx(age_angle_bound(omega, age), abs=1e-15)
    assert age_angle_bound(3, 4) == np.pi
    assert age_angle_bound(0, 4) == 0


def test_sequence_bound_matches_exact_two_dimensional_radial_tail():
    sigma, count, alpha = 0.02, 1000, 0.01
    radius = gaussian_sequence_radius(sigma, count, alpha)
    assert count * np.exp(-(radius**2) / (2 * sigma**2)) == pytest.approx(alpha, rel=1e-12)
    assert gaussian_sequence_radius(0, count, alpha) == 0
    assert -2 * np.log(0.01) == pytest.approx(9.210340371976184)


@pytest.mark.parametrize("rate,age", [(-1, 1), (1, -1), (np.nan, 1), (1, np.inf)])
def test_age_bound_rejects_invalid_assumptions(rate, age):
    with pytest.raises(ValueError):
        age_angle_bound(rate, age)


@pytest.mark.parametrize(
    "sigma,n,alpha", [(-1, 1, 0.1), (1, 0, 0.1), (1, 1, 0), (1, 1, 1), (1, True, 0.1)]
)
def test_probability_bound_rejects_invalid_assumptions(sigma, n, alpha):
    with pytest.raises(ValueError):
        gaussian_sequence_radius(sigma, n, alpha)


def test_unknown_sensor_bias_keeps_absolute_tilt_ambiguity():
    result = bias_information(9.81)
    assert result["calibrated_state_dimension"] - result["calibrated_rank"] == 2
    assert result["unknown_bias_state_dimension"] - result["unknown_bias_rank"] == 4
    N, F, H = (np.array(result[k]) for k in ("null_vectors", "F", "H"))
    np.testing.assert_array_equal(H @ N, np.zeros((5, 4)))
    np.testing.assert_array_equal((F @ N)[:, :3], np.zeros((17, 3)))
    np.testing.assert_array_equal(F @ N[:, 3], -N[:, 2])
    assert np.linalg.matrix_rank(N) == 4
