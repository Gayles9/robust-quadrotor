"""Independent quadrature, rotation, Gaussian conditioning and contract checks."""

from math import prod

import numpy as np
import pytest

from experiments import prearm_alignment_feasibility as base
from experiments import prearm_nonlinear_uncertainty as study
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_consistency import eskf_right_local_error
from quadrotor_math.rotations import rotation_matrix_body_to_world


@pytest.mark.parametrize("order", [5, 7])
def test_positive_normal_rule_moments_independent_of_hermite_construction(order):
    x, w = study.normal_rule(order)
    assert x.shape == (order**4, 4) and w.min() > 0
    assert w.sum() == pytest.approx(1, abs=2e-15)
    for power in range(10):
        expected = 0 if power % 2 else prod(range(1, power, 2))
        for axis in range(4):
            assert w @ x[:, axis] ** power == pytest.approx(expected, abs=3e-12)
    assert w @ (x[:, 0] ** 2 * x[:, 1] ** 4 * x[:, 3] ** 6) == pytest.approx(45)
    assert not x.flags.writeable and not w.flags.writeable


@pytest.mark.parametrize("roll,pitch", [(0, 0), (0.03, -0.04), (-0.02, 0.015)])
def test_quaternion_map_and_logs_match_independent_matrix_and_eskf(roll, pitch):
    s = -base.G * base.euler_rotation(roll, pitch)[2]
    heading = np.array([-0.13, 0, 0.11])
    values = study.gravity_quaternions(np.tile(s, (3, 1)), heading)
    center_q_WB = values[1]
    for yaw, q_WB, error in zip(
        heading, values, study.local_logs(center_q_WB, values), strict=True
    ):
        np.testing.assert_allclose(
            rotation_matrix_body_to_world(q_WB), base.euler_rotation(roll, pitch, yaw), atol=3e-16
        )
        nominal = EskfNominalState(np.zeros(3), np.zeros(3), center_q_WB, np.zeros(3), np.zeros(3))
        reference = EskfNominalState(np.zeros(3), np.zeros(3), q_WB, np.zeros(3), np.zeros(3))
        np.testing.assert_allclose(
            error, eskf_right_local_error(nominal, reference)[6:9], atol=2e-16
        )
    np.testing.assert_allclose(
        study.local_logs(-center_q_WB, -values), study.local_logs(center_q_WB, values), atol=1e-16
    )


@pytest.mark.parametrize("s", [np.array([0, 0, -9.81]), np.array([0.25, -0.3, -9.8])])
def test_leading_heading_inclination_mixed_term_from_finite_differences(s):
    R, J, u = base.inclination(s)
    eps_a, eps_y = 1e-3, 1e-4
    for axis in range(3):
        total = np.zeros(3)
        for sign_a in (-1, 1):
            for sign_y in (-1, 1):
                adjusted = s - sign_a * eps_a * np.eye(3)[axis]
                # Matrix form is independent of the candidate quaternion implementation.
                Rp = base.inclination(adjusted)[0]
                Ryaw = base.euler_rotation(0, 0, sign_y * eps_y)
                total += sign_a * sign_y * base.rotation_log(R.T @ Ryaw @ Rp)
        numerical = total / (4 * eps_a * eps_y)
        np.testing.assert_allclose(numerical, -0.5 * np.cross(u, J[:, axis]), atol=2e-9)


def test_bias_conditioning_from_explicit_latent_increment_covariance():
    n = 201
    weights = np.arange(n - 1, 0, -1) / n
    latent = np.zeros((2, 1 + 2 * (n - 1) + n))
    latent[:, 0] = base.BA
    latent[0, 1:n] = weights * base.WA * np.sqrt(base.DT)
    latent[1, 1:n] = base.WA * np.sqrt(base.DT)
    latent[0, -n:] = base.SA / n
    C = latent @ latent.T
    Va, gain, residual, _ = study.bias_conditioning(n)
    assert Va == pytest.approx(C[0, 0])
    assert gain == pytest.approx(C[1, 0] / C[0, 0])
    transform = np.array([[-gain, 1], [1, 0]])
    conditional = transform @ C @ transform.T
    assert conditional[0, 0] == pytest.approx(residual)
    assert abs(conditional[0, 1]) < 2e-18
    assert gain * gain * Va + residual == pytest.approx(C[1, 1])


def test_quadrature_linear_limit_reproduces_all_first_order_joint_blocks():
    s, n = np.array([0.15, -0.2, -9.81]), 201
    _, J, u = base.inclination(s)
    x, w = study.normal_rule(5)
    Va, gain, residual, Pg = study.bias_conditioning(n)
    eta = np.sqrt(Va) * x[:, :3]
    theta = -eta @ J.T + base.HEADING * x[:, 3, None] * u
    joint = np.column_stack((theta, gain * eta, np.zeros_like(eta)))
    C = joint.T @ (w[:, None] * joint)
    C[3:6, 3:6] += residual * np.eye(3)
    C[6:, 6:] += Pg * np.eye(3)
    np.testing.assert_allclose(C, base.covariance(s, n), atol=4e-18)


def test_nonlinear_covariance_preserves_bias_marginal_and_modeled_heading_prior():
    s = np.array([0.2, -0.2, -9.81])
    result = study.nonlinear_moments(s, 201)
    original = base.covariance(s, 201)
    np.testing.assert_allclose(result.joint_covariance[3:, 3:], original[3:, 3:], atol=2e-18)
    assert np.linalg.eigvalsh(result.joint_covariance).min() > 0
    assert np.linalg.norm(result.centered_mean_B) <= 1e-13
    assert np.linalg.norm(result.shift_B) > 1e-10
    assert np.linalg.norm(result.joint_covariance[:3, 3:6]) > 1e-5
    x, w = study.normal_rule(5)
    Va = study.bias_conditioning(201)[0]
    samples = study.gravity_quaternions(s - np.sqrt(Va) * x[:, :3], base.HEADING * x[:, 3])
    yaw = np.array(
        [np.arctan2(R[1, 0], R[0, 0]) for R in map(rotation_matrix_body_to_world, samples)]
    )
    assert w @ yaw == pytest.approx(0, abs=1e-16)
    assert w @ yaw**2 == pytest.approx(base.HEADING**2, abs=2e-17)


def test_fixed_higher_order_accuracy_reference_without_candidate_selection():
    result = study.accuracy_reference()
    assert len(result["records"]) == 12
    assert result["passed"] is True


@pytest.mark.parametrize("s", [np.zeros(3), np.array([np.nan, 0, -9.81]), np.array([9.81, 0, 0])])
def test_invalid_gravity_is_rejected(s):
    with pytest.raises(ValueError):
        study.nonlinear_moments(s, 201)


def test_unsupported_orders_and_unconverged_mean_reject(monkeypatch):
    with pytest.raises(ValueError, match="frozen candidate"):
        study.normal_rule(3)
    monkeypatch.setattr(
        study, "local_logs", lambda center, samples: np.tile([0.01, 0, 0], (len(samples), 1))
    )
    with pytest.raises(ValueError, match="eight corrections"):
        study.nonlinear_moments(np.array([0, 0, -9.81]), 201)


def test_original_motion_support_and_ambiguity_fixtures_with_new_covariance(monkeypatch):
    original_evaluate = base.evaluate

    def evaluate(time_s, accel, gyro, *, supported):
        first = original_evaluate(time_s, accel, gyro, supported=supported)
        return study.candidate(accel.mean(axis=0), first)[0]

    monkeypatch.setattr(base, "evaluate", evaluate)
    cases = base.fixtures()
    for name in ("impulse", "vibration", "changing_gravity"):
        assert "imu_variation" in cases[name]["reasons"]
    assert "mean_gyro" in cases["mean_rate"]["reasons"]
    assert not cases["ambiguous_acceleration"]["reasons"]
    assert "assertion required" in cases["missing_support"]


def test_handoff_retains_nonlinear_correlations_and_fresh_walk_only():
    P = study.nonlinear_moments(np.array([0.2, -0.1, -9.81]), 201).joint_covariance
    released = base.release_covariance(P, base.DT)
    np.testing.assert_array_equal(released[:3, 3:6], P[:3, 3:6])
    np.testing.assert_array_equal(released[:3, :3], P[:3, :3])
    assert np.linalg.eigvalsh(released).min() > 0


def test_prior_authentication_precedes_any_population_or_output(tmp_path):
    (tmp_path / "report.json").write_text("{}")
    with pytest.raises(ValueError, match="digest"):
        study.run(tmp_path, tmp_path / "never-created")
    assert not (tmp_path / "never-created").exists()


def test_all_population_outcomes_retained_and_independent_streams(monkeypatch):
    monkeypatch.setattr(study, "TRIALS", 3)
    summary, arrays = study.population(1)
    assert arrays["errors"].shape == (3, 9)
    assert arrays["rejected"].shape == (3,)
    assert summary["trials"] == 3
    new, old = study.draw_trial(1, 0, 201), base.draw_trial(1, 0, 201)
    assert not np.array_equal(new["accel"], old["accel"])
    np.testing.assert_array_equal(study.draw_trial(1, 0, 201)["accel"], new["accel"])


def test_audit_rejects_nonidentical_original_history(monkeypatch):
    monkeypatch.setattr(study, "TRIALS", 1)
    with pytest.raises(ValueError, match="does not reconstruct"):
        study.population(1, {"gaussian_errors": np.zeros((1, 9))})
