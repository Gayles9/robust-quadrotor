"""Independent frame, latent-noise, observability and sample-ownership checks."""

import numpy as np
import pytest

from experiments import prearm_alignment_feasibility as study
from quadrotor_math.rotations import rotation_matrix_body_to_world


def test_allocation_satisfies_both_budgets_and_rounding_without_score():
    result = study.allocation()
    assert result["samples"] == 201
    assert result["acquisition_span_s"] == 0.5
    assert result["accel_white_sigma_deg"] < 0.02
    assert result["terminal_gyro_three_sigma_deg_s"] < 0.03
    n = result["minimum_accel_samples"]
    assert np.rad2deg(study.SA / np.sqrt(n - 1) / study.G) > 0.02
    assert np.rad2deg(study.SA / np.sqrt(n) / study.G) <= 0.02
    n = result["minimum_gyro_samples"]
    for count, passes in ((n - 1, False), (n, True)):
        A = study.walk_factors(count)[0]
        value = 3 * np.rad2deg(np.sqrt(study.SG**2 / count + study.WG**2 * A))
        assert bool(value <= 0.03) is passes


@pytest.mark.parametrize("angles", [(0, 0, 0), (0.1, -0.08, 0.3), (-0.05, 0.07, -0.2)])
def test_supported_gravity_signs_and_unobservable_heading(angles):
    roll, pitch, yaw = angles
    # Independent quaternion composition through public rotation implementation.
    cr, sr, cp, sp, cy, sy = [v for a in angles for v in (np.cos(a / 2), np.sin(a / 2))]
    q_WB = np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )
    R = rotation_matrix_body_to_world(q_WB)
    np.testing.assert_allclose(study.euler_rotation(roll, pitch, yaw), R, atol=2e-16)
    reconstructed, _, _ = study.inclination(-study.G * R[2])
    np.testing.assert_allclose(reconstructed[2], R[2], atol=2e-16)
    assert abs(np.arctan2(reconstructed[1, 0], reconstructed[0, 0])) < 1e-16
    # Many headings give precisely the same stationary accelerometer signal.
    np.testing.assert_allclose(study.euler_rotation(roll, pitch, yaw + 1)[2], R[2], atol=2e-16)


@pytest.mark.parametrize("s", [np.array([0, 0, -9.81]), np.array([0.3, -0.4, -9.79])])
def test_right_local_jacobian_against_matrix_finite_differences(s):
    R, J, u = study.inclination(s)
    numerical = np.empty((3, 3))
    for j in range(3):
        step = np.eye(3)[j] * 1e-5
        plus, minus = study.inclination(s + step)[0], study.inclination(s - step)[0]
        dR = R.T @ (plus - minus) / 2e-5
        numerical[:, j] = [dR[2, 1], dR[0, 2], dR[1, 0]]
    np.testing.assert_allclose(J, numerical, atol=2e-11)
    np.testing.assert_allclose(u, R[2], atol=1e-15)


@pytest.mark.parametrize("n", [2, 3, 201])
def test_walk_terms_against_explicit_independent_latent_increments(n):
    weights = np.arange(n - 1, 0, -1) / n
    A, B, T = study.walk_factors(n)
    assert A == pytest.approx(study.DT * weights @ weights)
    assert A == pytest.approx(study.DT * (1 - weights) @ (1 - weights))
    assert B == pytest.approx(study.DT * weights.sum())
    assert T == pytest.approx(study.DT * np.ones(n - 1).sum())


def test_joint_covariance_from_independent_latent_map_and_cross_sign():
    n, s = 201, np.array([0.2, -0.3, -9.81])
    _, J, u = study.inclination(s)
    columns = []
    col = np.zeros(9)
    col[:3] = study.HEADING * u
    columns.append(col)
    for axis in range(3):
        col = np.zeros(9)
        col[:3], col[3 + axis] = -J[:, axis] * study.BA, study.BA
        columns.append(col)
        for k in range(1, n):
            weight = (n - k) / n
            col = np.zeros(9)
            col[:3] = -J[:, axis] * weight * study.WA * np.sqrt(study.DT)
            col[3 + axis] = study.WA * np.sqrt(study.DT)
            columns.append(col)
            col = np.zeros(9)
            col[6 + axis] = (1 - weight) * study.WG * np.sqrt(study.DT)
            columns.append(col)
        for _ in range(n):
            col = np.zeros(9)
            col[:3] = -J[:, axis] * study.SA / n
            columns.append(col)
            col = np.zeros(9)
            col[6 + axis] = -study.SG / n
            columns.append(col)
    latent = np.array(columns).T
    P = study.covariance(s, n)
    np.testing.assert_allclose(P, latent @ latent.T, atol=4e-18)
    assert np.linalg.eigvalsh(P).min() > 0
    level = study.covariance(np.array([0, 0, -study.G]), n)
    assert level[0, 4] > 0 and level[1, 3] < 0
    assert level[2, 2] == pytest.approx(study.HEADING**2)


def test_whitening_includes_walk_correlation_and_removes_constant():
    n = 201
    times = np.arange(n) * study.DT
    for W, sigma, walk in zip(
        study.contrast_whiteners(n), (study.SA, study.SG), (study.WA, study.WG), strict=True
    ):
        C = sigma**2 * np.eye(n) + walk**2 * np.minimum.outer(times, times)
        np.testing.assert_allclose(W @ C @ W.T, np.eye(n - 1), atol=2e-14)
        np.testing.assert_allclose(W @ np.ones(n), 0, atol=2e-12)


def test_handoff_preserves_cross_covariance_and_adds_only_new_walk():
    P = study.covariance(np.array([0.1, 0.2, -9.81]), 201)
    release = study.release_covariance(P, study.DT)
    expected = np.diag([0.0] * 3 + [study.WA**2 * study.DT] * 3 + [study.WG**2 * study.DT] * 3)
    np.testing.assert_allclose(release - P, expected, atol=1e-19)
    np.testing.assert_array_equal(release[:3, 3:6], P[:3, 3:6])
    # Fresh sample means block independence is valid, unlike reusing the mean.
    joint = np.zeros((15, 15))
    joint[:9, :9], joint[9:, 9:] = release, np.diag([study.SA**2] * 3 + [study.SG**2] * 3)
    assert np.linalg.eigvalsh(joint).min() > 0


@pytest.mark.parametrize("delay", [0, 0.005, -0.0025, np.nan])
def test_handoff_rejects_reuse_or_gap(delay):
    with pytest.raises(ValueError, match="fresh endpoint"):
        study.release_covariance(np.eye(9), delay)


@pytest.mark.parametrize("s", [np.zeros(3), np.array([9.81, 0, 0]), np.array([0, 0, np.nan])])
def test_invalid_gravity_has_no_candidate(s):
    with pytest.raises(ValueError):
        study.inclination(s)


def test_stress_rejections_and_required_undetectable_counterexample():
    cases = study.fixtures()
    for name in ("impulse", "vibration", "changing_gravity"):
        assert "imu_variation" in cases[name]["reasons"]
    assert "mean_gyro" in cases["mean_rate"]["reasons"]
    assert not cases["ambiguous_acceleration"]["reasons"]
    # On the moving level vehicle f=a-g; on the supported tilted vehicle f=-R.T g.
    a = np.array(cases["indistinguishable_world_acceleration_m_s2"])
    np.testing.assert_allclose(
        a - [0, 0, study.G], -study.G * study.euler_rotation(0, np.deg2rad(1))[2]
    )
    assert np.linalg.norm(a) > 0.17
    assert "assertion required" in cases["missing_support"]


def test_heading_coupling_tightens_world_axis_uncertainty_when_tilted():
    time = np.arange(201) * study.DT
    level = study.evaluate(
        time, np.tile([0, 0, -study.G], (201, 1)), np.zeros((201, 3)), supported=True
    )
    tilted = study.evaluate(
        time,
        np.tile(-study.G * study.euler_rotation(0.1, 0)[2], (201, 1)),
        np.zeros((201, 3)),
        supported=True,
    )
    assert level.radius_99_deg < 0.75 < tilted.radius_99_deg
    assert "axis_uncertainty" in tilted.reasons


def test_all_errors_including_rejected_samples_are_retained(tmp_path, monkeypatch):
    monkeypatch.setattr(study, "TRIALS", 3)
    report = study.run(tmp_path / "evidence")
    arrays = np.load(tmp_path / "evidence/trials.npz", allow_pickle=False)
    assert arrays["gaussian_whitened"].shape == (3, 9)
    expected = np.linalg.eigvalsh(np.cov(arrays["gaussian_whitened"], rowvar=False))
    np.testing.assert_array_equal(expected, report["gaussian"]["whitened_covariance_eigenvalues"])
    assert report["flight_qualified"] is False
    with pytest.raises(ValueError, match="preserve previous results"):
        study.run(tmp_path / "evidence")
