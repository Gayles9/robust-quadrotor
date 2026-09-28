"""Independent conditioning, reset, startup information and evidence boundaries."""

from dataclasses import replace

import numpy as np
import pytest

from experiments import attitude_startup_audit as audit
from quadrotor_math.eskf import EskfNominalState, eskf_reset_jacobian
from quadrotor_math.eskf_endpoint import EskfEndpointState, update_eskf_endpoint


def nominal():
    return EskfNominalState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
    )


@pytest.mark.parametrize(
    "phi",
    [
        np.zeros(3),
        np.array([1e-8, -2e-8, 3e-8]),
        np.array([0.4, -0.6, 0.8]),
        np.array([0, 0, np.pi]),
    ],
)
def test_quadrature_reset_matches_closed_form_and_negative_skew_sign(phi):
    correction = np.zeros(15)
    correction[6:9] = phi
    np.testing.assert_allclose(
        audit.right_jacobian(phi), eskf_reset_jacobian(correction)[6:9, 6:9], atol=8e-16
    )
    if np.any(phi):
        assert not np.array_equal(audit.right_jacobian(phi), audit.right_jacobian(-phi))


@pytest.mark.parametrize("bad", [np.zeros(2), np.zeros((1, 3)), np.array([0, np.nan, 0])])
def test_reset_rejects_invalid_vectors(bad):
    with pytest.raises(ValueError, match="finite rotation vector"):
        audit.right_jacobian(bad)


@pytest.mark.parametrize("size", [1, 3])
def test_full_joint_conditioning_including_sample_memory(size):
    rng = np.random.default_rng(260026)
    L = rng.normal(size=(21, 21)) * 0.03
    C = L @ L.T
    H = np.zeros((size, 21))
    H[:, :size] = np.eye(size) * (-1 if size == 1 else 1)
    R = np.eye(size) * 0.02**2
    residual = np.linspace(-0.01, 0.03, size)
    current = EskfEndpointState(nominal(), np.linspace(-0.01, 0.02, 6), C)
    out, diagnostic = update_eskf_endpoint(current, residual, np.zeros(size), H[:, :15], R)
    correction, covariance = audit.independent_condition(C, H, R, residual)
    np.testing.assert_allclose(correction[:15], diagnostic.error_state_correction, atol=2e-16)
    np.testing.assert_allclose(covariance, out.joint_covariance, atol=8e-17)
    np.testing.assert_allclose(
        current.imu_noise_mean_B + correction[15:], out.imu_noise_mean_B, atol=2e-17
    )
    assert np.linalg.norm(correction[15:]) > 0


def test_conditioning_rejects_bad_shape_nonfinite_and_singular_innovation():
    C, H, R, residual = np.eye(21), np.eye(21)[:1], np.eye(1), np.zeros(1)
    for args in (
        (C[:15, :15], H, R, residual),
        (C, H[:, :15], R, residual),
        (C, H, np.eye(2), residual),
        (C, H, R, np.array([np.nan])),
    ):
        with pytest.raises(ValueError):
            audit.independent_condition(*args)
    with pytest.raises(np.linalg.LinAlgError):
        audit.independent_condition(np.zeros((21, 21)), H, np.zeros((1, 1)), residual)


@pytest.mark.parametrize("sign", [-1, 1])
def test_tilt_ignores_yaw_and_quaternion_sign(sign):
    identity = np.array([1.0, 0, 0, 0])
    yaw = np.array([np.cos(0.4), 0, 0, np.sin(0.4)])
    pitch = np.array([np.cos(0.1), 0, np.sin(0.1), 0])
    assert audit.tilt_deg(identity, sign * yaw) == 0
    assert audit.tilt_deg(identity, sign * pitch) == pytest.approx(np.rad2deg(0.2))


def test_local_information_exhibits_tilt_bias_and_yaw_ambiguity():
    result = audit.level_hover_information()
    assert result["rank"] == 11 and result["null_residual_max"] == 0
    null = np.array(result["null_vectors_columns"])
    assert np.linalg.matrix_rank(null) == 4
    # Direct independent horizontal acceleration equation, not the matrix routine.
    np.testing.assert_array_equal(9.81 * null[6] - null[10], 0)
    np.testing.assert_array_equal(-9.81 * null[7] - null[9], 0)
    assert result["three_position_acceleration_noise_std_m_s2"] == pytest.approx(
        np.sqrt(6) * 0.02 / 0.2**2
    )
    assert result["equivalent_small_angle_noise_std_deg"] == pytest.approx(7.153181662694274)


@pytest.mark.parametrize("bad", [0, -1, np.nan, np.inf])
def test_information_rejects_nonphysical_gravity(bad):
    with pytest.raises(ValueError, match="positive finite gravity"):
        audit.level_hover_information(bad)


def test_prior_comparison_is_not_silent_retuning():
    before = audit.configuration("nominal_hover", "campaign")["estimator_configuration"]
    result = audit.prior_audit()
    np.testing.assert_allclose(result["variance_ratio_prior_to_truth"][:12], 6.75)
    np.testing.assert_allclose(result["variance_ratio_prior_to_truth"][12:], 25 / 3)
    after = audit.configuration("nominal_hover", "campaign")["estimator_configuration"]
    np.testing.assert_array_equal(before.initial_covariance, after.initial_covariance)


def test_no_report_or_output_created_for_wrong_oracle_digest(tmp_path, monkeypatch):
    monkeypatch.setattr(audit, "authenticate_campaign", lambda *args: {})
    (tmp_path / "report.json").write_text("{}")
    output = tmp_path / "never-created"
    with pytest.raises(ValueError, match="digest mismatch"):
        audit.run(tmp_path, tmp_path, output)
    assert not output.exists()


def test_actual_error_can_increase_while_uncertainty_decreases():
    # A two-variable scalar linear-Gaussian analog with valid correlation.
    # This proves non-monotonic realized error is not a sign-bug criterion.
    C = np.eye(21)
    C[0, 6] = C[6, 0] = 0.5
    H = np.zeros((1, 21))
    H[0, 0] = 1
    correction, posterior = audit.independent_condition(C, H, np.array([[1.0]]), np.array([-0.4]))
    assert abs(0.1 - correction[6]) > 0.1
    assert posterior[6, 6] < C[6, 6]


def test_audit_rejects_tampered_saved_state_before_scoring(monkeypatch):
    from types import SimpleNamespace

    config = audit.configuration("nominal_hover", "campaign")["estimator_configuration"]
    fake = SimpleNamespace(
        measurements=SimpleNamespace(time_s=np.array([0.0])),
        estimates=SimpleNamespace(
            events=(), states=(replace(config.initial_state, position_W=np.ones(3)),)
        ),
    )
    monkeypatch.setattr(audit, "unpack_history", lambda arrays: (fake, None))
    with pytest.raises(ValueError, match="exact state reconstruction"):
        audit.audit_history({"mission_q_WB": np.array([[1.0, 0, 0, 0]])})
