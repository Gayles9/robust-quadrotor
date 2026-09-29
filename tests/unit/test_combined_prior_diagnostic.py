"""Independent identities for the saved-data tradeoff diagnosis."""

from types import SimpleNamespace

import numpy as np
import pytest

from experiments.combined_prior_diagnostic import (
    authenticate_inputs,
    capture_updates,
    command_terms,
    correction_change,
    gaussian_split,
    signed_mse,
)
from experiments.supported_start import configuration_for, prepare
from quadrotor_math import eskf_replay
from quadrotor_math.eskf_endpoint import initialize_eskf_endpoint
from quadrotor_math.eskf_replay import EskfObservationKind, EskfReplayObservation


def test_covariance_changes_gain_without_guaranteeing_realized_error_reduction():
    covariance = np.eye(21)
    covariance[0, 0] = covariance[3, 3] = 2
    covariance[0, 3] = covariance[3, 0] = 1
    H = np.eye(21)[:1]
    first = gaussian_split(covariance, H, np.ones((1, 1)), np.array([0.4]), np.array([-0.2]))
    assert first["gain"][0, 0] == pytest.approx(2 / 3)
    assert first["gain"][3, 0] == pytest.approx(1 / 3)
    assert first["noise"][0] == pytest.approx(0.8 / 3)
    assert first["prediction"][0] == pytest.approx(-0.4 / 3)
    conditioned = covariance - np.outer(covariance[:, 3], covariance[3]) / 2
    second = gaussian_split(conditioned, H, np.ones((1, 1)), np.array([0.4]), np.array([-0.2]))
    assert second["gain"][0, 0] == pytest.approx(0.6)
    assert second["gain"][3, 0] == 0
    change = correction_change(first, second)
    np.testing.assert_allclose(
        change["gain"], second["correction"] - first["correction"], atol=1e-16
    )
    np.testing.assert_array_equal(change["innovation"], np.zeros(21))


def test_gain_and_innovation_change_identity_retains_correlated_cancellation():
    first = dict(gain=np.array([[2.0], [1.0]]), innovation=np.array([3.0]))
    second = dict(gain=np.array([[1.0], [-1.0]]), innovation=np.array([6.0]))
    change = correction_change(first, second)
    np.testing.assert_array_equal(change["gain"], [-3.0, -6.0])
    np.testing.assert_array_equal(change["innovation"], [3.0, -3.0])
    np.testing.assert_array_equal(change["total"], [0.0, -9.0])
    rejected = dict(gain=np.zeros((2, 1)), innovation=np.array([99.0]))
    np.testing.assert_array_equal(correction_change(first, rejected)["total"], [-6.0, -3.0])


@pytest.mark.parametrize("bad", ["shape", "nan"])
def test_correction_change_rejects_incompatible_inputs(bad):
    first = dict(gain=np.ones((2, 1)), innovation=np.ones(1))
    second = dict(
        gain=np.ones((2, 2)) if bad == "shape" else np.full((2, 1), np.nan), innovation=np.ones(1)
    )
    with pytest.raises(ValueError):
        correction_change(first, second)


def test_signed_mse_is_additive_and_can_be_negative_with_different_durations():
    channels = np.tile(np.array([[[3.0, 0, 0], [-2.0, 0, 0]]]), (3, 1, 1))
    error = channels.sum(axis=1)
    np.testing.assert_array_equal(signed_mse(error, channels), [3.0, -2.0])
    shortened = channels[:2] * 2
    delta = signed_mse(shortened.sum(axis=1), shortened) - signed_mse(error, channels)
    assert delta.sum() == 3
    with pytest.raises(ValueError, match="response sum"):
        signed_mse(error + 0.1, channels)
    with pytest.raises(ValueError):
        signed_mse(error, channels[:, :, :2])


def command_fixture():
    a = dict(
        mission_time_s=np.array([0.0, 0.1, 0.2]),
        mission_position_control_time_s=np.array([0.0, 0.2]),
    )
    for name in (
        "position_W",
        "velocity_W",
        "reference_position_W",
        "reference_velocity_W",
        "reference_acceleration_W",
    ):
        a["mission_" + name] = np.zeros((3, 3))
    a["estimate_position_W"] = np.tile([1.0, 0, -1.0], (3, 1))
    a["estimate_velocity_W"] = np.tile([0.0, 2.0, 0], (3, 1))
    a["mission_requested_acceleration_W"] = np.tile([-1.0, -10.0, 3.0], (2, 1))
    args = dict(
        position_controller=SimpleNamespace(
            position_gain_W=np.array([1.0, 2, 3]), velocity_gain_W=np.array([4.0, 5, 6])
        )
    )
    return a, args


def test_requested_acceleration_uses_navigation_error_with_negative_pd_sign():
    a, args = command_fixture()
    terms = command_terms(a, args)
    np.testing.assert_array_equal(terms[:, 2], np.tile([-1.0, 0, 3.0], (2, 1)))
    np.testing.assert_array_equal(terms[:, 3], np.tile([0.0, -10, 0], (2, 1)))
    a["mission_position_control_time_s"][1] = 0.15
    with pytest.raises(ValueError, match="outer clock"):
        command_terms(a, args)


@pytest.mark.parametrize("rejected", [False, True])
def test_observer_preserves_ordered_update_and_independently_checks_joint_covariance(rejected):
    args = configuration_for(prepare("nominal_hover", "smoke"), "aligned")
    config = args["estimator_configuration"]
    endpoint = initialize_eskf_endpoint(
        config.initial_state, config.initial_covariance, config.sampled_imu_noise
    )
    position = endpoint.nominal_state.position_W
    observations = (
        EskfReplayObservation(
            EskfObservationKind.LOCAL_POSITION, 0, 0, 0, position + (1 if rejected else 0.01)
        ),
        EskfReplayObservation(
            EskfObservationKind.BAROMETRIC_ALTITUDE,
            0,
            0,
            0,
            np.array(
                [
                    config.barometric_reference_altitude
                    + config.barometric_altitude_bias
                    - position[2]
                    + 0.01
                ]
            ),
        ),
    )
    a = dict(mission_time_s=np.array([0.0]), mission_position_W=np.array([position]))
    original = eskf_replay._correct_eskf_epoch
    inputs = (
        endpoint.nominal_state,
        endpoint.joint_covariance[:15, :15],
        endpoint,
        observations,
        0,
        config,
    )
    expected = original(*inputs)
    with capture_updates(a, args) as trace:
        actual = eskf_replay._correct_eskf_epoch(*inputs)
    assert eskf_replay._correct_eskf_epoch is original
    np.testing.assert_array_equal(actual[2].joint_covariance, expected[2].joint_covariance)
    assert len(trace["events"]) == 2
    assert trace["events"][0]["sensor"] == "position"
    assert trace["events"][1]["sensor"] == "altitude"
    assert max(trace["maximum"].values()) < 1e-10
    if rejected:
        np.testing.assert_array_equal(trace["events"][0]["gain"], np.zeros((21, 3)))
    with pytest.raises(RuntimeError):
        with capture_updates(a, args):
            raise RuntimeError("injected")
    assert eskf_replay._correct_eskf_epoch is original


def test_authentication_rejects_changed_report_before_reading_histories(tmp_path):
    (tmp_path / "report.json").write_text("{}")
    with pytest.raises(ValueError, match="byte digest"):
        authenticate_inputs(dict(original=tmp_path, boundary=tmp_path, combined=tmp_path))
