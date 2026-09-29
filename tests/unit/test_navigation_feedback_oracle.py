"""ADR0033 outer-only isolation, restoration, saved replay and unchanged scoring."""

import copy

import numpy as np
import pytest

from experiments.estimated_feedback_evidence import load_history, pack_history, save_history
from experiments.navigation_feedback_oracle import (
    audit_oracle,
    compare,
    navigation_oracle,
    noise_pairing,
)
from experiments.supported_start import configuration_for, fly
from experiments.supported_validation_protocol import Job, prepare_job
from quadrotor_math import estimated_mission, mission_simulation
from quadrotor_math.position_control import compute_position_control


@pytest.mark.parametrize("fail", [False, True])
def test_only_outer_navigation_changes_and_patches_restore(monkeypatch, fail):
    truth = (np.ones(3), np.ones(3) * 2, np.array([1.0, 0, 0, 0]), np.zeros(3))
    feedback = (np.ones(3) * 3, np.ones(3) * 4, np.array([1.0, 0, 0, 0]), np.ones(3) * 5)
    sentinel = object()
    received = []

    def control(*args):
        received.append(args)
        return sentinel

    def simulate(*args, **kwargs):
        returned = args[10](0, 0.0, truth, np.zeros(4))
        assert returned is feedback  # Guard, completion and inner inputs remain estimates.
        assert kwargs == {"observation_guard": sentinel}
        result = mission_simulation.compute_position_control(
            returned[0], returned[1], sentinel, sentinel
        )
        assert result is sentinel
        if fail:
            raise RuntimeError("injected")
        return result

    monkeypatch.setattr(estimated_mission, "_simulate_mission", simulate)
    monkeypatch.setattr(mission_simulation, "compute_position_control", control)
    try:
        with navigation_oracle() as trace:
            result = estimated_mission._simulate_mission(
                *([None] * 10), lambda *a: feedback, observation_guard=sentinel
            )
            assert result is sentinel
    except RuntimeError:
        assert fail
    assert estimated_mission._simulate_mission is simulate
    assert mission_simulation.compute_position_control is control
    np.testing.assert_array_equal(received[0][0], truth[0])
    np.testing.assert_array_equal(received[0][1], truth[1])
    assert received[0][2:] == (sentinel, sentinel)
    assert trace[0]["index"] == 0 and trace[0]["time_s"] == 0
    assert trace[0]["estimated_position_W"] == feedback[0].tolist()
    assert trace[0]["feedback_velocity_W"] == truth[1].tolist()


def test_wrong_estimate_is_rejected_before_outer_command(monkeypatch):
    truth = (np.ones(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3))
    estimate = (np.zeros(3), np.zeros(3), truth[2], truth[3])

    def simulate(*args, **kwargs):
        args[10](0, 0.0, truth, np.zeros(4))
        mission_simulation.compute_position_control(truth[0], estimate[1], None, None)

    monkeypatch.setattr(estimated_mission, "_simulate_mission", simulate)
    with pytest.raises(ValueError, match="same-epoch estimated navigation"):
        with navigation_oracle():
            estimated_mission._simulate_mission(*([None] * 10), lambda *a: estimate)


@pytest.fixture(scope="module")
def smoke_pair():
    prepared = prepare_job(Job("nominal_hover", 47821), "smoke")
    baseline, baseline_diagnostic = fly(prepared, "aligned")
    with navigation_oracle() as trace:
        oracle, diagnostic = fly(prepared, "aligned")
    return prepared, baseline, baseline_diagnostic, oracle, diagnostic, trace


def test_saved_smoke_reconstructs_oracle_and_preserves_noise(smoke_pair, tmp_path):
    prepared, baseline, _, oracle, diagnostic, trace = smoke_pair
    arrays = pack_history(oracle, oracle.mission)
    records = save_history(tmp_path, 0, arrays)
    restored = load_history(tmp_path, 0, records)
    for key in arrays:
        np.testing.assert_array_equal(restored[key], arrays[key])
    checked = audit_oracle(prepared, oracle, diagnostic, trace)
    assert max(checked.values()) < 1e-12
    pairing = noise_pairing(
        pack_history(baseline, baseline.mission), restored, configuration_for(prepared, "aligned")
    )
    assert max(pairing.values()) < 1e-12
    args = configuration_for(prepared, "aligned")
    m = oracle.mission
    for row, k in enumerate(np.searchsorted(m.time_s, m.position_control_time_s)):
        from quadrotor_math.missions import mission_reference

        reference, _ = mission_reference(args["plan"], float(m.time_s[k]))
        expected = compute_position_control(
            m.position_W[k], m.velocity_W[k], reference, args["position_controller"]
        )
        np.testing.assert_array_equal(
            m.requested_acceleration_W[row], expected.requested_acceleration_W
        )
    assert not np.array_equal(
        baseline.mission.requested_acceleration_W, oracle.mission.requested_acceleration_W
    )


@pytest.mark.parametrize(
    "field", ["index", "time_s", "estimated_velocity_W", "feedback_position_W"]
)
def test_trace_tampering_is_rejected(smoke_pair, field):
    prepared, _, _, oracle, diagnostic, original = smoke_pair
    trace = copy.deepcopy(original)
    trace[0][field] = -1 if field in ("index", "time_s") else [100.0, 100.0, 100.0]
    with pytest.raises(ValueError, match="trace"):
        audit_oracle(prepared, oracle, diagnostic, trace)


def test_pairing_rejects_changed_noise_and_bias(smoke_pair):
    prepared, baseline, _, oracle, _, _ = smoke_pair
    a, b = pack_history(baseline, baseline.mission), pack_history(oracle, oracle.mission)
    args = configuration_for(prepared, "aligned")
    b["angular_velocity_measurements_B"] = b["angular_velocity_measurements_B"].copy()
    b["angular_velocity_measurements_B"][0, 0] += 0.01
    with pytest.raises(ValueError, match="paired random draws"):
        noise_pairing(a, b, args)
    b = pack_history(oracle, oracle.mission)
    b["true_accelerometer_bias_B"] = b["true_accelerometer_bias_B"].copy()
    b["true_accelerometer_bias_B"][0, 0] += 0.01
    with pytest.raises(ValueError, match="bias draws"):
        noise_pairing(a, b, args)


@pytest.mark.parametrize("failure", ["none", "hover", "rmse", "incomplete"])
def test_headroom_requires_all_frozen_conditions(failure):
    baseline = {"hover_peak_m": 0.1018, "tracking_rmse_m": 0.0649}
    oracle = {"hover_peak_m": 0.04, "tracking_rmse_m": 0.04, "flight_passed": True}
    if failure == "hover":
        oracle["hover_peak_m"] = 0.09
        oracle["flight_passed"] = False
    if failure == "rmse":
        oracle["tracking_rmse_m"] = 0.07
    if failure == "incomplete":
        oracle["flight_passed"] = False
    comparison = compare(baseline, oracle)
    assert comparison["useful_headroom"] == (failure == "none")
    assert comparison["qualification"] is False
