"""Independent local-model checks and isolation of the rejected experiment."""

from dataclasses import asdict

import numpy as np
import pytest

from experiments import feedback_motor_damping_probe as probe
from experiments.estimated_feedback_evidence import plain
from experiments.feedback_bandwidth_validation import make_configuration
from quadrotor_math import estimated_mission, mission_simulation
from quadrotor_math.cascade_analysis import hover_axis_zero_order_hold


def test_continuous_characteristic_matches_hand_binomial_and_physical_generator():
    result = probe.local_model()
    coefficients = [0.025, 1.5, 36, 432, 2592, 6220.8]
    np.testing.assert_allclose(result["characteristic_coefficients"], coefficients)
    generator = np.zeros((5, 5))
    generator[0, 1], generator[1, 2], generator[2, 3], generator[3, 4] = 1, 9.81, 1, 1
    generator[4] = [-432 * 14.4 / 9.81, -432 * 6 / 9.81, -432, -36, -1.5]
    generator[4] /= 0.025
    np.testing.assert_allclose(np.poly(generator), np.array(coefficients) / 0.025, rtol=2e-14)
    poles = result["equivalent_poles"]
    assert max(poles.real) < -5.6
    assert min(-poles.real / abs(poles)) > 0.73


def test_sampled_transition_matches_explicit_held_outer_command():
    state = np.array([0.02, -0.03, 0.01, -0.05, 0.1])
    expected = state.copy()
    desired = -(14.4 * state[0] + 6 * state[1]) / 9.81
    phi, gamma = hover_axis_zero_order_hold(9.81, 0.025, 0.01)
    for _ in range(2):
        command = 36 * (12 * (desired - expected[2]) - expected[3]) - 0.5 * expected[4]
        expected = phi @ expected + gamma * command
    np.testing.assert_allclose(
        probe.local_model()["lifted_transition"] @ state, expected, atol=1e-15
    )


@pytest.mark.parametrize("seed", [95000, 96000, True, 30.0, -1])
def test_reserved_or_invalid_seeds_cannot_be_evaluated(seed):
    with pytest.raises(ValueError, match="observed"):
        probe.configuration(seed)


def test_configuration_preserves_truth_sensor_estimator_reference_and_limits():
    old = make_configuration({"case": "hover", "seed": 30, "noiseless": False}, design_version=2)
    new = probe.configuration(30)
    for key in old:
        a = asdict(old[key]) if hasattr(old[key], "__dataclass_fields__") else old[key]
        b = asdict(new[key]) if hasattr(new[key], "__dataclass_fields__") else new[key]
        if key in ("position_controller", "attitude_controller"):
            names = (
                ("position_gain_W", "velocity_gain_W")
                if key == "position_controller"
                else ("attitude_gain_B", "rate_gain_B")
            )
            for name in names:
                assert a[name][2] == b[name][2]
                b[name] = a[name]
        assert plain(a) == plain(b)


@pytest.mark.parametrize("horizon", [0, -1, 0.001, 10.1, float("nan"), True, np.bool_(True)])
def test_invalid_horizons_fail_before_simulation(tmp_path, horizon):
    with pytest.raises(ValueError, match="horizon_s"):
        probe.run(30, tmp_path, horizon_s=horizon)


def test_short_prefix_is_not_a_pass_and_restores_hooks(tmp_path):
    original_inner = mission_simulation.compute_attitude_control
    original_simulate = estimated_mission._simulate_mission
    result = probe.run(30, tmp_path, horizon_s=0.04)
    assert result["peak_m"] is None and not result["passes_prefix_screen"]
    assert mission_simulation.compute_attitude_control is original_inner
    assert estimated_mission._simulate_mission is original_simulate
    with np.load(tmp_path / result["file"], allow_pickle=False) as saved:
        assert saved["trace"].shape == (17, 10)
        assert saved["nominal_rotors"].shape == (4, 4)
        np.testing.assert_array_equal(saved["nominal_rotors"][0], np.full(4, np.sqrt(9.81 / 4e-5)))
    with pytest.raises(FileExistsError):
        probe.run(30, tmp_path, horizon_s=0.04)


def test_hooks_restore_after_unexpected_failure(monkeypatch, tmp_path):
    original_inner = mission_simulation.compute_attitude_control

    def broken(*args, **kwargs):
        raise RuntimeError("injected")

    monkeypatch.setattr(estimated_mission, "_simulate_mission", broken)
    with pytest.raises(RuntimeError, match="injected"):
        probe.run(30, tmp_path, horizon_s=0.04)
    assert estimated_mission._simulate_mission is broken
    assert mission_simulation.compute_attitude_control is original_inner
