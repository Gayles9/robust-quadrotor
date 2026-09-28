"""Independent physical checks and isolation checks for the ADR 0025 diagnostics."""

import copy
import json

import numpy as np
import pytest

from experiments.early_flight_diagnostic import (
    authenticate_campaign,
    down_axis,
    force_budget,
    held_indices,
    independent_metrics,
    scalar_motor_step,
)
from experiments.early_flight_oracle import attitude_oracle, noise_pairing
from experiments.robustness_protocol import configuration
from quadrotor_math import estimated_mission, mission_simulation
from quadrotor_math.attitude_control import compute_attitude_control
from quadrotor_math.rotations import rotation_matrix_body_to_world


def test_down_axis_matches_independent_rotation_matrix():
    q = np.random.default_rng(93).normal(size=(200, 4))
    q /= np.linalg.norm(q, axis=1)[:, None]
    expected = np.array([rotation_matrix_body_to_world(row)[:, 2] for row in q])
    np.testing.assert_allclose(down_axis(q), expected, atol=1e-15, rtol=0)
    np.testing.assert_array_equal(down_axis(-q), down_axis(q))


@pytest.mark.parametrize("q", [np.zeros((1, 4)), np.ones(4), np.full((1, 4), np.nan)])
def test_down_axis_rejects_invalid_input(q):
    with pytest.raises(ValueError):
        down_axis(q)


def test_hold_clock_at_boundary_and_terminal():
    np.testing.assert_array_equal(
        held_indices(np.array([0.0, 0.02, 0.04]), np.array([0.0, 0.01, 0.02, 0.039, 0.04, 0.06])),
        [0, 0, 1, 1, 2, 2],
    )


@pytest.mark.parametrize("clock", [np.array([]), np.array([0.01]), np.array([0.0, 0.0])])
def test_hold_rejects_invalid_clock(clock):
    with pytest.raises(ValueError):
        held_indices(clock, np.array([0.0, 0.01]))


@pytest.mark.parametrize("speed,command", [(495.0, 520.0), (520.0, 490.0), (500.0, 500.0)])
def test_motor_integrals_against_numerical_quadrature(speed, command):
    h, tau, mass, kf, gravity = 0.02, 0.025, 1.1, 1e-5, 9.81
    t = np.linspace(0, h, 100001)
    omega = command + (speed - command) * np.exp(-t / tau)
    acceleration = gravity - 4 * kf * omega**2 / mass
    expected_z = 0.1 + 0.2 * h + np.trapezoid((h - t) * acceleration, t)
    expected_v = 0.2 + np.trapezoid(acceleration, t)
    z, v, end = scalar_motor_step(
        0.1, 0.2, speed, command, mass=mass, kf=kf, gravity=gravity, tau=tau, h=h
    )
    assert z == pytest.approx(expected_z, abs=2e-12)
    assert v == pytest.approx(expected_v, abs=2e-12)
    assert end == pytest.approx(omega[-1], abs=1e-12)


def test_vertical_equilibrium_and_ned_sign():
    hover = np.sqrt(1.1 * 9.81 / (4e-5))
    z, v, _ = scalar_motor_step(
        0, 0, hover, hover, mass=1.1, kf=1e-5, gravity=9.81, tau=0.025, h=0.02
    )
    assert abs(z) < 1e-16 and abs(v) < 1e-15
    nominal = np.sqrt(9.81 / (4e-5))
    z, v, _ = scalar_motor_step(
        0, 0, nominal, nominal, mass=1.1, kf=1e-5, gravity=9.81, tau=0.025, h=0.02
    )
    deficit = 9.81 * (1 - 1 / 1.1)
    assert z == pytest.approx(0.5 * deficit * 0.02**2)
    assert v == pytest.approx(deficit * 0.02)


def test_force_budget_mass_and_tilt_have_physical_sign():
    args = configuration("mass_tracking", "campaign")
    angle = 0.1
    a = {
        "mission_time_s": np.array([0.0, 0.01]),
        "mission_control_time_s": np.array([0.0]),
        "mission_position_control_time_s": np.array([0.0]),
        "mission_q_WB": np.tile([np.cos(angle / 2), 0, np.sin(angle / 2), 0], (2, 1)),
        "estimate_q_WB": np.tile([1.0, 0, 0, 0], (2, 1)),
        "mission_q_reference_WB": np.array([[1.0, 0, 0, 0]]),
        "mission_actual_rotor_omega": np.full((2, 4), np.sqrt(9.81 / 4e-5)),
        "mission_collective_thrust": np.array([9.81]),
        "mission_feasible_acceleration_W": np.zeros((1, 3)),
    }
    b = force_budget(a, args)
    assert b["attitude_estimation"][0, 0] == pytest.approx(-9.81 / 1.1 * np.sin(angle))
    assert b["actual"][0, 2] == pytest.approx(9.81 - 9.81 / 1.1 * np.cos(angle))
    np.testing.assert_allclose(b["mass"][:, 2], 9.81 * (1 - 1 / 1.1))
    np.testing.assert_array_equal(b["attitude_tracking"], np.zeros((2, 3)))


def test_report_tampering_rejected_before_payloads(tmp_path):
    (tmp_path / "report.json").write_text("{}")
    with pytest.raises(ValueError, match="byte digest"):
        authenticate_campaign(tmp_path, "baseline")


def test_scores_include_startup_and_both_hover_endpoints():
    a = {
        "mission_time_s": np.array([0.0, 5.0, 11.0, 12.0]),
        "mission_position_W": np.array([[3.0, 0, 0], [1.0, 0, 0], [2.0, 0, 0], [0.0, 0, 0]]),
        "mission_reference_position_W": np.zeros((4, 3)),
        "mission_velocity_W": np.zeros((4, 3)),
    }
    scores = independent_metrics(a, "nominal_hover")
    assert scores["tracking_rmse_m"] == pytest.approx(np.sqrt(14 / 4))
    assert scores["hover_peak_m"] == 2


@pytest.mark.parametrize("fail", [False, True])
def test_oracle_changes_only_inner_quaternion_and_restores(monkeypatch, fail):
    truth = (np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3))
    feedback = (np.ones(3), np.ones(3) * 2, np.array([0.99, 0.01, 0.02, 0.03]), np.ones(3) * 3)
    sentinel = object()
    received = []

    def inner(*args):
        received.append(args)
        return sentinel

    def simulate(*args, **kwargs):
        returned = args[10](0, 0.0, truth, np.zeros(4))
        assert returned is feedback  # Guard and outer-loop input remains the original object.
        assert kwargs == {"observation_guard": sentinel}
        result = mission_simulation.compute_attitude_control(
            returned[2], returned[3], sentinel, 9.81, sentinel
        )
        assert result is sentinel
        if fail:
            raise RuntimeError("deliberate failure")
        return result

    monkeypatch.setattr(estimated_mission, "_simulate_mission", simulate)
    monkeypatch.setattr(mission_simulation, "compute_attitude_control", inner)
    try:
        with attitude_oracle() as trace:
            result = estimated_mission._simulate_mission(
                *([None] * 10), lambda *a: feedback, observation_guard=sentinel
            )
            assert result is sentinel
    except RuntimeError:
        assert fail
    assert estimated_mission._simulate_mission is simulate
    assert mission_simulation.compute_attitude_control is inner
    np.testing.assert_array_equal(received[0][0], truth[2])
    assert received[0][1] is feedback[3]
    assert received[0][2:] == (sentinel, 9.81, sentinel)
    assert len(trace) == 1 and trace[0]["estimate_q_WB"] == feedback[2].tolist()


def test_short_oracle_commands_use_truth_attitude_and_estimated_rate():
    args = configuration("nominal_hover", "smoke")
    with attitude_oracle() as trace:
        result = estimated_mission.simulate_estimated_mission(**args)
    m = result.mission
    assert len(trace) == len(m.control_time_s) > 1
    assert not np.array_equal(m.q_WB[0], result.estimates.states[0].q_WB)
    for i, t in enumerate(m.control_time_s):
        k = int(round(t / args["numerics"].time_step_s))
        expected = compute_attitude_control(
            m.q_WB[k],
            result.angular_velocity_estimate_B[k],
            m.q_reference_WB[i],
            m.collective_thrust[i],
            args["attitude_controller"],
        )
        np.testing.assert_array_equal(
            m.commanded_rotor_omega[i], expected.allocation.commanded_rotor_omega
        )
        assert trace[i]["feedback_q_WB"] == m.q_WB[k].tolist()


def noise_fixture(speed, position):
    events = [
        {
            "observation": {
                "acquisition_index": 0,
                "measurement": (position + np.array([0.01, 0.02, 0.03])).tolist(),
            }
        },
        {"observation": {"acquisition_index": 0, "measurement": [-position[2] + 0.04]}},
    ]
    return {
        "mission_time_s": np.array([0.0, 0.0025]),
        "mission_actual_rotor_omega": np.full((2, 4), speed),
        "mission_position_W": np.tile(position, (2, 1)),
        "mission_omega_B": np.zeros((2, 3)),
        "specific_force_measurements_B": np.tile([0.001, 0.002, -4e-5 * speed**2 + 0.003], (2, 1)),
        "angular_velocity_measurements_B": np.full((2, 3), 0.003),
        "true_accelerometer_bias_B": np.zeros((2, 3)),
        "true_gyroscope_bias_B": np.zeros((2, 3)),
        "metadata_json": np.frombuffer(json.dumps({"events": events}).encode(), dtype=np.uint8),
    }


def test_noise_pairing_removes_changed_physical_trajectory():
    a = noise_fixture(500.0, np.zeros(3))
    b = noise_fixture(530.0, np.array([0.1, 0.2, 0.3]))
    assert max(noise_pairing(a, b).values()) < 2e-15


@pytest.mark.parametrize(
    "field",
    [
        "specific_force_measurements_B",
        "angular_velocity_measurements_B",
        "true_accelerometer_bias_B",
        "true_gyroscope_bias_B",
        "metadata_json",
    ],
)
def test_noise_pairing_rejects_changed_random_draws(field):
    a = noise_fixture(500.0, np.zeros(3))
    b = copy.deepcopy(a)
    if field == "metadata_json":
        m = json.loads(b[field].tobytes())
        m["events"][0]["observation"]["measurement"][0] += 0.1
        b[field] = np.frombuffer(json.dumps(m).encode(), dtype=np.uint8)
    else:
        b[field][0, 0] += 0.1
    with pytest.raises(ValueError, match="draws differ"):
        noise_pairing(a, b)
