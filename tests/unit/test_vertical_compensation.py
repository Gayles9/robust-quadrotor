"""Independent scalar-law checks and explicit opt-in mission contracts."""

from dataclasses import asdict, replace

import numpy as np
import pytest

from experiments.estimated_feedback_evidence import pack_history
from experiments.robustness_protocol import configuration, policies
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.observation_health import ObservationHealthMonitor
from quadrotor_math.position_control import PositionReference, compute_position_control
from quadrotor_math.vertical_compensation import (
    VerticalCompensationPolicy,
    VerticalIntegralCompensator,
)


def make(bound=1.5):
    return VerticalIntegralCompensator(
        configuration("nominal_tracking", "smoke")["position_controller"],
        VerticalCompensationPolicy(0.5, bound, 0.02),
    )


def target(accel=0):
    return PositionReference(np.zeros(3), np.zeros(3), np.array([0.0, 0.0, accel]), 0.0)


def step(c, z=0.1, *, healthy=True, limited=False, reference=None):
    return c.step(
        len(c.history) * 0.02,
        np.array([0.0, 0.0, z]),
        np.zeros(3),
        target() if reference is None else reference,
        observations_healthy=healthy,
        previous_inner_limited=limited,
    )


@pytest.mark.parametrize("field", ["integral_gain", "maximum_acceleration", "update_period_s"])
@pytest.mark.parametrize("bad", [True, 0, -1, np.inf, np.nan, "1"])
def test_policy_rejects_invalid_units(field, bad):
    values = dict(integral_gain=0.5, maximum_acceleration=1.5, update_period_s=0.02)
    values[field] = bad
    with pytest.raises(ValueError):
        VerticalCompensationPolicy(**values)


def test_sign_current_command_and_next_integral_are_distinct():
    c = make()
    first = step(c)
    assert c.acceleration == pytest.approx(-0.001)
    assert first.requested_acceleration_W[2] == pytest.approx(-2.25 * 0.1)
    second = step(c)
    assert second.requested_acceleration_W[2] == pytest.approx(-2.25 * 0.1 - 0.001)
    assert c.acceleration == pytest.approx(-0.002)
    assert first.collective_thrust < second.collective_thrust
    assert np.array_equal(first.requested_acceleration_W[:2], second.requested_acceleration_W[:2])


def test_unhealthy_and_inner_limiting_freeze_without_catchup():
    c = make()
    step(c)
    before = c.acceleration
    for _ in range(20):
        step(c, healthy=False)
    assert c.acceleration == before and c.history[-1].update_reason == "unhealthy"
    step(c, limited=True)
    assert c.acceleration == before and c.history[-1].update_reason == "inner_limit"
    step(c)
    assert c.acceleration == pytest.approx(before - 0.001)


def test_outer_saturation_prevents_outward_growth_but_allows_unwind():
    c = make()
    step(c, z=-0.1)
    assert c.acceleration == pytest.approx(0.001)
    step(c, z=-0.1, reference=target(3))
    assert c.history[-1].update_reason == "outer_limit"
    assert c.acceleration == pytest.approx(0.001)
    step(c, z=0.1, reference=target(3))
    assert c.history[-1].update_reason == "integrating"
    assert c.acceleration == pytest.approx(0.0)


@pytest.mark.parametrize("z", [-0.1, 0.1])
def test_state_bound_and_reset(z):
    c = make(0.01)
    for _ in range(30):
        step(c, z)
    assert c.acceleration == np.copysign(0.01, -z)
    assert c.history[-1].bound_limited
    c.reset()
    assert c.acceleration == 0 and c.history == ()
    step(c, z)
    assert c.history[0].time_s == 0


@pytest.mark.parametrize(
    "bad", ["clock", "position", "velocity", "reference", "health", "limit", "overflow"]
)
def test_invalid_step_is_atomic(bad):
    c = make()
    step(c)
    before = c.history
    args = dict(
        time_s=0.02,
        position_W=np.array([0.0, 0.0, 0.1]),
        velocity_W=np.zeros(3),
        reference=target(),
        observations_healthy=True,
        previous_inner_limited=False,
    )
    if bad == "clock":
        args["time_s"] = 0.04
    elif bad == "position":
        args["position_W"] = np.array([0.0, 0.0, np.nan])
    elif bad == "velocity":
        args["velocity_W"] = np.zeros(2)
    elif bad == "reference":
        args["reference"] = None
    elif bad == "health":
        args["observations_healthy"] = 1
    elif bad == "limit":
        args["previous_inner_limited"] = 1
    else:
        args["position_W"] = np.full(3, 1e308)
    with pytest.raises((TypeError, ValueError)):
        c.step(**args)
    assert c.history == before and c.acceleration == before[-1].next_acceleration
    step(c)


def test_zero_error_is_exact_pd_and_owned_parameters():
    args = configuration("nominal_tracking", "smoke")
    c = make()
    actual = step(c, z=0)
    expected = compute_position_control(
        np.zeros(3), np.zeros(3), target(), args["position_controller"]
    )
    for name, value in asdict(actual).items():
        np.testing.assert_array_equal(value, getattr(expected, name))
    assert c.matches(args["position_controller"])
    assert not c.matches(replace(args["position_controller"], nominal_mass=1.1))
    with pytest.raises(ValueError):
        make(2.1)


def test_nominal_polynomial_has_declared_factored_poles():
    np.testing.assert_allclose(
        np.polymul([1, 0.5], np.polymul([1, 0.5], [1, 2])), [1, 3, 2.25, 0.5], rtol=0, atol=0
    )
    r = 1 / 1.1
    tau = 0.025
    matrix = np.array(
        [
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [-r * 2.25 / tau, -r * 3 / tau, -1 / tau, r / tau],
            [-0.5, 0, 0, 0.0],
        ]
    )
    assert np.max(np.linalg.eigvals(matrix).real) < -0.4


def test_sampled_lag_model_rejects_constant_mass_load_without_truth_input():
    # Independent exact held-input linear plant, not the mission integrator.
    h, tau, ratio = 0.02, 0.025, 1 / 1.1
    decay = np.exp(-h / tau)
    phi = np.array(
        [[1, h, tau * (h - tau * (1 - decay))], [0, 1, tau * (1 - decay)], [0, 0, decay]]
    )
    gamma = np.array(
        [h * h / 2 - tau * h + tau * tau * (1 - decay), h - tau * (1 - decay), 1 - decay]
    )
    transition = np.eye(4)
    transition[:3, :3] = phi
    transition[:3] += np.outer(gamma, [-ratio * 2.25, -ratio * 3, 0, ratio])
    transition[3, 0] = -0.5 * h
    assert np.max(abs(np.linalg.eigvals(transition))) < 1
    plant = np.zeros(3)
    disturbance = (1 - ratio) * 9.81
    c = make()
    for k in range(1000):
        command = c.step(
            k * h,
            np.array([0.0, 0.0, plant[0]]),
            np.array([0.0, 0.0, plant[1]]),
            target(),
            observations_healthy=k >= 20,
            previous_inner_limited=False,
        )
        plant = phi @ plant + gamma * (ratio * command.feasible_acceleration_W[2])
        plant += disturbance * np.array([h * h / 2, h, 0.0])
    assert abs(plant[0]) < 0.002
    assert c.acceleration == pytest.approx(-0.981, abs=0.01)
    assert all(abs(s.next_acceleration) <= 1.5 for s in c.history)


def test_disabled_option_keeps_every_default_payload_byte():
    args = configuration("nominal_tracking", "smoke")
    first = simulate_estimated_mission(**args)
    second = simulate_estimated_mission(**args, vertical_compensation=None)
    a, b = pack_history(first, first.mission), pack_history(second, second.mission)
    assert all(a[k].tobytes() == b[k].tobytes() for k in a)


@pytest.mark.parametrize("bad", ["type", "used", "parameters", "clock", "health", "geometric"])
def test_mission_preflight_precedes_random_streams(bad, monkeypatch):
    args = configuration("nominal_tracking", "smoke")
    h, _ = policies(args, "smoke")
    health = ObservationHealthMonitor(h)
    c = make()
    if bad == "type":
        c = True
    elif bad == "used":
        step(c)
    elif bad == "parameters":
        c = VerticalIntegralCompensator(
            replace(args["position_controller"], nominal_mass=1.1), c.policy
        )
    elif bad == "clock":
        c = VerticalIntegralCompensator(
            args["position_controller"], replace(c.policy, update_period_s=0.01)
        )
    elif bad == "health":
        health = None
    elif bad == "geometric":
        args["geometric_controller"] = True

    def fail(*a, **kw):
        raise AssertionError("RNG reached")

    monkeypatch.setattr("quadrotor_math.estimated_mission.create_run_random_streams", fail)
    with pytest.raises((TypeError, ValueError)):
        simulate_estimated_mission(**args, observation_health=health, vertical_compensation=c)
