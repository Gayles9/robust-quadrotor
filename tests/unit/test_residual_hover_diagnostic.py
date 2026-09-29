"""Independent response identities and diagnostic failure checks for ADR 0032."""

import numpy as np
import pytest

from experiments.early_flight_diagnostic import check_plant
from experiments.estimated_feedback_evidence import pack_history
from experiments.hover_response import cascade_model, cascade_transition, pd_response
from experiments.residual_hover_diagnostic import (
    CHANNELS,
    authenticate,
    command_residual,
    local_cascade,
    response_budget,
)
from experiments.supported_start import configuration_for, fly
from experiments.supported_validation_protocol import Job, prepare_job


def test_sample_held_pd_matches_analytic_constant_acceleration():
    t = np.arange(5) * 0.1
    p0 = np.array([[0.2, -0.1, 0.3]])
    v0 = np.array([[0.1, 0.2, -0.1]])
    kp, kv = np.array([1.0, 2.0, 3.0]), np.array([2.0, 3.0, 4.0])
    forcing = np.tile([0.5, 0.1, -0.3], (4, 1, 1))
    zeros = np.zeros_like(forcing)
    p, v = pd_response(t, np.zeros(4, dtype=int), kp, kv, forcing, zeros, zeros, p0, v0)
    acceleration = forcing[0] - kp * p0 - kv * v0
    np.testing.assert_allclose(
        p[:, 0], p0 + t[:, None] * v0 + t[:, None] ** 2 / 2 * acceleration, atol=1e-15, rtol=0
    )
    np.testing.assert_allclose(v[:, 0], v0 + t[:, None] * acceleration, atol=1e-15, rtol=0)


def test_forcing_channels_cancel_and_increments_propagate():
    t = np.arange(41) * 0.0025
    held = np.arange(40) // 8 * 8
    force = np.zeros((40, 3, 3))
    force[:, 0] = [1.0, -2.0, 3.0]
    force[:, 1] = -force[:, 0]
    dp, dv = np.zeros_like(force), np.zeros_like(force)
    dp[0, 2] = [0.1, 0.2, 0.3]
    dv[0, 2] = [-0.1, 0.3, 0.2]
    p, v = pd_response(
        t, held, np.ones(3), np.ones(3), force, dp, dv, np.zeros((3, 3)), np.zeros((3, 3))
    )
    np.testing.assert_array_equal(p[:, 0], -p[:, 1])
    np.testing.assert_array_equal(v[:, 0], -v[:, 1])
    np.testing.assert_array_equal(p[1, 2], dp[0, 2])
    np.testing.assert_array_equal(v[1, 2], dv[0, 2])
    np.testing.assert_allclose(p[8, 2], dp[0, 2] + 7 * 0.0025 * dv[0, 2], atol=1e-15)


@pytest.mark.parametrize("bad", ["future", "nan", "shape"])
def test_response_rejects_invalid_forcing_or_ownership(bad):
    force = np.zeros((2, 1, 3))
    hold = np.zeros(2, dtype=int)
    if bad == "future":
        hold[1] = 2
    elif bad == "nan":
        force[0, 0, 0] = np.nan
    else:
        force = np.zeros((2, 3))
    with pytest.raises(ValueError):
        pd_response(
            np.arange(3) * 0.01,
            hold,
            np.ones(3),
            np.ones(3),
            force,
            np.zeros_like(force),
            np.zeros_like(force),
            np.zeros((1, 3)),
            np.zeros((1, 3)),
        )


def test_cascade_flow_matches_analytic_motor_lag_and_composes():
    h, tau = 0.0025, 0.025
    flow = cascade_transition(h, tau)
    initial = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
    state = flow @ initial
    decay = np.exp(-h / tau)
    assert state[4] == pytest.approx(decay, abs=1e-15)
    assert state[3] == pytest.approx(tau * (1 - decay), abs=1e-15)
    assert state[2] == pytest.approx(tau * h - tau * tau * (1 - decay), abs=1e-15)
    np.testing.assert_allclose(flow @ flow, cascade_transition(2 * h, tau), atol=2e-16, rtol=0)


def test_cascade_feedback_sign_and_clocks():
    time = np.arange(17) * 0.0025
    errors = np.zeros((17, 2))
    initial = np.zeros((5, 2))
    initial[0, 0] = 0.1
    states, eig = cascade_model(
        time,
        initial,
        errors,
        errors,
        errors,
        errors,
        kp=1.0,
        kv=1.8,
        ka=3.0,
        kr=12.0,
        tau=0.025,
        outer_stride=8,
        inner_stride=4,
    )
    assert states[1, 5, 0] == -0.1
    assert states[1, 6, 0] == pytest.approx(-3.6)
    assert states[1, 4, 0] < 0
    np.testing.assert_array_equal(states[1:9, 5, 0], np.full(8, -0.1))
    assert states[9, 5, 0] != states[8, 5, 0]
    assert states[-1, 0, 0] < initial[0, 0]
    np.testing.assert_array_equal(states[:, :, 1], np.zeros((17, 7)))
    assert np.max(np.abs(eig)) < 1


def test_report_tampering_is_rejected_before_any_payload(tmp_path):
    (tmp_path / "report.json").write_text("{}")
    with pytest.raises(ValueError, match="byte digest"):
        authenticate(tmp_path)


def test_release_interval_split_preserves_response_at_and_after_boundary():
    t = np.arange(401) * 0.0025
    forcing = np.ones((400, 1, 3))
    early = (t[:-1] < 0.5)[:, None, None]
    hold = np.arange(400) // 8 * 8
    zeros = np.zeros_like(forcing)

    def response(f):
        return pd_response(
            t, hold, np.ones(3), np.ones(3), f, zeros, zeros, np.zeros((1, 3)), np.zeros((1, 3))
        )

    full_p, full_v = response(forcing)
    early_p, early_v = response(forcing * early)
    late_p, late_v = response(forcing * ~early)
    np.testing.assert_array_equal(late_p[:201], np.zeros((201, 1, 3)))
    assert np.all(late_p[201] > 0)
    assert np.all(early_p[201] != early_p[200])
    np.testing.assert_allclose(early_p + late_p, full_p, atol=2e-15, rtol=0)
    np.testing.assert_allclose(early_v + late_v, full_v, atol=2e-15, rtol=0)


def test_full_smoke_response_reconstructs_both_loops_and_release():
    prepared = prepare_job(Job("nominal_hover", 47811), "smoke")
    args = configuration_for(prepared, "aligned")
    result, _ = fly(prepared, "aligned")
    a = pack_history(result, result.mission)
    assert max(check_plant(a, args).values()) <= 1e-12
    assert command_residual(a, args) <= 1e-12
    b = response_budget(a, args)
    assert float(b["acceleration_closure"]) <= 1e-11
    assert float(b["response_closure"]) <= 1e-10
    np.testing.assert_allclose(
        b["response_position_W"].sum(axis=1), a["mission_position_W"], atol=1e-12, rtol=0
    )
    np.testing.assert_allclose(
        b["response_error_W"].sum(axis=1),
        a["mission_position_W"] - a["mission_reference_position_W"],
        atol=1e-12,
        rtol=0,
    )
    np.testing.assert_array_equal(b["actual_acceleration_W"][0], [0.0, 0.0, 9.81])
    np.testing.assert_array_equal(
        b["forcing_W"][:, CHANNELS.index("mass")], np.zeros((len(a["mission_time_s"]) - 1, 3))
    )
    # Every smoke interval is earlier than the frozen .5 s release boundary.
    for j, name in enumerate(CHANNELS):
        if name != "reference":
            np.testing.assert_array_equal(
                b["early_response_position_W"][:, j], b["response_position_W"][:, j]
            )
    _, model = local_cascade(a, args)
    assert np.isfinite(model["horizontal_rms_discrepancy_m"])
    a["mission_commanded_rotor_omega"] = a["mission_commanded_rotor_omega"].copy()
    a["mission_commanded_rotor_omega"][0, 0] += 0.01
    with pytest.raises(ValueError, match="command reconstruction"):
        command_residual(a, args)
