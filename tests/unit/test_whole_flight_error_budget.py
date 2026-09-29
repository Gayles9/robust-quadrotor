"""Independent signs, chronology, worst-case attainment and evidence guards for ADR0039."""

import hashlib
import json

import numpy as np
import pytest

from experiments import whole_flight_error_budget as budget
from experiments.attitude_startup_audit import level_hover_information
from experiments.hover_response import cascade_model
from experiments.supported_validation_protocol import Job, configuration


def parameters():
    return dict(
        h=0.0025,
        tau=0.025,
        kp=1.0,
        kv=1.8,
        ka=3.0,
        kr=12.0,
        gravity=9.81,
        outer_stride=8,
        inner_stride=4,
    )


def evolve(time, initial, drives, params):
    return cascade_model(
        time, initial, *drives, **{k: v for k, v in params.items() if k not in ("h", "gravity")}
    )[0]


@pytest.mark.parametrize("frequency", [0.0, 0.01, 0.2, 2.5, 25.0])
def test_frequency_matches_separate_real_cascade(frequency):
    params = parameters()
    assert budget.check_phasor(budget.lift_cascade(**params), params, frequency) < 1e-10


def test_dc_equilibrium_including_signed_inclination_and_rate():
    lift = budget.lift_cascade(**parameters())
    _, X, Y = budget.phasor(lift, 0)
    expected = np.tile([-1, -1.8, -9.81, -3.27], (8, 1))
    np.testing.assert_allclose(Y, expected, atol=1e-12, rtol=0)
    np.testing.assert_allclose(X[1:], 0, atol=1e-12, rtol=0)


def test_lift_matches_arbitrary_inputs_and_initial_state_with_original_clocks():
    params = parameters()
    lift = budget.lift_cascade(**params)
    rng = np.random.default_rng(39001)
    time = np.arange(161) * params["h"]
    drives = [rng.normal(0, 0.01, (len(time), 2)) for _ in range(4)]
    initial = rng.normal(0, 0.01, (5, 2))
    state = evolve(time, initial, drives, params)
    x = initial.copy()
    for k in range(0, len(time) - 1, 8):
        U = np.array(
            [
                drives[group][k + phase] / (9.81 if group >= 2 else 1)
                for phase, group in zip(lift.phases, lift.groups, strict=True)
            ]
        )
        np.testing.assert_allclose(lift.C @ x + lift.D @ U, state[k : k + 8, 0], atol=1e-12, rtol=0)
        x = lift.A @ x + lift.B @ U
    np.testing.assert_allclose(x, state[-1, :5], atol=1e-12, rtol=0)


@pytest.mark.parametrize("group", range(4))
def test_impulse_bound_is_attained_by_adversarial_vector_signs(group):
    params = parameters()
    lift = budget.lift_cascade(**params)
    period, phase = 60, 7
    bounds, kernel = budget.impulse_bounds(lift, period)
    time = np.arange(period * 8 + phase + 1) * params["h"]
    drives = [np.zeros((len(time), 2)) for _ in range(4)]
    direction = np.array([0.6, -0.8])
    for m in range(period + 1):
        coefficients = lift.D[phase] if m == period else kernel[period - m - 1, phase]
        for slot in np.flatnonzero(lift.groups == group):
            k = m * 8 + lift.phases[slot]
            if k < len(time):
                drives[group][k] = (
                    np.sign(coefficients[slot]) * direction * (9.81 if group >= 2 else 1)
                )
    state = evolve(time, np.zeros((5, 2)), drives, params)
    np.testing.assert_allclose(
        state[-1, 0], direction * bounds[period, phase, group], atol=1e-12, rtol=0
    )
    # The bound includes only acquired samples: before the first flow, all gains are zero.
    np.testing.assert_array_equal(bounds[0, 0], np.zeros(4))


def test_gram_preserves_cancellation_nonzero_means_and_vector_axes():
    channel = np.array(
        [
            [[1.0, 2.0, 0], [-1.0, -2.0, 0], [0.0, 0.0, 3.0]],
            [[2.0, 1.0, 0], [-2.0, -1.0, 0], [0.0, 0.0, 3.0]],
        ]
    )
    value = budget.gram_budget(channel, channel.sum(axis=1))
    assert value["mse_m2"] == 9
    assert value["diagonal_m2"] == 19
    assert value["cross_m2"] == -10
    assert value["mean_error_W"] == [0, 0, 3]
    assert value["signed_shares_m2"] == [0, 0, 9]
    assert value["triangle_rms_bound_m"] > value["rms_m"]


@pytest.mark.parametrize("bad", ["sum", "nan", "empty", "shape"])
def test_gram_rejects_invalid_or_mismatched_data(bad):
    channels, error = np.zeros((2, 2, 3)), np.zeros((2, 3))
    if bad == "sum":
        error[0, 1] = 1
    elif bad == "nan":
        channels[0, 0, 0] = np.nan
    elif bad == "empty":
        channels, error = channels[:0], error[:0]
    else:
        channels = channels[:, :, :2]
    with pytest.raises(ValueError):
        budget.gram_budget(channels, error)


def test_information_nullspace_and_why_velocity_is_not_new_rank():
    result = budget.information_structure(9.81)
    original = level_hover_information()
    np.testing.assert_array_equal(result["null_vectors"], original["null_vectors_columns"])
    assert result["cases"]["existing_position"]["rank"] == original["rank"] == 11
    assert result["cases"]["added_velocity"]["rank"] == 11
    assert result["cases"]["added_inclination"]["rank"] == 13
    np.testing.assert_array_equal(
        result["cases"]["added_velocity"]["null_output_norm"], np.zeros(4)
    )
    # Directly solve the null equations, without constructing an observability matrix.
    F, N = np.array(result["F"]), np.array(result["null_vectors"])
    np.testing.assert_array_equal((F @ N)[:, :3], np.zeros((15, 3)))
    np.testing.assert_array_equal(F @ N[:, 3], -N[:, 2])
    np.testing.assert_array_equal(N[6:8, :2], np.eye(2))
    np.testing.assert_array_equal(N[6:8, 2:], np.zeros((2, 2)))


def test_sensor_difference_noise_includes_shared_sample_covariance():
    value = budget.sensor_budget(configuration(Job("nominal_hover", 47001), "campaign"))
    assert value["difference_velocity_std_m_s"] == pytest.approx([np.sqrt(2) * 0.1] * 3)
    assert value["difference_acceleration_std_m_s2"] == pytest.approx([np.sqrt(6) * 0.5] * 3)
    assert value["position_difference_velocity_covariance_m2_s"] == pytest.approx([0.002] * 3)
    assert value["successive_difference_velocity_covariance_m2_s2"] == pytest.approx([-0.01] * 3)


@pytest.mark.parametrize(
    "key,value", [("outer_stride", 7), ("inner_stride", 0), ("gravity", -1), ("h", np.nan)]
)
def test_lift_rejects_bad_physics_or_clocks(key, value):
    params = parameters()
    params[key] = value
    with pytest.raises(ValueError):
        budget.lift_cascade(**params)


@pytest.mark.parametrize("frequency", [-1, 25.01, np.nan])
def test_frequency_rejects_invalid_or_aliased_outer_band(frequency):
    with pytest.raises(ValueError):
        budget.phasor(budget.lift_cascade(**parameters()), frequency)


def test_manifest_tampering_is_rejected_before_analysis(tmp_path, monkeypatch):
    payload = tmp_path / "data.npz"
    payload.write_bytes(b"original")
    manifest = json.dumps(
        [dict(file=payload.name, bytes=8, sha256=hashlib.sha256(b"original").hexdigest())]
    ).encode()
    (tmp_path / "manifest.json").write_bytes(manifest)
    monkeypatch.setattr(budget, "MANIFEST_SHA", hashlib.sha256(manifest).hexdigest())
    payload.write_bytes(b"tampered")
    with pytest.raises(ValueError, match="digest"):
        budget.authenticate(tmp_path)
