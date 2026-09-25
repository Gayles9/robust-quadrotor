"""Independent mathematics, finite-budget policy and experimental isolation."""

from dataclasses import asdict

import numpy as np
import pytest

from experiments import feedback_codesign as design
from experiments import feedback_motor_damping_probe as preceding
from experiments.estimated_feedback_evidence import plain
from experiments.feedback_bandwidth_validation import make_configuration
from quadrotor_math import estimated_mission, mission_simulation
from quadrotor_math.cascade_analysis import HorizontalCascade, hover_axis_zero_order_hold

GAINS = design.Gains(14.4, 6, 12, 36, 0.5)


def empty_data():
    return {
        "errors": np.zeros((1001, 1, 2, 4)),
        "initial": np.zeros((1, 2, 5)),
        "actual": np.zeros((1001, 1, 2)),
    }


@pytest.mark.parametrize("bad", [True, np.bool_(False), float("nan"), float("inf"), 1j, "4", [4]])
def test_invalid_gain_types_and_values(bad):
    with pytest.raises(ValueError, match="gains"):
        design.Gains(bad, 4, 8, 16, 0)


@pytest.mark.parametrize(
    "index,value", [(0, 3.9), (1, 12.1), (2, 3.9), (3, 80.1), (4, -0.1), (4, 2.1)]
)
def test_frozen_gain_bounds(index, value):
    values = GAINS.array()
    values[index] = value
    with pytest.raises(ValueError, match="bounds"):
        design.Gains(*values)


def test_design_grid_determinism_anchors_and_coordinate_roundtrip():
    first = design.design_points()
    second = design.design_points()
    assert first == second and len(first) == 512
    assert first[:2] == [design.Gains(6.4, 4, 8, 16, 0), GAINS]
    for g in first:
        np.testing.assert_allclose(design.decode(design.encode(g)).array(), g.array(), rtol=1e-14)
    z = np.array([design.encode(g) for g in first[2:]])
    for j in range(5):
        np.testing.assert_array_equal(np.sort((510 * z[:, j]).astype(int)), np.arange(510))


@pytest.mark.parametrize(
    "point", [np.zeros(4), np.full(5, np.nan), np.full(5, -0.01), np.full(5, 1.01)]
)
def test_invalid_design_points(point):
    with pytest.raises(ValueError, match="point"):
        design.decode(point)


def test_zero_damping_matches_existing_multirate_cascade():
    a, _ = design.lifted_model(design.Gains(6.4, 4, 8, 16, 0))
    expected = HorizontalCascade(6.4, 4, 8, 16, 9.81, 0.025, 0.01, 2).lifted_transition()
    np.testing.assert_array_equal(a, expected)
    np.testing.assert_array_equal(
        design.lifted_model(GAINS)[0], preceding.local_model()["lifted_transition"]
    )


def test_noise_lift_matches_two_explicit_ticks_and_error_signs():
    a, b = design.lifted_model(GAINS)
    state = np.array([0.02, -0.03, 0.01, -0.05, 0.1])
    errors = np.array([0.7, -0.9, 0.3, -0.5, -0.2, 0.4])
    expected = state.copy()
    desired = -(14.4 * (state[0] + 0.02 * errors[0]) + 6 * (state[1] + 0.02 * errors[1])) / 9.81
    phi, gamma = hover_axis_zero_order_hold(9.81, 0.025, 0.01)
    for k in range(2):
        rate = 12 * (desired - expected[2] - np.deg2rad(1) * errors[2 + 2 * k])
        command = 36 * (rate - expected[3] - 0.002 * errors[3 + 2 * k]) - 0.5 * expected[4]
        expected = phi @ expected + gamma * command
    np.testing.assert_allclose(a @ state + b @ errors, expected, atol=3e-15)
    equilibrium = np.linalg.solve(np.eye(5) - a, b[:, 0])
    np.testing.assert_allclose(equilibrium, [-0.02, 0, 0, 0, 0], atol=2e-13)


def test_lyapunov_noise_energy_matches_independent_impulse_sum():
    a, b = design.lifted_model(GAINS)
    impulse = b.copy()
    energy = 0.0
    for _ in range(500):
        energy += np.sum(impulse[0] ** 2)
        impulse = a @ impulse
    np.testing.assert_allclose(design.noise_response(a, b) ** 2, energy, rtol=2e-12)
    with pytest.raises(ValueError, match="stable"):
        design.noise_response(np.eye(5), b)


def test_replay_zero_and_constant_position_error_equilibrium():
    data = empty_data()
    row = design.replay([GAINS], data)[0]
    assert all(value == 0 for value in row.values())
    data["errors"][:, 0, 0, 0] = 0.01
    data["initial"][0, 0, 0] = -0.01
    row = design.replay([GAINS], data)[0]
    assert row["peak_m"] == pytest.approx(0.01)
    assert row["position_mse_m2"] == pytest.approx(0.0001)
    assert row["moment_peak_Nm"] == 0


def test_north_east_inertia_and_all_objective_components():
    data = empty_data()
    data["initial"][0, :, 0] = 0.001
    row = design.replay([GAINS], data)[0]
    # At t=0, u=-kr*ka*kp*p/g, North pitch inertia is .025 not roll .020.
    expected = 0.025 * 36 * 12 * 14.4 * 0.001 / 9.81
    assert row["moment_peak_Nm"] == pytest.approx(expected)
    terms = dict(
        peak_m=0.07,
        position_mse_m2=0.03**2,
        moment_mse_Nm2=0.15**2,
        moment_peak_Nm=0.8,
        noise_response_m=0.02,
    )
    assert design.objective(terms) == pytest.approx(1.5)
    for key in terms:
        smaller = terms | {key: 0.0}
        assert design.objective(smaller) < design.objective(terms)


@pytest.mark.parametrize("key", ["errors", "initial", "actual"])
def test_replay_rejects_nonfinite_and_bad_shapes(key):
    data = empty_data()
    data[key].flat[0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        design.replay([GAINS], data)
    data = empty_data()
    data[key] = data[key][:-1]
    with pytest.raises(ValueError, match="shape"):
        design.replay([GAINS], data)


def test_source_identity_before_development_decode(tmp_path):
    (tmp_path / "report.json").write_text("{}")
    with pytest.raises(ValueError, match="identity"):
        design.load_development(tmp_path)


def test_search_budget_and_selection_are_not_nonlinear_sweeps(monkeypatch):
    sizes = []

    def evaluate(gains, data):
        sizes.append(len(gains))
        return [
            {
                "gains": g.array().tolist(),
                "admissible": True,
                "objective": float(np.sum(g.array() ** 2)),
            }
            for g in gains
        ]

    monkeypatch.setattr(design, "evaluate", evaluate)
    rows = design.synthesize(empty_data())
    assert sizes == [512, 10, 10, 10] and len(rows) == 542
    assert design.select(rows)["objective"] == min(r["objective"] for r in rows)
    with pytest.raises(ValueError, match="no admissible"):
        design.select([{"admissible": False}])


def test_no_feasible_candidate_preserves_failed_ledger(monkeypatch):
    ledger = [{"admissible": False}] * 512
    monkeypatch.setattr(design, "evaluate", lambda *args: ledger)
    assert design.synthesize(empty_data()) is ledger


@pytest.mark.parametrize("mutation", ["source", "inputs", "budget", "winner", "failed"])
def test_selected_candidate_refuses_stale_or_changed_records(monkeypatch, mutation):
    monkeypatch.setattr(design, "source_sha256", lambda root: "current")
    row = {"gains": GAINS.array().tolist(), "admissible": True, "objective": 1.0}
    record = {
        "source_sha256": "current",
        "input_report_sha256": design.REPORT_SHA256,
        "evaluations": [row],
        "selected": row,
        "budget": 542,
    }
    assert design.selected_gains(record) == GAINS
    if mutation == "source":
        record["source_sha256"] = "stale"
    elif mutation == "inputs":
        record["input_report_sha256"] = "different"
    elif mutation == "budget":
        record["budget"] = 543
    elif mutation == "winner":
        record["selected"] = row | {"objective": 2.0}
    else:
        record["selected"] = None
    with pytest.raises(ValueError):
        design.selected_gains(record)


@pytest.mark.parametrize("seed", [95000, 96000, True, 30.0, -1])
def test_reserved_and_invalid_seeds_rejected(seed):
    with pytest.raises(ValueError, match="observed"):
        design.candidate_configuration(seed, GAINS)


def test_configuration_changes_only_declared_gains():
    original = make_configuration(
        {"case": "hover", "seed": 30, "noiseless": False}, design_version=2
    )
    candidate = design.candidate_configuration(30, GAINS)
    for key in original:
        a = (
            asdict(original[key])
            if hasattr(original[key], "__dataclass_fields__")
            else original[key]
        )
        b = (
            asdict(candidate[key])
            if hasattr(candidate[key], "__dataclass_fields__")
            else candidate[key]
        )
        if key in ("position_controller", "attitude_controller"):
            fields = (
                ("position_gain_W", "velocity_gain_W")
                if key == "position_controller"
                else ("attitude_gain_B", "rate_gain_B")
            )
            for field in fields:
                assert a[field][2] == b[field][2]
                b[field] = a[field]
        assert plain(a) == plain(b)


def test_shared_execution_preserves_previous_probe_exactly(tmp_path):
    original_inner = mission_simulation.compute_attitude_control
    original_sim = estimated_mission._simulate_mission
    old_dir, new_dir = tmp_path / "old", tmp_path / "new"
    old_dir.mkdir()
    new_dir.mkdir()
    old = preceding.run(30, old_dir, horizon_s=0.04)
    new = design.run_candidate(30, new_dir, GAINS, horizon_s=0.04)
    assert not new["passes_margin_screen"] and not new["passes_prefix_screen"]
    assert old["sha256"] == new["sha256"]
    assert mission_simulation.compute_attitude_control is original_inner
    assert estimated_mission._simulate_mission is original_sim


@pytest.mark.parametrize("damping", [-0.1, 2.1, True, np.nan])
def test_private_probe_rejects_invalid_damping_before_execution(tmp_path, damping):
    with pytest.raises(ValueError, match="damping"):
        preceding._run_configuration(30, {}, tmp_path, damping=damping)


def test_margin_is_stricter_than_inherited_requirement(monkeypatch, tmp_path):
    monkeypatch.setattr(
        design,
        "_run_configuration",
        lambda *a, **k: {
            "passes_prefix_screen": True,
            "peak_m": 0.075,
        },
    )
    assert not design.run_candidate(30, tmp_path, GAINS)["passes_margin_screen"]
