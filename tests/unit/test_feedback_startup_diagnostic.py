"""Oracle isolation and exact prefix preservation for the diagnostic experiment."""

import numpy as np
import pytest

from experiments.feedback_startup_diagnostic import (
    FIELDS,
    MODES,
    authenticate_original_prefix,
    run_prefix,
    select_feedback,
)
from quadrotor_math import estimated_mission


@pytest.mark.parametrize("mode", MODES)
def test_only_declared_feedback_channels_are_replaced_without_mutation(mode):
    truth = tuple(np.full(n, i + 1.0) for i, n in enumerate((3, 3, 4, 3)))
    estimate = tuple(-a for a in truth)
    before = [a.copy() for a in (*truth, *estimate)]
    selected = select_feedback(truth, estimate, mode)
    for i, value in enumerate(selected):
        assert value is (truth[i] if i in MODES[mode] else estimate[i])
    for current, previous in zip((*truth, *estimate), before, strict=True):
        np.testing.assert_array_equal(current, previous)


@pytest.mark.parametrize(
    "seed,mode,horizon",
    [
        (95000, "estimated", 1),
        (93003, "bad", 1),
        (True, "estimated", 1),
        (93003, "estimated", 10.1),
        (93003, "estimated", 0),
        (93003, "estimated", float("nan")),
        (93003, "estimated", 0.001),
    ],
)
def test_invalid_or_unobserved_experiments_are_rejected(seed, mode, horizon):
    with pytest.raises(ValueError):
        run_prefix(seed, mode, horizon)


def test_noop_is_deterministic_and_private_hook_is_restored():
    original = estimated_mission._simulate_mission
    a, b = run_prefix(93003, "estimated", 0.04), run_prefix(93003, "estimated", 0.04)
    assert estimated_mission._simulate_mission is original
    assert len(a["time_s"]) == 17
    for key in a:
        np.testing.assert_array_equal(a[key], b[key])
    for name in FIELDS:
        np.testing.assert_array_equal(a["feedback_" + name], a["estimate_" + name])


def test_hook_restores_on_unexpected_error(monkeypatch):
    def broken(*args, **kwargs):
        raise RuntimeError("injected failure")

    monkeypatch.setattr(estimated_mission, "_simulate_mission", broken)
    with pytest.raises(RuntimeError, match="injected failure"):
        run_prefix(93003, "estimated", 0.04)
    assert estimated_mission._simulate_mission is broken


def test_truth_all_affects_feedback_without_overwriting_estimator():
    a = run_prefix(93003, "truth_all", 0.04)
    for name in FIELDS:
        np.testing.assert_array_equal(a["feedback_" + name], a["truth_" + name])
    assert not np.array_equal(a["estimate_position_W"], a["truth_position_W"])


def test_prefix_authentication_detects_one_changed_value():
    a = run_prefix(93003, "estimated", 0.01)
    archive = {
        "mission_time_s": a["time_s"],
        "mission_reference_position_W": a["reference_position_W"],
        **{"mission_" + n: a["truth_" + n].copy() for n in FIELDS},
        **{"estimate_" + n: a["estimate_" + n] for n in FIELDS[:3]},
        "angular_velocity_estimate_B": a["estimate_omega_B"],
    }
    authenticate_original_prefix(a, archive)
    archive["mission_position_W"][1, 0] = np.nextafter(archive["mission_position_W"][1, 0], np.inf)
    with pytest.raises(ValueError, match="truth_position_W"):
        authenticate_original_prefix(a, archive)
