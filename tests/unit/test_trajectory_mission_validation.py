"""Frozen campaign definition, failure scoring and mathematical timing evidence."""

from dataclasses import replace

import numpy as np

from experiments.position_control_validation import make_case
from experiments.trajectory_mission_validation import CASES, TARGETS, build_mission, score_result
from quadrotor_math.mission_simulation import simulate_mission
from quadrotor_math.missions import MinimumSnapMissionSegment


def test_fixed_protocol_timing_and_phase_acceptance():
    plan, timing = build_mission()
    assert CASES == ("nominal", "refined", "offset_positive", "offset_negative", "mild_wind")
    assert TARGETS["position_rmse_m"] == 0.15 and TARGETS["position_peak_m"] == 0.25
    assert timing["scale"] == 1.25**6
    assert len(timing["attempts"]) == 7
    assert all(not x["feasibility"]["violations"] for x in timing["attempts"][-1:])
    assert all(x["feasibility"]["violations"] for x in timing["attempts"][:-1])
    assert sum(isinstance(s, MinimumSnapMissionSegment) for s in plan.segments) == 3
    assert plan.completion_position_tolerance_m == 0.08
    assert plan.completion_velocity_tolerance_m_s == 0.08


def test_initial_abort_scores_failure_without_empty_window_nan():
    initial, body, world, outer, inner, plan, safety, numerics = make_case(
        {"case": "smoke", "seed": None, "refinement": 1}
    )
    initial = replace(initial, position_W=np.array([4.0, 0, 0]))
    result = simulate_mission(
        initial,
        np.full(4, np.sqrt(9.81 / 4e-5)),
        body,
        inner.nominal_rotors,
        world,
        outer,
        inner,
        plan,
        safety,
        numerics,
    )
    score = score_result(result)
    assert not score["passed"] and not score["complete"]
    assert score["abort_reason"] == "geofence"
    assert score["position_rmse_m"] is None
