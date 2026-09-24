"""Analytic references, supervisor transitions, guard priority and invalid input."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.attitude_control_validation import axis_quaternion
from experiments.position_control_validation import baseline_mission, safety_limits
from quadrotor_math.missions import (
    MissionPhase as P,
)
from quadrotor_math.missions import (
    MissionSegment,
    MissionState,
    advance_mission,
    mission_guard_reason,
    mission_reference,
    sample_segment,
)
from quadrotor_math.missions import (
    ReferenceKind as K,
)


def segment():
    return MissionSegment(P.TAKEOFF, 4.0, np.array([1.0, 2, 3]), np.array([2.0, -1, -2]), K.SMOOTH)


@pytest.mark.parametrize("time", [0.0, 4.0])
def test_exact_rest_endpoints(time):
    s = segment()
    target = sample_segment(s, time, 1.0)
    np.testing.assert_array_equal(
        target.position_W, s.start_position_W if time == 0 else s.end_position_W
    )
    np.testing.assert_array_equal(target.velocity_W, 0)
    np.testing.assert_array_equal(target.acceleration_W, 0)
    assert target.yaw_rad == 1.0


@pytest.mark.parametrize("time", [0.3, 1.0, 2.0, 3.1])
def test_quintic_derivatives_independently(time):
    h = 1e-4
    a, b, c = (sample_segment(segment(), time + d, 0.0) for d in (-h, 0, h))
    np.testing.assert_allclose((c.position_W - a.position_W) / (2 * h), b.velocity_W, atol=5e-9)
    np.testing.assert_allclose((c.velocity_W - a.velocity_W) / (2 * h), b.acceleration_W, atol=6e-9)
    expected = segment().start_position_W + (
        segment().end_position_W - segment().start_position_W
    ) * np.polyval([6, -15, 10, 0, 0, 0], time / 4)
    np.testing.assert_allclose(b.position_W, expected, atol=2e-15)


def test_hold_step_and_clamping_semantics():
    hold = replace(segment(), end_position_W=segment().start_position_W, kind=K.HOLD)
    for time in [-1, 0, 1, 4, 5]:
        np.testing.assert_array_equal(
            sample_segment(hold, time, 0).position_W, hold.start_position_W
        )
    step = replace(segment(), phase=P.TRACK, kind=K.STEP)
    np.testing.assert_array_equal(sample_segment(step, -1, 0).position_W, step.start_position_W)
    np.testing.assert_array_equal(sample_segment(step, 0, 0).position_W, step.end_position_W)


def test_right_continuous_boundaries_and_final_hold():
    plan = baseline_mission("vertical_step")
    for time, phase in [
        (0, P.INITIALIZE),
        (1, P.TAKEOFF),
        (5, P.TRACK),
        (15, P.LAND),
        (19, P.LAND),
        (100, P.LAND),
    ]:
        assert mission_reference(plan, time)[1] == phase
    assert mission_reference(plan, 7 - 1e-10)[0].position_W[2] == -1
    assert mission_reference(plan, 7)[0].position_W[2] == -1.5


def test_completion_dwell_resets_on_exit_and_terminal_latches():
    plan = baseline_mission("smoke")
    state = advance_mission(MissionState(), plan, 0.04, np.zeros(3), np.zeros(3))
    assert state.phase == P.LAND and state.settled_since_s == 0.04
    state = advance_mission(state, plan, 0.045, np.ones(3), np.zeros(3))
    assert state.settled_since_s is None
    state = advance_mission(state, plan, 0.05, np.zeros(3), np.zeros(3))
    state = advance_mission(state, plan, 0.062, np.zeros(3), np.zeros(3))
    assert state.phase == P.COMPLETE
    assert advance_mission(state, plan, 0.1, np.ones(3), np.ones(3), "geofence") is state


def test_speed_band_timeout_and_guard_priority():
    plan = baseline_mission("smoke")
    state = advance_mission(MissionState(), plan, 0.04, np.zeros(3), np.ones(3))
    assert state.settled_since_s is None
    state = advance_mission(state, plan, 0.08, np.zeros(3), np.ones(3))
    assert state.phase == P.ABORT and state.reason == "landing_timeout"
    assert advance_mission(state, plan, 0.09, np.zeros(3), np.zeros(3)) is state
    settled = advance_mission(MissionState(), plan, 0.04, np.zeros(3), np.zeros(3))
    assert advance_mission(settled, plan, 0.06, np.zeros(3), np.zeros(3), "tilt").reason == "tilt"


def test_geofence_closed_boundaries_tilt_and_priority():
    limits = safety_limits()
    identity = np.array([1.0, 0, 0, 0])
    assert mission_guard_reason(limits.maximum_position_W, identity, limits) is None
    assert mission_guard_reason(limits.minimum_position_W, identity, limits) is None
    outside = limits.maximum_position_W.copy()
    outside[0] += 1e-10
    tilted = axis_quaternion(np.array([1.0, 0, 0]), np.deg2rad(36))
    assert mission_guard_reason(outside, tilted, limits) == "geofence"
    assert mission_guard_reason(np.zeros(3), tilted, limits) == "tilt"
    assert (
        mission_guard_reason(np.zeros(3), axis_quaternion(np.array([0.0, 0, 1]), np.pi), limits)
        is None
    )


@pytest.mark.parametrize(
    "changes",
    [
        {"duration_s": 0},
        {"duration_s": True},
        {"duration_s": np.inf},
        {"kind": "smooth"},
        {"kind": K.HOLD},
        {"kind": K.STEP},
        {"phase": P.ABORT},
        {"phase": 1},
        {"start_position_W": np.ones(2)},
        {"end_position_W": np.full(3, np.nan)},
    ],
)
def test_invalid_segments(changes):
    with pytest.raises(ValueError):
        replace(segment(), **changes)


@pytest.mark.parametrize(
    "changes",
    [
        {"segments": ()},
        {"segments": (segment(),)},
        {"completion_dwell_s": 0},
        {"completion_timeout_s": 0.001},
        {"yaw_rad": True},
        {"completion_velocity_tolerance_m_s": np.nan},
    ],
)
def test_invalid_plans(changes):
    with pytest.raises(ValueError):
        replace(baseline_mission("smoke"), **changes)


def test_disconnected_plan_rejects_and_arrays_are_owned():
    plan = baseline_mission("smoke")
    segments = list(plan.segments)
    segments[2] = replace(segments[2], start_position_W=np.ones(3), end_position_W=np.ones(3))
    with pytest.raises(ValueError, match="connected"):
        replace(plan, segments=tuple(segments))
    snapshot = replace(plan)
    assert not np.shares_memory(
        snapshot.segments[0].start_position_W, plan.segments[0].start_position_W
    )
    assert not snapshot.segments[0].start_position_W.flags.writeable


@pytest.mark.parametrize(
    "changes",
    [
        {"maximum_tilt_rad": np.pi / 2},
        {"maximum_tilt_rad": True},
        {"maximum_position_W": np.full(3, np.inf)},
        {"minimum_position_W": np.full(3, 5.0)},
    ],
)
def test_invalid_safety(changes):
    with pytest.raises(ValueError):
        replace(safety_limits(), **changes)


@pytest.mark.parametrize(
    "changes",
    [
        {"phase": 0},
        {"phase": P.ABORT},
        {"reason": "tilt"},
        {"settled_since_s": 1},
        {"time_s": -1},
        {"time_s": True},
    ],
)
def test_invalid_supervisor_state(changes):
    with pytest.raises(ValueError):
        MissionState(**changes)


def test_backward_time_and_nonfinite_reference_fail():
    with pytest.raises(ValueError, match="backward"):
        advance_mission(
            MissionState(time_s=1), baseline_mission("smoke"), 0, np.zeros(3), np.zeros(3)
        )
    for bad in [True, np.nan, np.inf]:
        with pytest.raises(ValueError):
            sample_segment(segment(), bad, 0)


def test_reference_overflow_and_indistinguishable_clock_fail():
    with pytest.raises(ValueError, match="finite"):
        sample_segment(
            replace(
                segment(), start_position_W=np.full(3, -1e308), end_position_W=np.full(3, 1e308)
            ),
            2.0,
            0.0,
        )
    plan = baseline_mission("smoke")
    segments = (replace(plan.segments[0], duration_s=1e100), *plan.segments[1:])
    with pytest.raises(ValueError, match="clock"):
        replace(plan, segments=segments)
