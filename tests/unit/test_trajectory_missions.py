"""Polynomial reference integration without changing historical mission contracts."""

from dataclasses import asdict, replace

import numpy as np
import pytest

from experiments.position_control_validation import baseline_mission, make_case
from quadrotor_math.minimum_snap import minimum_snap_trajectory
from quadrotor_math.mission_simulation import simulate_mission
from quadrotor_math.missions import (
    MinimumSnapMissionSegment,
    MissionSegment,
    mission_reference,
    sample_segment,
)
from quadrotor_math.missions import (
    MissionPhase as P,
)
from quadrotor_math.missions import (
    ReferenceKind as K,
)


def segment(phase=P.TRACK, points=None, durations=None):
    if points is None:
        points = np.array([[0.0, 0, 0], [0.1, 0.1, -0.1]])
    if durations is None:
        durations = np.array([3.0])
    curve = minimum_snap_trajectory(points, durations)
    return MinimumSnapMissionSegment(
        phase, float(curve.knot_times_s[-1]), points[0], points[-1], K.MINIMUM_SNAP, curve
    )


def test_polynomial_adapter_uses_analytic_p_v_a_and_owns_curve():
    s = segment()
    for t in [0.0, 0.3, 1.5, 2.99]:
        result = sample_segment(s, t, 0.3)
        for r, value in enumerate((result.position_W, result.velocity_W, result.acceleration_W)):
            np.testing.assert_allclose(value, s.trajectory.evaluate(t, r), atol=1e-13)
        assert result.yaw_rad == 0.3
    for t in [-1.0, 3.0, 4.0]:
        result = sample_segment(s, t, 0.0)
        np.testing.assert_array_equal(
            result.position_W, s.start_position_W if t < 0 else s.end_position_W
        )
        np.testing.assert_array_equal(result.velocity_W, 0)
        np.testing.assert_array_equal(result.acceleration_W, 0)
    copy = replace(s)
    assert not np.shares_memory(copy.trajectory.coefficients_W, s.trajectory.coefficients_W)


def test_historical_segment_schema_does_not_gain_null_fields():
    old = baseline_mission("square")
    assert set(asdict(old.segments[0])) == {
        "phase",
        "duration_s",
        "start_position_W",
        "end_position_W",
        "kind",
    }


@pytest.mark.parametrize(
    "change",
    [
        {"duration_s": 4.0},
        {"start_position_W": np.ones(3)},
        {"end_position_W": np.ones(3)},
        {"kind": K.SMOOTH},
        {"phase": P.INITIALIZE},
        {"trajectory": None},
    ],
)
def test_segment_rejects_inconsistent_or_unsupported_representation(change):
    with pytest.raises((ValueError, TypeError)):
        replace(segment(), **change)


def test_plain_segment_cannot_claim_polynomial_kind():
    with pytest.raises(ValueError, match="MinimumSnapMissionSegment"):
        MissionSegment(P.TRACK, 1.0, np.zeros(3), np.zeros(3), K.MINIMUM_SNAP)


def test_segment_rejects_nonrest_and_discontinuous_curves():
    s = segment(points=np.zeros((3, 3)), durations=np.ones(2))
    c = s.trajectory.coefficients_W.copy()
    c[1, 0, 0] = 0.1
    with pytest.raises(ValueError, match="C3"):
        replace(s, trajectory=replace(s.trajectory, coefficients_W=c))
    d = np.zeros((3, 3))
    d[0, 0] = 0.1
    with pytest.raises(ValueError, match="rest"):
        replace(
            segment(),
            trajectory=minimum_snap_trajectory(
                np.array([[0.0, 0, 0], [0.1, 0.1, -0.1]]), np.array([3.0]), d
            ),
        )


def simulate(plan, monkeypatch=None):
    initial, body, world, outer, inner, _, safety, numerics = make_case(
        {"case": "smoke", "seed": None, "refinement": 1}
    )
    if monkeypatch is not None:

        def forbidden(*args, **kwargs):
            raise AssertionError("preflight must happen before plant execution")

        monkeypatch.setattr("quadrotor_math.mission_simulation._plant_step", forbidden)
    return simulate_mission(
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


def test_internal_geofence_excursion_rejected_before_plant(monkeypatch):
    plan = baseline_mission("smoke")
    track = segment(points=np.array([[0.0, 0, 0], [4.0, 0, 0], [0.0, 0, 0]]), durations=np.ones(2))
    plan = replace(plan, segments=(*plan.segments[:2], track, plan.segments[-1]))
    with pytest.raises(ValueError, match="geofence"):
        simulate(plan, monkeypatch)


def test_short_equilibrium_preserves_clocks_completion_and_phase_selection():
    old = baseline_mission("smoke")
    track = segment(points=np.zeros((2, 3)), durations=np.array([0.01]))
    plan = replace(old, segments=(*old.segments[:2], track, old.segments[-1]))
    assert mission_reference(plan, 0.02)[1] == P.TRACK
    assert mission_reference(plan, 0.03)[1] == P.LAND
    first, second = simulate(old), simulate(plan)
    for name in (
        "time_s",
        "position_W",
        "velocity_W",
        "q_WB",
        "phase",
        "control_time_s",
        "position_control_time_s",
        "commanded_rotor_omega",
    ):
        np.testing.assert_array_equal(getattr(first, name), getattr(second, name))
    assert second.phase[-1] == P.COMPLETE
