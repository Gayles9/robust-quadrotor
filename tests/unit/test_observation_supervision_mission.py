"""Public feedback integration, phase guards, command prefixes and saved evidence."""

from dataclasses import fields, replace

import numpy as np
import pytest

from experiments.estimated_feedback_evidence import (
    load_history,
    pack_history,
    save_history,
    unpack_history,
)
from experiments.estimated_feedback_validation import make_configuration
from quadrotor_math.eskf_innovation import EskfInnovationPolicy
from quadrotor_math.eskf_replay import EskfReplayStatus
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.geometric_control import GeometricControllerParameters
from quadrotor_math.missions import MissionPhase
from quadrotor_math.observation_health import (
    ObservationHealthConfiguration,
    ObservationHealthMonitor,
    ObservationHealthPolicy,
)
from quadrotor_math.observation_supervision import (
    ObservationSupervisionPolicy,
    ObservationSupervisor,
)
from quadrotor_math.run_configuration import SensorSchedule


def inputs(mode="nominal", *, geometric=False, noiseless=False):
    args = make_configuration({"case": "smoke", "seed": 31, "noiseless": noiseless})
    # All four phase labels use the existing zero-position HOLD reference.
    # This short regression fixture tests supervision, not trajectory tracking.
    args["plan"] = replace(
        args["plan"],
        segments=tuple(replace(s, duration_s=0.1) for s in args["plan"].segments),
    )
    args["sensors"] = replace(
        args["sensors"],
        local_position_schedule=SensorSchedule(0.02, 0.013 if mode == "position_delayed" else 0),
        barometric_altitude_schedule=SensorSchedule(
            0.01, 0.03 if "altitude_delayed" in mode else 0
        ),
    )
    if geometric:
        args["geometric_controller"] = GeometricControllerParameters()
    config = args["estimator_configuration"]
    if "rejected" in mode:
        config = replace(
            config,
            innovation_policy=EskfInnovationPolicy(
                1e-15 if mode == "position_rejected" else 11.345,
                1e-15 if mode == "altitude_rejected" else 6.635,
            ),
        )
    if mode == "optional_altitude_disabled":
        config = replace(config, fuse_barometric_altitude=False)
    args["estimator_configuration"] = config
    return args


def observer(args, *, position_timeout=0.08, altitude_timeout=0.06):
    sensors, config = args["sensors"], args["estimator_configuration"]
    policies = tuple(
        ObservationHealthPolicy(
            s.sample_period_s,
            s.delivery_delay_s,
            args["numerics"].time_step_s,
            2,
            4,
            3,
            2,
            5,
            enabled,
        )
        for s, enabled in zip(
            (sensors.local_position_schedule, sensors.barometric_altitude_schedule),
            (config.fuse_local_position, config.fuse_barometric_altitude),
            strict=True,
        )
    )
    health = ObservationHealthMonitor(ObservationHealthConfiguration(*policies))
    supervisor = ObservationSupervisor(
        health, ObservationSupervisionPolicy(position_timeout, altitude_timeout)
    )
    return health, supervisor


def execute(args, health, supervisor):
    return simulate_estimated_mission(
        **args, observation_health=health, observation_supervision=supervisor
    )


@pytest.mark.parametrize("geometric", [False, True])
@pytest.mark.parametrize(
    "mode", ["nominal", "optional_altitude_disabled", "optional_altitude_delayed"]
)
def test_healthy_or_explicitly_optional_stream_preserves_every_existing_payload(
    mode, geometric, tmp_path
):
    args = inputs(mode, geometric=geometric)
    health, supervisor = observer(args, altitude_timeout=None if "optional" in mode else 0.06)
    baseline = simulate_estimated_mission(**args)
    result = execute(args, health, supervisor)
    before = pack_history(baseline, baseline.mission)
    after = pack_history(result, result.mission)
    assert before.keys() == after.keys()
    for key in before:
        assert before[key].dtype == after[key].dtype
        assert before[key].shape == after[key].shape
        assert before[key].tobytes() == after[key].tobytes(), key
    assert supervisor.abort_reason is None
    left, right = tmp_path / "baseline", tmp_path / "supervised"
    left.mkdir()
    right.mkdir()
    assert save_history(left, 0, before) == save_history(right, 0, after)


def assert_mission_prefix(baseline, result):
    # Every state, reference, force, command and limitation flag retains parity.
    # Only the terminal phase differs when the new guard stops earlier.
    for field in fields(result.mission):
        if field.name in ("phase", "abort_reason"):
            continue
        actual, expected = (
            getattr(result.mission, field.name),
            getattr(baseline.mission, field.name),
        )
        np.testing.assert_array_equal(actual, expected[: len(actual)], err_msg=field.name)
    np.testing.assert_array_equal(
        result.mission.phase[:-1], baseline.mission.phase[: len(result.mission.phase) - 1]
    )


@pytest.mark.parametrize("geometric", [False, True])
@pytest.mark.parametrize(
    "mode,deadline",
    [
        ("position_delayed", 0.08),
        ("altitude_delayed", 0.06),
        ("position_rejected", 0.08),
        ("altitude_rejected", 0.06),
    ],
)
def test_fault_abort_timing_command_prefix_and_authenticated_reconstruction(
    mode, deadline, geometric, tmp_path
):
    args = inputs(mode, geometric=geometric)
    health, supervisor = observer(args)
    baseline = simulate_estimated_mission(**args)
    result = execute(args, health, supervisor)
    label = "local_position" if mode.startswith("position") else "barometric_altitude"
    assert result.mission.abort_reason == f"observation_{label}_timeout"
    assert result.mission.phase[-1] == MissionPhase.ABORT
    assert result.mission.time_s[-1] == deadline
    assert supervisor.latest.time_s == deadline
    assert len(supervisor.history) == len(health.history) == len(result.mission.time_s)
    assert np.all(result.mission.control_time_s < deadline)
    assert np.all(result.mission.position_control_time_s < deadline)
    assert_mission_prefix(baseline, result)
    expected = EskfReplayStatus.STALE if mode.endswith("delayed") else EskfReplayStatus.REJECTED
    assert any(e.status is expected for e in result.estimates.events)
    records = save_history(tmp_path, 0, pack_history(result, baseline.mission))
    restored, comparator = unpack_history(load_history(tmp_path, 0, records))
    baseline_records = save_history(tmp_path, 1, pack_history(baseline, baseline.mission))
    restored_baseline, _ = unpack_history(load_history(tmp_path, 1, baseline_records))
    np.testing.assert_array_equal(comparator.phase, baseline.mission.phase)
    assert comparator.abort_reason == baseline.mission.abort_reason
    assert_mission_prefix(restored_baseline, restored)
    replay_health, replay_supervisor = observer(args)
    for k, time in enumerate(restored.estimates.time_s):
        replay_health.step(
            float(time),
            tuple(e for e in restored.estimates.events if e.observation.delivery_index == k),
        )
        replay_supervisor.step()
    assert replay_health.history == health.history
    assert replay_supervisor.history == supervisor.history
    assert restored.mission.abort_reason == supervisor.abort_reason
    path = tmp_path / records[0]["file"]
    path.write_bytes(path.read_bytes() + b"corrupt")
    with pytest.raises(ValueError, match="digest"):
        load_history(tmp_path, 0, records)


@pytest.mark.parametrize(
    "deadline,prior_phase",
    [(0.1, MissionPhase.INITIALIZE), (0.2, MissionPhase.TAKEOFF), (0.3, MissionPhase.TRACK)],
)
def test_phase_boundaries_do_not_reset_startup_loss_budget(deadline, prior_phase):
    args = inputs("position_delayed", noiseless=True)
    health, supervisor = observer(args, position_timeout=deadline)
    result = execute(args, health, supervisor)
    assert result.mission.abort_reason == "observation_local_position_timeout"
    assert result.mission.time_s[-1] == deadline
    assert result.mission.phase[-2] == prior_phase
    assert supervisor.latest.local_position_unhealthy_since_s == 0
    assert np.all(result.mission.control_time_s < deadline)


def test_observation_abort_wins_over_same_epoch_landing_completion():
    args = inputs("position_delayed", noiseless=True)
    baseline = simulate_estimated_mission(**args)
    assert baseline.mission.phase[-1] == MissionPhase.COMPLETE
    deadline = float(baseline.mission.time_s[-1])
    health, supervisor = observer(args, position_timeout=deadline)
    result = execute(args, health, supervisor)
    assert result.mission.phase[-2] == MissionPhase.LAND
    assert result.mission.phase[-1] == MissionPhase.ABORT
    assert result.mission.abort_reason == "observation_local_position_timeout"
    assert result.mission.time_s[-1] == deadline
    assert_mission_prefix(baseline, result)


@pytest.mark.parametrize("guard", ["truth", "estimate"])
def test_existing_safety_guard_keeps_priority_at_observation_deadline(guard, monkeypatch):
    import quadrotor_math.mission_simulation as module

    args = inputs("position_delayed")
    health, supervisor = observer(args)
    original = module.mission_guard_reason
    calls = 0

    def guard_at_deadline(position, q_WB, safety):
        nonlocal calls
        is_target = calls % 2 == (0 if guard == "truth" else 1)
        calls += 1
        if supervisor.latest.time_s == 0.08 and is_target:
            return "geofence"
        return original(position, q_WB, safety)

    monkeypatch.setattr(module, "mission_guard_reason", guard_at_deadline)
    result = execute(args, health, supervisor)
    assert result.mission.abort_reason == f"{guard}_geofence"
    assert supervisor.abort_reason == "observation_local_position_timeout"
    assert result.mission.time_s[-1] == 0.08
    assert np.all(result.mission.control_time_s < 0.08)


@pytest.mark.parametrize("bad", ["type", "missing_monitor", "different_monitor", "used"])
def test_supervisor_preflight_rejects_mismatch_before_rng_or_plant(bad, monkeypatch):
    import quadrotor_math.estimated_mission as module

    args = inputs()
    health, supervisor = observer(args)
    if bad == "type":
        supervisor = True
    elif bad == "missing_monitor":
        health = None
    elif bad == "different_monitor":
        health, _ = observer(args)
    else:
        health.step(0)
        supervisor.step()
        health.reset()

    def forbidden(*args, **kwargs):
        pytest.fail("preflight must finish before RNG or plant")

    monkeypatch.setattr(module, "create_run_random_streams", forbidden)
    monkeypatch.setattr(module, "_simulate_mission", forbidden)
    with pytest.raises((TypeError, ValueError)):
        execute(args, health, supervisor)


def test_both_reset_produce_identical_decisions_and_saved_payloads():
    args = inputs("altitude_delayed")
    health, supervisor = observer(args)
    first = execute(args, health, supervisor)
    decisions = supervisor.history
    health.reset()
    supervisor.reset()
    second = execute(args, health, supervisor)
    assert supervisor.history == decisions
    before, after = pack_history(first, first.mission), pack_history(second, second.mission)
    assert all(before[k].tobytes() == after[k].tobytes() for k in before)
