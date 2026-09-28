"""Passive integration, complete byte parity, replay, and evidence authentication."""

from dataclasses import replace

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
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.geometric_control import GeometricControllerParameters
from quadrotor_math.observation_health import (
    ObservationHealthConfiguration,
    ObservationHealthMonitor,
    ObservationHealthPolicy,
    ObservationHealthState,
)
from quadrotor_math.run_configuration import SensorSchedule


def inputs(mode="cascade"):
    args = make_configuration({"case": "smoke", "seed": 31, "noiseless": False})
    args["plan"] = replace(
        args["plan"],
        segments=tuple(replace(segment, duration_s=0.1) for segment in args["plan"].segments),
    )
    args["sensors"] = replace(
        args["sensors"],
        local_position_schedule=SensorSchedule(0.02, 0.013 if mode == "delayed" else 0),
        barometric_altitude_schedule=SensorSchedule(0.01, 0.03 if mode == "delayed" else 0),
    )
    if mode == "geometric":
        args["geometric_controller"] = GeometricControllerParameters()
    if mode == "disabled":
        args["estimator_configuration"] = replace(
            args["estimator_configuration"],
            fuse_local_position=False,
            fuse_barometric_altitude=False,
        )
    if mode == "rejected":
        args["estimator_configuration"] = replace(
            args["estimator_configuration"], innovation_policy=EskfInnovationPolicy(1e-15, 1e-15)
        )
    if mode == "abort":
        args["initial_state"] = replace(args["initial_state"], position_W=np.array([4.0, 0, 0]))
    return args


def observer(args):
    sensors = args["sensors"]
    config = args["estimator_configuration"]
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
    return ObservationHealthMonitor(ObservationHealthConfiguration(*policies))


@pytest.mark.parametrize(
    "mode", ["cascade", "geometric", "delayed", "disabled", "rejected", "abort"]
)
def test_monitor_preserves_all_histories_saved_payloads_and_outcomes(mode, tmp_path):
    args = inputs(mode)
    baseline = simulate_estimated_mission(**args)
    monitor = observer(args)
    observed = simulate_estimated_mission(**args, observation_health=monitor)
    # This packs every original public numerical field, commands, phases,
    # all estimator corrections/innovations and both abort reasons.
    before = pack_history(baseline, baseline.mission)
    after = pack_history(observed, observed.mission)
    assert before.keys() == after.keys()
    for key in before:
        assert before[key].dtype == after[key].dtype
        assert before[key].shape == after[key].shape
        assert before[key].tobytes() == after[key].tobytes(), key
    left, right = tmp_path / "baseline", tmp_path / "monitored"
    left.mkdir()
    right.mkdir()
    records = save_history(left, 0, before)
    assert save_history(right, 0, after) == records
    reconstructed, _ = unpack_history(load_history(right, 0, records))
    assert reconstructed.mission.abort_reason == baseline.mission.abort_reason
    assert len(monitor.history) == len(observed.mission.time_s)
    replay = observer(args)
    for k, time in enumerate(observed.estimates.time_s):
        delivered = tuple(e for e in observed.estimates.events if e.observation.delivery_index == k)
        assert replay.step(float(time), delivered) == monitor.history[k]
    if mode in ("delayed", "rejected"):
        assert monitor.latest.local_position.state is ObservationHealthState.LOST
    if mode == "disabled":
        assert monitor.latest.local_position.state is ObservationHealthState.DISABLED
    if mode == "abort":
        assert len(monitor.history) == 1 and monitor.latest.time_s == 0
    # The existing evidence boundary still fails closed on damage.
    path = right / records[0]["file"]
    path.write_bytes(path.read_bytes() + b"corrupt")
    with pytest.raises(ValueError, match="digest"):
        load_history(right, 0, records)


@pytest.mark.parametrize("bad", ["period", "delay", "clock", "enabled", "used", "start", "type"])
def test_policy_mismatch_rejected_before_rng_or_plant(bad, monkeypatch):
    import quadrotor_math.estimated_mission as module

    args = inputs()
    monitor = observer(args)
    if bad in ("period", "delay", "clock", "enabled"):
        name, value = {
            "period": ("sample_period_s", 0.04),
            "delay": ("delivery_delay_s", 0.01),
            "clock": ("check_interval_s", 0.005),
            "enabled": ("enabled", False),
        }[bad]
        monitor = ObservationHealthMonitor(
            replace(
                monitor.configuration,
                local_position=replace(monitor.configuration.local_position, **{name: value}),
            )
        )
    elif bad == "used":
        monitor.step(0)
    elif bad == "start":
        monitor.reset(initial_time_s=1)
    else:
        monitor = True

    def forbidden(*args, **kwargs):
        pytest.fail("preflight must finish before generating observations or advancing plant")

    monkeypatch.setattr(module, "create_run_random_streams", forbidden)
    monkeypatch.setattr(module, "_simulate_mission", forbidden)
    with pytest.raises((ValueError, TypeError)):
        simulate_estimated_mission(**args, observation_health=monitor)


def test_reset_makes_repeated_mission_trace_identical():
    args = inputs("abort")
    monitor = observer(args)
    simulate_estimated_mission(**args, observation_health=monitor)
    first = monitor.history
    with pytest.raises(ValueError, match="fresh"):
        simulate_estimated_mission(**args, observation_health=monitor)
    monitor.reset()
    simulate_estimated_mission(**args, observation_health=monitor)
    assert monitor.history == first
