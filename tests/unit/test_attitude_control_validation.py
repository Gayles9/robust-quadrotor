"""Evidence-ledger integrity, independent metrics and deterministic experiment tests."""

from copy import deepcopy

import numpy as np
import pytest

from experiments.attitude_control_validation import (
    _execute,
    _result,
    canonical_json,
    make_case,
    planned_jobs,
    run_validation,
    score_case,
    settling_time,
    source_sha256,
    validate_report,
    validation_protocol,
)
from experiments.plot_attitude_control import render_report


@pytest.fixture(scope="module")
def smoke():
    return run_validation("smoke")


def test_partitions_and_repeatability(smoke):
    assert canonical_json(smoke) == canonical_json(run_validation("smoke", workers=2))
    seed_sets = [
        {job["seed"] for job in planned_jobs(partition)}
        for partition in ("smoke", "development", "validation")
    ]
    assert all(
        not (first & second) for i, first in enumerate(seed_sets) for second in seed_sets[i + 1 :]
    )
    assert len(planned_jobs("validation")) == 30
    assert smoke["summary"]["passed"]
    validate_report(smoke)


def test_seeded_inputs_repeat_and_fixed_timing():
    job = planned_jobs("development")[0]
    state1, _, schedule1 = make_case(job)
    state2, _, schedule2 = make_case(job)
    np.testing.assert_array_equal(state1.q_WB, state2.q_WB)
    np.testing.assert_array_equal(state1.omega_B, state2.omega_B)
    np.testing.assert_array_equal(schedule1.reference_q_WB, schedule2.reference_q_WB)
    pulse = make_case({"case": "pulse", "seed": None, "refinement": 1})[2]
    assert np.flatnonzero(pulse.disturbance_moment_B[:, 0])[0] == 400
    assert np.flatnonzero(pulse.disturbance_moment_B[:, 0])[-1] == 479
    fine = make_case({"case": "pulse", "seed": None, "refinement": 2})[2]
    np.testing.assert_array_equal(fine.disturbance_moment_B[::2], pulse.disturbance_moment_B)
    assert fine.controller_stride * fine.time_step_s == 0.01


def test_settling_uses_last_departure_and_rate_band():
    time = np.arange(6) * 0.1
    error = np.deg2rad(np.array([5.0, 0, 2, 0, 0, 0]))
    rate = np.array([0.0, 0, 0, 0.1, 0, 0])
    assert settling_time(time, error, rate, 0) == 0.4
    assert settling_time(time, error, rate, 0.2) == 0.2
    rate[-1] = 0.1
    assert settling_time(time, error, rate, 0) is None
    assert settling_time(time, np.zeros(6), np.zeros(6), 0.2) == 0


def test_persistent_torque_last_event_is_its_start(smoke):
    result = _result(smoke["trials"][0]["history"])
    metrics = score_case({"case": "persistent"}, result)
    assert metrics["last_event_s"] == 0.5


@pytest.mark.parametrize(
    "mutation",
    ["missing", "duplicate", "reordered", "metrics", "summary", "clock", "nan", "protocol"],
)
def test_report_rejects_corruption(smoke, mutation):
    report = deepcopy(smoke)
    if mutation == "missing":
        report["trials"].pop()
    elif mutation == "duplicate":
        report["trials"][1] = deepcopy(report["trials"][0])
    elif mutation == "reordered":
        report["trials"].reverse()
    elif mutation == "metrics":
        report["trials"][0]["metrics"]["final_error_deg"] = 0
    elif mutation == "summary":
        report["summary"]["planned"] = 100
    elif mutation == "clock":
        report["trials"][0]["history"]["time_s"][1] += 1e-6
    elif mutation == "nan":
        report["trials"][0]["history"]["omega_B"][1][0] = float("nan")
    elif mutation == "protocol":
        report["protocol"]["controller"]["attitude_gain_B"][0] *= 2
    with pytest.raises(ValueError):
        validate_report(report)


def test_numerical_failure_is_retained_and_prevents_pass(monkeypatch):
    def fail(*args, **kwargs):
        raise ValueError("injected failure")

    monkeypatch.setattr("experiments.attitude_control_validation.simulate_attitude_control", fail)
    report = run_validation("smoke")
    assert report["summary"]["numerical_failures"] == 2
    assert not report["summary"]["passed"]
    assert all(row["error"] == "injected failure" for row in report["trials"])
    validate_report(report)


def test_plot_headless_and_no_overwrite(smoke, tmp_path):
    output = tmp_path / "plots"
    assert render_report(smoke, output) == ["attitude_ensemble.png"]
    assert (output / "attitude_ensemble.png").read_bytes().startswith(b"\x89PNG")
    with pytest.raises(FileExistsError):
        render_report(smoke, output)
    corrupted = deepcopy(smoke)
    corrupted["trials"].pop()
    with pytest.raises(ValueError):
        render_report(corrupted, tmp_path / "invalid")
    assert not (tmp_path / "invalid").exists()


def test_source_hash_includes_untracked_sources_and_boundaries(tmp_path):
    (tmp_path / "src/quadrotor_math").mkdir(parents=True)
    (tmp_path / "experiments").mkdir()
    (tmp_path / "pyproject.toml").write_text("project")
    (tmp_path / "uv.lock").write_text("lock")
    file = tmp_path / "src/quadrotor_math/new.py"
    file.write_text("a")
    first = source_sha256(tmp_path)
    file.write_text("b")
    assert source_sha256(tmp_path) != first


@pytest.mark.parametrize("bad", [0, -1, True, 1.5])
def test_workers_reject(bad):
    with pytest.raises(ValueError):
        run_validation("smoke", bad)


def test_unknown_protocol_and_failed_job():
    with pytest.raises(ValueError):
        validation_protocol("unregistered")
    row = _execute(({"case": "unknown", "seed": None, "refinement": 1}, 0.04))
    assert row["status"] == "failed"
    assert row["error_type"] == "ValueError"
