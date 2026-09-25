"""Frozen ledgers, corruption rejection, lossless chunks and paired evidence."""

import hashlib
import json
from copy import deepcopy

import numpy as np
import pytest

from experiments.attitude_control_validation import canonical_json
from experiments.estimated_feedback_evidence import (
    load_history,
    pack_history,
    save_history,
    unpack_history,
)
from experiments.estimated_feedback_validation import (
    load_report,
    planned_jobs,
    run_validation,
    save_report,
    validate_report,
    validation_protocol,
)
from experiments.plot_estimated_feedback import render_report


@pytest.fixture(scope="module")
def smoke():
    return run_validation("smoke")


def test_workers_seed_separation_and_lossless_object_roundtrip(smoke):
    other = run_validation("smoke", workers=2)
    assert canonical_json(other["summary"]) == canonical_json(smoke["summary"])
    for first, second in zip(smoke["trials"], other["trials"], strict=True):
        assert canonical_json(first["metrics"]) == canonical_json(second["metrics"])
        restored = pack_history(*unpack_history(first["history"]))
        for key, value in first["history"].items():
            np.testing.assert_array_equal(value, second["history"][key])
            np.testing.assert_array_equal(value, restored[key])
    groups = [
        {r["seed"] for r in planned_jobs(partition)}
        for partition in ("smoke", "fixed", "development", "validation")
    ]
    assert all(not (a & b) for i, a in enumerate(groups) for b in groups[i + 1 :])
    assert len(planned_jobs("validation")) == 10
    validate_report(smoke)


@pytest.mark.parametrize(
    "bad",
    [
        "missing",
        "duplicate",
        "reordered",
        "metric",
        "summary",
        "protocol",
        "job_type",
        "state",
        "covariance",
        "rate",
        "clock",
        "reference",
        "command",
        "phase",
        "dtype",
        "extra_array",
    ],
)
def test_rejects_ledger_and_history_corruption(smoke, bad):
    r = deepcopy(smoke)
    row = r["trials"][0]
    data = row["history"]
    if bad == "missing":
        r["trials"].pop()
    elif bad == "duplicate":
        r["trials"][1] = deepcopy(row)
    elif bad == "reordered":
        r["trials"].reverse()
    elif bad == "metric":
        row["metrics"]["estimation_position_rmse_m"] += 0.01
    elif bad == "summary":
        r["summary"]["planned"] = 0
    elif bad == "protocol":
        r["protocol"]["targets"]["position_rmse_m"] = 10
    elif bad == "job_type":
        row["job"]["noiseless"] = 0
    elif bad == "state":
        data["estimate_position_W"][1, 0] += 0.01
    elif bad == "covariance":
        data["covariances"][1, 0, 0] += 0.1
    elif bad == "rate":
        data["angular_velocity_estimate_B"][1, 0] += 0.01
    elif bad == "clock":
        data["mission_time_s"][1] += 1e-5
    elif bad == "reference":
        data["mission_reference_position_W"][1, 0] += 0.01
    elif bad == "command":
        data["mission_commanded_rotor_omega"][0, 0] += 0.01
    elif bad == "phase":
        data["mission_phase"][1] = 1
    elif bad == "dtype":
        data["estimate_position_W"] = data["estimate_position_W"].astype(np.float32)
    elif bad == "extra_array":
        data["unused_truth_override"] = np.zeros(3)
    with pytest.raises((ValueError, KeyError, TypeError)):
        validate_report(r)


def test_failed_trials_stay_in_the_ledger(monkeypatch):
    def fail(**kwargs):
        raise ValueError("deliberate numerical failure")

    monkeypatch.setattr(
        "experiments.estimated_feedback_validation.simulate_estimated_mission", fail
    )
    report = run_validation("smoke")
    assert report["summary"] == {
        "planned": 2,
        "numerical_failures": 2,
        "acceptance_failures": 0,
        "passed": False,
    }
    assert all(row["error"] == "deliberate numerical failure" for row in report["trials"])
    validate_report(report)


def test_complete_bundle_roundtrip_digest_and_no_overwrite(smoke, tmp_path):
    directory = tmp_path / "bundle"
    save_report(smoke, directory)
    loaded = load_report(directory)
    for a, b in zip(smoke["trials"], loaded["trials"], strict=True):
        for key in a["history"]:
            np.testing.assert_array_equal(a["history"][key], b["history"][key])
    with pytest.raises(FileExistsError):
        save_report(smoke, directory)
    path = directory / "trial-000-cov-000.npz"
    path.write_bytes(path.read_bytes() + b"modified")
    with pytest.raises(ValueError, match="digest"):
        load_report(directory)


def test_multiple_covariance_chunks_preserve_every_row(smoke, tmp_path, monkeypatch):
    monkeypatch.setattr("experiments.estimated_feedback_evidence.HISTORY_CHUNK_ROWS", 7)
    data = smoke["trials"][0]["history"]
    records = save_history(tmp_path, 0, data)
    assert len(records) > 3
    restored = load_history(tmp_path, 0, records)
    for key, value in data.items():
        np.testing.assert_array_equal(value, restored[key])
    with pytest.raises(ValueError, match="count"):
        load_history(tmp_path, 0, records[:-1])
    wrong = deepcopy(records)
    wrong[1]["file"] = "../covariance.npz"
    with pytest.raises(ValueError, match="filename"):
        load_history(tmp_path, 0, wrong)


@pytest.mark.parametrize("bad", ["duplicate_json", "nan", "path", "format", "missing_chunk"])
def test_bundle_rejects_ambiguous_metadata(smoke, tmp_path, bad):
    directory = tmp_path / "bundle"
    save_report(smoke, directory)
    path = directory / "report.json"
    data = json.loads(path.read_bytes())
    if bad == "duplicate_json":
        path.write_text('{"format_version":1,"format_version":1}')
    else:
        if bad == "nan":
            data["trials"][0]["metrics"]["estimation_position_rmse_m"] = float("nan")
        elif bad == "path":
            data["trials"][0]["history_parts"][0]["file"] = "../data.npz"
        elif bad == "format":
            data["format_version"] = True
        else:
            data["trials"][0]["history_parts"].pop()
        path.write_text(json.dumps(data))
    with pytest.raises((ValueError, KeyError)):
        load_report(directory)


def test_pickle_arrays_rejected_even_with_matching_digest(smoke, tmp_path):
    directory = tmp_path / "bundle"
    save_report(smoke, directory)
    path = directory / "trial-000-data.npz"
    with path.open("wb") as stream:
        np.savez(stream, payload=np.array([object()], dtype=object))
    report_path = directory / "report.json"
    report = json.loads(report_path.read_bytes())
    report["trials"][0]["history_parts"][0]["sha256"] = hashlib.sha256(
        path.read_bytes()
    ).hexdigest()
    report_path.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="Object arrays"):
        load_report(directory)


def test_plot_validates_first_and_has_input_identity(smoke, tmp_path):
    assert render_report(smoke, tmp_path / "plots") == ["feedback_ensemble.png"]
    assert (tmp_path / "plots/feedback_ensemble.png").read_bytes().startswith(b"\x89PNG")
    manifest = json.loads((tmp_path / "plots/plot_manifest.json").read_bytes())
    assert len(manifest["input_sha256"]) == len(manifest["plotter_sha256"]) == 64
    with pytest.raises(FileExistsError):
        render_report(smoke, tmp_path / "plots")
    invalid = deepcopy(smoke)
    invalid["trials"].pop()
    with pytest.raises(ValueError):
        render_report(invalid, tmp_path / "invalid")
    assert not (tmp_path / "invalid").exists()


@pytest.mark.parametrize("bad", [True, 0, -1, 1.5])
def test_invalid_worker_count(bad):
    with pytest.raises(ValueError):
        run_validation("smoke", bad)


def test_invalid_partition():
    with pytest.raises(ValueError):
        validation_protocol("future")
