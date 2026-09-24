"""Evidence integrity, complete trial accounting and frozen metric oracles."""

import json
from copy import deepcopy

import numpy as np
import pytest

from experiments.attitude_control_validation import canonical_json
from experiments.plot_position_control import render_report
from experiments.position_control_validation import (
    load_report,
    longest_active_duration,
    planned_jobs,
    run_validation,
    save_report,
    validate_report,
    validation_protocol,
)


@pytest.fixture(scope="module")
def smoke():
    return run_validation("smoke")


def test_worker_determinism_exact_ledger_and_seed_separation(smoke):
    other = run_validation("smoke", 2)
    assert canonical_json(smoke["summary"]) == canonical_json(other["summary"])
    for a, b in zip(smoke["trials"], other["trials"], strict=True):
        assert canonical_json(a["metrics"]) == canonical_json(b["metrics"])
        for name in a["history"]:
            np.testing.assert_array_equal(a["history"][name], b["history"][name])
    seeds = [
        {job["seed"] for job in planned_jobs(partition)}
        for partition in ("smoke", "development", "validation")
    ]
    assert all(not (a & b) for i, a in enumerate(seeds) for b in seeds[i + 1 :])
    assert len(planned_jobs("validation")) == 10
    validate_report(smoke)


def test_duration_oracle_uses_interval_widths_not_counts():
    flags = np.array([False, True, True, False, True])
    assert longest_active_duration(flags, np.array([0.1, 0.03, 0.05, 0.1, 0.02])) == pytest.approx(
        0.08
    )
    assert longest_active_duration(np.array([], dtype=bool), np.array([])) == 0


@pytest.mark.parametrize(
    "kind",
    [
        "missing",
        "duplicate",
        "reordered",
        "metric",
        "summary",
        "clock",
        "initial",
        "reference",
        "phase",
        "protocol",
        "outer",
        "thrust",
        "nan",
    ],
)
def test_report_rejects_corruption(smoke, kind):
    report = deepcopy(smoke)
    row = report["trials"][0]
    if kind == "missing":
        report["trials"].pop()
    elif kind == "duplicate":
        report["trials"][1] = deepcopy(row)
    elif kind == "reordered":
        report["trials"].reverse()
    elif kind == "metric":
        row["metrics"]["position_rmse_m"] = 0
    elif kind == "summary":
        report["summary"]["planned"] = 0
    elif kind == "clock":
        row["history"]["time_s"][1] += 1e-6
    elif kind == "initial":
        row["history"]["position_W"][0, 0] += 0.01
    elif kind == "reference":
        row["history"]["reference_position_W"][1, 0] += 0.01
    elif kind == "phase":
        row["history"]["phase"][1] = 1
    elif kind == "protocol":
        report["protocol"]["targets"]["position_rmse_m"] = 1
    elif kind == "outer":
        row["history"]["requested_acceleration_W"][0, 0] += 0.01
    elif kind == "thrust":
        row["history"]["collective_thrust"][0] += 0.01
    elif kind == "nan":
        row["history"]["velocity_W"][1, 0] = np.nan
    with pytest.raises(ValueError):
        validate_report(report)


def test_failure_ledger_retained(monkeypatch):
    def fail(*args, **kwargs):
        raise ValueError("injected numerical failure")

    monkeypatch.setattr("experiments.position_control_validation.simulate_mission", fail)
    report = run_validation("smoke")
    assert report["summary"]["numerical_failures"] == 2
    assert not report["summary"]["passed"]
    assert all(row["error"] == "injected numerical failure" for row in report["trials"])
    validate_report(report)


def test_bundle_roundtrip_no_overwrite_and_digest(smoke, tmp_path):
    output = tmp_path / "bundle"
    save_report(smoke, output)
    loaded = load_report(output)
    assert canonical_json(loaded["summary"]) == canonical_json(smoke["summary"])
    for first, second in zip(loaded["trials"], smoke["trials"], strict=True):
        for name in first["history"]:
            np.testing.assert_array_equal(first["history"][name], second["history"][name])
    with pytest.raises(FileExistsError):
        save_report(smoke, output)
    path = output / "trial-000.npz"
    path.write_bytes(path.read_bytes() + b"corruption")
    with pytest.raises(ValueError, match="digest"):
        load_report(output)


@pytest.mark.parametrize("kind", ["duplicate_json", "path", "format", "nan"])
def test_bundle_metadata_rejects(smoke, tmp_path, kind):
    save_report(smoke, tmp_path / "bundle")
    path = tmp_path / "bundle/report.json"
    data = json.loads(path.read_bytes())
    if kind == "duplicate_json":
        path.write_text('{"format_version":1,"format_version":1}')
    else:
        if kind == "path":
            data["trials"][0]["history_file"] = "../trial-000.npz"
        elif kind == "format":
            data["format_version"] = True
        else:
            data["trials"][0]["metrics"]["position_rmse_m"] = float("nan")
        path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_report(tmp_path / "bundle")


def test_headless_plot_validate_first_no_overwrite(smoke, tmp_path):
    names = render_report(smoke, tmp_path / "plots")
    assert names == ["mission_ensemble.png"]
    assert (tmp_path / "plots" / names[0]).read_bytes().startswith(b"\x89PNG")
    with pytest.raises(FileExistsError):
        render_report(smoke, tmp_path / "plots")
    invalid = deepcopy(smoke)
    invalid["trials"].pop()
    with pytest.raises(ValueError):
        render_report(invalid, tmp_path / "invalid")
    assert not (tmp_path / "invalid").exists()


@pytest.mark.parametrize("bad", [0, -1, True, 1.5])
def test_bad_workers(bad):
    with pytest.raises(ValueError):
        run_validation("smoke", bad)


def test_bad_partition():
    with pytest.raises(ValueError):
        validation_protocol("unknown")
