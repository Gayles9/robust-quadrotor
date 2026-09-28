"""Complete candidate evidence, causal integration and retained comparison failures."""

import hashlib
import json
import shutil

import pytest

from experiments.attitude_control_validation import canonical_json
from experiments.robustness_validation import run_campaign
from experiments.vertical_compensation_validation import (
    BASELINE_REPORT_SHA256,
    POLICY,
    protocol,
    read_baseline,
    run,
    verify,
)


def test_one_frozen_candidate_preserves_all_original_cases():
    p = protocol("campaign", BASELINE_REPORT_SHA256)
    assert hashlib.sha256(canonical_json(p)).hexdigest() == (
        "6bab1e089730485310a9d69a3d49361618c73982663b75f2a9efe8fc9fa9635e"
    )
    assert p["compensation_policy"] == {
        "integral_gain": 0.5,
        "maximum_acceleration": 1.5,
        "update_period_s": 0.02,
    }
    assert len(p["base_protocol"]["cases"]) == 12
    assert p["base_protocol"]["modes"] == ["off", "on"]
    assert p["base_protocol"]["flight_limits"]["hover_peak_m"] == 0.08
    assert p["base_protocol"]["flight_limits"]["rmse_m"] == 0.15
    with pytest.raises(ValueError, match="baseline"):
        protocol("campaign", "0" * 64)


@pytest.fixture(scope="module")
def saved(tmp_path_factory):
    root = tmp_path_factory.mktemp("vertical")
    baseline = root / "baseline"
    run_campaign(baseline, "smoke")
    candidate = root / "candidate"
    report = run(candidate, baseline, "smoke")
    assert report["summary"]["executions"] == 4
    assert report["summary"]["numerical_failures"] == 0
    return baseline, candidate


def test_saved_reconstruction_and_nonqualification(saved):
    baseline, candidate = saved
    report = verify(candidate, baseline)
    assert report["summary"]["response_passes"] == 2
    assert not report["summary"]["candidate_accepted"]
    for row in report["cases"]:
        assert row["metrics"]["common_prefix_equal"]
        for mode in ("off", "on"):
            record = row["modes"][mode]["diagnostic"]
            d = json.loads((candidate / row["name"] / record["file"]).read_bytes())
            times = {s["time_s"]: s for s in d["health"]}
            for sample in d["vertical"]:
                health = times[sample["time_s"]]
                healthy = all(
                    health[k]["state"] == "healthy"
                    for k in ("local_position", "barometric_altitude")
                )
                assert sample["observations_healthy"] == healthy
                if not healthy:
                    assert sample["next_acceleration"] == sample["applied_acceleration"]
                    assert sample["update_reason"] == "unhealthy"
                assert abs(sample["next_acceleration"]) <= POLICY.maximum_acceleration
                assert sample["time_s"] < row["metrics"]["modes"][mode]["terminal_time_s"]


@pytest.fixture
def copied(saved, tmp_path):
    baseline, candidate = saved
    directory = tmp_path / "copy"
    shutil.copytree(candidate, directory)
    return baseline, directory, json.loads((directory / "report.json").read_bytes())


@pytest.mark.parametrize(
    "change", ["protocol", "source", "software", "cases", "metrics", "comparison", "summary"]
)
def test_changed_report_claims_rejected(copied, change):
    baseline, directory, r = copied
    match = {
        "protocol": "frozen candidate",
        "source": "recorded execution source",
        "software": "software provenance",
        "cases": "missing or reordered",
        "metrics": "metrics differ",
        "comparison": "comparison differs",
        "summary": "summary differs",
    }[change]
    if change == "protocol":
        r["protocol"]["compensation_policy"]["integral_gain"] = 1.0
        r["protocol_sha256"] = hashlib.sha256(canonical_json(r["protocol"])).hexdigest()
    elif change == "source":
        r["source_sha256"] = "0" * 64
    elif change == "software":
        r["software"]["git_worktree_clean"] = "yes"
    elif change == "cases":
        r["cases"].reverse()
    elif change == "metrics":
        r["cases"][0]["metrics"]["modes"]["on"]["tracking_rmse_m"] = -1
    elif change == "comparison":
        r["cases"][0]["comparison"]["metric_deltas"]["on"]["tracking_rmse_m"] = -1
    else:
        r["summary"]["candidate_accepted"] = True
    (directory / "report.json").write_bytes(canonical_json(r))
    with pytest.raises(ValueError, match=match):
        verify(directory, baseline)


def test_reauthenticated_integral_trace_must_follow_law(copied):
    baseline, directory, r = copied
    row = r["cases"][0]
    record = row["modes"]["on"]["diagnostic"]
    path = directory / row["name"] / record["file"]
    d = json.loads(path.read_bytes())
    d["vertical"][1]["next_acceleration"] += 0.1
    path.write_bytes(canonical_json(d))
    record["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    (directory / "report.json").write_bytes(canonical_json(r))
    with pytest.raises(ValueError, match="vertical compensation reconstruction"):
        verify(directory, baseline)


def test_baseline_bytes_and_identity_are_required(saved, tmp_path):
    baseline, _ = saved
    destination = tmp_path / "baseline"
    shutil.copytree(baseline, destination)
    with pytest.raises(ValueError, match="audited campaign"):
        read_baseline(destination, "campaign")
    (destination / "nominal_tracking/trial-000-data.npz").write_bytes(b"damaged")
    with pytest.raises(ValueError, match="baseline history byte digest"):
        read_baseline(destination, "smoke")


def test_one_failed_mode_is_retained_without_losing_its_successful_partner(
    saved, tmp_path, monkeypatch
):
    import experiments.robustness_validation as base

    baseline, _ = saved
    original = base.simulate_estimated_mission

    def fail_on(**kwargs):
        if kwargs["observation_supervision"] is not None:
            raise RuntimeError("deliberate candidate failure")
        return original(**kwargs)

    monkeypatch.setattr(base, "simulate_estimated_mission", fail_on)
    directory = tmp_path / "failures"
    report = run(directory, baseline, "smoke")
    assert report["summary"]["numerical_failures"] == 2
    assert not report["summary"]["candidate_accepted"]
    for row in report["cases"]:
        assert row["modes"]["off"]["status"] == "ok"
        assert row["modes"]["on"]["status"] == "error"
        assert not row["comparison"]["passed"]
    assert verify(directory, baseline) == report
