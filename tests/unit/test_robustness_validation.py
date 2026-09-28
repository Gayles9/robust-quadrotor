"""The frozen campaign retains failures and distrusts saved claims and payloads."""

import hashlib
import json
import shutil

import numpy as np
import pytest

from experiments.attitude_control_validation import canonical_json
from experiments.robustness_evidence import load_json
from experiments.robustness_protocol import CASES, configuration, faults, protocol
from experiments.robustness_validation import run_campaign, verify_report


@pytest.mark.parametrize(
    ("partition", "digest"),
    [
        ("smoke", "10897d014c60935ade47dcabc5115efb01ed1fb434d07bad66256e80255bbbd9"),
        ("campaign", "7a686a341305a6a58a0afcf5d20cb4929844165165f7aa70bc11679d99843c30"),
    ],
)
def test_frozen_protocol(partition, digest):
    assert hashlib.sha256(canonical_json(protocol(partition))).hexdigest() == digest


def test_complete_campaign_and_fixed_exposure_counts():
    definition = protocol("campaign")
    assert [row["name"] for row in definition["cases"]] == list(CASES)
    assert definition["modes"] == ["off", "on"]
    assert [len(faults(case, "campaign", configuration(case, "campaign"))) for case in CASES] == [
        0,
        0,
        10,
        10,
        10,
        50,
        50,
        50,
        2,
        10,
        0,
        0,
    ]
    base = definition["cases"][1]["configuration"]
    for case, changed in (
        ("wind_tracking", {"truth_world", "truth_body"}),
        ("mass_tracking", {"truth_body"}),
    ):
        actual = next(r["configuration"] for r in definition["cases"] if r["name"] == case)
        assert {k for k in base if base[k] != actual[k]} == changed


@pytest.fixture(scope="module")
def saved_campaign(tmp_path_factory):
    directory = tmp_path_factory.mktemp("robustness") / "smoke"
    report = run_campaign(directory, "smoke")
    assert report["summary"] == {
        "cases": 2,
        "executions": 4,
        "numerical_failures": 0,
        "response_passes": 2,
        "required_flight_cases": 1,
        "flight_passes": 0,
        "passed": False,
    }
    return directory


def test_authenticated_replay_preserves_failed_flight(saved_campaign):
    report = verify_report(saved_campaign)
    assert not report["summary"]["passed"]
    for row in report["cases"]:
        assert row["metrics"]["common_prefix_equal"]
        assert row["metrics"]["response_passed"]
    nominal = report["cases"][0]["metrics"]["modes"]
    assert nominal["on"]["abort_reason"] == "landing_timeout"
    assert nominal["off"] == nominal["on"]


@pytest.fixture
def copied_campaign(saved_campaign, tmp_path):
    directory = tmp_path / "copy"
    shutil.copytree(saved_campaign, directory)
    return directory, json.loads((directory / "report.json").read_bytes())


def save_report(directory, report):
    (directory / "report.json").write_bytes(canonical_json(report))


@pytest.mark.parametrize(
    "change", ["source", "software", "protocol", "missing", "order", "modes", "metrics", "summary"]
)
def test_report_claims_are_recomputed(copied_campaign, change):
    directory, report = copied_campaign
    match = {
        "source": "recorded execution source",
        "software": "software provenance",
        "protocol": "frozen definition",
        "missing": "missing or reordered",
        "order": "missing or reordered",
        "modes": "case/mode ledger",
        "metrics": "metrics differ",
        "summary": "summary differs",
    }[change]
    if change == "source":
        report["source_sha256"] = "0" * 64
    elif change == "software":
        report["software"]["git_worktree_clean"] = "true"
    elif change == "protocol":
        report["protocol"]["flight_limits"]["rmse_m"] = 10.0
        report["protocol_sha256"] = hashlib.sha256(canonical_json(report["protocol"])).hexdigest()
        (directory / "protocol.json").write_bytes(canonical_json(report["protocol"]))
    elif change == "missing":
        report["cases"].pop()
    elif change == "order":
        report["cases"].reverse()
    elif change == "modes":
        del report["cases"][0]["modes"]["on"]
    elif change == "metrics":
        report["cases"][0]["metrics"]["modes"]["on"]["tracking_rmse_m"] = 0.0
    else:
        report["summary"]["passed"] = True
    save_report(directory, report)
    with pytest.raises(ValueError, match=match):
        verify_report(directory)


@pytest.mark.parametrize("payload", ["history", "diagnostic"])
def test_byte_authentication_precedes_decoding(copied_campaign, payload):
    directory, report = copied_campaign
    row = report["cases"][0]
    item = row["modes"]["off"][payload]
    record = item[0] if payload == "history" else item
    (directory / row["name"] / record["file"]).write_bytes(b"not decodable")
    with pytest.raises(ValueError, match="byte digest mismatch"):
        verify_report(directory)


@pytest.mark.parametrize("change", ["sources", "health", "supervision"])
def test_reauthenticated_diagnostic_must_reconstruct(copied_campaign, change):
    directory, report = copied_campaign
    row = report["cases"][0]
    mode = "on" if change == "supervision" else "off"
    record = row["modes"][mode]["diagnostic"]
    path = directory / row["name"] / record["file"]
    diagnostic = json.loads(path.read_bytes())
    if change == "sources":
        # Remove the last source of one stream, retaining valid survivor identities.
        kind = diagnostic["fault_records"][-1]["source"]["kind"]
        indices = [
            i for i, r in enumerate(diagnostic["fault_records"]) if r["source"]["kind"] == kind
        ]
        diagnostic["fault_records"].pop(indices[-1])
        match = "incomplete original acquisition ledger"
    else:
        diagnostic[change].pop()
        match = "health reconstruction" if change == "health" else "supervisor reconstruction"
    path.write_bytes(canonical_json(diagnostic))
    record["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    save_report(directory, report)
    with pytest.raises(ValueError, match=match):
        verify_report(directory)


def test_reauthenticated_commands_must_follow_controller(copied_campaign):
    directory, report = copied_campaign
    row = report["cases"][0]
    record = row["modes"]["off"]["history"][0]
    path = directory / row["name"] / record["file"]
    with np.load(path, allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in archive.files}
    for prefix in ("mission_", "baseline_"):
        arrays[prefix + "commanded_rotor_omega"][0, 0] += 0.01
    np.savez_compressed(path, **arrays)
    record["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    save_report(directory, report)
    with pytest.raises(ValueError, match="inner control dataflow mismatch"):
        verify_report(directory)


def test_duplicate_json_keys_rejected_after_authentication(tmp_path):
    data = b'{"health": [], "health": [1]}'
    (tmp_path / "diagnostic.json").write_bytes(data)
    with pytest.raises(ValueError, match="duplicate"):
        load_json(
            tmp_path,
            "diagnostic.json",
            {"file": "diagnostic.json", "sha256": hashlib.sha256(data).hexdigest()},
        )


def test_numerical_failures_are_retained_for_all_executions(tmp_path, monkeypatch):
    def fail(**kwargs):
        raise RuntimeError("deliberate numerical failure")

    monkeypatch.setattr("experiments.robustness_validation.simulate_estimated_mission", fail)
    directory = tmp_path / "failures"
    report = run_campaign(directory, "smoke")
    assert report["summary"]["executions"] == report["summary"]["numerical_failures"] == 4
    assert not report["summary"]["passed"]
    assert not list(directory.rglob("*.npz"))
    assert len(list(directory.rglob("failure-*.json"))) == 4
    assert verify_report(directory) == report
    for row in report["cases"]:
        for item in row["modes"].values():
            assert item["error"] == "RuntimeError: deliberate numerical failure"


def test_runner_refuses_existing_output_and_invalid_worker_count(tmp_path):
    with pytest.raises(FileExistsError):
        run_campaign(tmp_path, "smoke")
    for workers in (0, 3, True):
        with pytest.raises(ValueError, match="workers"):
            run_campaign(tmp_path / "unused", "smoke", workers)
    assert not (tmp_path / "unused").exists()
