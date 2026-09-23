"""Fresh protocol isolation, compatibility, pairing and complete report accounting."""

import json
from copy import deepcopy

import pytest

from experiments import eskf_validation as experiment


def test_fresh_protocol_keeps_original_hashes_and_disjoint_seeds():
    originals = {
        "smoke": "7cf471729803d0ef11ffb126a9b2e90e767d10afccc71180f74778721c4f0ce6",
        "development": "998813847853b3cad682bf1346dda7dcd1b358c7bda456a5243148ed5dbf1651",
        "validation": "aec7acf36a2facb36cb6406ae6ab23106b159e79dd4400b0853268e1f9fddd12",
    }
    old_seeds = set()
    for p, digest in originals.items():
        protocol = experiment.validation_protocol(p)
        assert experiment.protocol_sha256(protocol) == digest
        old_seeds.update(j["seed"] for j in protocol["jobs"])
    sets = []
    for p in originals:
        protocol = experiment.validation_protocol(p, "endpoint")
        seeds = {j["seed"] for j in protocol["jobs"]}
        assert not seeds & old_seeds
        assert all(not seeds & s for s in sets)
        sets.append(seeds)
        assert all("first_order" in j["variants"] for j in protocol["jobs"])
    full = experiment.validation_protocol("validation", "endpoint")
    assert len(full["jobs"]) == 100
    assert sum(len(j["variants"]) for j in full["jobs"]) == 380
    assert sets[-1] == set(range(50000, 50100))


@pytest.fixture(scope="module")
def smoke():
    return experiment.run_validation("smoke", propagation="endpoint")


def test_endpoint_smoke_is_worker_deterministic_validated_and_paired(smoke):
    assert smoke == experiment.run_validation("smoke", workers=2, propagation="endpoint")
    assert smoke == experiment.validate_validation_report(smoke)
    assert smoke["numerical_failure_count"] == 0
    for trial in smoke["trials"]:
        legacy = experiment.run_validation_trial(
            (
                "excited",
                trial["seed"],
                40,
                ("nominal",),
                "representative" in trial["variants"]["first_order"],
            )
        )
        assert trial["variants"]["first_order"] == legacy["variants"]["nominal"]
        assert "sampled_imu_noise" in trial
    assert smoke["assessment"]["paired_first_order"] is not None


@pytest.mark.parametrize("bad", ["protocol", "duplicate", "missing", "stale_summary"])
def test_new_protocol_rejects_inconsistent_evidence(smoke, bad):
    report = deepcopy(smoke)
    if bad == "protocol":
        report["protocol"]["propagation"] = "first_order"
    if bad == "duplicate":
        report["trials"][1] = deepcopy(report["trials"][0])
    if bad == "missing":
        del report["trials"][0]["variants"]["first_order"]
    if bad == "stale_summary":
        report["assessment"]["paired_first_order"]["accuracy_target_met"] = False
    with pytest.raises(ValueError):
        experiment.validate_validation_report(report)


def test_cli_and_plot_explicitly_identify_endpoint_protocol(tmp_path):
    from experiments.plot_eskf_validation import main as plot_main

    path = tmp_path / "result.json"
    assert (
        experiment.main(
            ["--propagation", "endpoint", "--partition", "smoke", "--output", str(path)]
        )
        == 0
    )
    report = json.loads(path.read_text())
    assert report["protocol"]["version"] == 2
    assert report["protocol"]["propagation"] == "endpoint"
    assert plot_main(["--input", str(path), "--output", str(tmp_path / "plots")]) == 0


def test_unknown_propagation_is_rejected():
    with pytest.raises(ValueError, match="propagation"):
        experiment.validation_protocol("smoke", "typo")
