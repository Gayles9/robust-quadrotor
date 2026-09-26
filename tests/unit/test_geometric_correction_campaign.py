"""Candidate scope, held-out ledger and unchanged estimator/comparator contracts."""

from dataclasses import asdict

import numpy as np
import pytest

from experiments.attitude_control_validation import canonical_json
from experiments.estimated_feedback_evidence import plain
from experiments.geometric_correction_validation import (
    PROFILES,
    configuration,
    planned_jobs,
    study_summary,
    verify_payloads,
)
from experiments.geometric_reimplementation_validation import configuration as original
from experiments.geometric_reimplementation_validation import jobs, name_of


def test_frozen_development_and_qualification_are_complete_and_disjoint():
    development = planned_jobs("development")
    qualification = planned_jobs("qualification")
    assert development == [jobs()[i] for i in (14, 15, 23, 25)]
    assert qualification[:28] == jobs()
    assert len(qualification) == len({name_of(j) for j in qualification}) == 40
    assert {j["seed"] for j in qualification[28:]} == set(range(95000, 95004)) | set(
        range(96000, 96004)
    )
    assert not {name_of(j) for j in development} & {name_of(j) for j in qualification[28:]}


@pytest.mark.parametrize("stage", ["development", "qualification"])
def test_candidate_missing_or_failed_rows_never_qualify(stage, tmp_path):
    planned = planned_jobs(stage)
    rows = [dict(index=i, job=j, error="retained failure") for i, j in enumerate(planned)]
    report = study_summary(rows, stage, tmp_path)
    assert report["complete_ledger"]
    assert not report["candidate_qualified"]
    assert not report["candidate_tracking_and_effort_passed"]
    assert not study_summary(rows[:-1], stage, tmp_path)["complete_ledger"]
    duplicate = rows[:-1] + [rows[0]]
    assert not study_summary(duplicate, stage, tmp_path)["complete_ledger"]


@pytest.mark.parametrize("frequency,stiffness", PROFILES)
def test_candidate_changes_only_declared_geometric_parameters(frequency, stiffness):
    for job in planned_jobs("development"):
        before, after = original(job), configuration(job, frequency, stiffness)
        for key in before:
            if job["controller"] == "geometric" and key in (
                "geometric_controller",
                "position_controller",
            ):
                continue
            a, b = before[key], after[key]
            if hasattr(a, "__dataclass_fields__"):
                a, b = asdict(a), asdict(b)
            assert canonical_json(plain(a)) == canonical_json(plain(b))
        if job["controller"] == "geometric":
            assert after["geometric_controller"].rebase_estimator_corrections is True
            assert after["geometric_controller"].filter_pole_rad_s == 30
            assert after["geometric_controller"].attitude_stiffness == stiffness
            outer = after["position_controller"]
            np.testing.assert_array_equal(outer.position_gain_W, [frequency**2] * 2 + [2.25])
            np.testing.assert_array_equal(outer.velocity_gain_W, [1.8 * frequency] * 2 + [3.0])


@pytest.mark.parametrize("value", [True, np.bool_(True), 0, 3, float("nan")])
def test_undeclared_candidates_rejected(value):
    with pytest.raises(ValueError, match="declared candidate"):
        configuration(jobs()[15], value)


def test_evidence_gate_detects_empty_missing_and_changed_payloads(tmp_path):
    import hashlib

    path = tmp_path / "flight"
    path.mkdir()
    payload = b"independently known payload"
    (path / "data.npz").write_bytes(payload)
    rows = [
        dict(
            directory="flight",
            files=[dict(file="data.npz", sha256=hashlib.sha256(payload).hexdigest())],
        )
    ]
    assert verify_payloads(rows, tmp_path)["passed"]
    for damaged in (b"", b"modified payload"):
        (path / "data.npz").write_bytes(damaged)
        assert not verify_payloads(rows, tmp_path)["passed"]
    (path / "data.npz").unlink()
    assert not verify_payloads(rows, tmp_path)["passed"]
    assert not verify_payloads([dict(directory="failed", error="simulation failure")], tmp_path)[
        "passed"
    ]
    assert not verify_payloads([], tmp_path)["passed"]
