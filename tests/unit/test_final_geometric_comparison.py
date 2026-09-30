"""Fixed search, independent comparator selection, complete replay and held-out gates."""

import copy
import json
from dataclasses import asdict
from unittest.mock import patch

import numpy as np
import pytest

from experiments import final_geometric_comparison as study
from experiments import supported_geometric_comparison as previous


def ledger(stage="development", names=None):
    names = names or tuple(study.BY_NAME)
    rows = []
    for job in study.clean_jobs(stage):
        row = dict(job=asdict(job), arms={})
        for name in names:
            scale = 0.8 if name == "axis-h10-f1" else 0.9 if name == "cascade-f1.25" else 1
            row["arms"][name] = dict(
                audit=dict(passed=True),
                metrics=dict(
                    passed=True,
                    position_rmse_m=0.05 * scale,
                    position_peak_m=0.07,
                    moment_effort_N2_m2_s=0.01,
                ),
            )
        rows.append(row)
    return rows


def test_fixed_profiles_ledger_and_independent_cascade_selection():
    assert len(study.PROFILES) == 9
    assert sum(p.controller == "axis" for p in study.PROFILES) == 6
    assert len(study.clean_jobs("development")) == 4
    assert len(study.clean_jobs("validation")) == 12
    assert {j.seed for j in study.clean_jobs("development")}.isdisjoint(
        j.seed for j in study.clean_jobs("validation")
    )
    choice = study.select(ledger())
    assert choice["accepted"]
    assert (choice["geometric"], choice["cascade"]) == ("axis-h10-f1", "cascade-f1.25")
    assert choice["primary"]["balanced_ratios"]["position_rmse_m"] == pytest.approx(8 / 9)
    assert study.definition("development", None, {})["decision_sha256"] == study.ADR_SHA
    # Canonical JSON sorts keys; authentication cannot depend on dictionary order.
    assert study.select(json.loads(json.dumps(ledger(), sort_keys=True))) == choice
    assert study.arms_for("validation", {"geometric": "axis-h10-f1", "cascade": "cascade-f1"}) == (
        "axis-h10-f1",
        "cascade-f1",
    )


@pytest.mark.parametrize(
    "failure", ["missing", "duplicate", "audit", "physical", "effort", "opportunity"]
)
def test_selection_fails_closed_without_hiding_outcomes(failure):
    rows = ledger()
    if failure == "missing":
        rows.pop()
    elif failure == "duplicate":
        rows[-1] = copy.deepcopy(rows[0])
    elif failure == "audit":
        rows[0]["arms"]["axis-h10-f1"]["audit"]["passed"] = False
    else:
        for row in rows:
            for profile in study.PROFILES:
                if profile.controller != "axis":
                    continue
                metrics = row["arms"][profile.name]["metrics"]
                if failure == "physical":
                    metrics["passed"] = False
                elif failure == "effort":
                    metrics["moment_effort_N2_m2_s"] = 0.020001
                else:
                    metrics["position_rmse_m"] = 0.049
    assert not study.select(rows)["accepted"]


def test_failed_development_blocks_later_seed_construction(tmp_path):
    rows = ledger()
    rows.pop()
    protocol = study.definition("development", None, {})
    (tmp_path / "protocol.json").write_text(json.dumps(protocol))
    (tmp_path / "report.json").write_text(
        json.dumps(dict(protocol=protocol, cases=rows, summary=study.select(rows)))
    )
    with patch.object(previous, "prepare", side_effect=AssertionError("fresh seed opened")):
        with pytest.raises(ValueError, match="later scientific stages remain unopened"):
            study.run(tmp_path / "fresh", "validation", 1, development=tmp_path)
    assert not (tmp_path / "fresh").exists()


def validation_ledger():
    choice = dict(geometric="axis-h10-f1", cascade="cascade-f1.25")
    rows = ledger("validation", study.arms_for("validation", choice))
    for case in study.supported_validation_protocol.FAULTS:
        rows.append(
            dict(
                job=asdict(study.Job(case, 97000)),
                arms={name: dict(audit=dict(passed=True)) for name in ("off", "on")},
                comparison=dict(passed=True),
            )
        )
    return choice, rows


@pytest.mark.parametrize(
    "failure", [None, "category", "individual", "effort", "fault", "peak_physical", "ledger"]
)
def test_prospective_fresh_decision_records_tradeoffs(failure):
    choice, rows = validation_ledger()
    if failure == "category":
        for row in rows[4:12]:
            if row["job"]["case"] == "wind_spline":
                row["arms"][choice["geometric"]]["metrics"]["position_rmse_m"] = 0.048
    elif failure == "individual":
        rows[0]["arms"][choice["geometric"]]["metrics"]["position_rmse_m"] = 0.05
    elif failure == "effort":
        rows[0]["arms"][choice["geometric"]]["metrics"]["moment_effort_N2_m2_s"] = 0.020001
    elif failure == "fault":
        rows[-1]["comparison"]["passed"] = False
    elif failure == "peak_physical":
        rows[0]["arms"][choice["geometric"]]["metrics"]["passed"] = False
    elif failure == "ledger":
        rows.pop()
    summary = study.validation_summary(rows, choice)
    assert summary["accepted"] is (failure is None)
    if failure is None:
        assert summary["descriptive_balanced_rmse_interval"] == pytest.approx([8 / 9, 8 / 9])


@pytest.mark.parametrize("case", ["hover", "position_dropout"])
def test_saved_axis_pipeline_reconstructs_full_history_and_rejects_tampering(tmp_path, case):
    names = ("axis-h10-f1", "axis-h20-f1.25", "cascade-f1.25")
    job = study.Job(case, 40)
    kwargs = dict(output=tmp_path, names=names, fault=case != "hover", partition="smoke")
    row = study.execute_case(job, **kwargs)
    assert all(item.get("audit", {}).get("passed") for item in row["arms"].values()), row
    assert all(item["audit"]["release_accuracy"]["passed"] for item in row["arms"].values())
    assert study.execute_case(job, verify=True, **kwargs) == row
    arm = next(iter(row["arms"]))
    path = tmp_path / job.name / f"configuration-{arm}.json"
    path.write_bytes(path.read_bytes() + b" ")
    with pytest.raises(ValueError, match="digest"):
        study.execute_case(job, verify=True, **kwargs)


def test_profile_changes_only_declared_controller_parameters_and_context_restores():
    base = previous.configuration(study.Job("hover", 40), "smoke")
    for profile in study.PROFILES:
        configured = study.configure(base, profile)
        for key in base:
            if key not in ("position_controller", "geometric_controller"):
                assert study.plain_configuration(configured[key]) == study.plain_configuration(
                    base[key]
                )
        np.testing.assert_array_equal(
            configured["position_controller"].position_gain_W[2:],
            base["position_controller"].position_gain_W[2:],
        )
    original = previous.shaped_reference
    with pytest.raises(RuntimeError):
        with study.reference_context(study.BY_NAME["axis-h10-f1"]):
            assert previous.shaped_reference is not original
            raise RuntimeError("injected")
    assert previous.shaped_reference is original
