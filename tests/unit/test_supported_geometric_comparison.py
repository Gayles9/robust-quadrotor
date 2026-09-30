"""Fixed composition, complete saved replay, prospective gates and seed isolation."""

import copy
import json
from dataclasses import asdict, replace
from unittest.mock import patch

import numpy as np
import pytest

from experiments import robustness_evidence
from experiments import supported_geometric_comparison as study
from experiments.combined_supported_prior import combined_prediction
from experiments.supported_start import configuration_for, fly, plain_configuration
from quadrotor_math import mission_simulation
from quadrotor_math.geometric_control import GeometricDomainError


@pytest.fixture(scope="module")
def prepared():
    return study.prepare(study.Job("hover", 40), "smoke")


def test_fixed_ledger_and_no_opening_reserved_configuration():
    development, validation = study.jobs("development"), study.jobs("validation")
    assert sum(len(study.arms(j)) for j in development) == 28
    assert sum(len(study.arms(j)) for j in validation) == 40
    assert len({j.name for j in development}) == 12
    assert len({j.name for j in validation}) == 16
    assert {j.seed for j in development}.isdisjoint(j.seed for j in validation)
    assert study.definition("development")["decision_sha256"] == study.ADR_SHA
    with pytest.raises(ValueError):
        study.jobs("sweep")
    with pytest.raises(ValueError):
        study.Job("hover", True)


def test_pair_configurations_isolate_estimation_and_controller(prepared):
    configs = {}
    for arm in study.ARMS:
        _, selected, mode = study.arm_start(prepared, arm)
        configs[arm] = configuration_for(selected, mode)
    baseline, geometric, cascade = (configs[a] for a in study.ARMS)
    for key in baseline:
        if key != "estimator_configuration":
            assert plain_configuration(baseline[key]) == plain_configuration(geometric[key])
    for key in geometric:
        if key != "geometric_controller":
            assert plain_configuration(geometric[key]) == plain_configuration(cascade[key])
    assert cascade["allow_minimum_snap"] is True
    assert "geometric_controller" not in cascade
    assert geometric["geometric_controller"].filter_pole_rad_s == 10
    assert np.all(geometric["estimator_configuration"].initial_covariance[3:6] == 0)
    assert np.all(np.diag(baseline["estimator_configuration"].initial_covariance)[3:6] > 0)
    assert np.all(geometric["initial_actual_rotor_omega"] == 0)
    assert np.all(geometric["initial_state"].velocity_W == 0)


def test_shaping_context_restores_both_paths_on_exception():
    before = mission_simulation.build_geometric_reference
    replay = robustness_evidence.build_geometric_reference
    with pytest.raises(RuntimeError, match="injected"):
        with study.coherent_reference():
            assert mission_simulation.build_geometric_reference is study.shaped_reference
            assert robustness_evidence.build_geometric_reference is study.shaped_reference
            raise RuntimeError("injected")
    assert mission_simulation.build_geometric_reference is before
    assert robustness_evidence.build_geometric_reference is replay


@pytest.mark.parametrize("case", ["hover", "position_dropout"])
def test_short_complete_pipeline_saves_and_reconstructs_every_arm(tmp_path, case):
    job = study.Job(case, 40)
    row = study.execute_job(job, output=tmp_path, partition="smoke")
    assert all(item.get("audit", {}).get("passed") for item in row["arms"].values()), row
    assert all("history" in item for item in row["arms"].values())
    assert all(
        item["audit"]["sample_memory_epochs"] == round(item["metrics"]["duration_s"] / 0.0025) + 1
        for item in row["arms"].values()
    )
    repeated = study.execute_job(job, output=tmp_path, partition="smoke", verify=True)
    assert repeated == row
    if case == "hover":
        assert not row["comparison"]["passed"]  # A short fixture cannot cover 5..65 s.
        assert row["arms"]["combined_geometric"]["audit"]["release_accuracy"]["passed"]
        assert max(row["noise_pairing"]["combined_cascade"].values()) < 1e-12
    config = tmp_path / job.name / f"configuration-{study.arms(job)[0]}.json"
    config.write_bytes(config.read_bytes() + b" ")
    with pytest.raises(ValueError, match="digest"):
        study.execute_job(job, output=tmp_path, partition="smoke", verify=True)


def clean_values():
    return {
        a: dict(passed=True, position_rmse_m=0.05, moment_effort_N2_m2_s=0.1, hold_peak_m=0.07)
        for a in study.ARMS
    }


@pytest.mark.parametrize("case", ["hover", "spline", "wind_spline"])
def test_frozen_comparison_gates(case):
    values = clean_values()
    job = study.Job(case, 40)
    assert study.clean_comparison(job, values)["passed"]
    values["combined_geometric"]["moment_effort_N2_m2_s"] = 0.20001
    assert not study.clean_comparison(job, values)["passed"]
    values = clean_values()
    values["combined_geometric"]["passed"] = False
    assert not study.clean_comparison(job, values)["passed"]
    if case != "hover":
        values = clean_values()
        values["combined_geometric"]["position_rmse_m"] = 0.05001
        assert not study.clean_comparison(job, values)["passed"]
        values = clean_values()
        values["combined_cascade"]["passed"] = False
        assert not study.clean_comparison(job, values)["passed"]


def ledger():
    rows = []
    for job in study.jobs("development"):
        values = clean_values()
        values["unaligned_geometric"]["hold_peak_m"] = 0.12
        rows.append(
            dict(
                job=asdict(job),
                arms={
                    a: dict(audit=dict(passed=True), history=[], metrics=values.get(a, {}))
                    for a in study.arms(job)
                },
                comparison=dict(passed=True),
            )
        )
    return rows


@pytest.mark.parametrize("fault", ["missing", "duplicate", "audit", "comparison", "hover"])
def test_summary_fails_closed(fault):
    rows = ledger()
    assert study.summarize(rows, "development")["accepted"]
    broken = copy.deepcopy(rows)
    if fault == "missing":
        broken.pop()
    elif fault == "duplicate":
        broken[-1] = broken[0]
    elif fault == "audit":
        broken[0]["arms"]["combined_geometric"]["audit"]["passed"] = False
    elif fault == "comparison":
        broken[-1]["comparison"]["passed"] = False
    else:
        for row in broken[:2]:
            row["arms"]["combined_geometric"]["metrics"]["hold_peak_m"] = 0.13
    assert not study.summarize(broken, "development")["accepted"]


def test_failed_development_blocks_validation_before_seed_configuration(tmp_path):
    rows = ledger()
    rows[-1]["comparison"]["passed"] = False
    protocol = {**study.definition("development"), "development_report_sha256": None}
    (tmp_path / "protocol.json").write_text(json.dumps(protocol))
    (tmp_path / "report.json").write_text(
        json.dumps(
            dict(protocol=protocol, cases=rows, summary=study.summarize(rows, "development"))
        )
    )
    with patch.object(study, "configuration", side_effect=AssertionError("reserved seed opened")):
        with pytest.raises(ValueError, match="reserved validation remains unopened"):
            study.run(tmp_path / "fresh", "validation", 1, development=tmp_path)
    assert not (tmp_path / "fresh").exists()


def test_geometric_domain_abort_replays_without_terminal_command(prepared):
    original, selected, mode = study.arm_start(prepared, "combined_geometric")

    def limited(*args, **kwargs):
        if args[7] == 2:
            raise GeometricDomainError("shaped_reference_domain")
        return study.shaped_reference(*args, **kwargs)

    with patch.object(mission_simulation, "build_geometric_reference", limited):
        with combined_prediction(original, selected, "online") as trace:
            result, diagnostic = fly(selected, mode)
    diagnostic["release_prediction"] = trace
    assert result.mission.abort_reason == "shaped_reference_domain"
    assert result.mission.control_time_s[-1] < result.mission.time_s[-1]
    # audit_arm's usual shaping context is replaced only to reproduce this injected domain.
    builder = study.shaped_reference

    def replay_limited(*args, **kwargs):
        if args[7] == 2:
            raise GeometricDomainError("shaped_reference_domain")
        return builder(*args, **kwargs)

    with patch.object(study, "shaped_reference", replay_limited):
        assert study.audit_arm(original, selected, mode, "combined_geometric", result, diagnostic)[
            "passed"
        ]


def test_later_zero_rotor_is_not_permitted(prepared):
    original, selected, mode = study.arm_start(prepared, "combined_geometric")
    with study.coherent_reference(), combined_prediction(original, selected, "online"):
        result, _ = fly(selected, mode)
    assert not study.score(result.mission, hover=False)["actual_rotor_at_bound_after_release"]
    speeds = result.mission.actual_rotor_omega.copy()
    speeds[1, 0] = 0
    tampered = replace(result.mission, actual_rotor_omega=speeds)
    assert study.score(tampered, hover=False)["actual_rotor_at_bound_after_release"]
