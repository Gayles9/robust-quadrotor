"""Independent seed, fault-source and fail-closed campaign checks."""

import copy
import json
from dataclasses import replace

import numpy as np
import pytest

from experiments import supported_repeatability as campaign
from experiments import supported_start as boundary
from experiments import supported_validation_protocol as protocol
from experiments.estimated_feedback_validation import make_configuration
from experiments.robustness_protocol import configuration as original_configuration
from experiments.robustness_protocol import faults
from quadrotor_math.prearm_alignment import PrearmStatus


def test_fixed_jobs_have_independent_seeds_and_distinct_pair_meanings():
    jobs = protocol.jobs()
    assert len(jobs) == 17 and len({j.name for j in jobs}) == 17
    assert [(j.case, j.seed) for j in jobs[:9]] == [
        (case, seed)
        for seed in (47001, 47002, 47003)
        for case in ("nominal_hover", "nominal_tracking", "wind_tracking")
    ]
    assert all(j.seed == 47004 for j in jobs[9:])
    assert [j.case for j in jobs[9:]] == [
        "position_dropout",
        "position_rejection",
        "position_delay",
        "altitude_dropout",
        "altitude_rejection",
        "altitude_delay",
        "position_recovery",
        "landing_position_dropout",
    ]
    assert protocol.modes(jobs[0]) == ("unaligned", "aligned")
    assert protocol.modes(jobs[-1]) == ("off", "on")


@pytest.mark.parametrize("case", ["nominal_hover", "wind_tracking", "position_rejection"])
def test_reseeding_changes_true_pose_bias_and_noise_without_changing_prior_or_laws(case):
    job = protocol.Job(case, 48001)
    args = protocol.configuration(job, "campaign")
    original = original_configuration(case, "campaign")
    independent = make_configuration(dict(case="hover", seed=48001, noiseless=False))
    for key in original:
        if key not in ("initial_state", "sensors"):
            assert boundary.plain_configuration(args[key]) == boundary.plain_configuration(
                original[key]
            )
    assert args["sensors"].root_seed == 48001
    assert boundary.plain_configuration(args["initial_state"]) == boundary.plain_configuration(
        independent["initial_state"]
    )
    assert boundary.plain_configuration(args["sensors"].imu) == boundary.plain_configuration(
        independent["sensors"].imu
    )
    assert [boundary.plain_configuration(f) for f in faults(case, "campaign", args)] == [
        boundary.plain_configuration(f) for f in faults(case, "campaign", original)
    ]
    assert not np.array_equal(args["initial_state"].q_WB, original["initial_state"].q_WB)
    np.testing.assert_array_equal(args["estimator_configuration"].initial_state.q_WB, [1, 0, 0, 0])


@pytest.mark.parametrize("case", ["position_dropout", "position_rejection", "position_delay"])
def test_faulted_sensor_draws_are_checked_against_original_acquisition_ledger(case):
    prepared = protocol.prepare_job(protocol.Job(case, 48001), "smoke")
    result, diagnostic = protocol.execute(prepared, "off")
    assert prepared.status is PrearmStatus.RELEASED
    affected = [
        r
        for r in diagnostic["fault_records"]
        if r["fault"]["dropout"] or r["fault"]["delay_steps"] or r["fault"]["offset"] is not None
    ]
    assert affected
    checked = boundary.audit(prepared, "aligned", result, diagnostic, supervision="off")
    assert checked["maximum_draw_error"] < 1e-12
    damaged = copy.deepcopy(diagnostic)
    damaged["fault_records"][0]["source"]["measurement"][0] += 0.001
    with pytest.raises(ValueError):
        boundary.audit(prepared, "aligned", result, damaged, supervision="off")


def metrics(peak=0.05, rmse=0.05):
    return dict(
        hover_peak_m=peak,
        tracking_rmse_m=rmse,
        flight_conditions=dict(
            complete=True, rmse=True, final_position=True, final_speed=True, full_hover=True
        ),
        flight_passed=True,
    )


def test_clean_comparison_does_not_average_away_regression_or_failed_limit():
    unaligned = metrics(0.06)
    assert protocol.clean_comparison("nominal_hover", unaligned, metrics())["passed"]
    assert not protocol.clean_comparison("nominal_hover", unaligned, metrics(0.061))["passed"]
    bad = metrics(0.04)
    bad["flight_conditions"]["complete"] = False
    bad["flight_passed"] = False
    assert not protocol.clean_comparison("nominal_hover", unaligned, bad)["passed"]
    # Passing flight thresholds do not imply every numerical metric improved.
    assert protocol.clean_comparison("nominal_tracking", metrics(rmse=0.04), metrics(rmse=0.06))[
        "passed"
    ]


def rows():
    return [
        dict(
            name=j.name,
            status="released",
            modes={m: dict(status="ok", metrics=metrics()) for m in protocol.modes(j)},
            comparison=dict(passed=True),
        )
        for j in protocol.jobs()
    ]


def test_summary_requires_all_pairs_and_keeps_rejection_in_denominator():
    original = rows()
    summary = campaign.summarize(original)
    assert summary["integration_go"] and summary["flights"] == 34
    assert not summary["mass_qualified"] and not summary["hardware_qualified"]
    for changed in (original[:-1], original[::-1], original + original[:1]):
        with pytest.raises(ValueError, match="job ledger"):
            campaign.summarize(changed)
    rejected = copy.deepcopy(original)
    rejected[0].update(status="rejected", modes={}, reasons=["unsupported"])
    assert not campaign.summarize(rejected)["integration_go"]
    failed = copy.deepcopy(original)
    failed[0]["comparison"]["passed"] = False
    assert not campaign.summarize(failed)["integration_go"]


def test_rejected_support_is_saved_and_neither_arm_flies(tmp_path, monkeypatch):
    prior = boundary.support_evidence
    monkeypatch.setattr(
        boundary, "support_evidence", lambda *args: replace(prior(*args), motors_off=False)
    )

    def forbidden(*args, **kwargs):
        raise AssertionError("rejected acquisition flew")

    monkeypatch.setattr(protocol, "execute", forbidden)
    row = campaign.execute_job(protocol.Job("nominal_hover", 48002), tmp_path, "smoke")
    assert row["status"] == "rejected" and not row["modes"]
    assert json.loads((tmp_path / row["name"] / "case.json").read_text())["reasons"]
    assert (tmp_path / row["name"] / "support.npz").exists()


@pytest.fixture(scope="module")
def saved_pair(tmp_path_factory):
    root = tmp_path_factory.mktemp("supported-independent")
    job = protocol.Job("position_dropout", 48002)
    row = campaign.execute_job(job, root, "smoke")
    return root, job, row


def test_saved_fault_pair_reconstructs_response_and_support_boundary(saved_pair):
    root, job, row = saved_pair
    checked = campaign.verify_job(root, job, row, "smoke")
    assert checked == row
    assert row["comparison"]["passed"]
    assert row["response"]["response_conditions"]["common_prefix"]
    assert not row["response"]["flight_required"]


def test_saved_metric_claims_cannot_be_promoted(saved_pair):
    root, job, row = saved_pair
    changed = copy.deepcopy(row)
    changed["modes"]["on"]["metrics"]["tracking_rmse_m"] = 0.0
    with pytest.raises(ValueError):
        campaign.verify_job(root, job, changed, "smoke")
