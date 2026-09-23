"""Regression checks for protocol ownership, trial accounting and evidence timing."""

from copy import deepcopy

import numpy as np
import pytest
from matplotlib.figure import Figure

from experiments import eskf_validation as experiment
from experiments.plot_eskf_validation import render_validation_report
from quadrotor_math.eskf_faults import inject_eskf_observation_faults
from quadrotor_math.eskf_replay import replay_eskf
from quadrotor_math.eskf_synthetic import make_eskf_synthetic_case


@pytest.fixture(scope="module")
def report():
    return experiment.run_validation("smoke")


def test_protocol_metadata_cannot_mutate_covariance_variants():
    protocol = experiment.validation_protocol("smoke")
    original = experiment.SENSITIVITY.copy()
    try:
        protocol["sensitivity_covariance_multipliers"]["q_low"] = (7.0, 8.0)
        assert experiment.SENSITIVITY == original
        assert (
            experiment.validation_protocol("smoke")["sensitivity_covariance_multipliers"]
            == original
        )
    finally:
        experiment.SENSITIVITY.clear()
        experiment.SENSITIVITY.update(original)


@pytest.mark.parametrize("variant", ["fault_typo", "nomnial", ""])
def test_unknown_variant_cannot_be_reported_as_success(variant):
    case = make_eskf_synthetic_case(10, number_of_steps=2)
    injected = inject_eskf_observation_faults(case.measurements, ())
    with pytest.raises(ValueError, match="variant"):
        experiment._evaluate_variant(case, variant, injected, False)


def test_duplicate_seed_cannot_narrow_independent_seed_reference_band(report):
    duplicate = deepcopy(report["trials"][0])
    with pytest.raises(ValueError, match="duplicate"):
        experiment.summarize_validation([report["trials"][0], duplicate])


@pytest.mark.parametrize("change", ["missing_trial", "missing_variant", "unexpected_variant"])
def test_summary_requires_every_declared_job_and_variant(report, change):
    trials = deepcopy(report["trials"])
    if change == "missing_trial":
        trials.pop()
    elif change == "missing_variant":
        del trials[0]["variants"]["q_low"]
    else:
        trials[0]["variants"]["unexpected"] = trials[0]["variants"]["nominal"]
    with pytest.raises(ValueError, match="job|variant"):
        experiment.summarize_validation(trials, protocol=report["protocol"])


def test_diagnostic_epochs_follow_actual_clock_including_last_pre_return_sample():
    case = make_eskf_synthetic_case(
        10, family="stationary", number_of_steps=220, time_step_s=0.05, stochastic=False
    )
    injected = inject_eskf_observation_faults(case.measurements, ())
    record = experiment._evaluate_variant(case, "nominal", injected, False)
    replay = replay_eskf(case.measurements, case.configuration)
    np.testing.assert_array_equal(record["time_s"], case.measurements.time_s)
    # Last available epochs at/before the declared 7.9, 9.99 and 11.0 seconds.
    indices = [158, 199, 220]
    np.testing.assert_array_equal(record["dropout_time_s"], replay.time_s[indices])
    np.testing.assert_array_equal(
        record["dropout_horizontal_variance_m2"],
        replay.covariances[indices, 0, 0] + replay.covariances[indices, 1, 1],
    )


def test_summary_rejects_equal_length_but_different_nees_epochs(report):
    trials = deepcopy(report["trials"])
    trials[0]["variants"]["nominal"]["time_s"] = (
        np.array(trials[1]["variants"]["nominal"]["time_s"]) + 0.001
    ).tolist()
    with pytest.raises(ValueError, match="epochs"):
        experiment.summarize_validation(trials)


@pytest.mark.parametrize(
    "change",
    [
        "protocol_hash",
        "protocol",
        "missing_trial",
        "duplicate_trial",
        "groups",
        "assessment",
        "counts",
        "epochs",
    ],
)
def test_plot_rejects_inconsistent_evidence_before_creating_output(report, tmp_path, change):
    altered = deepcopy(report)
    if change == "protocol_hash":
        altered["protocol_sha256"] = "0" * 64
    elif change == "protocol":
        altered["protocol"]["targets"]["position_rmse_reduction"] = 0.0
        altered["protocol_sha256"] = experiment.protocol_sha256(altered["protocol"])
    elif change == "missing_trial":
        altered["trials"].pop()
    elif change == "duplicate_trial":
        altered["trials"].append(deepcopy(altered["trials"][0]))
    elif change == "groups":
        altered["groups"]["excited/nominal"]["ensemble"]["nees"]["descriptive_mean"] += 1
    elif change == "assessment":
        altered["assessment"]["execution_passed"] = False
    elif change == "counts":
        altered["numerical_failure_count"] = 1
    else:
        altered["trials"][0]["variants"]["nominal"]["time_s"][-1] += 0.01
    output = tmp_path / change
    with pytest.raises(ValueError):
        render_validation_report(altered, output)
    assert not output.exists()


def test_legacy_v1_report_is_validated_with_its_declared_clock_without_mutation(report):
    legacy = deepcopy(report)
    legacy["report_schema"]["version"] = 1
    for trial in legacy["trials"]:
        for record in trial["variants"].values():
            record.pop("time_s")
    before = deepcopy(legacy)
    normalized = experiment.validate_validation_report(legacy)
    assert normalized == report
    assert legacy == before


def test_finite_json_and_failed_seed_accounting_remain_enforced(report):
    broken = deepcopy(report)
    broken["trials"][0]["variants"]["nominal"]["nees"][0] = float("nan")
    with pytest.raises(ValueError):
        experiment.validate_validation_report(broken)
    failed = deepcopy(report["trials"])
    failed[0]["variants"]["nominal"] = {
        "status": "numerical_failure",
        "stage": "replay_or_evaluation",
        "error": "fixture",
    }
    rebuilt = experiment._assemble_report(report["protocol"], failed)
    validated = experiment.validate_validation_report(rebuilt)
    assert validated["numerical_failure_count"] == 1
    assert validated["groups"]["excited/nominal"]["ensemble"] is None
    assert not validated["assessment"]["execution_passed"]


def test_legacy_conversion_does_not_replace_conflicting_explicit_epochs(report):
    legacy = deepcopy(report)
    legacy["report_schema"]["version"] = 1
    legacy["trials"][0]["variants"]["nominal"]["time_s"][-1] = 1.0
    with pytest.raises(ValueError, match="epochs"):
        experiment.validate_validation_report(legacy)


def test_short_plot_does_not_extend_axes_to_faults_outside_its_horizon(
    report, tmp_path, monkeypatch
):
    figures = {}

    def capture(figure, path, **kwargs):
        figures[path.name] = figure

    monkeypatch.setattr(Figure, "savefig", capture)
    render_validation_report(report, tmp_path / "short")
    for axis in figures["faults.png"].axes[:2]:
        assert axis.get_xlim()[1] < 0.5
        assert not axis.patches
    assert [text.get_text() for text in figures["faults.png"].axes[2].texts] == ["N/A"] * 4
    handles = figures["faults.png"].axes[2].get_legend().legend_handles
    assert handles[0].get_facecolor() != handles[1].get_facecolor()
