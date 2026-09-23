"""Frozen protocol, all-seed accounting, label isolation and headless reporting."""

import json
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from experiments import eskf_validation as experiment
from experiments.plot_eskf_validation import main as plot_main
from experiments.plot_eskf_validation import render_validation_report
from quadrotor_math.eskf_faults import EskfObservationFault, inject_eskf_observation_faults
from quadrotor_math.eskf_innovation import chi_square_99_percent_eskf_innovation_policy
from quadrotor_math.eskf_replay import EskfObservationKind as Kind
from quadrotor_math.eskf_replay import replay_eskf
from quadrotor_math.eskf_synthetic import make_eskf_synthetic_case
from quadrotor_math.run_manifest import SoftwareProvenance


def test_partitions_are_disjoint_and_full_scope_is_frozen():
    protocols = [experiment.validation_protocol(p) for p in experiment.PARTITIONS]
    sets = [{job["seed"] for job in protocol["jobs"]} for protocol in protocols]
    assert sets == [set(range(10, 12)), set(range(4000, 4005)), set(range(40000, 40100))]
    assert all(not a & b for i, a in enumerate(sets) for b in sets[i + 1 :])
    jobs = protocols[-1]["jobs"]
    assert len(jobs) == 100
    assert sum("fault_gated" in job["variants"] for job in jobs) == 20
    assert sum("q_low" in job["variants"] for job in jobs) == 10
    assert all(job["number_of_steps"] == 3000 for job in jobs)
    assert protocols[-1]["gate"] == {
        "local_position_nis_threshold": 11.345,
        "barometric_altitude_nis_threshold": 6.635,
    }
    assert experiment.protocol_sha256(protocols[-1]) != experiment.protocol_sha256(
        {**protocols[-1], "version": 2}
    )


@pytest.fixture(scope="module")
def smoke():
    return experiment.run_validation("smoke")


def test_smoke_is_repeatable_across_worker_counts_and_has_complete_variants(smoke):
    assert smoke == experiment.run_validation("smoke", workers=2)
    assert smoke["numerical_failure_count"] == 0
    assert smoke["nominal_or_gated_divergence_count"] == 0
    assert not smoke["assessment"]["acceptance_partition"]
    for trial in smoke["trials"]:
        assert len(trial["variants"]) == 8
        assert len(trial["initial_covariance"]) == 15
        for name, variant in trial["variants"].items():
            assert variant["status"] == "completed"
            assert len(variant["nees"]) == 41
            assert variant["minimum_normalized_covariance_eigenvalue"] > 0
            assert variant["max_quaternion_squared_norm_defect"] < 2e-12
            if name.startswith("fault_"):
                assert sum(e["status"] == "pending" for e in variant["fault_events"]) == 2
                assert variant["fault_detection"]["local_position"]["recall"] is None


@pytest.mark.parametrize("workers", [True, 0, -1, 9, 1.5])
def test_invalid_workers(workers):
    with pytest.raises(ValueError):
        experiment.run_validation("smoke", workers)


def test_fault_protocol_acquisitions_offsets_and_counts_are_predeclared():
    case = make_eskf_synthetic_case(4000)
    plan = experiment.validation_fault_plan(case.measurements, 4000)
    again = experiment.validation_fault_plan(case.measurements, 4000)
    assert len(plan) == len(again)
    for kind, outliers, dropped, delayed in [
        (Kind.LOCAL_POSITION, 41, 20, 21),
        (Kind.BAROMETRIC_ALTITUDE, 21, 0, 11),
    ]:
        faults = [f for f in plan if f.kind is kind]
        assert sum(f.offset is not None for f in faults) == outliers
        assert sum(f.dropout for f in faults) == dropped
        assert sum(f.delay_steps > 0 for f in faults) == delayed
    for one, two in zip(plan, again, strict=True):
        if one.offset is not None:
            assert 2 <= np.linalg.norm(one.offset) <= 3
            np.testing.assert_array_equal(one.offset, two.offset)
    # No white-noise stream is consumed by plan construction.
    original = case.measurements.specific_force_measurements_B.copy()
    np.testing.assert_array_equal(case.measurements.specific_force_measurements_B, original)


def test_confusion_metrics_exclude_missing_and_delayed_observations():
    case = make_eskf_synthetic_case(1, family="stationary", number_of_steps=40, stochastic=False)
    injected = inject_eskf_observation_faults(
        case.measurements,
        (
            EskfObservationFault(Kind.LOCAL_POSITION, 0, dropout=True),
            EskfObservationFault(Kind.LOCAL_POSITION, 1, offset=np.array([10.0, 0, 0])),
            EskfObservationFault(
                Kind.LOCAL_POSITION, 2, delay_steps=1, offset=np.array([10.0, 0, 0])
            ),
            EskfObservationFault(Kind.LOCAL_POSITION, 4, delay_steps=1),
        ),
    )
    result = replay_eskf(
        injected.measurements,
        replace(
            case.configuration, innovation_policy=chi_square_99_percent_eskf_innovation_policy()
        ),
    )
    metrics = experiment.fault_detection_metrics(injected, result)["local_position"]
    assert metrics["true_positive"] == 1
    assert metrics["true_negative"] == 1
    assert metrics["false_positive"] == metrics["false_negative"] == 0
    assert (
        metrics["dropped"]
        == metrics["stale"]
        == metrics["pending"]
        == metrics["ineligible_outliers"]
        == 1
    )
    assert metrics["source_count"] == 5
    assert metrics["precision"] == metrics["recall"] == 1
    ungated = replay_eskf(injected.measurements, case.configuration)
    assert (
        experiment.fault_detection_metrics(injected, ungated)["local_position"]["false_negative"]
        == 1
    )
    with pytest.raises(ValueError, match="aligned"):
        experiment.fault_detection_metrics(injected, replace(result, time_s=result.time_s + 0.1))
    wrong = replay_eskf(case.measurements, case.configuration)
    with pytest.raises(ValueError):
        experiment.fault_detection_metrics(injected, wrong)


def test_all_failure_paths_keep_seeds_and_suppress_ensembles(monkeypatch):
    original = experiment.make_eskf_synthetic_case

    def fail(seed, **kwargs):
        if seed == 10:
            raise ValueError("generation failure fixture")
        return original(seed, **kwargs)

    monkeypatch.setattr(experiment, "make_eskf_synthetic_case", fail)
    report = experiment.run_validation("smoke")
    assert report["numerical_failure_count"] == 8
    assert len(report["trials"]) == 2
    assert all(group["ensemble"] is None for group in report["groups"].values())
    assert report["assessment"]["nominal_targets"] is None
    assert all(value is None for value in report["assessment"]["paired_sensitivity"].values())
    assert all(v["stage"] == "generation" for v in report["trials"][0]["variants"].values())


def test_one_variant_failure_does_not_remove_the_other_variants(monkeypatch):
    original = experiment._evaluate_variant

    def fail(case, variant, *args):
        if variant == "q_low":
            raise ValueError("Q diagnostic failure fixture")
        return original(case, variant, *args)

    monkeypatch.setattr(experiment, "_evaluate_variant", fail)
    trial = experiment.run_validation_trial(experiment.planned_jobs("smoke")[0])
    assert trial["variants"]["q_low"]["status"] == "numerical_failure"
    assert trial["variants"]["nominal"]["status"] == "completed"
    assert len(trial["variants"]) == 8


def test_finite_divergence_remains_in_ensemble_and_selection_misalignment_fails(smoke):
    trials = deepcopy(smoke["trials"])
    trials[0]["variants"]["nominal"]["metrics"]["diverged"] = True
    groups = experiment.summarize_validation(trials)
    assert groups["excited/nominal"]["ensemble"]["divergence_count"] == 1
    assert groups["excited/nominal"]["planned_count"] == 2
    trials[0]["variants"]["nominal"]["nis"]["local_position"]["time_s"][0] = 0.0001
    with pytest.raises(ValueError, match="epochs"):
        experiment.summarize_validation(trials)


def test_replay_receives_only_measurements_and_nominal_configuration(monkeypatch):
    original = experiment.replay_eskf
    calls = []

    def spy(data, configuration):
        assert set(data.__dataclass_fields__) == {
            "time_s",
            "specific_force_measurements_B",
            "angular_velocity_measurements_B",
            "observations",
        }
        assert "reference" not in configuration.__dataclass_fields__
        calls.append(1)
        return original(data, configuration)

    monkeypatch.setattr(experiment, "replay_eskf", spy)
    experiment.run_validation_trial(experiment.planned_jobs("smoke")[0])
    assert len(calls) == 8


def test_cli_provenance_exclusive_output_failure_exit_and_source_mutation(
    tmp_path, monkeypatch, smoke
):
    provenance = SoftwareProvenance(".1", "3.12.14", np.__version__, "a" * 40, False)
    monkeypatch.setattr(experiment, "capture_software_provenance", lambda root: provenance)
    monkeypatch.setattr(experiment, "run_validation", lambda *args: deepcopy(smoke))
    output = tmp_path / "result.json"
    assert experiment.main(["--output", str(output)]) == 0
    saved = json.loads(output.read_text())
    assert saved["software_provenance"]["git_commit_sha"] == "a" * 40
    assert len(saved["source_sha256"]) == 64
    with pytest.raises(FileExistsError):
        experiment.main(["--output", str(output)])
    monkeypatch.setattr(
        experiment,
        "run_validation",
        lambda *args: {**deepcopy(smoke), "numerical_failure_count": 1},
    )
    assert experiment.main(["--output", str(tmp_path / "failed.json")]) == 1
    hashes = iter(["before", "after"])
    monkeypatch.setattr(experiment, "_source_sha256", lambda root: next(hashes))
    with pytest.raises(RuntimeError, match="changed"):
        experiment.main(["--output", str(tmp_path / "changed.json")])
    assert not (tmp_path / "changed.json").exists()


def test_headless_figures_have_input_identity_and_refuse_overwrite(tmp_path, smoke):
    source = tmp_path / "report.json"
    source.write_text(json.dumps(smoke))
    output = tmp_path / "figures"
    assert plot_main(["--input", str(source), "--output", str(output)]) == 0
    metadata = json.loads((output / "figures.json").read_text())
    assert len(metadata["figures"]) == 6
    for name in metadata["figures"]:
        assert (output / name).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    with pytest.raises(FileExistsError):
        render_validation_report(smoke, output)
    bad = deepcopy(smoke)
    bad["report_schema"]["version"] = 99
    with pytest.raises(ValueError):
        render_validation_report(bad, tmp_path / "bad")
    assert not (tmp_path / "bad").exists()


def test_incomplete_ensembles_are_omitted_visibly(tmp_path, smoke):
    report = deepcopy(smoke)
    for group in report["groups"].values():
        group["ensemble"] = None
    report["assessment"]["paired_sensitivity"] = {}
    assert render_validation_report(report, tmp_path / "incomplete") == []
