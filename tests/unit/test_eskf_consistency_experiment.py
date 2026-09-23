"""Frozen nominal ensemble integration, reproducibility and honest failure accounting."""

import importlib.util
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_consistency import eskf_right_local_error
from quadrotor_math.eskf_innovation import EskfInnovationPolicy
from quadrotor_math.eskf_replay import replay_eskf
from quadrotor_math.eskf_run_replay import eskf_replay_input_from_run_artifact
from quadrotor_math.run_generation import generate_run_artifact_data
from quadrotor_math.run_manifest import SoftwareProvenance, decode_run_manifest

SPEC = importlib.util.spec_from_file_location(
    "consistency_experiment", Path(__file__).parents[2] / "experiments/eskf_consistency.py"
)
assert SPEC is not None and SPEC.loader is not None
experiment = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = experiment
SPEC.loader.exec_module(experiment)
PROVENANCE = SoftwareProvenance("0.1.0", "3.12.14", np.__version__, "a" * 40, False)


def test_protocol_partitions_are_fixed_disjoint_and_content_addressed():
    protocols = [
        experiment.study_protocol(p, PROVENANCE) for p in ("smoke", "development", "validation")
    ]
    sets = [set(p["seeds"]) for p in protocols]
    assert sets == [{0, 1}, set(range(1000, 1010)), set(range(20000, 20100))]
    assert all(not a & b for i, a in enumerate(sets) for b in sets[i + 1 :])
    assert [p["number_of_steps"] for p in protocols] == [21, 201, 201]
    assert experiment.protocol_sha256(protocols[0]) == experiment.protocol_sha256(
        experiment.study_protocol("smoke", replace(PROVENANCE, git_commit_sha="b" * 40))
    )
    changed = dict(protocols[0], prior_block_standard_deviations=[1, 1, 1, 1, 1])
    assert experiment.protocol_sha256(changed) != experiment.protocol_sha256(protocols[0])
    with pytest.raises(ValueError):
        experiment.study_protocol("invented", PROVENANCE)


@pytest.mark.parametrize("scenario", ["hover", "translating_yaw"])
def test_scenario_truth_matches_independent_analytic_motion(scenario):
    configuration = experiment.run_configuration(scenario, 0, 21)
    artifact = generate_run_artifact_data(configuration)
    times = artifact.truth_time_s
    velocity = np.zeros(3) if scenario == "hover" else np.array([0.5, -0.25, 0.1])
    rate = 0.0 if scenario == "hover" else 0.4
    np.testing.assert_allclose(
        artifact.truth_position_history_W, times[:, None] * velocity, atol=2e-14
    )
    np.testing.assert_allclose(
        artifact.truth_q_history_WB,
        np.column_stack(
            [
                np.cos(rate * times / 2),
                np.zeros_like(times),
                np.zeros_like(times),
                np.sin(rate * times / 2),
            ]
        ),
        atol=2e-14,
    )


@pytest.mark.parametrize("scenario", ["hover", "translating_yaw"])
def test_prior_is_independent_draw_with_correct_first_bias_step_variance(scenario):
    run = experiment.run_configuration(scenario, 5, 21)
    prior = experiment.replay_configuration(scenario, run)
    scenario_id = 0 if scenario == "hover" else 1
    rng = np.random.Generator(
        np.random.PCG64(np.random.SeedSequence([0x45534B46, 1, scenario_id, 5], pool_size=4))
    )
    std = np.repeat([0.2, 0.1, 0.01, 0.02, 0.002], 3)
    expected_error = rng.standard_normal(15) * std
    dt = 0.01
    velocity = np.zeros(3) if scenario == "hover" else np.array([0.5, -0.25, 0.1])
    rate = 0 if scenario == "hover" else 0.4
    anchor = EskfNominalState(
        velocity * dt,
        velocity,
        np.array([np.cos(rate * dt / 2), 0, 0, np.sin(rate * dt / 2)]),
        np.zeros(3),
        np.zeros(3),
    )
    np.testing.assert_allclose(
        eskf_right_local_error(prior.initial_state, anchor), expected_error, atol=1e-16
    )
    variances = std**2
    variances[9:12] += 0.001**2 * dt
    variances[12:15] += 0.0001**2 * dt
    np.testing.assert_allclose(prior.initial_covariance, np.diag(variances), rtol=1e-15)
    assert isinstance(prior.innovation_policy, EskfInnovationPolicy)
    assert prior.innovation_policy.local_position_nis_threshold is None
    assert prior.innovation_policy.barometric_altitude_nis_threshold is None
    # Neither generation nor construction of another seed affects the prior stream.
    generate_run_artifact_data(run)
    again = experiment.replay_configuration(scenario, run)
    np.testing.assert_array_equal(again.initial_state.q_WB, prior.initial_state.q_WB)


def test_nis_summary_retains_rejections_and_reports_unscored_dispositions():
    run = experiment.run_configuration("hover", 0, 21)
    measured = eskf_replay_input_from_run_artifact(generate_run_artifact_data(run))
    config = replace(
        experiment.replay_configuration("hover", run),
        innovation_policy=EskfInnovationPolicy(1e-20, 1e-20),
    )
    result = replay_eskf(measured, config)
    for series in experiment.innovation_series(result).values():
        assert series.counts["rejected"] == len(series.scores)
        assert series.counts["fused"] == 0
        assert len(series.scores) > 0
        assert np.all(series.scores > 1e-20)
    disabled = replay_eskf(
        measured, replace(config, fuse_local_position=False, fuse_barometric_altitude=False)
    )
    for series in experiment.innovation_series(disabled).values():
        assert series.counts["disabled"] > 0
        assert series.counts["unscored"] == series.counts["disabled"]
        assert len(series.scores) == 0


@pytest.fixture(scope="module")
def smoke():
    return experiment.run_study("smoke", PROVENANCE)


def test_smoke_has_complete_manifest_seed_error_and_event_evidence(smoke):
    assert smoke["protocol_sha256"] == experiment.protocol_sha256(smoke["protocol"])
    assert smoke["numerical_failure_count"] == 0
    assert smoke["divergence_count"] == 0
    for scenario in smoke["scenarios"]:
        assert len(scenario["trials"]) == 2
        assert scenario["ensemble"] is not None
        for trial in scenario["trials"]:
            manifest = decode_run_manifest(json.dumps(trial["run_manifest"]).encode())
            assert manifest.run_configuration.root_seed == trial["seed"]
            assert manifest.software_provenance == PROVENANCE
            assert len(trial["nees"]) == 21
            assert len(trial["time_s"]) == 21
            assert trial["time_s"][0] == 0.01
            assert set(trial["fused_metrics"]["rmse"]) == set(experiment.BLOCK_UNITS)
            assert trial["nis"]["local_position"]["counts"]["fused"] == 4
            assert trial["nis"]["barometric_altitude"]["counts"]["fused"] == 2
            assert all(np.isfinite(trial["nees"]))


def test_repeat_study_is_byte_deterministic(smoke):
    assert experiment.canonical_json(
        experiment.run_study("smoke", PROVENANCE)
    ) == experiment.canonical_json(smoke)


def test_failed_seed_is_retained_and_suppresses_ensemble_without_silencing_other_trials(
    monkeypatch,
):
    original = experiment.generate_run_artifact_data

    def fail_one(run):
        if run.root_seed == 0:
            raise ValueError("injected numerical failure")
        return original(run)

    monkeypatch.setattr(experiment, "generate_run_artifact_data", fail_one)
    result = experiment.run_study("smoke", PROVENANCE)
    assert result["numerical_failure_count"] == 2
    for scenario in result["scenarios"]:
        assert scenario["ensemble"] is None
        assert len(scenario["trials"]) == 2
        assert scenario["trials"][0]["failure_stage"] == "generation"
        assert "injected numerical failure" in scenario["trials"][0]["error"]
        assert scenario["trials"][1]["status"] == "completed"


def test_metrics_keep_physical_blocks_and_predeclared_divergence_thresholds():
    errors = np.zeros((2, 15))
    errors[:, 0:3] = [[3, 4, 0], [0, 0, 0]]
    metrics = experiment.error_metrics(errors)
    assert metrics["rmse"]["position_m"] == pytest.approx(np.sqrt(12.5))
    assert metrics["max_position_m"] == 5
    assert not metrics["diverged"]
    errors[1, 0] = 10
    assert not experiment.error_metrics(errors)["diverged"]
    errors[1, 0] = np.nextafter(10.0, np.inf)
    assert experiment.error_metrics(errors)["diverged"]
    errors[1, 0] = 0
    errors[1, 6] = np.pi / 2
    assert not experiment.error_metrics(errors)["diverged"]
    errors[1, 6] += 1e-10
    assert experiment.error_metrics(errors)["diverged"]


def test_cli_exclusive_output_and_reproducible_metadata(tmp_path, monkeypatch):
    output = tmp_path / "report.json"
    monkeypatch.setattr(experiment, "capture_software_provenance", lambda root: PROVENANCE)
    assert experiment.main(["--partition", "smoke", "--output", str(output)]) == 0
    report = json.loads(output.read_text())
    assert len(report["source_sha256"]) == 64
    assert report["generated_at_utc"].endswith("+00:00")
    before = output.read_bytes()
    with pytest.raises(FileExistsError):
        experiment.main(["--partition", "smoke", "--output", str(output)])
    assert output.read_bytes() == before


def test_truth_access_occurs_only_after_both_measurement_replays(monkeypatch):
    generate = experiment.generate_run_artifact_data
    replay = experiment.replay_eskf
    reference = experiment.eskf_reference_from_run_artifact
    calls = 0
    allow_truth = False

    class Guard:
        def __init__(self, data):
            self.data = data

        def __getattr__(self, name):
            if (name.startswith("truth_") and name != "truth_time_s") or "bias_history" in name:
                assert allow_truth, f"Premature truth access: {name}"
            return getattr(self.data, name)

    def measured_replay(data, config):
        nonlocal calls
        calls += 1
        return replay(data, config)

    def evaluate_reference(data):
        nonlocal allow_truth
        assert calls == 2
        allow_truth = True
        return reference(data)

    monkeypatch.setattr(experiment, "generate_run_artifact_data", lambda run: Guard(generate(run)))
    monkeypatch.setattr(experiment, "replay_eskf", measured_replay)
    monkeypatch.setattr(experiment, "eskf_reference_from_run_artifact", evaluate_reference)
    assert experiment._run_trial("hover", 0, 21, PROVENANCE).consistency is not None


def test_prior_does_not_consume_truth_configuration_or_initial_truth_state():
    run = experiment.run_configuration("hover", 0, 21)

    class NominalOnly:
        def __getattr__(self, name):
            assert name in ("nominal", "numerics", "root_seed"), name
            return getattr(run, name)

    prior = experiment.replay_configuration("hover", NominalOnly())
    assert prior.initial_time_s == 0.01


@pytest.mark.parametrize("call_to_fail,stage", [(1, "fused_replay"), (2, "dead_reckoning_replay")])
def test_replay_numerical_failure_records_seed_and_stage(monkeypatch, call_to_fail, stage):
    original = experiment.replay_eskf
    count = 0

    def fail_at(data, config):
        nonlocal count
        count += 1
        if count == call_to_fail:
            raise ValueError("injected replay failure")
        return original(data, config)

    monkeypatch.setattr(experiment, "replay_eskf", fail_at)
    trial = experiment._run_trial("hover", 1, 21, PROVENANCE)
    assert trial.consistency is None
    assert trial.report["seed"] == 1
    assert trial.report["failure_stage"] == stage


def test_singular_covariance_is_an_evaluation_failure_not_zero_nees(monkeypatch):
    original = experiment.replay_eskf

    def singular(data, config):
        result = original(data, config)
        return replace(result, covariances=np.zeros_like(result.covariances))

    monkeypatch.setattr(experiment, "replay_eskf", singular)
    trial = experiment._run_trial("hover", 0, 21, PROVENANCE)
    assert trial.report["failure_stage"] == "reference_and_consistency"
    assert "NEES" in trial.report["error"]
    assert "nees" not in trial.report


@pytest.mark.parametrize("changed", ["nees", "local_position", "barometric_altitude"])
def test_ensemble_rejects_misaligned_epochs(changed):
    one = experiment._run_trial("hover", 0, 21, PROVENANCE)
    two = experiment._run_trial("hover", 1, 21, PROVENANCE)
    if changed == "nees":
        two = replace(
            two, consistency=replace(two.consistency, time_s=two.consistency.time_s + 1e-15)
        )
    else:
        nis = dict(two.nis)
        nis[changed] = replace(nis[changed], time_s=nis[changed].time_s + 1e-15)
        two = replace(two, nis=nis)
    with pytest.raises(ValueError, match="epochs"):
        experiment._ensemble([one, two])


def test_finite_divergent_trials_remain_in_ensemble_and_cli_returns_nonzero(tmp_path, monkeypatch):
    original = experiment.error_metrics

    def divergent(errors):
        return dict(original(errors), diverged=True)

    monkeypatch.setattr(experiment, "error_metrics", divergent)
    monkeypatch.setattr(experiment, "capture_software_provenance", lambda root: PROVENANCE)
    output = tmp_path / "divergent.json"
    assert experiment.main(["--output", str(output)]) == 1
    report = json.loads(output.read_text())
    assert report["divergence_count"] == 4
    assert report["numerical_failure_count"] == 0
    assert all(s["ensemble"] is not None for s in report["scenarios"])


def test_cli_rejects_source_mutation_during_execution(tmp_path, monkeypatch):
    calls = iter(["before", "after"])
    monkeypatch.setattr(experiment, "_source_sha256", lambda root: next(calls))
    monkeypatch.setattr(experiment, "capture_software_provenance", lambda root: PROVENANCE)
    output = tmp_path / "changed.json"
    with pytest.raises(RuntimeError, match="source files changed"):
        experiment.main(["--output", str(output)])
    assert not output.exists()
