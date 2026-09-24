"""Settling-margin revision; preserved prior evidence and complete parallel audits."""

import hashlib
from copy import deepcopy
from dataclasses import asdict

import numpy as np
import pytest
from test_cascade_analysis import (
    test_lifted_model_matches_nonlinear_plant_jacobian as check_nonlinear_jacobian,
)

from experiments.attitude_control_validation import canonical_json
from experiments.estimated_feedback_evidence import plain
from experiments.feedback_bandwidth_validation import (
    designed_horizontal_cascade,
    load_report,
    make_configuration,
    planned_jobs,
    run_validation,
    save_report,
    validate_report,
    validation_protocol,
)


def test_version_one_protocols_are_preserved_exactly():
    hashes = {
        "fixed": "80df4d0315667dc87e57078ae74bfa43bfa59cc9e7f1a0c81dd04c69f381d93e",
        "development": "4acb0aa56eb4e75fb3097dbb7a4b3fda4404a666599464d3abdcd7fe2877859b",
        "validation": "f7be987e273d1095ee20b3376ac10c949f78e82f550f8dcea9fe361798642eca",
    }
    for partition, expected in hashes.items():
        p = validation_protocol(partition)
        assert hashlib.sha256(canonical_json(p)).hexdigest() == expected
        assert p == validation_protocol(partition, design_version=1)


def test_all_real_continuous_design_and_sampled_margin():
    m = designed_horizontal_cascade(2)
    # Hand-expanded .025*(s+8)^5, independent literal coefficients.
    expected = [0.025, 1, 16, 128, 512, 819.2]
    np.testing.assert_allclose(m.characteristic_polynomial(), expected, rtol=2e-15)
    np.testing.assert_allclose(np.poly(m.continuous_matrix()), np.array(expected) / 0.025)
    poles = np.log(np.linalg.eigvals(m.lifted_transition()).astype(complex)) / 0.02
    assert max(poles.real) < -3.9
    assert min(-poles.real / abs(poles)) > 0.76
    old = designed_horizontal_cascade(1)
    assert m.gravity / m.position_gain < 0.7 * old.gravity / old.position_gain
    assert m.velocity_gain / m.position_gain < 0.8 * old.velocity_gain / old.position_gain
    assert m.attitude_gain * m.rate_gain < old.attitude_gain * old.rate_gain
    assert m.rate_gain < old.rate_gain


@pytest.mark.parametrize("axis, body_axis, sign", [(0, 1, -1), (1, 0, 1)])
def test_revised_profile_matches_both_nonlinear_axis_jacobians(axis, body_axis, sign):
    m = designed_horizontal_cascade(2)
    check_nonlinear_jacobian(
        axis, body_axis, sign, m.position_gain, m.velocity_gain, m.attitude_gain, m.rate_gain
    )


def test_revision_changes_only_four_gains_and_keeps_population_prior():
    job = planned_jobs("fixed")[1]
    a, b = make_configuration(job), make_configuration(job, design_version=2)
    for key in a:
        va = asdict(a[key]) if hasattr(a[key], "__dataclass_fields__") else a[key]
        vb = asdict(b[key]) if hasattr(b[key], "__dataclass_fields__") else b[key]
        if key in ("position_controller", "attitude_controller"):
            names = (
                ("position_gain_W", "velocity_gain_W")
                if key == "position_controller"
                else ("attitude_gain_B", "rate_gain_B")
            )
            for name in names:
                assert va[name][2] == vb[name][2]
                assert not np.array_equal(va[name][:2], vb[name][:2])
                vb[name] = va[name]
        assert canonical_json(plain(va)) == canonical_json(plain(vb))


def test_seed_exposure_and_version_boundaries_are_explicit():
    fresh = {j["seed"] for j in planned_jobs("validation", design_version=2)}
    observed = {
        j["seed"] for p in ("smoke", "fixed", "development", "validation") for j in planned_jobs(p)
    }
    development = planned_jobs("development", design_version=2)
    assert len(fresh) == 30 and not fresh & observed
    assert not fresh & {j["seed"] for j in development}
    assert {91001, 91011, 91016, 91019} <= {j["seed"] for j in development}
    assert len(development) == 13
    assert planned_jobs("fixed", design_version=2) == planned_jobs("fixed")
    protocol = validation_protocol("validation", design_version=2)
    assert (
        protocol["version"] == 2
        and "29/30" in protocol["revision_evidence"]["preceding_validation"]
    )
    assert protocol["inherited_contract"] == validation_protocol("fixed")["inherited_contract"]


@pytest.mark.parametrize("bad", [True, 0, 3, 1.0, "2"])
def test_invalid_design_version(bad):
    with pytest.raises(ValueError, match="design_version"):
        designed_horizontal_cascade(bad)
    with pytest.raises(ValueError, match="design_version"):
        planned_jobs("fixed", design_version=bad)
    with pytest.raises(ValueError, match="design_version"):
        run_validation("smoke", design_version=bad)


@pytest.fixture(scope="module")
def revised_smoke():
    return run_validation("smoke", design_version=2)


def test_parallel_audit_and_simulation_preserve_every_array(revised_smoke, tmp_path):
    parallel = run_validation("smoke", workers=2, design_version=2)
    for a, b in zip(revised_smoke["trials"], parallel["trials"], strict=True):
        assert canonical_json(a["metrics"]) == canonical_json(b["metrics"])
        for key in a["history"]:
            np.testing.assert_array_equal(a["history"][key], b["history"][key])
    validate_report(revised_smoke)
    validate_report(parallel, workers=2)
    save_report(revised_smoke, tmp_path / "serial")
    save_report(parallel, tmp_path / "parallel", workers=2)
    serial_paths = sorted((tmp_path / "serial").iterdir())
    parallel_paths = sorted((tmp_path / "parallel").iterdir())
    assert [p.name for p in serial_paths] == [p.name for p in parallel_paths]
    assert all(
        a.read_bytes() == b.read_bytes() for a, b in zip(serial_paths, parallel_paths, strict=True)
    )
    loaded = load_report(tmp_path / "parallel", workers=2)
    assert loaded["summary"]["passed"]
    assert loaded["protocol"]["version"] == 2


@pytest.mark.parametrize("part", ["metrics", "estimated_command", "paired_command", "covariance"])
def test_parallel_audit_propagates_failures_before_publication(revised_smoke, tmp_path, part):
    bad = deepcopy(revised_smoke)
    row = bad["trials"][-1]
    if part == "metrics":
        row["metrics"]["peak_attitude_estimation_deg"] += 1
    elif part == "estimated_command":
        row["history"]["mission_commanded_rotor_omega"][0, 0] += 1
    elif part == "paired_command":
        row["history"]["baseline_commanded_rotor_omega"][0, 0] += 1
    else:
        row["history"]["covariances"][1, 0, 0] += 1
    with pytest.raises(ValueError):
        save_report(bad, tmp_path / "bad", workers=2)
    assert not (tmp_path / "bad").exists()


def test_old_report_still_roundtrips_with_parallel_audit(tmp_path):
    old = run_validation("smoke")
    save_report(old, tmp_path / "old", workers=2)
    loaded = load_report(tmp_path / "old", workers=2)
    assert loaded["protocol"] == old["protocol"]
    assert loaded["summary"] == old["summary"]


@pytest.mark.parametrize("bad", [True, 0, -1, 1.5])
def test_invalid_audit_workers(revised_smoke, tmp_path, bad):
    with pytest.raises(ValueError, match="workers"):
        save_report(revised_smoke, tmp_path / "bad", workers=bad)
    assert not (tmp_path / "bad").exists()


def test_plot_uses_recorded_design_version(revised_smoke, tmp_path, monkeypatch):
    from experiments import plot_feedback_bandwidth as plot

    actual = plot.designed_horizontal_cascade
    requested = []

    def record(version):
        requested.append(version)
        return actual(version)

    monkeypatch.setattr(plot, "designed_horizontal_cascade", record)
    plot.render_report(revised_smoke, tmp_path / "plots")
    assert requested == [2]


@pytest.mark.parametrize("seed", [30, 91001])
def test_startup_regressions_keep_the_same_takeoff_and_hold_start(seed):
    from dataclasses import replace

    from quadrotor_math.estimated_mission import simulate_estimated_mission

    kw = make_configuration({"case": "hover", "seed": seed, "noiseless": False}, design_version=2)
    plan = kw["plan"]
    kw["plan"] = replace(
        plan,
        segments=tuple(
            replace(s, duration_s=5.0) if i == 2 else s for i, s in enumerate(plan.segments)
        ),
    )
    r = simulate_estimated_mission(**kw).mission
    selected = (r.time_s >= 5) & (r.time_s <= 10)
    assert (
        max(np.linalg.norm(r.position_W[selected] - r.reference_position_W[selected], axis=1))
        < 0.07
    )
    assert r.abort_reason is None and not np.any(r.inner_limit_flags)
