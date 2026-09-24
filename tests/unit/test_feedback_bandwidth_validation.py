"""Frozen independent campaign, protected baseline, and evidence integrity."""

from copy import deepcopy
from dataclasses import asdict, replace

import numpy as np
import pytest

from experiments.attitude_control_validation import canonical_json
from experiments.estimated_feedback_evidence import plain
from experiments.estimated_feedback_validation import make_configuration as original_configuration
from experiments.estimated_feedback_validation import validation_protocol as original_protocol
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


def test_exact_coefficient_match_and_sampled_stability():
    m = designed_horizontal_cascade()
    desired = 0.025 * np.polymul(np.polymul([1, 6, 9], [1, 6]), [1, 28, 392])
    np.testing.assert_allclose(m.characteristic_polynomial(), desired, rtol=1e-15)
    poles = np.log(np.linalg.eigvals(m.lifted_transition()).astype(complex)) / 0.02
    assert np.max(poles.real) < -2.4
    assert np.min(-poles.real / abs(poles)) > 0.60


def test_profile_changes_only_documented_gains_and_initial_covariance():
    job = planned_jobs("fixed")[1]
    original, new = original_configuration(job), make_configuration(job)
    for key in original:
        if key not in ("position_controller", "attitude_controller", "estimator_configuration"):

            def value(x):
                return plain(asdict(x)) if hasattr(x, "__dataclass_fields__") else plain(x)

            assert canonical_json(value(original[key])) == canonical_json(value(new[key]))
    for key, fields in [
        ("attitude_controller", ("attitude_gain_B", "rate_gain_B")),
        ("position_controller", ("position_gain_W", "velocity_gain_W")),
    ]:
        a, b = asdict(original[key]), asdict(new[key])
        for field in fields:
            assert a[field][2] == b[field][2]
            assert not np.shares_memory(getattr(original[key], field), getattr(new[key], field))
            b[field] = a[field]
        assert canonical_json(plain(a)) == canonical_json(plain(b))
    half_width = np.array([0.02] * 6 + [np.deg2rad(2)] * 3 + [0.02] * 3 + [0.003] * 3)
    a, b = asdict(original["estimator_configuration"]), asdict(new["estimator_configuration"])
    np.testing.assert_array_equal(b["initial_covariance"], np.diag(half_width**2 / 3))
    b["initial_covariance"] = a["initial_covariance"]
    assert canonical_json(plain(a)) == canonical_json(plain(b))


def test_noiseless_prior_and_population_not_realization_based():
    noiseless = make_configuration({"case": "square", "seed": None, "noiseless": True})
    np.testing.assert_array_equal(noiseless["estimator_configuration"].initial_covariance, 0)
    a, b = [make_configuration({"case": "hover", "seed": s, "noiseless": False}) for s in [1, 2]]
    np.testing.assert_array_equal(
        a["estimator_configuration"].initial_covariance,
        b["estimator_configuration"].initial_covariance,
    )
    assert not np.array_equal(a["initial_state"].position_W, b["initial_state"].position_W)
    # Independent numerical quadrature of the uniform second moment.
    x, w = np.polynomial.legendre.leggauss(3)
    half_width = np.array([0.02] * 6 + [np.deg2rad(2)] * 3 + [0.02] * 3 + [0.003] * 3)
    expected = np.array([np.dot(w, (width * x) ** 2) / 2 for width in half_width])
    np.testing.assert_allclose(
        np.diag(a["estimator_configuration"].initial_covariance), expected, rtol=2e-15
    )


def test_legacy_protocol_hashes_unchanged_and_seeds_disjoint():
    import hashlib

    hashes = {
        "fixed": "dcc035d1589e82bb7f7d3490f38f6765a62015a44ef064bd6fa0a6cdd3d5ca15",
        "development": "d2dffe95e0ec54ed836dd3be6aea183e778318ef390ab388ba22bbaccf5b8b53",
        "validation": "e942a867424a6c4d1578c949286f90f99171ae183655808d276e9cd451e66927",
    }
    for partition, digest in hashes.items():
        assert hashlib.sha256(canonical_json(original_protocol(partition))).hexdigest() == digest
    sets = [
        {j["seed"] for j in planned_jobs(p)}
        for p in ("smoke", "fixed", "development", "validation")
    ]
    assert all(not a & b for i, a in enumerate(sets) for b in sets[i + 1 :])
    assert len(planned_jobs("validation")) == 30
    assert len([j for j in planned_jobs("validation") if j["case"] == "hover"]) == 20
    assert not sets[-1] & set(range(90000, 90010))
    assert validation_protocol("fixed")["jobs"] == original_protocol("fixed")["jobs"]
    assert validation_protocol("fixed")["inherited_contract"] == original_protocol("fixed")


@pytest.fixture(scope="module")
def smoke():
    return run_validation("smoke")


def test_complete_worker_parity_and_persistence(smoke, tmp_path):
    other = run_validation("smoke", workers=2)
    assert smoke["summary"]["passed"]
    for a, b in zip(smoke["trials"], other["trials"], strict=True):
        assert canonical_json(a["metrics"]) == canonical_json(b["metrics"])
        for key in a["history"]:
            np.testing.assert_array_equal(a["history"][key], b["history"][key])
    save_report(smoke, tmp_path / "evidence")
    loaded = load_report(tmp_path / "evidence")
    assert loaded["summary"] == smoke["summary"]
    for a, b in zip(smoke["trials"], loaded["trials"], strict=True):
        for key in a["history"]:
            np.testing.assert_array_equal(a["history"][key], b["history"][key])
    with pytest.raises(FileExistsError):
        save_report(smoke, tmp_path / "evidence")
    path = tmp_path / "evidence/trial-000-cov-000.npz"
    path.write_bytes(path.read_bytes() + b"corrupt")
    with pytest.raises(ValueError, match="digest"):
        load_report(tmp_path / "evidence")


@pytest.mark.parametrize(
    "bad",
    [
        "missing",
        "reorder",
        "metric",
        "summary",
        "profile",
        "covariance",
        "command",
        "baseline_command",
        "baseline_moment",
        "baseline_flag",
    ],
)
def test_rejects_corruption(smoke, bad):
    r = deepcopy(smoke)
    h = r["trials"][0]["history"]
    if bad == "missing":
        r["trials"].pop()
    elif bad == "reorder":
        r["trials"].reverse()
    elif bad == "metric":
        r["trials"][0]["metrics"]["peak_attitude_estimation_deg"] += 1
    elif bad == "summary":
        r["summary"]["passed"] = False
    elif bad == "profile":
        r["protocol"]["configurations"][0]["attitude_controller"]["attitude_gain_B"][0] += 1
    elif bad == "covariance":
        h["covariances"][1, 0, 0] += 1
    elif bad == "command":
        h["mission_commanded_rotor_omega"][0, 0] += 1
    elif bad == "baseline_command":
        h["baseline_commanded_rotor_omega"][0, 0] += 1
    elif bad == "baseline_moment":
        h["baseline_moment_requested_B"][0, 0] += 0.01
    else:
        h["baseline_inner_limit_flags"][0, 0] = not h["baseline_inner_limit_flags"][0, 0]
    with pytest.raises(ValueError):
        validate_report(r)


def test_failures_retained(monkeypatch):
    def fail(**kwargs):
        raise ValueError("deliberate failure")

    monkeypatch.setattr(
        "experiments.feedback_bandwidth_validation.simulate_estimated_mission", fail
    )
    r = run_validation("smoke")
    assert r["summary"]["numerical_failures"] == 2
    assert not r["summary"]["passed"]
    validate_report(r)


@pytest.mark.parametrize("bad", [True, 0, -1, 1.5])
def test_invalid_workers(bad):
    with pytest.raises(ValueError):
        run_validation("smoke", bad)


def test_unknown_partition():
    with pytest.raises(ValueError):
        planned_jobs("unknown")


def test_plot_validation_and_identity(smoke, tmp_path):
    import json

    from experiments.plot_feedback_bandwidth import render_report

    output = tmp_path / "plots"
    names = render_report(smoke, output)
    assert set(names) == {"feedback_ensemble.png", "feedback_local_design.png"}
    manifest = json.loads((output / "plot_manifest.json").read_bytes())
    assert manifest["files"] == names
    assert len(manifest["extension_plotter_sha256"]) == 64
    assert all((output / name).read_bytes().startswith(b"\x89PNG") for name in names)
    with pytest.raises(FileExistsError):
        render_report(smoke, output)
    bad = deepcopy(smoke)
    bad["trials"].pop()
    with pytest.raises(ValueError):
        render_report(bad, tmp_path / "bad")
    assert not (tmp_path / "bad").exists()


def test_original_campaign_also_checks_paired_inner_commands():
    from experiments.estimated_feedback_validation import run_validation as old_run
    from experiments.estimated_feedback_validation import validate_report as old_validate

    original = deepcopy(old_run("smoke"))
    old_validate(original)
    original["trials"][0]["history"]["baseline_moment_requested_B"][0, 0] += 0.01
    with pytest.raises(ValueError, match="inner command"):
        old_validate(original)


def test_seed_30_startup_regression_without_moving_takeoff_or_hover_start():
    from quadrotor_math.estimated_mission import simulate_estimated_mission

    job = {"case": "hover", "seed": 30, "noiseless": False}
    kwargs = make_configuration(job)
    # Bounded CI regression, not the full 60-second campaign acceptance.
    plan = kwargs["plan"]
    segments = tuple(
        replace(s, duration_s=5.0) if i == 2 else s for i, s in enumerate(plan.segments)
    )
    kwargs["plan"] = replace(plan, segments=segments)
    result = simulate_estimated_mission(**kwargs)
    r = result.mission
    window = (r.time_s >= 5) & (r.time_s <= 10)
    assert max(np.linalg.norm(r.position_W[window] - r.reference_position_W[window], axis=1)) < 0.08
    assert r.abort_reason is None
    assert not np.any(r.inner_limit_flags)
