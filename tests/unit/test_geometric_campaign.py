"""Frozen case accounting and failure retention for new reconstruction evidence."""

from copy import deepcopy

import numpy as np

from experiments.geometric_reimplementation_validation import (
    configuration,
    jobs,
    name_of,
    summarize,
)


def test_frozen_cases_are_unique_and_include_failed_case_accounting(tmp_path):
    planned = jobs()
    assert len(planned) == len({name_of(j) for j in planned}) == 28
    rows = [dict(index=i, job=j, error="test failure") for i, j in enumerate(planned)]
    report = summarize(rows, tmp_path)
    assert report["complete_ledger"]
    assert not report["true_state_passed"] and not report["all_performance_passed"]
    assert not summarize(rows[:-1], tmp_path)["complete_ledger"]
    duplicate = deepcopy(rows)
    duplicate[-1] = duplicate[0]
    assert not summarize(duplicate, tmp_path)["complete_ledger"]


def test_matched_spline_pairs_have_identical_physical_and_estimator_inputs():
    base = dict(mode="estimated", case="mild_wind", seed=30, repeat=False)
    a = configuration({**base, "controller": "cascade"})
    b = configuration({**base, "controller": "geometric"})
    for name in ("position_gain_W", "velocity_gain_W"):
        np.testing.assert_array_equal(
            getattr(a["position_controller"], name), getattr(b["position_controller"], name)
        )
    np.testing.assert_array_equal(a["initial_state"].position_W, b["initial_state"].position_W)
    np.testing.assert_array_equal(
        a["estimator_configuration"].initial_covariance,
        b["estimator_configuration"].initial_covariance,
    )
    assert a["sensors"].root_seed == b["sensors"].root_seed == 30
    assert a["allow_minimum_snap"] is True
    assert "geometric_controller" in b
