"""Independent boundary and dataflow checks for the supported-start experiment."""

from dataclasses import replace

import numpy as np
import pytest

from experiments import supported_start as study
from quadrotor_math import estimated_mission
from quadrotor_math.eskf_endpoint import initialize_eskf_endpoint
from quadrotor_math.imu import accelerometer_specific_force_measurement_body
from quadrotor_math.prearm_alignment import PrearmStatus
from quadrotor_math.randomness import create_run_random_streams
from quadrotor_math.rotations import rotation_matrix_body_to_world


@pytest.mark.parametrize("case", study.CASES)
def test_fixture_balances_gravity_drag_and_preserves_pose(case):
    prepared = study.prepare(case, "smoke")
    original = prepared.original
    a = prepared.evidence
    R = rotation_matrix_body_to_world(original["initial_state"].q_WB)
    relative_B = R.T @ -original["truth_world"].wind_velocity_W
    drag_B = -original["truth_body"].quadratic_drag_coefficient_B * np.abs(relative_B) * relative_B
    gravity_W = np.array([0.0, 0, 9.81])
    balance = a["support_force_W"] + (R @ drag_B) + original["truth_body"].mass * gravity_W
    np.testing.assert_allclose(balance, 0, rtol=0, atol=2e-15)
    np.testing.assert_array_equal(a["velocity_W"], 0)
    np.testing.assert_array_equal(a["omega_B"], 0)
    np.testing.assert_array_equal(a["actual_rotor_omega"], 0)
    np.testing.assert_array_equal(
        a["position_W"], np.tile(original["initial_state"].position_W, (202, 1))
    )
    np.testing.assert_array_equal(a["q_WB"], np.tile(original["initial_state"].q_WB, (202, 1)))
    np.testing.assert_allclose(
        a["ideal_specific_force_B"], np.tile(-R.T @ gravity_W, (202, 1)), rtol=0, atol=2e-15
    )
    assert prepared.status is PrearmStatus.RELEASED and prepared.release is not None
    assert prepared.release.aligned_sample_ids == (0, 200)
    assert prepared.release.fresh_sample.sample_id == 201


def test_supported_arms_differ_only_in_alignment_prior_and_keep_original_navigation():
    prepared = study.prepare("nominal_tracking", "smoke")
    first = study.configuration_for(prepared, "unaligned")
    second = study.configuration_for(prepared, "aligned")
    for key in first:
        if key != "estimator_configuration":
            assert study.plain_configuration(first[key]) == study.plain_configuration(second[key])
    a, b = first["estimator_configuration"], second["estimator_configuration"]
    original = prepared.original["estimator_configuration"]
    for config in (a, b):
        np.testing.assert_array_equal(
            config.initial_covariance[:6, :6], original.initial_covariance[:6, :6]
        )
        np.testing.assert_array_equal(config.initial_covariance[:6, 6:], 0)
        np.testing.assert_array_equal(
            config.initial_state.position_W, original.initial_state.position_W
        )
        np.testing.assert_array_equal(
            config.initial_state.velocity_W, original.initial_state.velocity_W
        )
    endpoint = initialize_eskf_endpoint(b.initial_state, b.initial_covariance, b.sampled_imu_noise)
    np.testing.assert_array_equal(
        endpoint.joint_covariance, prepared.release.endpoint.joint_covariance
    )
    np.testing.assert_array_equal(
        endpoint.imu_noise_mean_B, prepared.release.endpoint.imu_noise_mean_B
    )
    assert np.max(np.abs(endpoint.joint_covariance[6:9, 9:12])) > 1e-5
    expected = original.initial_covariance.copy()
    expected[9:15, 9:15] += original.sampled_imu_noise.bias_walk_spectral_density_B * 0.5025
    np.testing.assert_array_equal(a.initial_covariance, expected)
    np.testing.assert_array_equal(a.initial_state.q_WB, [1, 0, 0, 0])


@pytest.mark.parametrize("fail", [False, True])
def test_boundary_adapter_replaces_only_first_physical_acceleration_and_restores(fail):
    prepared = study.prepare("nominal_hover", "smoke")
    args = study.configuration_for(prepared, "aligned")
    imu = args["sensors"].imu
    rng = create_run_random_streams(args["sensors"].root_seed).accelerometer_measurement_noise
    independent = create_run_random_streams(
        args["sensors"].root_seed
    ).accelerometer_measurement_noise
    original = estimated_mission.accelerometer_specific_force_measurement_body
    try:
        with study.release_measurement(prepared) as trace:
            first = estimated_mission.accelerometer_specific_force_measurement_body(
                np.zeros(3),
                imu.initial_accelerometer_bias_B,
                imu.accelerometer_noise_standard_deviation_B,
                rng,
            )
            np.testing.assert_array_equal(first, prepared.release.fresh_sample.specific_force_B)
            accelerometer_specific_force_measurement_body(
                np.zeros(3),
                imu.initial_accelerometer_bias_B,
                imu.accelerometer_noise_standard_deviation_B,
                independent,
            )
            if fail:
                raise RuntimeError("injected")
            second = estimated_mission.accelerometer_specific_force_measurement_body(
                np.ones(3),
                imu.initial_accelerometer_bias_B,
                imu.accelerometer_noise_standard_deviation_B,
                rng,
            )
            expected = accelerometer_specific_force_measurement_body(
                np.ones(3),
                imu.initial_accelerometer_bias_B,
                imu.accelerometer_noise_standard_deviation_B,
                independent,
            )
            np.testing.assert_array_equal(second, expected)
            assert trace["samples"] == 2 and trace["supported_samples"] == 1
    except RuntimeError:
        assert fail
    assert estimated_mission.accelerometer_specific_force_measurement_body is original


def test_short_flight_begins_with_exact_fresh_sample_and_continuous_zero_motors():
    prepared = study.prepare("nominal_tracking", "smoke")
    result, diagnostic = study.fly(prepared, "aligned")
    a = prepared.evidence
    np.testing.assert_array_equal(
        result.measurements.specific_force_measurements_B[0], a["specific_force_B"][-1]
    )
    np.testing.assert_array_equal(
        result.measurements.angular_velocity_measurements_B[0], a["angular_velocity_B"][-1]
    )
    for field in ("position_W", "velocity_W", "q_WB", "omega_B", "actual_rotor_omega"):
        np.testing.assert_array_equal(getattr(result.mission, field)[0], a[field][-1])
    assert np.all(result.mission.actual_rotor_omega[1] > 0)
    assert np.all(result.mission.actual_rotor_omega[1] < result.mission.commanded_rotor_omega[0])
    assert diagnostic["boundary"]["supported_samples"] == 1
    assert diagnostic["boundary"]["samples"] == len(result.mission.time_s)
    assert (
        study.verify_noise(result, study.configuration_for(prepared, "aligned"), supported=True)[
            "maximum_draw_error"
        ]
        < 1e-12
    )
    study.audit(prepared, "aligned", result, diagnostic)


def test_rejected_acquisition_cannot_fly(monkeypatch):
    original = study.support_evidence
    monkeypatch.setattr(
        study, "support_evidence", lambda *args: replace(original(*args), motors_off=False)
    )
    prepared = study.prepare("nominal_hover", "smoke")
    assert prepared.release is None and prepared.status is PrearmStatus.REJECTED
    with pytest.raises(ValueError, match="release"):
        study.fly(prepared, "aligned")


def test_unknown_case_and_mode_rejected():
    with pytest.raises(ValueError):
        study.prepare("new_seed")
    with pytest.raises(ValueError):
        study.configuration_for(study.prepare("nominal_hover", "smoke"), "oracle")


def test_noise_verifier_rejects_changed_draw_without_relabeling_physics():
    prepared = study.prepare("nominal_tracking", "smoke")
    result, _ = study.fly(prepared, "aligned")
    force = result.measurements.specific_force_measurements_B.copy()
    force[3, 0] += 0.001
    changed = replace(
        result, measurements=replace(result.measurements, specific_force_measurements_B=force)
    )
    with pytest.raises(ValueError, match="random draw"):
        study.verify_noise(changed, study.configuration_for(prepared, "aligned"), supported=True)


def score_fixture():
    from quadrotor_math.missions import MissionPhase

    return dict(
        mission_time_s=np.array([0.0, 5, 11, 15.5]),
        mission_position_W=np.array([[0.2, 0, 0], [0.04, 0, 0], [0.08, 0, 0], [0.01, 0, 0]]),
        mission_reference_position_W=np.zeros((4, 3)),
        mission_velocity_W=np.zeros((4, 3)),
        mission_phase=np.array([0, 0, 0, MissionPhase.COMPLETE]),
        mission_q_WB=np.tile([1.0, 0, 0, 0], (4, 1)),
        estimate_q_WB=np.tile([1.0, 0, 0, 0], (4, 1)),
    )


def test_scoring_keeps_startup_full_hover_endpoints_and_extra_support_time():
    from experiments.supported_start_validation import score

    result = score(score_fixture(), "nominal_hover")
    assert result["tracking_rmse_m"] == pytest.approx(
        np.sqrt((0.2**2 + 0.04**2 + 0.08**2 + 0.01**2) / 4)
    )
    assert result["hover_peak_m"] == 0.08
    assert result["elapsed_including_support_s"] == 16.0025
    assert result["flight_passed"]


@pytest.mark.parametrize("end", [4.0, 10.0])
def test_short_flight_cannot_pass_hover_even_with_small_error(end):
    from experiments.supported_start_validation import score

    a = score_fixture()
    a["mission_time_s"] = np.array([0.0, 1, 2, end])
    a["mission_position_W"][:] = 0
    assert not score(a, "nominal_hover")["flight_conditions"]["full_hover"]


def test_comparison_retains_both_comparators_passing_conditions_and_mass_failure():
    import copy

    from experiments.supported_start_validation import compare, score, summarize

    good = score(score_fixture(), "nominal_hover")
    worse = copy.deepcopy(good)
    worse["hover_peak_m"] = 0.080001
    worse["flight_passed"] = False
    worse["flight_conditions"]["full_hover"] = False
    assert not compare("nominal_hover", good, good, worse)["passed"]
    failed = copy.deepcopy(good)
    failed["flight_conditions"]["complete"] = False
    assert not compare("nominal_tracking", good, good, failed)["passed"]
    assert not summarize([])["supported_initialization_go"]
    rows = []
    for case in study.CASES:
        value = copy.deepcopy(good)
        if case == "mass_tracking":
            value["flight_passed"] = False
        rows.append(
            dict(
                name=case,
                status="released",
                modes={mode: dict(status="ok", metrics=value) for mode in study.MODES},
                comparison=dict(passed=True),
            )
        )
    summary = summarize(rows)
    assert summary["supported_initialization_go"] and not summary["all_four_aligned_flights_pass"]
