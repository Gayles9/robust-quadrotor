"""Independent geometric, covariance and truth-alignment evaluation oracles."""

from dataclasses import replace
from math import pi

import numpy as np
import pytest

from quadrotor_math.eskf import EskfNominalState, inject_eskf_error_state
from quadrotor_math.eskf_consistency import (
    EskfConsistencyHistory,
    EskfReferenceHistory,
    eskf_reference_from_run_artifact,
    eskf_right_local_error,
    evaluate_eskf_replay,
    normalized_estimation_error_squared,
    sampled_imu_continuous_noise_covariance,
)
from quadrotor_math.eskf_replay import EskfReplayResult
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.run_configuration import ImuParameters


def _state():
    return EskfNominalState(
        np.array([1.0, -2, 3]),
        np.array([0.5, -0.2, 0.1]),
        np.array([0.5, 0.5, 0.5, 0.5]),
        np.zeros(3),
        np.zeros(3),
    )


@pytest.mark.parametrize("seed", range(12))
def test_right_error_inverts_injection_and_matches_matrix_log(seed):
    nominal = _state()
    expected = np.random.default_rng(seed).normal(size=15) * 0.1
    reference = inject_eskf_error_state(nominal, expected)
    error = eskf_right_local_error(nominal, reference)
    np.testing.assert_allclose(error, expected, atol=3e-16, rtol=2e-14)
    relative = rotation_matrix_body_to_world(nominal.q_WB).T @ rotation_matrix_body_to_world(
        reference.q_WB
    )
    angle = np.arccos((np.trace(relative) - 1) / 2)
    axis_sine = (
        np.array(
            [
                relative[2, 1] - relative[1, 2],
                relative[0, 2] - relative[2, 0],
                relative[1, 0] - relative[0, 1],
            ]
        )
        / 2
    )
    np.testing.assert_allclose(error[6:9], axis_sine * angle / np.sin(angle), atol=5e-16)
    assert error.flags.owndata and not error.flags.writeable


@pytest.mark.parametrize("axis", range(3))
@pytest.mark.parametrize("angle", [0.0, 1e-300, 1e-12, -0.3, pi - 1e-9, pi + 1e-9])
@pytest.mark.parametrize("signs", [(1, 1), (-1, 1), (1, -1), (-1, -1)])
def test_principal_rotation_axes_small_angles_and_quaternion_signs(axis, angle, signs):
    nominal = replace(_state(), q_WB=np.array([float(signs[0]), 0, 0, 0]))
    reference_q_WB = np.zeros(4)
    reference_q_WB[0] = np.cos(angle / 2)
    reference_q_WB[axis + 1] = np.sin(angle / 2)
    reference = replace(nominal, q_WB=signs[1] * reference_q_WB)
    expected = np.zeros(3)
    expected[axis] = angle if angle <= pi else angle - 2 * pi
    np.testing.assert_allclose(
        eskf_right_local_error(nominal, reference)[6:9], expected, rtol=2e-15, atol=0
    )


@pytest.mark.parametrize("axis", range(3))
def test_exact_pi_has_deterministic_sign_representative(axis):
    nominal = replace(_state(), q_WB=np.array([1.0, 0, 0, 0]))
    q_WB = np.zeros(4)
    q_WB[axis + 1] = -1
    expected = np.zeros(3)
    expected[axis] = pi
    for sign in (-1, 1):
        error = eskf_right_local_error(nominal, replace(nominal, q_WB=sign * q_WB))
        np.testing.assert_array_equal(error[6:9], expected)


def test_error_is_right_local_not_world_local():
    nominal = replace(_state(), q_WB=np.array([np.sqrt(0.5), 0, 0, np.sqrt(0.5)]))
    correction = np.zeros(15)
    correction[6] = 0.1
    reference = inject_eskf_error_state(nominal, correction)
    np.testing.assert_allclose(
        eskf_right_local_error(nominal, reference)[6:9], [0.1, 0, 0], atol=1e-16
    )


def test_error_revalidates_inputs_and_rejects_arithmetic_overflow():
    with pytest.raises(TypeError):
        eskf_right_local_error(None, _state())
    huge = replace(_state(), position_W=np.full(3, 1e308))
    with pytest.raises(ValueError, match="finite"):
        eskf_right_local_error(huge, replace(huge, position_W=-huge.position_W))
    mutated = _state()
    mutated.q_WB.flags.writeable = True
    mutated.q_WB[:] = 0
    with pytest.raises(ValueError):
        eskf_right_local_error(mutated, _state())


@pytest.mark.parametrize("seed", range(8))
def test_nees_matches_independent_solve_and_change_of_units(seed):
    rng = np.random.default_rng(seed)
    matrix = rng.normal(size=(15, 15))
    covariance = matrix @ matrix.T + np.eye(15)
    error = rng.normal(size=15)
    expected = error @ np.linalg.solve(covariance, error)
    assert normalized_estimation_error_squared(error, covariance) == pytest.approx(expected)
    scales = np.geomspace(1e-90, 1e90, 15)
    assert normalized_estimation_error_squared(
        error * scales, covariance * np.outer(scales, scales)
    ) == pytest.approx(expected)


@pytest.mark.parametrize("variance", [1e-300, 1.0, 1e300])
def test_nees_diagonal_and_zero_error(variance):
    covariance = np.eye(15) * variance
    assert normalized_estimation_error_squared(
        np.ones(15) * np.sqrt(variance), covariance
    ) == pytest.approx(15)
    assert normalized_estimation_error_squared(np.zeros(15), covariance) == 0


@pytest.mark.parametrize(
    "bad",
    [
        np.zeros((15, 15)),
        np.diag([0.0] + [1.0] * 14),
        -np.eye(15),
        np.ones((15, 15)),
        np.eye(14),
        np.full((15, 15), np.nan),
        np.eye(15, dtype=complex),
        np.eye(15, dtype=bool),
        np.triu(np.ones((15, 15))),
    ],
)
def test_nees_requires_finite_real_symmetric_positive_definite_covariance(bad):
    with pytest.raises(ValueError):
        normalized_estimation_error_squared(np.zeros(15), bad)


@pytest.mark.parametrize(
    "bad",
    [
        np.zeros(14),
        np.zeros((15, 1)),
        [0.0] * 15,
        np.full(15, np.inf),
        np.ones(15, dtype=complex),
        np.ones(15, dtype=bool),
    ],
)
def test_nees_rejects_invalid_error(bad):
    with pytest.raises(ValueError):
        normalized_estimation_error_squared(bad, np.eye(15))


def test_nees_rejects_unrepresentable_statistic():
    with pytest.raises(ValueError, match="NEES"):
        normalized_estimation_error_squared(np.full(15, 1e308), np.eye(15) * 1e-300)


def test_seeded_gaussian_nees_has_expected_first_two_moments():
    errors = np.random.default_rng(341).normal(size=(1500, 15))
    scores = np.array([normalized_estimation_error_squared(e, np.eye(15)) for e in errors])
    assert abs(scores.mean() - 15) < 0.4
    assert abs(scores.var() - 30) < 4


def test_history_exact_alignment_post_update_covariance_and_ownership():
    states = (_state(), _state())
    times = np.array([0.01, 0.02])
    reference = EskfReferenceHistory(times, states)
    error = np.zeros(15)
    error[0] = 2
    result = EskfReplayResult(
        times,
        tuple(inject_eskf_error_state(s, -error) for s in states),
        np.stack([np.eye(15), np.eye(15) * 2]),
        (),
    )
    evaluated = evaluate_eskf_replay(result, reference)
    np.testing.assert_allclose(evaluated.nees, [4, 2], rtol=3e-16)
    np.testing.assert_array_equal(evaluated.error_states, np.stack([error, error]))
    for array in (
        reference.time_s,
        reference.states[0].q_WB,
        evaluated.time_s,
        evaluated.error_states,
        evaluated.nees,
    ):
        assert array.flags.owndata and not array.flags.writeable
    assert not np.shares_memory(reference.states[0].q_WB, states[0].q_WB)
    with pytest.raises(ValueError, match="exact"):
        evaluate_eskf_replay(result, replace(reference, time_s=times + 1e-15))
    with pytest.raises(ValueError, match="exact"):
        evaluate_eskf_replay(result, EskfReferenceHistory(times[:1], states[:1]))


@pytest.mark.parametrize(
    "times",
    [
        np.array([]),
        np.array([-1.0]),
        np.array([1.0, 1.0]),
        np.array([np.nan, 1.0]),
        np.array([2.0, 1.0]),
    ],
)
def test_reference_rejects_invalid_epochs(times):
    with pytest.raises(ValueError):
        EskfReferenceHistory(times, tuple(_state() for _ in times))


def test_histories_validate_dimensions_and_nonnegative_nees():
    with pytest.raises(ValueError):
        EskfReferenceHistory(np.array([0.1]), ())
    with pytest.raises(ValueError):
        EskfReferenceHistory(np.array([0.1]), (None,))
    with pytest.raises(ValueError):
        EskfConsistencyHistory(np.array([0.1]), np.zeros((1, 15)), np.array([-1.0]))
    with pytest.raises(ValueError):
        EskfConsistencyHistory(np.array([0.1]), np.zeros((1, 14)), np.array([0.0]))


class _TruthOnly:
    def __init__(self, **overrides):
        self.values = {
            "truth_time_s": np.arange(4) * 0.01,
            "truth_position_history_W": np.arange(12).reshape(4, 3).astype(float),
            "truth_velocity_history_W": np.zeros((4, 3)),
            "truth_q_history_WB": np.tile([1.0, 0, 0, 0], (4, 1)),
            "accelerometer_bias_history_B": np.arange(12).reshape(4, 3).astype(float) * 0.01,
            "gyroscope_bias_history_B": np.arange(12).reshape(4, 3).astype(float) * 0.001,
        } | overrides

    def __getattr__(self, name):
        if name not in self.values:
            raise AssertionError(f"Evaluator read unexpected field {name}")
        return self.values[name]


def test_truth_adapter_aligns_completed_rows_and_biases_without_reading_sensors():
    source = _TruthOnly()
    reference = eskf_reference_from_run_artifact(source)
    np.testing.assert_array_equal(reference.time_s, [0.01, 0.02, 0.03])
    for k, state in enumerate(reference.states, 1):
        np.testing.assert_array_equal(state.position_W, source.truth_position_history_W[k])
        np.testing.assert_array_equal(
            state.accelerometer_bias_B, source.accelerometer_bias_history_B[k]
        )
        np.testing.assert_array_equal(state.gyroscope_bias_B, source.gyroscope_bias_history_B[k])


@pytest.mark.parametrize("name", list(_TruthOnly().values))
def test_truth_adapter_revalidates_all_consumed_arrays(name):
    source = _TruthOnly()
    changed = source.values[name].copy()
    changed.flat[0] = np.nan
    with pytest.raises(ValueError):
        eskf_reference_from_run_artifact(_TruthOnly(**{name: changed}))


@pytest.mark.parametrize(
    "clock",
    [
        np.array([0.0]),
        np.array([0.1, 0.2, 0.3, 0.4]),
        np.array([0, 0.01, 0.021, 0.03]),
        np.array([0, 1e-16, 2e-16, 3e-16]),
    ],
)
def test_truth_adapter_rejects_incompatible_clock(clock):
    with pytest.raises(ValueError):
        eskf_reference_from_run_artifact(_TruthOnly(truth_time_s=clock))


def _imu():
    return ImuParameters(
        np.zeros(3),
        np.array([1.0, 2, 3]),
        np.array([0.1, 0.2, 0.3]),
        np.zeros(3),
        np.array([0.01, 0.02, 0.03]),
        np.array([0.001, 0.002, 0.003]),
    )


@pytest.mark.parametrize("dt", [0.001, 0.01, 0.2])
def test_explicit_sample_noise_conversion_matches_increment_variances(dt):
    imu = _imu()
    Qc = sampled_imu_continuous_noise_covariance(imu, dt)
    expected = np.concatenate(
        [
            imu.accelerometer_noise_standard_deviation_B**2 * dt,
            imu.gyroscope_noise_standard_deviation_B**2 * dt,
            imu.accelerometer_bias_random_walk_density_B**2,
            imu.gyroscope_bias_random_walk_density_B**2,
        ]
    )
    np.testing.assert_allclose(Qc, np.diag(expected), rtol=5e-16)
    np.testing.assert_allclose(
        np.diag(Qc)[:6] * dt,
        np.concatenate(
            [imu.accelerometer_noise_standard_deviation_B, imu.gyroscope_noise_standard_deviation_B]
        )
        ** 2
        * dt**2,
    )
    assert Qc.flags.owndata and not Qc.flags.writeable


@pytest.mark.parametrize("dt", [0, -1, np.nan, np.inf, True, "0.01"])
def test_noise_conversion_rejects_invalid_dt(dt):
    with pytest.raises(ValueError):
        sampled_imu_continuous_noise_covariance(_imu(), dt)


@pytest.mark.parametrize("value", [1e300, 1e-300])
@pytest.mark.parametrize(
    "field",
    [
        "accelerometer_noise_standard_deviation_B",
        "gyroscope_noise_standard_deviation_B",
        "accelerometer_bias_random_walk_density_B",
        "gyroscope_bias_random_walk_density_B",
    ],
)
def test_noise_conversion_rejects_unrepresentable_positive_variance(value, field):
    with pytest.raises(ValueError):
        sampled_imu_continuous_noise_covariance(replace(_imu(), **{field: np.full(3, value)}), 0.01)
