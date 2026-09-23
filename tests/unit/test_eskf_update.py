"""Same-epoch ESKF measurement correction and right-local reset evidence."""

from dataclasses import FrozenInstanceError, fields, replace

import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.eskf import (
    EskfMeasurementUpdate,
    EskfNominalState,
    eskf_barometric_altitude_measurement_model,
    eskf_local_position_measurement_model,
    eskf_reset_jacobian,
    inject_eskf_error_state,
    predict_eskf,
    reset_eskf_covariance,
    update_eskf_barometric_altitude,
    update_eskf_linear_measurement,
    update_eskf_local_position,
)


def _state() -> EskfNominalState:
    return EskfNominalState(
        np.array([1.0, -2.0, -3.0]),
        np.array([0.2, -0.1, 0.3]),
        np.array([0.5, -0.5, 0.5, 0.5]),
        np.array([0.1, -0.2, 0.3]),
        np.array([0.01, -0.02, 0.03]),
    )


def _cross(vector: NDArray[np.float64]) -> NDArray[np.float64]:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _exp_matrix(phi: NDArray[np.float64]) -> NDArray[np.float64]:
    """Independent Rodrigues rotation, used only by finite-difference oracles."""
    theta = np.linalg.norm(phi)
    if theta == 0.0:
        return np.eye(3)
    axis_cross = _cross(phi / theta)
    return np.eye(3) + np.sin(theta) * axis_cross + (1.0 - np.cos(theta)) * axis_cross @ axis_cross


def _local_log_near_identity(rotation: NDArray[np.float64]) -> NDArray[np.float64]:
    # Only rotations near identity occur in the central differences below.
    sine_axis = 0.5 * np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )
    sine = np.linalg.norm(sine_axis)
    return sine_axis if sine == 0.0 else np.arcsin(sine) / sine * sine_axis


def test_measurement_models_match_ned_signs_and_nominal_biases() -> None:
    state = _state()
    position, H_position = eskf_local_position_measurement_model(state, np.array([0.4, -0.5, 0.6]))
    altitude, H_altitude = eskf_barometric_altitude_measurement_model(state, 100.0, 1.2)
    np.testing.assert_allclose(position, [1.4, -2.5, -2.4], atol=1e-15, rtol=0.0)
    np.testing.assert_allclose(altitude, [104.2], atol=1e-14, rtol=0.0)
    expected = np.zeros((3, 15))
    expected[:, :3] = np.eye(3)
    np.testing.assert_array_equal(H_position, expected)
    np.testing.assert_array_equal(H_altitude, -expected[2:3])
    assert not np.shares_memory(position, state.position_W)


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_measurement_jacobians_in_all_error_directions(sign: float) -> None:
    state = replace(_state(), q_WB=sign * _state().q_WB)
    bias = np.array([0.3, -0.2, 0.1])
    _, H_position = eskf_local_position_measurement_model(state, bias)
    _, H_altitude = eskf_barometric_altitude_measurement_model(state, 10.0, -0.3)
    finite_difference = np.empty((4, 15))
    step = 1e-5
    for column in range(15):
        delta = np.eye(15)[column] * step
        plus = inject_eskf_error_state(state, delta)
        minus = inject_eskf_error_state(state, -delta)
        # Evaluate the sensor equations directly, without using the model helpers.
        z_plus = np.r_[plus.position_W + bias, 10.0 - plus.position_W[2] - 0.3]
        z_minus = np.r_[minus.position_W + bias, 10.0 - minus.position_W[2] - 0.3]
        finite_difference[:, column] = (z_plus - z_minus) / (2.0 * step)
    np.testing.assert_allclose(
        finite_difference, np.vstack((H_position, H_altitude)), atol=1e-10, rtol=0.0
    )


@pytest.mark.parametrize(
    "phi",
    [
        np.zeros(3),
        np.array([1e-10, -2e-10, 3e-10]),
        np.array([1e-4, 0.0, 0.0]),
        np.array([0.3, -0.4, 0.2]),
        np.array([2.4, 0.5, -0.7]),
    ],
)
def test_reset_jacobian_matches_independent_finite_differences(phi: NDArray[np.float64]) -> None:
    correction = np.arange(15, dtype=np.float64) * 0.01
    correction[6:9] = phi
    actual = eskf_reset_jacobian(correction)
    finite_difference = np.eye(15)
    step = 1e-6
    base_inverse = _exp_matrix(-phi)
    for column in range(3):
        perturbation = step * np.eye(3)[column]
        plus = _local_log_near_identity(base_inverse @ _exp_matrix(phi + perturbation))
        minus = _local_log_near_identity(base_inverse @ _exp_matrix(phi - perturbation))
        finite_difference[6:9, 6 + column] = (plus - minus) / (2 * step)
    np.testing.assert_allclose(actual, finite_difference, atol=5e-10, rtol=0.0)


def test_reset_transforms_every_attitude_cross_covariance() -> None:
    rng = np.random.Generator(np.random.PCG64(4001))
    factor = rng.standard_normal((15, 15))
    covariance = factor @ factor.T + np.eye(15)
    correction = np.zeros(15)
    correction[6:9] = [0.1, -0.2, 0.3]
    Gamma = eskf_reset_jacobian(correction)
    result = reset_eskf_covariance(covariance, correction)
    np.testing.assert_allclose(result, Gamma @ covariance @ Gamma.T, atol=1e-14, rtol=1e-14)
    assert not np.allclose(result[:6, 6:9], covariance[:6, 6:9])
    assert not np.allclose(result[6:9, 9:], covariance[6:9, 9:])
    np.testing.assert_array_equal(result.view(np.uint64), result.T.copy().view(np.uint64))
    assert np.linalg.eigvalsh(result).min() > 0.0
    assert not np.shares_memory(result, covariance)


def test_zero_reset_is_owned_bit_exact_identity() -> None:
    covariance = np.diag(np.geomspace(1e-24, 1e24, 15))
    np.testing.assert_array_equal(eskf_reset_jacobian(np.zeros(15)), np.eye(15))
    result = reset_eskf_covariance(covariance, np.zeros(15))
    assert result.tobytes() == covariance.tobytes()
    assert not np.shares_memory(result, covariance)


def test_scalar_barometric_update_matches_closed_form() -> None:
    state = _state()
    covariance = np.eye(15)
    covariance[2, 2] = 4.0
    result = update_eskf_barometric_altitude(state, covariance, 106.0, 1.0, 100.0, 1.0)
    # Predicted altitude is 104 m: positive residual moves NED z upward (negative).
    np.testing.assert_array_equal(result.innovation, [2.0])
    np.testing.assert_array_equal(result.innovation_covariance, [[5.0]])
    expected_gain = np.zeros((15, 1))
    expected_gain[2, 0] = -0.8
    np.testing.assert_allclose(result.kalman_gain, expected_gain, atol=2e-16, rtol=0.0)
    np.testing.assert_allclose(
        result.nominal_state.position_W, [1.0, -2.0, -4.6], atol=1e-15, rtol=0.0
    )
    expected_covariance = covariance.copy()
    expected_covariance[2, 2] = 0.8
    np.testing.assert_allclose(result.covariance, expected_covariance, atol=2e-16, rtol=0.0)


def test_diagonal_position_update_has_independent_analytic_gains() -> None:
    state = _state()
    covariance = np.diag(np.arange(1, 16, dtype=np.float64))
    noise = np.diag([3.0, 6.0, 9.0])
    bias = np.array([0.2, -0.4, 0.6])
    residual = np.array([4.0, -8.0, 12.0])
    result = update_eskf_local_position(
        state, covariance, state.position_W + bias + residual, noise, bias
    )
    expected_gain = np.zeros((15, 3))
    expected_gain[:3] = 0.25 * np.eye(3)
    np.testing.assert_allclose(result.kalman_gain, expected_gain, atol=1e-16, rtol=0.0)
    np.testing.assert_allclose(
        result.nominal_state.position_W, state.position_W + residual / 4, atol=1e-15, rtol=0.0
    )
    expected_covariance = covariance.copy()
    expected_covariance[:3, :3] *= 0.75
    np.testing.assert_allclose(result.covariance, expected_covariance, atol=1e-15, rtol=0.0)


def test_zero_innovation_preserves_state_bytes_but_updates_covariance() -> None:
    state = replace(_state(), q_WB=_state().q_WB * (1.0 + 1e-13))
    result = update_eskf_local_position(state, np.eye(15), state.position_W, np.eye(3), np.zeros(3))
    for field in fields(state):
        before, after = getattr(state, field.name), getattr(result.nominal_state, field.name)
        assert before.tobytes() == after.tobytes()
        assert not np.shares_memory(before, after)
    np.testing.assert_array_equal(result.error_state_correction, np.zeros(15))
    np.testing.assert_allclose(np.diag(result.covariance)[:3], 0.5)


def test_correlated_update_agrees_with_independent_gaussian_conditioning() -> None:
    rng = np.random.Generator(np.random.PCG64(81))
    factor = rng.normal(size=(15, 15))
    covariance = factor @ factor.T + 0.3 * np.eye(15)
    noise_factor = rng.normal(size=(3, 3))
    noise = noise_factor @ noise_factor.T + 0.5 * np.eye(3)
    state = _state()
    residual = np.array([0.05, -0.03, 0.02])
    result = update_eskf_local_position(
        state, covariance, state.position_W + residual, noise, np.zeros(3)
    )
    S = covariance[:3, :3] + noise
    gain = np.linalg.solve(S, covariance[:3, :]).T
    correction = gain @ residual
    # Conditional Gaussian covariance, independently of the Joseph implementation.
    conditional = covariance - gain @ covariance[:3, :]
    Gamma = eskf_reset_jacobian(correction)
    expected_covariance = Gamma @ conditional @ Gamma.T
    np.testing.assert_allclose(result.kalman_gain, gain, atol=1e-14, rtol=1e-14)
    np.testing.assert_allclose(result.error_state_correction, correction, atol=1e-14, rtol=1e-14)
    np.testing.assert_allclose(result.covariance, expected_covariance, atol=3e-14, rtol=1e-13)
    expected_state = inject_eskf_error_state(state, correction)
    for field in fields(state):
        np.testing.assert_allclose(
            getattr(result.nominal_state, field.name),
            getattr(expected_state, field.name),
            atol=1e-14,
            rtol=1e-14,
        )
    assert np.linalg.norm(correction[6:9]) > 0.001
    assert np.linalg.norm(correction[9:12]) > 0.001
    assert np.linalg.norm(correction[12:15]) > 0.001


def test_update_returns_frozen_independently_owned_arrays() -> None:
    state = _state()
    covariance = np.eye(15)
    measurement = state.position_W + 1.0
    noise = np.eye(3)
    bias = np.zeros(3)
    originals = [array.copy() for array in (covariance, measurement, noise, bias)]
    result = update_eskf_local_position(state, covariance, measurement, noise, bias)
    assert isinstance(result, EskfMeasurementUpdate)
    with pytest.raises(FrozenInstanceError):
        result.covariance = covariance
    for field in fields(result):
        value = getattr(result, field.name)
        if isinstance(value, np.ndarray):
            assert value.flags.owndata and value.flags.c_contiguous and not value.flags.writeable
            assert value.dtype == np.dtype(np.float64)
            for source in (covariance, measurement, noise, bias):
                assert not np.shares_memory(value, source)
    for value, original in zip((covariance, measurement, noise, bias), originals, strict=True):
        np.testing.assert_array_equal(value, original)


def _linear_arguments() -> dict[str, object]:
    H = np.zeros((3, 15))
    H[:, :3] = np.eye(3)
    return dict(
        nominal_state=_state(),
        covariance=np.eye(15),
        measurement=np.ones(3),
        predicted_measurement=np.zeros(3),
        measurement_jacobian=H,
        measurement_noise_covariance=np.eye(3),
    )


@pytest.mark.parametrize(
    ("name", "invalid"),
    [
        ("measurement", np.zeros(0)),
        ("measurement", np.zeros((3, 1))),
        ("measurement", np.array(0.0)),
        ("predicted_measurement", np.zeros(2)),
        ("predicted_measurement", np.zeros((3, 1))),
        ("covariance", np.eye(14)),
        ("measurement_jacobian", np.zeros((15, 3))),
        ("measurement_noise_covariance", np.eye(2)),
    ],
)
def test_update_rejects_incompatible_shapes(name: str, invalid: NDArray[np.float64]) -> None:
    arguments = _linear_arguments()
    arguments[name] = invalid
    with pytest.raises(ValueError, match=name):
        update_eskf_linear_measurement(**arguments)


@pytest.mark.parametrize(
    "name",
    [
        "measurement",
        "predicted_measurement",
        "covariance",
        "measurement_jacobian",
        "measurement_noise_covariance",
    ],
)
@pytest.mark.parametrize("invalid", [np.nan, np.inf, -np.inf])
def test_update_rejects_nonfinite_inputs(name: str, invalid: float) -> None:
    arguments = _linear_arguments()
    arguments[name].flat[0] = invalid
    with pytest.raises(ValueError, match=name):
        update_eskf_linear_measurement(**arguments)


def test_update_validates_all_shapes_before_value_domains() -> None:
    arguments = _linear_arguments()
    arguments["measurement"][:] = np.nan
    arguments["measurement_noise_covariance"] = np.eye(2)
    with pytest.raises(ValueError, match="measurement_noise_covariance must have shape"):
        update_eskf_linear_measurement(**arguments)


@pytest.mark.parametrize("name", ["covariance", "measurement_noise_covariance"])
@pytest.mark.parametrize(
    "kind", ["asymmetric", "negative_diagonal", "indefinite", "zero_row", "subnormal_negative"]
)
def test_update_rejects_invalid_covariances_at_local_scale(name: str, kind: str) -> None:
    arguments = _linear_arguments()
    values = arguments[name]
    values[-1, -1] = 1e24
    if kind == "asymmetric":
        values[0, 1], values[1, 0] = 1e-12, 2e-12
    elif kind == "negative_diagonal":
        values[0, 0] = -1e-20
    elif kind == "indefinite":
        values[:2, :2] = [[1e-20, 2e-20], [2e-20, 1e-20]]
    elif kind == "zero_row":
        values[0, 0] = 0.0
        values[0, 1] = values[1, 0] = np.nextafter(0.0, 1.0)
    else:
        values[0, 0] = -np.nextafter(0.0, 1.0)
    with pytest.raises(ValueError, match=name):
        update_eskf_linear_measurement(**arguments)


@pytest.mark.parametrize("kind", ["zero", "duplicate_exact_observations"])
def test_update_rejects_singular_innovation_covariance(kind: str) -> None:
    arguments = _linear_arguments()
    arguments["measurement_noise_covariance"][:] = 0.0
    if kind == "zero":
        arguments["covariance"][:] = 0.0
    else:
        arguments["measurement_jacobian"][1] = arguments["measurement_jacobian"][0]
    with pytest.raises(
        ValueError, match="innovation_covariance must be numerically positive definite"
    ):
        update_eskf_linear_measurement(**arguments)


def test_zero_measurement_noise_collapses_observed_covariance() -> None:
    state = _state()
    result = update_eskf_local_position(
        state, np.eye(15), np.array([5.0, 6.0, 7.0]), np.zeros((3, 3)), np.zeros(3)
    )
    np.testing.assert_array_equal(result.nominal_state.position_W, [5.0, 6.0, 7.0])
    expected = np.eye(15)
    expected[:3, :3] = 0.0
    np.testing.assert_array_equal(result.covariance, expected)


@pytest.mark.parametrize("subnormal", [False, True])
@pytest.mark.filterwarnings("error")
def test_uninformative_update_preserves_covariance_bits(subnormal: bool) -> None:
    arguments = _linear_arguments()
    covariance = np.eye(15) * (np.nextafter(0.0, 1.0) if subnormal else 0.3)
    arguments["covariance"] = covariance
    arguments["measurement_jacobian"][:] = 0.0
    arguments["measurement_noise_covariance"] = np.eye(3) * np.nextafter(0.0, 1.0)
    result = update_eskf_linear_measurement(**arguments)
    assert result.covariance.tobytes() == covariance.tobytes()
    np.testing.assert_array_equal(result.error_state_correction, np.zeros(15))


@pytest.mark.filterwarnings("error")
def test_scaled_gain_handles_extreme_diagonal_measurement_units() -> None:
    diagonal = np.array([1e-300, 1.0, 1e300])
    covariance = np.eye(15)
    covariance[:3, :3] = np.diag(diagonal)
    result = update_eskf_local_position(
        _state(), covariance, _state().position_W, np.diag(diagonal), np.zeros(3)
    )
    # Scaling, two solves, and unscaling introduce a few float64 roundoff units.
    roundoff = 8.0 * np.finfo(np.float64).eps
    np.testing.assert_allclose(result.kalman_gain[:3], 0.5 * np.eye(3), atol=roundoff, rtol=0.0)
    np.testing.assert_allclose(
        np.diag(result.covariance)[:3] / diagonal, 0.5, atol=roundoff, rtol=0.0
    )


@pytest.mark.parametrize(
    "kind", ["innovation", "innovation_covariance", "jacobian_product", "correction"]
)
@pytest.mark.filterwarnings("error")
def test_update_rejects_finite_arithmetic_overflow_atomically(kind: str) -> None:
    arguments = _linear_arguments()
    huge = np.finfo(float).max
    if kind == "innovation":
        arguments["measurement"][:] = huge
        arguments["predicted_measurement"][:] = -huge
    elif kind == "innovation_covariance":
        arguments["covariance"][:] = np.eye(15) * huge
        arguments["measurement_noise_covariance"][:] = np.eye(3) * huge
    elif kind == "jacobian_product":
        arguments["measurement_jacobian"][:] = huge
    else:
        arguments["measurement_jacobian"] *= 1e-160
        arguments["measurement_noise_covariance"][:] = 0.0
        arguments["measurement"][:] = 1e200
    before = {
        name: value.copy() for name, value in arguments.items() if isinstance(value, np.ndarray)
    }
    with pytest.raises(ValueError, match="finite"):
        update_eskf_linear_measurement(**arguments)
    for name, original in before.items():
        assert arguments[name].tobytes() == original.tobytes()


@pytest.mark.parametrize("invalid", [np.zeros(14), np.full(15, np.nan), np.full(15, np.inf)])
def test_reset_rejects_invalid_correction(invalid: NDArray[np.float64]) -> None:
    with pytest.raises(ValueError, match="error_state_correction"):
        eskf_reset_jacobian(invalid)


@pytest.mark.filterwarnings("error")
def test_reset_rejects_rotation_norm_overflow() -> None:
    correction = np.zeros(15)
    correction[6:9] = np.finfo(float).max
    with pytest.raises(ValueError, match="finite"):
        eskf_reset_jacobian(correction)


@pytest.mark.parametrize("invalid", [np.zeros(2), np.array([np.nan, 0.0, 0.0])])
def test_position_model_rejects_invalid_bias(invalid: NDArray[np.float64]) -> None:
    with pytest.raises(ValueError, match="position_bias_W"):
        eskf_local_position_measurement_model(_state(), invalid)


@pytest.mark.parametrize("name", ["reference_altitude", "barometric_altitude_bias"])
@pytest.mark.parametrize("invalid", [np.nan, np.inf, -np.inf])
def test_altitude_model_rejects_invalid_scalars(name: str, invalid: float) -> None:
    arguments = dict(reference_altitude=0.0, barometric_altitude_bias=0.0)
    arguments[name] = invalid
    with pytest.raises(ValueError, match=name):
        eskf_barometric_altitude_measurement_model(_state(), **arguments)


@pytest.mark.parametrize("model", ["position", "altitude"])
@pytest.mark.filterwarnings("error")
def test_measurement_models_reject_finite_overflow(model: str) -> None:
    state = replace(_state(), position_W=np.full(3, -np.finfo(float).max))
    with pytest.raises(ValueError, match="finite"):
        if model == "position":
            eskf_local_position_measurement_model(state, state.position_W)
        else:
            eskf_barometric_altitude_measurement_model(state, np.finfo(float).max, 0.0)


@pytest.mark.parametrize("variance", [-1.0, np.nan, np.inf])
def test_altitude_wrapper_rejects_invalid_variance(variance: float) -> None:
    with pytest.raises(ValueError, match="measurement_noise_covariance"):
        update_eskf_barometric_altitude(_state(), np.eye(15), 10.0, variance, 0.0, 0.0)


def test_quaternion_sign_equivalence_through_nonzero_update() -> None:
    state = _state()
    factor = np.eye(15)
    factor[6:9, :3] = 0.2 * np.eye(3)
    covariance = factor @ factor.T
    positive = update_eskf_local_position(
        state, covariance, state.position_W + 0.5, np.eye(3), np.zeros(3)
    )
    negative = update_eskf_local_position(
        replace(state, q_WB=-state.q_WB), covariance, state.position_W + 0.5, np.eye(3), np.zeros(3)
    )
    np.testing.assert_array_equal(positive.covariance, negative.covariance)
    np.testing.assert_array_equal(positive.nominal_state.q_WB, -negative.nominal_state.q_WB)


@pytest.mark.parametrize(
    "name",
    ["covariance", "innovation", "innovation_covariance", "kalman_gain", "error_state_correction"],
)
def test_result_constructor_owns_and_validates_arrays(name: str) -> None:
    good = update_eskf_local_position(_state(), np.eye(15), np.zeros(3), np.eye(3), np.zeros(3))
    original = getattr(good, name)
    value = original.copy()
    copied = replace(good, **{name: value})
    value.flat[0] += 1.0
    np.testing.assert_array_equal(getattr(copied, name), original)
    with pytest.raises(ValueError, match=name):
        replace(good, **{name: np.full(original.shape, np.nan)})
    with pytest.raises(ValueError, match=name):
        replace(good, **{name: np.zeros((2, 2, 2))})


@pytest.mark.parametrize("moving", [False, True])
def test_prediction_update_composition_corrects_vertical_bias_drift(moving: bool) -> None:
    """Isolate observable vertical bias on a known stationary/constant-velocity path."""
    dt = 0.01
    velocity = np.array([1.0, -0.5, 0.2]) if moving else np.zeros(3)
    state = EskfNominalState(
        np.array([0.0, 0.0, 0.5]),
        velocity + [0.0, 0.0, 0.1],
        np.array([1.0, 0.0, 0.0, 0.0]),
        np.zeros(3),
        np.zeros(3),
    )
    dead_reckoning = state
    covariance = np.eye(15) * 0.1
    continuous_noise = np.eye(12) * 1e-8
    measured_force = np.array([0.0, 0.0, -9.81 + 0.15])
    corrected_errors, dead_errors = [], []
    for step in range(1, 801):
        truth_position = velocity * (step * dt)
        state, covariance = predict_eskf(
            state, covariance, measured_force, np.zeros(3), 9.81, continuous_noise, dt
        )
        dead_reckoning, _ = predict_eskf(
            dead_reckoning,
            np.zeros((15, 15)),
            measured_force,
            np.zeros(3),
            9.81,
            np.zeros((12, 12)),
            dt,
        )
        if step % 10 == 0:
            result = update_eskf_local_position(
                state, covariance, truth_position, np.eye(3) * 1e-4, np.zeros(3)
            )
            state, covariance = result.nominal_state, result.covariance
        if step % 20 == 0:
            result = update_eskf_barometric_altitude(
                state, covariance, 100.0 - truth_position[2], 2e-4, 100.0, 0.0
            )
            state, covariance = result.nominal_state, result.covariance
        corrected_errors.append(np.linalg.norm(state.position_W - truth_position))
        dead_errors.append(np.linalg.norm(dead_reckoning.position_W - truth_position))
        assert np.all(np.isfinite(covariance))
        np.testing.assert_array_equal(
            covariance.view(np.uint64), covariance.T.copy().view(np.uint64)
        )
        assert np.linalg.eigvalsh(covariance).min() >= -1e-12
        assert abs(np.linalg.norm(state.q_WB) - 1.0) < 1e-12
    # The dead-reckoning vertical error follows e_p(0)+e_v(0)t+0.5*b_a*t².
    np.testing.assert_allclose(
        dead_errors[-1], 0.5 + 0.1 * 8.0 + 0.5 * 0.15 * 8.0**2, atol=2e-12, rtol=0.0
    )
    assert corrected_errors[-1] < 0.005
    assert abs(state.accelerometer_bias_B[2] - 0.15) < 0.001
    assert np.sqrt(np.mean(np.square(corrected_errors))) < 0.1 * np.sqrt(
        np.mean(np.square(dead_errors))
    )


@pytest.mark.parametrize("seed", range(12))
@pytest.mark.parametrize("measurement_size", [1, 3])
def test_seeded_updates_match_independent_conditioning_and_rotation(
    seed: int, measurement_size: int
) -> None:
    rng = np.random.Generator(np.random.PCG64(seed))
    factor = rng.standard_normal((15, 15))
    covariance = factor @ factor.T + np.eye(15)
    noise_factor = rng.standard_normal((measurement_size, measurement_size))
    noise = noise_factor @ noise_factor.T + np.eye(measurement_size)
    H = np.zeros((measurement_size, 15))
    H[:, :measurement_size] = np.eye(measurement_size)
    residual = rng.standard_normal(measurement_size) * 0.01
    state = _state()
    result = update_eskf_linear_measurement(
        state, covariance, residual, np.zeros(measurement_size), H, noise
    )
    S = H @ covariance @ H.T + noise
    gain = np.linalg.solve(S, H @ covariance).T
    correction = gain @ residual
    conditional_covariance = covariance - gain @ H @ covariance
    Gamma = eskf_reset_jacobian(correction)
    np.testing.assert_allclose(
        result.covariance, Gamma @ conditional_covariance @ Gamma.T, atol=5e-14, rtol=1e-12
    )
    np.testing.assert_allclose(result.kalman_gain, gain, atol=1e-14, rtol=1e-14)

    # Independent quaternion-to-matrix expression; verify right multiplication.
    def matrix(q_WB: NDArray[np.float64]) -> NDArray[np.float64]:
        scalar, vector = q_WB[0], q_WB[1:]
        return (
            (scalar**2 - vector @ vector) * np.eye(3)
            + 2 * np.outer(vector, vector)
            + 2 * scalar * _cross(vector)
        )

    np.testing.assert_allclose(
        matrix(result.nominal_state.q_WB),
        matrix(state.q_WB) @ _exp_matrix(correction[6:9]),
        atol=8e-16,
        rtol=0.0,
    )
    assert np.linalg.eigvalsh(result.covariance).min() > 0.0


def test_independent_same_epoch_position_and_altitude_updates_match_joint_solution() -> None:
    state = _state()
    factor = np.eye(15)
    factor[3:6, :3] = 0.4 * np.eye(3)
    covariance = factor @ factor.T
    position = state.position_W + [0.1, -0.2, 0.3]
    altitude = 104.0
    position_result = update_eskf_local_position(
        state, covariance, position, np.eye(3) * 0.2, np.zeros(3)
    )
    sequential = update_eskf_barometric_altitude(
        position_result.nominal_state, position_result.covariance, altitude, 0.4, 100.0, 0.0
    )
    H = np.zeros((4, 15))
    H[:3, :3] = np.eye(3)
    H[3, 2] = -1.0
    joint = update_eskf_linear_measurement(
        state,
        covariance,
        np.r_[position, altitude],
        np.r_[state.position_W, 100.0 - state.position_W[2]],
        H,
        np.diag([0.2, 0.2, 0.2, 0.4]),
    )
    for field in fields(state):
        np.testing.assert_allclose(
            getattr(sequential.nominal_state, field.name),
            getattr(joint.nominal_state, field.name),
            atol=1e-15,
            rtol=0.0,
        )
    np.testing.assert_allclose(sequential.covariance, joint.covariance, atol=3e-16, rtol=0.0)


def test_rotating_constant_world_acceleration_composition() -> None:
    acceleration_W = np.array([0.2, -0.1, 0.05])
    velocity_W = np.array([1.0, 0.2, -0.3])
    accelerometer_bias_B = np.array([0.01, -0.02, 0.03])
    gyroscope_bias_B = np.array([0.001, -0.002, 0.003])
    omega_B = np.array([0.0, 0.0, 0.2])
    state = EskfNominalState(
        np.zeros(3),
        velocity_W,
        np.array([1.0, 0.0, 0.0, 0.0]),
        accelerometer_bias_B,
        gyroscope_bias_B,
    )
    covariance = np.eye(15) * 0.01
    dt = 0.005
    for index in range(1, 401):
        previous_time = (index - 1) * dt
        time = index * dt
        R_WB = _exp_matrix(omega_B * previous_time)
        measured_force = R_WB.T @ (acceleration_W - [0.0, 0.0, 9.81]) + accelerometer_bias_B
        state, covariance = predict_eskf(
            state,
            covariance,
            measured_force,
            omega_B + gyroscope_bias_B,
            9.81,
            np.eye(12) * 1e-8,
            dt,
        )
        truth_position_W = velocity_W * time + 0.5 * acceleration_W * time**2
        if index % 20 == 0:
            result = update_eskf_local_position(
                state, covariance, truth_position_W, np.eye(3) * 0.01, np.zeros(3)
            )
            state, covariance = result.nominal_state, result.covariance
        np.testing.assert_allclose(state.position_W, truth_position_W, atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(
            state.velocity_W, velocity_W + acceleration_W * time, atol=1e-12, rtol=0.0
        )
        np.testing.assert_allclose(
            state.q_WB, [np.cos(0.1 * time), 0.0, 0.0, np.sin(0.1 * time)], atol=1e-12, rtol=0.0
        )
