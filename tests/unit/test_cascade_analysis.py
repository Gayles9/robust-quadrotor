"""Local continuous and multirate hover models, independently checked."""

from dataclasses import replace

import numpy as np
import pytest

from quadrotor_math.cascade_analysis import HorizontalCascade, hover_axis_zero_order_hold


def model():
    return HorizontalCascade(1.0, 1.8, 6.0, 24.0, 9.81, 0.025, 0.01, 2)


def test_characteristic_polynomial_and_poles():
    m = model()
    np.testing.assert_allclose(m.characteristic_polynomial(), [0.025, 1, 24, 144, 259.2, 144])
    np.testing.assert_allclose(
        np.poly(m.continuous_matrix()), m.characteristic_polynomial() / 0.025
    )
    poles = np.linalg.eigvals(m.continuous_matrix())
    assert np.max(poles.real) < -1.2
    slow = poles[abs(poles) < 5]
    assert np.min(-slow.real / abs(slow)) > 0.99
    legacy = replace(m, attitude_gain=3.0, rate_gain=12.0)
    old = np.linalg.eigvals(legacy.continuous_matrix())
    assert 0.57 < np.min(-old.real / abs(old)) < 0.58


@pytest.mark.parametrize("step", [0.01, 0.0025, 1e-8])
def test_exact_hold_against_analytic_constant_command_and_motor_decay(step):
    phi, gamma = hover_axis_zero_order_hold(9.81, 0.025, step)
    # Equilibrium angular acceleration a=u gives an exact polynomial trajectory.
    initial = np.array([0.1, 0.2, 0.3, 0.4, 0.5])
    p, v, eta, r, a = initial
    expected = [
        p + v * step + 9.81 * (eta * step**2 / 2 + r * step**3 / 6 + a * step**4 / 24),
        v + 9.81 * (eta * step + r * step**2 / 2 + a * step**3 / 6),
        eta + r * step + a * step**2 / 2,
        r + a * step,
        a,
    ]
    np.testing.assert_allclose(phi @ initial + gamma * a, expected, atol=1e-15)
    assert phi[4, 4] == pytest.approx(np.exp(-step / 0.025), abs=1e-15)
    assert gamma[4] == pytest.approx(-np.expm1(-step / 0.025), abs=1e-15)
    # Composition for the same held command is the semigroup property.
    half, half_g = hover_axis_zero_order_hold(9.81, 0.025, step / 2)
    np.testing.assert_allclose(phi, half @ half, atol=1e-15)
    np.testing.assert_allclose(gamma, half @ half_g + half_g, atol=1e-15)


def test_lifted_map_matches_explicit_multirate_execution():
    m = model()
    initial = np.array([0.02, -0.03, 0.01, -0.05, 0.1])
    phi, gamma = hover_axis_zero_order_hold(m.gravity, m.motor_time_constant_s, m.inner_period_s)
    expected = initial.copy()
    desired = -(m.position_gain * initial[0] + m.velocity_gain * initial[1]) / m.gravity
    for _ in range(m.outer_stride):
        command = m.rate_gain * (m.attitude_gain * (desired - expected[2]) - expected[3])
        expected = phi @ expected + gamma * command
    np.testing.assert_allclose(m.lifted_transition() @ initial, expected, atol=1e-15)
    assert max(abs(np.linalg.eigvals(m.lifted_transition()))) < 1
    # The outer reference really is held, not secretly refreshed at inner ticks.
    refreshed = replace(m, outer_stride=1).lifted_transition()
    assert not np.allclose(m.lifted_transition(), refreshed @ refreshed)


def test_sampled_limit_matches_continuous_generator():
    m = replace(model(), inner_period_s=1e-7)
    np.testing.assert_allclose(
        (m.lifted_transition() - np.eye(5)) / (2e-7), m.continuous_matrix(), atol=0.02, rtol=0.0002
    )


def test_arrays_owned_readonly_and_gravity_similarity():
    m = model()
    values = [
        m.characteristic_polynomial(),
        m.continuous_matrix(),
        m.lifted_transition(),
        *hover_axis_zero_order_hold(9.81, 0.025, 0.01),
    ]
    for a in values:
        assert a.dtype == np.float64 and not a.flags.writeable
        assert a.flags.owndata and np.all(np.isfinite(a))
    np.testing.assert_allclose(
        np.sort_complex(np.linalg.eigvals(m.lifted_transition())),
        np.sort_complex(np.linalg.eigvals(replace(m, gravity=1).lifted_transition())),
    )


@pytest.mark.parametrize(
    "field",
    [
        "position_gain",
        "velocity_gain",
        "attitude_gain",
        "rate_gain",
        "gravity",
        "motor_time_constant_s",
        "inner_period_s",
    ],
)
@pytest.mark.parametrize("bad", [True, 0.0, -1.0, float("nan"), float("inf"), "1"])
def test_invalid_scalars(field, bad):
    with pytest.raises(ValueError):
        replace(model(), **{field: bad})


@pytest.mark.parametrize("bad", [True, 0, -1, 1.5, 10001])
def test_invalid_stride(bad):
    with pytest.raises(ValueError):
        replace(model(), outer_stride=bad)


def test_boundaries_and_arithmetic_failure():
    with pytest.raises(ValueError, match="period"):
        replace(model(), inner_period_s=0.026)
    with pytest.raises(ValueError):
        hover_axis_zero_order_hold(9.81, 0.025, 0)
    with pytest.raises(ValueError):
        hover_axis_zero_order_hold(9.81, 0.025, 0.03)
    with pytest.raises(ValueError):
        replace(model(), attitude_gain=1e300, rate_gain=1e300).continuous_matrix()


@pytest.mark.parametrize("axis,body_axis,sign", [(0, 1, -1), (1, 0, 1)])
@pytest.mark.parametrize(
    "kp,kv,ka,kr",
    [
        (1.0, 1.8, 3.0, 12.0),
        (1.0, 1.8, 6.0, 24.0),
        (3528 / 1003, 3192 / 1003, 6018 / 773, 773 / 40),
    ],
)
def test_lifted_model_matches_nonlinear_plant_jacobian(axis, body_axis, sign, kp, kv, ka, kr):
    from experiments.attitude_control_validation import axis_quaternion, baseline_parameters
    from experiments.position_control_validation import position_parameters
    from quadrotor_math.attitude_control import (
        allocate_limited_body_moment,
        attitude_error_body,
        compute_attitude_control,
    )
    from quadrotor_math.attitude_simulation import _plant_step, _wrench
    from quadrotor_math.position_control import PositionReference, compute_position_control
    from quadrotor_math.run_configuration import RigidBodyParameters, WorldParameters

    inner = replace(
        baseline_parameters(),
        attitude_gain_B=np.array([ka, ka, 2.0]),
        rate_gain_B=np.array([kr, kr, 8.0]),
    )
    outer = replace(
        position_parameters(),
        position_gain_W=np.array([kp, kp, 2.25]),
        velocity_gain_W=np.array([kv, kv, 3.0]),
    )
    body = RigidBodyParameters(1.0, inner.nominal_inertia_B, np.zeros(3))
    world = WorldParameters(9.81, np.zeros(3))
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    unit = np.eye(3)[body_axis]
    identity = np.array([1.0, 0, 0, 0])

    def advance(x):
        p, v, eta, rate, acceleration = x
        moment = unit * sign * acceleration * inner.nominal_inertia_B[body_axis, body_axis]
        actual = allocate_limited_body_moment(
            9.81, moment, inner.nominal_rotors
        ).commanded_rotor_omega
        state = (
            np.eye(3)[axis] * p,
            np.eye(3)[axis] * v,
            axis_quaternion(unit, sign * eta),
            unit * sign * rate,
        )
        desired = compute_position_control(state[0], state[1], ref, outer)
        for _ in range(2):
            command = compute_attitude_control(
                state[2], state[3], desired.q_reference_WB, desired.collective_thrust, inner
            )
            for _ in range(4):
                state, actual = _plant_step(
                    state,
                    actual,
                    command.allocation.commanded_rotor_omega,
                    np.zeros(3),
                    body,
                    inner.nominal_rotors,
                    world,
                    0.0025,
                )
        return np.array(
            [
                state[0][axis],
                state[1][axis],
                sign * attitude_error_body(identity, state[2])[body_axis],
                sign * state[3][body_axis],
                sign
                * _wrench(actual, inner.nominal_rotors)[1][body_axis]
                / inner.nominal_inertia_B[body_axis, body_axis],
            ]
        )

    eps = 1e-5
    jacobian = np.column_stack(
        [(advance(eps * e) - advance(-eps * e)) / (2 * eps) for e in np.eye(5)]
    )
    expected = replace(
        model(), position_gain=kp, velocity_gain=kv, attitude_gain=ka, rate_gain=kr
    ).lifted_transition()
    np.testing.assert_allclose(jacobian, expected, atol=3e-6, rtol=3e-5)
