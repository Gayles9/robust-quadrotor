"""Independent transfer identities and nonlinear checks of both sampled maps."""

import numpy as np
import pytest

from experiments.axis_shaped_geometric import (
    axis_shaped_reference,
    force_step,
    horizontal_transition,
    vertical_transition,
)
from experiments.final_geometric_comparison import BY_NAME, configure
from experiments.geometric_sampled_analysis import GeometricHoverMap
from experiments.geometric_transient_study import shaped_reference
from experiments.supported_geometric_comparison import Job, configuration
from quadrotor_math.geometric_control import GeometricDomainError
from quadrotor_math.geometric_filter import FeedbackDerivativeFilter
from quadrotor_math.position_control import PositionReference


def test_anisotropic_discrete_transfer_function_and_derivative_jets():
    h, omega = 0.02, 4.7
    poles = np.array([10.0, 10.0, 30.0])
    memory = FeedbackDerivativeFilter(h, 10)
    initial = memory
    for k in range(1500):
        first, second, memory = force_step(np.full(3, np.cos(omega * h * k)), memory, k, 30)
    # Independent frequency-domain oracle under the bilinear substitution.
    laplace = 2j / h * np.tan(omega * h / 2)
    response = (poles / (poles + laplace)) ** 3 * np.exp(1j * omega * h * k)
    np.testing.assert_allclose(memory.sections[2], response.real, atol=1e-13)
    np.testing.assert_allclose(first, (laplace * response).real, atol=1e-12)
    np.testing.assert_allclose(second, (laplace**2 * response).real, atol=1e-11)
    assert initial.next_index == 0
    np.testing.assert_array_equal(initial.sections, np.zeros((3, 3)))


def test_isotropic_compatibility_and_reset():
    outer = configuration(Job("hover", 40), "smoke")["position_controller"]
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0.1)
    memories = [FeedbackDerivativeFilter(0.02, 10) for _ in range(2)]
    rng = np.random.default_rng(40)
    for k in range(30):
        pos, vel, jerk, snap = rng.normal(0, 0.01, (4, 3))
        old, memories[0] = shaped_reference(pos, vel, ref, jerk, snap, outer, memories[0], k)
        new, memories[1] = axis_shaped_reference(
            pos, vel, ref, jerk, snap, outer, memories[1], k, vertical_pole_rad_s=10
        )
        for name in ("lift_W",):
            np.testing.assert_allclose(getattr(new, name), getattr(old, name), atol=1e-14)
        for name in ("q_reference_WB", "omega_reference_D", "alpha_reference_D"):
            np.testing.assert_allclose(
                getattr(new.rotation, name), getattr(old.rotation, name), atol=1e-14
            )
        np.testing.assert_array_equal(memories[0].sections, memories[1].sections)
    first, second, reset = force_step(
        np.array([1.0, 2, 3]), FeedbackDerivativeFilter(0.02, 10), 0, 30
    )
    np.testing.assert_array_equal(first, np.zeros(3))
    np.testing.assert_array_equal(second, np.zeros(3))
    np.testing.assert_array_equal(reset.sections, np.tile([1.0, 2, 3], (3, 1)))


@pytest.mark.parametrize("pole", [0, -1, True, float("nan"), float("inf"), 51, 1e-323])
def test_bad_vertical_poles(pole):
    with pytest.raises(ValueError):
        force_step(np.zeros(3), FeedbackDerivativeFilter(0.02, 10), 0, pole)


def test_failed_reference_keeps_memory_and_rejects_both_domains():
    outer = configuration(Job("hover", 40), "smoke")["position_controller"]
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0)
    clean = FeedbackDerivativeFilter(0.02, 10)
    with pytest.raises(GeometricDomainError, match="outer_reference_domain"):
        axis_shaped_reference(
            np.ones(3) * 100, np.zeros(3), ref, np.zeros(3), np.zeros(3), outer, clean, 0
        )
    assert clean.next_index == 0
    memory = FeedbackDerivativeFilter(
        0.02, 10, 1, np.array([100.0, 0, 0]), np.tile([100.0, 0, 0], (3, 1))
    )
    with pytest.raises(GeometricDomainError, match="shaped_reference_domain"):
        axis_shaped_reference(
            np.zeros(3), np.zeros(3), ref, np.zeros(3), np.zeros(3), outer, memory, 1
        )
    assert memory.next_index == 1
    with pytest.raises(ValueError, match="sample index"):
        force_step(np.zeros(3), clean, 1, 30)
    with pytest.raises(ValueError, match="correction channel"):
        axis_shaped_reference(
            np.zeros(3),
            np.zeros(3),
            ref,
            np.zeros(3),
            np.zeros(3),
            outer,
            clean,
            0,
            estimated_acceleration_W=np.zeros(3),
        )


@pytest.mark.parametrize("axis,body_axis,sign", [(0, 1, -1), (1, 0, 1)])
@pytest.mark.parametrize("pole,frequency", [(10, 1.0), (15, 1.25), (20, 1.25)])
def test_horizontal_map_against_independent_nonlinear_plant(axis, body_axis, sign, pole, frequency):
    from experiments.attitude_control_validation import axis_quaternion
    from quadrotor_math.attitude_control import allocate_limited_body_moment, attitude_error_body
    from quadrotor_math.attitude_simulation import _plant_step, _wrench
    from quadrotor_math.geometric_control import compute_geometric_control
    from quadrotor_math.geometric_reference import projected_collective_thrust
    from quadrotor_math.run_configuration import RigidBodyParameters, WorldParameters

    config = configure(
        configuration(Job("hover", 40), "smoke"), BY_NAME[f"axis-h{pole}-f{frequency:g}"]
    )
    inner, outer, gains = (
        config[k] for k in ("attitude_controller", "position_controller", "geometric_controller")
    )
    body = RigidBodyParameters(outer.nominal_mass, inner.nominal_inertia_B, np.zeros(3))
    world = WorldParameters(9.81, np.zeros(3))
    unit, direction = np.eye(3)[body_axis], np.eye(3)[axis]
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0)
    inertia = inner.nominal_inertia_B[body_axis, body_axis]

    def advance(x):
        p, v, eta, rate, acceleration = x[:5]
        actual = allocate_limited_body_moment(
            9.81, sign * acceleration * inertia * unit, inner.nominal_rotors
        ).commanded_rotor_omega
        state = direction * p, direction * v, axis_quaternion(unit, sign * eta), sign * rate * unit
        memory = FeedbackDerivativeFilter(
            0.02, pole, 1, -9.81 * x[5] * direction, -9.81 * np.outer(x[6:], direction)
        )
        target, memory = axis_shaped_reference(
            state[0], state[1], ref, np.zeros(3), np.zeros(3), outer, memory, 1
        )
        for _ in range(2):
            output = compute_geometric_control(
                state[2],
                state[3],
                target.rotation,
                projected_collective_thrust(target, state[2], outer),
                inner,
                gains,
            )
            for _ in range(4):
                state, actual = _plant_step(
                    state,
                    actual,
                    output.allocation.commanded_rotor_omega,
                    np.zeros(3),
                    body,
                    inner.nominal_rotors,
                    world,
                    0.0025,
                )
        return np.r_[
            state[0][axis],
            state[1][axis],
            sign * attitude_error_body(np.array([1.0, 0, 0, 0]), state[2])[body_axis],
            sign * state[3][body_axis],
            sign * _wrench(actual, inner.nominal_rotors)[1][body_axis] / inertia,
            -memory.previous[axis] / 9.81,
            -memory.sections[:, axis] / 9.81,
        ]

    epsilon = 1e-6
    numerical = np.column_stack(
        [(advance(epsilon * e) - advance(-epsilon * e)) / (2 * epsilon) for e in np.eye(9)]
    )
    model = horizontal_transition(pole, frequency, float(inertia))
    np.testing.assert_allclose(numerical, model, atol=3e-6, rtol=3e-5)
    assert np.max(np.abs(np.linalg.eigvals(model))) < 1
    if frequency == 1:
        np.testing.assert_allclose(
            model, GeometricHoverMap(float(inertia), pole, True).transition(), atol=1e-15
        )


def test_vertical_map_against_independent_nonlinear_plant():
    from quadrotor_math.attitude_control import allocate_limited_body_moment
    from quadrotor_math.attitude_simulation import _plant_step
    from quadrotor_math.run_configuration import RigidBodyParameters, WorldParameters

    config = configuration(Job("hover", 40), "smoke")
    inner, outer = config["attitude_controller"], config["position_controller"]
    body = RigidBodyParameters(outer.nominal_mass, inner.nominal_inertia_B, np.zeros(3))
    world = WorldParameters(9.81, np.zeros(3))
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0)
    direction = np.array([0.0, 0, 1])

    def advance(x):
        actual = allocate_limited_body_moment(
            9.81 - x[2], np.zeros(3), inner.nominal_rotors
        ).commanded_rotor_omega
        state = direction * x[0], direction * x[1], np.array([1.0, 0, 0, 0]), np.zeros(3)
        memory = FeedbackDerivativeFilter(
            0.02, 10, 1, -x[3] * direction, -np.outer(x[4:], direction)
        )
        target, memory = axis_shaped_reference(
            state[0], state[1], ref, np.zeros(3), np.zeros(3), outer, memory, 1
        )
        command = allocate_limited_body_moment(
            target.lift_W[2], np.zeros(3), inner.nominal_rotors
        ).commanded_rotor_omega
        for _ in range(8):
            state, actual = _plant_step(
                state, actual, command, np.zeros(3), body, inner.nominal_rotors, world, 0.0025
            )
        acceleration = 9.81 - inner.nominal_rotors.thrust_coefficient * np.sum(actual**2)
        return np.r_[
            state[0][2], state[1][2], acceleration, -memory.previous[2], -memory.sections[:, 2]
        ]

    epsilon = 1e-5
    numerical = np.column_stack(
        [(advance(epsilon * e) - advance(-epsilon * e)) / (2 * epsilon) for e in np.eye(7)]
    )
    np.testing.assert_allclose(numerical, vertical_transition(), atol=3e-7, rtol=3e-5)
    assert np.max(np.abs(np.linalg.eigvals(vertical_transition()))) < 1
