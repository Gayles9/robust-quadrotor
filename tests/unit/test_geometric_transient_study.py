"""Independent reference identities and nonlinear checks of the sampled model."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.geometric_reimplementation_validation import configuration, jobs
from experiments.geometric_sampled_analysis import GeometricHoverMap
from experiments.geometric_transient_study import candidate_configuration, shaped_reference
from quadrotor_math.geometric_filter import FeedbackDerivativeFilter
from quadrotor_math.position_control import PositionReference


def test_candidate_changes_only_filter_pole_and_keeps_comparator_exact():
    from dataclasses import asdict

    from experiments.attitude_control_validation import canonical_json
    from experiments.estimated_feedback_evidence import plain

    def encoded(c):
        return canonical_json(
            plain({k: asdict(v) if hasattr(v, "__dataclass_fields__") else v for k, v in c.items()})
        )

    for job in jobs():
        expected = configuration(job)
        if job["controller"] == "geometric":
            expected["geometric_controller"] = replace(
                expected["geometric_controller"], filter_pole_rad_s=10.0
            )
        assert encoded(candidate_configuration(job)) == encoded(expected)


def test_shaped_step_has_independent_bilinear_value_and_rotation():
    from quadrotor_math.rotations import rotation_matrix_body_to_world

    p = configuration(jobs()[15])["position_controller"]
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    memory = FeedbackDerivativeFilter(0.02, 10.0)
    _, memory = shaped_reference(
        np.zeros(3), np.zeros(3), ref, np.zeros(3), np.zeros(3), p, memory, 0
    )
    displacement = np.array([0.02, -0.01, 0.015])
    command, updated = shaped_reference(
        displacement, np.zeros(3), ref, np.zeros(3), np.zeros(3), p, memory, 1
    )
    b = 1 / 11
    c = p.nominal_mass * p.position_gain_W * displacement
    expected = p.nominal_mass * np.array([0, 0, p.nominal_gravity_acceleration]) + b**3 * c
    np.testing.assert_allclose(command.lift_W, expected, atol=1e-15)
    np.testing.assert_allclose(
        rotation_matrix_body_to_world(command.rotation.q_reference_WB)[:, 2],
        expected / np.linalg.norm(expected),
        atol=1e-15,
    )
    np.testing.assert_array_equal(memory.sections, np.zeros((3, 3)))
    assert updated.next_index == 2
    with pytest.raises(ValueError):
        shaped_reference(
            displacement,
            np.zeros(3),
            ref,
            np.zeros(3),
            np.zeros(3),
            p,
            memory,
            1,
            estimated_acceleration_W=np.zeros(3),
        )


def test_shaped_memory_cannot_bypass_acceleration_domain():
    from quadrotor_math.geometric_control import GeometricDomainError

    p = configuration(jobs()[15])["position_controller"]
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    memory = FeedbackDerivativeFilter(
        0.02, 10, 1, np.array([100, 0, 0]), np.tile([100.0, 0, 0], (3, 1))
    )
    with pytest.raises(GeometricDomainError, match="shaped_reference_domain"):
        shaped_reference(np.zeros(3), np.zeros(3), ref, np.zeros(3), np.zeros(3), p, memory, 1)


@pytest.mark.parametrize("axis,body_axis,sign", [(0, 1, -1), (1, 0, 1)])
@pytest.mark.parametrize("shaped,pole", [(False, 30.0), (True, 10.0)])
def test_sampled_map_matches_nonlinear_geometric_plant(axis, body_axis, sign, shaped, pole):
    from experiments.attitude_control_validation import axis_quaternion
    from quadrotor_math.attitude_control import allocate_limited_body_moment, attitude_error_body
    from quadrotor_math.attitude_simulation import _plant_step, _wrench
    from quadrotor_math.geometric_control import compute_geometric_control
    from quadrotor_math.geometric_reference import (
        build_geometric_reference,
        projected_collective_thrust,
    )
    from quadrotor_math.run_configuration import RigidBodyParameters, WorldParameters

    c = configuration(jobs()[15])
    inner = c["attitude_controller"]
    outer = c["position_controller"]
    gains = replace(c["geometric_controller"], filter_pole_rad_s=pole)
    body = RigidBodyParameters(outer.nominal_mass, inner.nominal_inertia_B, np.zeros(3))
    world = WorldParameters(9.81, np.zeros(3))
    unit = np.eye(3)[body_axis]
    direction = np.eye(3)[axis]
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0)
    J = inner.nominal_inertia_B[body_axis, body_axis]
    builder = shaped_reference if shaped else build_geometric_reference

    def advance(x):
        p, v, eta, rate, acceleration = x[:5]
        actual = allocate_limited_body_moment(
            9.81, sign * acceleration * J * unit, inner.nominal_rotors
        ).commanded_rotor_omega
        state = (
            direction * p,
            direction * v,
            axis_quaternion(unit, sign * eta),
            sign * rate * unit,
        )
        # c=-m*g*eta_d in the horizontal force coordinate.
        memory = FeedbackDerivativeFilter(
            0.02, pole, 1, -9.81 * x[5] * direction, -9.81 * np.outer(x[6:], direction)
        )
        target, memory = builder(
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
            sign * _wrench(actual, inner.nominal_rotors)[1][body_axis] / J,
            -memory.previous[axis] / 9.81,
            -memory.sections[:, axis] / 9.81,
        ]

    epsilon = 1e-6
    numerical = np.column_stack(
        [(advance(epsilon * e) - advance(-epsilon * e)) / (2 * epsilon) for e in np.eye(9)]
    )
    model = GeometricHoverMap(float(J), pole, shaped)
    np.testing.assert_allclose(numerical, model.transition(), atol=3e-6, rtol=3e-5)
    assert np.max(np.abs(np.linalg.eigvals(model.transition()))) < 1


@pytest.mark.parametrize(
    "kwargs",
    [
        {"inertia": 0},
        {"inertia": True},
        {"inertia": float("nan")},
        {"inertia": 0.02, "pole_rad_s": 51},
        {"inertia": 0.02, "shaped": 1},
    ],
)
def test_invalid_local_model(kwargs):
    with pytest.raises(ValueError):
        GeometricHoverMap(**kwargs)
