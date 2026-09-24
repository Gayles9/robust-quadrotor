"""Independent NED force/heading and boundary oracles for the outer loop."""

from dataclasses import FrozenInstanceError, fields, replace

import numpy as np
import pytest

from experiments.position_control_validation import position_parameters
from quadrotor_math.position_control import PositionReference, compute_position_control
from quadrotor_math.rotations import rotation_matrix_body_to_world


def reference(acceleration=None, yaw=0.0):
    return PositionReference(
        np.zeros(3),
        np.zeros(3),
        np.zeros(3) if acceleration is None else np.asarray(acceleration, dtype=float),
        yaw,
    )


def compute(acceleration=None, yaw=0.0, model=None):
    return compute_position_control(
        np.zeros(3),
        np.zeros(3),
        reference(acceleration, yaw),
        position_parameters() if model is None else model,
    )


@pytest.mark.parametrize(
    "yaw", [-np.pi, -np.pi + 1e-12, -1.7, -0.3, 0, 1.5, np.pi - 1e-12, np.pi, 3 * np.pi]
)
@pytest.mark.parametrize(
    "acceleration", [np.zeros(3), np.array([0.4, -0.3, 0.2]), np.array([-0.5, 0.4, -0.3])]
)
def test_force_reconstruction_and_exact_heading(yaw, acceleration):
    result = compute(acceleration, yaw)
    R_WB = rotation_matrix_body_to_world(result.q_reference_WB)
    np.testing.assert_allclose(R_WB.T @ R_WB, np.eye(3), atol=7e-16)
    assert np.linalg.det(R_WB) == pytest.approx(1, abs=9e-16)
    force = R_WB @ np.array([0.0, 0, -result.collective_thrust])
    np.testing.assert_allclose(force + [0, 0, 9.81], acceleration, atol=5e-15)
    heading = R_WB[:2, 0] / np.linalg.norm(R_WB[:2, 0])
    np.testing.assert_allclose(heading, [np.cos(yaw), np.sin(yaw)], atol=5e-16)
    np.testing.assert_allclose(result.feasible_acceleration_W, acceleration, atol=3e-15)
    assert not (result.acceleration_limited or result.tilt_limited or result.thrust_limited)


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("sign", [-1, 1])
def test_pd_signs_and_local_coefficients(axis, sign):
    model = position_parameters()
    step = np.eye(3)[axis] * sign * 1e-6
    p = compute_position_control(step, np.zeros(3), reference(), model)
    v = compute_position_control(np.zeros(3), step, reference(), model)
    np.testing.assert_allclose(
        p.requested_acceleration_W, -model.position_gain_W * step, atol=1e-20
    )
    np.testing.assert_allclose(
        v.requested_acceleration_W, -model.velocity_gain_W * step, atol=1e-20
    )
    if axis == 2:
        assert (p.collective_thrust > 9.81) == (sign > 0)  # too low -> more upward thrust


def test_independent_feasibility_projection():
    model = replace(
        position_parameters(), maximum_tilt_rad=np.deg2rad(5), maximum_collective_thrust=10.0
    )
    result = compute([10, -10, -10], model=model)
    assert result.acceleration_limited and result.tilt_limited and result.thrust_limited
    R_WB = rotation_matrix_body_to_world(result.q_reference_WB)
    assert np.arccos(R_WB[2, 2]) == pytest.approx(np.deg2rad(5))
    assert result.collective_thrust == 10
    expected_down = np.r_[
        -np.ones(1) / np.sqrt(2) * np.sin(np.deg2rad(5)),
        np.sin(np.deg2rad(5)) / np.sqrt(2),
        np.cos(np.deg2rad(5)),
    ]
    np.testing.assert_allclose(R_WB[:, 2], expected_down, atol=2e-16)
    np.testing.assert_allclose(
        result.feasible_acceleration_W, np.array([0.0, 0, 9.81]) - 10 * expected_down, atol=3e-15
    )


def test_positive_minimum_thrust_is_not_zero_freefall():
    model = replace(position_parameters(), minimum_collective_thrust=9.0)
    result = compute([0, 0, 2], model=model)
    assert result.thrust_limited and result.collective_thrust == 9
    assert result.feasible_acceleration_W[2] == pytest.approx(0.81)


def test_array_ownership_and_no_input_mutation():
    model = position_parameters()
    a = np.ones(3)
    target = PositionReference(a, a, a, -0.2)
    a[:] = 99
    np.testing.assert_array_equal(target.position_W, np.ones(3))
    result = compute_position_control(np.zeros(3), np.zeros(3), target, model)
    for value in (target, model, result):
        arrays = [
            getattr(value, f.name)
            for f in fields(value)
            if isinstance(getattr(value, f.name), np.ndarray)
        ]
        for array in arrays:
            assert array.flags.owndata and array.flags.c_contiguous and not array.flags.writeable
        assert all(
            not np.shares_memory(x, y) for i, x in enumerate(arrays) for y in arrays[i + 1 :]
        )
    with pytest.raises(FrozenInstanceError):
        target.yaw_rad = 3


@pytest.mark.parametrize("name", ["position_gain_W", "velocity_gain_W", "maximum_acceleration_W"])
@pytest.mark.parametrize(
    "value",
    [
        np.zeros(3),
        -np.ones(3),
        np.ones(2),
        np.ones(3, dtype=bool),
        np.ones(3, dtype=complex),
        np.full(3, np.nan),
        np.full(3, np.inf),
        [1.0, 1, 1],
    ],
)
def test_invalid_parameter_vectors(name, value):
    with pytest.raises(ValueError):
        replace(position_parameters(), **{name: value})


@pytest.mark.parametrize(
    "name",
    [
        "nominal_mass",
        "nominal_gravity_acceleration",
        "minimum_collective_thrust",
        "maximum_collective_thrust",
        "maximum_tilt_rad",
    ],
)
@pytest.mark.parametrize("value", [0, -1.0, np.nan, np.inf, True, 10**1000])
def test_invalid_parameter_scalars(name, value):
    with pytest.raises(ValueError):
        replace(position_parameters(), **{name: value})


@pytest.mark.parametrize(
    "change",
    [
        {"maximum_tilt_rad": np.pi / 2},
        {"maximum_acceleration_W": np.array([2.0, 2, 9.81])},
        {"minimum_collective_thrust": 9.81},
        {"maximum_collective_thrust": 9.81},
    ],
)
def test_singular_or_hover_infeasible_parameters(change):
    with pytest.raises(ValueError):
        replace(position_parameters(), **change)


@pytest.mark.parametrize(
    "bad",
    [
        np.ones(2),
        np.ones(3, dtype=bool),
        np.ones(3, dtype=complex),
        np.full(3, np.inf),
        [0.0, 0, 0],
    ],
)
def test_input_contracts(bad):
    with pytest.raises(ValueError):
        compute_position_control(bad, np.zeros(3), reference(), position_parameters())
    with pytest.raises(ValueError):
        replace(reference(), acceleration_W=bad)


@pytest.mark.parametrize("bad", [True, np.nan, np.inf, 10**1000])
def test_yaw_validation(bad):
    with pytest.raises(ValueError):
        reference(yaw=bad)


def test_overflow_is_not_hidden_by_clipping():
    model = replace(position_parameters(), position_gain_W=np.full(3, 1e308))
    with pytest.raises(ValueError, match="arithmetic"):
        compute_position_control(np.full(3, -1e308), np.zeros(3), reference(), model)


def test_seeded_bounded_outputs_for_large_feasible_requests():
    rng = np.random.Generator(np.random.PCG64(7715))
    model = position_parameters()
    for _ in range(60):
        result = compute(rng.uniform(-100, 100, 3), rng.uniform(-10, 10), model)
        R_WB = rotation_matrix_body_to_world(result.q_reference_WB)
        assert np.arccos(np.clip(R_WB[2, 2], -1, 1)) <= model.maximum_tilt_rad + 1e-14
        assert 2 <= result.collective_thrust <= 18
        np.testing.assert_allclose(
            result.feasible_acceleration_W,
            np.array([0, 0, 9.81]) - result.collective_thrust * R_WB[:, 2],
            atol=1e-14,
        )


def test_largest_quaternion_component_branches_near_tilt_domain_limit():
    model = replace(
        position_parameters(),
        maximum_acceleration_W=np.array([200.0, 200, 2]),
        maximum_tilt_rad=np.deg2rad(85),
    )
    branches = set()
    for heading in np.linspace(-np.pi, np.pi, 13):
        for direction in np.linspace(-np.pi, np.pi, 13):
            result = compute([150 * np.cos(direction), 150 * np.sin(direction), 0], heading, model)
            branches.add(int(np.argmax(np.abs(result.q_reference_WB))))
            R_WB = rotation_matrix_body_to_world(result.q_reference_WB)
            np.testing.assert_allclose(R_WB.T @ R_WB, np.eye(3), atol=2e-15)
            assert np.linalg.det(R_WB) == pytest.approx(1, abs=2e-15)
            np.testing.assert_allclose(
                R_WB[:2, 0] / np.linalg.norm(R_WB[:2, 0]),
                [np.cos(heading), np.sin(heading)],
                # Normalizing a near-vertical body's horizontal projection
                # amplifies matrix roundoff by at most 1/cos(maximum tilt).
                atol=16 * np.finfo(float).eps / np.cos(model.maximum_tilt_rad),
            )
            np.testing.assert_allclose(
                result.feasible_acceleration_W,
                np.array([0.0, 0, 9.81]) - result.collective_thrust * R_WB[:, 2],
                atol=2e-14,
            )
    # Explicit positive-down orientations with dominant x/y components; a
    # sparse heading/direction grid need not land on these narrow regions.
    for coordinate in (1, 2):
        expected_q_WB = np.array([np.sqrt((1 - 0.65**2) / 2), 0.0, 0.0, np.sqrt((1 - 0.65**2) / 2)])
        expected_q_WB[coordinate] = 0.65
        expected_R_WB = rotation_matrix_body_to_world(expected_q_WB)
        heading = np.arctan2(expected_R_WB[1, 0], expected_R_WB[0, 0])
        acceleration = np.array([0.0, 0, 9.81]) - 9.81 / expected_R_WB[2, 2] * expected_R_WB[:, 2]
        result = compute(acceleration, heading, model)
        branches.add(int(np.argmax(np.abs(result.q_reference_WB))))
        np.testing.assert_allclose(
            rotation_matrix_body_to_world(result.q_reference_WB), expected_R_WB, atol=2e-15
        )
    assert branches == {0, 1, 2, 3}
