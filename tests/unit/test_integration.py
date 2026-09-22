import numpy as np
import pytest

import quadrotor_math.dynamics as dynamics
from quadrotor_math.integration import (
    rigid_body_state_euler_step_from_rotor_speeds,
    rigid_body_state_rk4_step_from_rotor_speeds,
)
from quadrotor_math.rotations import (
    normalize_quaternion_body_to_world,
    quaternion_derivative_body_to_world,
    rotation_matrix_body_to_world,
)


def _level_hover_step(
    stepper,
    *,
    q_WB: np.ndarray | None = None,
    omega_B: np.ndarray | None = None,
    environment: dict[str, np.ndarray] | None = None,
):
    if q_WB is None:
        q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if omega_B is None:
        omega_B = np.zeros(3, dtype=np.float64)
    if environment is None:
        environment = {}
    return stepper(
        np.zeros(3, dtype=np.float64),
        np.array([2.0, 0.0, 0.0], dtype=np.float64),
        q_WB,
        omega_B,
        np.full(4, 500.0, dtype=np.float64),
        np.array(
            [
                [0.5, 0.0, 0.0],
                [0.0, 0.5, 0.0],
                [-0.5, 0.0, 0.0],
                [0.0, -0.5, 0.0],
            ],
            dtype=np.float64,
        ),
        np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64),
        1.0,
        np.eye(3, dtype=np.float64),
        10.0,
        1.0e-5,
        1.0e-6,
        0.1,
        **environment,
    )


def test_rigid_body_state_euler_step_from_rotor_speeds_advances_complete_state() -> None:
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.array([0.0, 0.0, 2.0], dtype=np.float64)

    rotor_omega = np.array([2.0, 0.0, 0.0, 0.0], dtype=np.float64)
    rotor_positions_B = np.array(
        [
            [2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    mass = 1.0
    inertia_B = np.diag(
        np.array([2.0, 3.0, 4.0], dtype=np.float64),
    )
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.1

    (
        next_position_W,
        next_velocity_W,
        next_q_WB,
        next_omega_B,
    ) = rigid_body_state_euler_step_from_rotor_speeds(
        position_W,
        velocity_W,
        q_WB,
        omega_B,
        rotor_omega,
        rotor_positions_B,
        rotor_spin_directions,
        mass,
        inertia_B,
        gravity_acceleration,
        thrust_coefficient,
        moment_coefficient,
        time_step,
    )

    expected_position_W = np.array(
        [10.1, 19.8, 30.3],
        dtype=np.float64,
    )
    expected_velocity_W = np.array(
        [1.0, -2.0, 3.681],
        dtype=np.float64,
    )
    quaternion_norm = np.sqrt(1.01)
    expected_q_WB = np.array(
        [
            1.0 / quaternion_norm,
            0.0,
            0.0,
            0.1 / quaternion_norm,
        ],
        dtype=np.float64,
    )
    expected_omega_B = np.array(
        [0.0, 0.2, 1.95],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        next_position_W,
        expected_position_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        next_velocity_W,
        expected_velocity_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        next_q_WB,
        expected_q_WB,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        next_omega_B,
        expected_omega_B,
        atol=1e-12,
    )


def test_euler_step_propagates_quadratic_drag_environment() -> None:
    calm_air = _level_hover_step(
        rigid_body_state_euler_step_from_rotor_speeds,
        environment={
            "wind_velocity_W": np.zeros(3),
            "quadratic_drag_coefficient_B": np.array([0.5, 0.0, 0.0]),
        },
    )
    following_wind = _level_hover_step(
        rigid_body_state_euler_step_from_rotor_speeds,
        environment={
            "wind_velocity_W": np.array([1.0, 0.0, 0.0]),
            "quadratic_drag_coefficient_B": np.array([0.5, 0.0, 0.0]),
        },
    )

    np.testing.assert_array_equal(calm_air[0], np.array([0.2, 0.0, 0.0]))
    np.testing.assert_array_equal(calm_air[1], np.array([1.8, 0.0, 0.0]))
    np.testing.assert_array_equal(calm_air[2], np.array([1.0, 0.0, 0.0, 0.0]))
    np.testing.assert_array_equal(calm_air[3], np.zeros(3))
    np.testing.assert_array_equal(following_wind[1], np.array([1.95, 0.0, 0.0]))


def test_rigid_body_state_rk4_step_from_rotor_speeds_advances_complete_state() -> None:
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.array([0.0, 0.0, 2.0], dtype=np.float64)

    rotor_omega = np.array([2.0, 0.0, 0.0, 0.0], dtype=np.float64)
    rotor_positions_B = np.array(
        [
            [2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    mass = 1.0
    inertia_B = np.diag(
        np.array([2.0, 3.0, 4.0], dtype=np.float64),
    )
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.1

    (
        next_position_W,
        next_velocity_W,
        next_q_WB,
        next_omega_B,
    ) = rigid_body_state_rk4_step_from_rotor_speeds(
        position_W,
        velocity_W,
        q_WB,
        omega_B,
        rotor_omega,
        rotor_positions_B,
        rotor_spin_directions,
        mass,
        inertia_B,
        gravity_acceleration,
        thrust_coefficient,
        moment_coefficient,
        time_step,
    )

    expected_position_W = np.array(
        [
            10.099975092370972,
            19.799997517165757,
            30.334050062655837,
        ],
        dtype=np.float64,
    )
    expected_velocity_W = np.array(
        [
            0.9990073165807823,
            -2.000123449300478,
            3.681003753240262,
        ],
        dtype=np.float64,
    )
    expected_q_WB = np.array(
        [
            0.9951156964771156,
            -0.0003300038558926743,
            0.004978286004512869,
            0.09858934217641005,
        ],
        dtype=np.float64,
    )
    expected_omega_B = np.array(
        [
            -0.009822789646669,
            0.1995693389233941,
            1.9500123170009223,
        ],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        next_position_W,
        expected_position_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        next_velocity_W,
        expected_velocity_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        next_q_WB,
        expected_q_WB,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        next_omega_B,
        expected_omega_B,
        atol=1e-12,
    )


def test_rk4_step_matches_independent_scalar_quadratic_drag_calculation() -> None:
    time_step = 0.1
    velocity = 2.0

    def acceleration(north_velocity: float) -> float:
        return -0.5 * abs(north_velocity) * north_velocity

    k1_position = velocity
    k1_velocity = acceleration(velocity)
    k2_position = velocity + 0.5 * time_step * k1_velocity
    k2_velocity = acceleration(velocity + 0.5 * time_step * k1_velocity)
    k3_position = velocity + 0.5 * time_step * k2_velocity
    k3_velocity = acceleration(velocity + 0.5 * time_step * k2_velocity)
    k4_position = velocity + time_step * k3_velocity
    k4_velocity = acceleration(velocity + time_step * k3_velocity)
    expected_position = (time_step / 6.0) * (
        k1_position + 2.0 * k2_position + 2.0 * k3_position + k4_position
    )
    expected_velocity = velocity + (time_step / 6.0) * (
        k1_velocity + 2.0 * k2_velocity + 2.0 * k3_velocity + k4_velocity
    )

    actual = _level_hover_step(
        rigid_body_state_rk4_step_from_rotor_speeds,
        environment={
            "wind_velocity_W": np.zeros(3),
            "quadratic_drag_coefficient_B": np.array([0.5, 0.0, 0.0]),
        },
    )

    np.testing.assert_allclose(actual[0], np.array([expected_position, 0.0, 0.0]), atol=1e-15)
    np.testing.assert_allclose(actual[1], np.array([expected_velocity, 0.0, 0.0]), atol=1e-15)


def test_rk4_recomputes_drag_at_each_projected_stage(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorded_velocities_W: list[np.ndarray] = []
    recorded_rotations_WB: list[np.ndarray] = []
    original_drag = dynamics.quadratic_drag_force_body

    def record_drag(
        velocity_W: np.ndarray,
        R_WB: np.ndarray,
        wind_velocity_W: np.ndarray,
        quadratic_drag_coefficient_B: np.ndarray,
    ) -> np.ndarray:
        recorded_velocities_W.append(velocity_W.copy())
        recorded_rotations_WB.append(R_WB.copy())
        return original_drag(
            velocity_W,
            R_WB,
            wind_velocity_W,
            quadratic_drag_coefficient_B,
        )

    monkeypatch.setattr(dynamics, "quadratic_drag_force_body", record_drag)
    omega_B = np.array([0.0, 0.0, 1.0])
    coefficient_B = np.array([0.5, 0.25, 0.0])
    _level_hover_step(
        rigid_body_state_rk4_step_from_rotor_speeds,
        omega_B=omega_B,
        environment={
            "wind_velocity_W": np.zeros(3),
            "quadratic_drag_coefficient_B": coefficient_B,
        },
    )

    time_step = 0.1
    initial_q_WB = np.array([1.0, 0.0, 0.0, 0.0])
    k1_q_WB = quaternion_derivative_body_to_world(initial_q_WB, omega_B)
    q_k2_WB = normalize_quaternion_body_to_world(initial_q_WB + 0.5 * time_step * k1_q_WB)
    k2_q_WB = quaternion_derivative_body_to_world(q_k2_WB, omega_B)
    q_k3_WB = normalize_quaternion_body_to_world(initial_q_WB + 0.5 * time_step * k2_q_WB)
    k3_q_WB = quaternion_derivative_body_to_world(q_k3_WB, omega_B)
    q_k4_WB = normalize_quaternion_body_to_world(initial_q_WB + time_step * k3_q_WB)
    expected_rotations_WB = [
        rotation_matrix_body_to_world(stage_q_WB)
        for stage_q_WB in (initial_q_WB, q_k2_WB, q_k3_WB, q_k4_WB)
    ]

    def drag_acceleration_W(velocity_W: np.ndarray, R_WB: np.ndarray) -> np.ndarray:
        velocity_air_B = R_WB.T @ velocity_W
        force_drag_B = -coefficient_B * np.abs(velocity_air_B) * velocity_air_B
        return R_WB @ force_drag_B

    velocity_k1_W = np.array([2.0, 0.0, 0.0])
    velocity_k2_W = velocity_k1_W + 0.5 * time_step * drag_acceleration_W(
        velocity_k1_W, expected_rotations_WB[0]
    )
    velocity_k3_W = velocity_k1_W + 0.5 * time_step * drag_acceleration_W(
        velocity_k2_W, expected_rotations_WB[1]
    )
    velocity_k4_W = velocity_k1_W + time_step * drag_acceleration_W(
        velocity_k3_W, expected_rotations_WB[2]
    )

    assert len(recorded_velocities_W) == 4
    np.testing.assert_allclose(
        recorded_velocities_W,
        [velocity_k1_W, velocity_k2_W, velocity_k3_W, velocity_k4_W],
        rtol=0.0,
        atol=1e-15,
    )
    np.testing.assert_allclose(
        recorded_rotations_WB,
        expected_rotations_WB,
        rtol=0.0,
        atol=1e-15,
    )
    assert not np.array_equal(recorded_rotations_WB[0], recorded_rotations_WB[1])
    for R_WB in recorded_rotations_WB:
        assert np.all(np.isfinite(R_WB))
        np.testing.assert_allclose(R_WB.T @ R_WB, np.eye(3), rtol=0.0, atol=1e-12)
        assert np.linalg.det(R_WB) == pytest.approx(1.0, abs=1e-12)


@pytest.mark.parametrize(
    "stepper",
    [
        rigid_body_state_euler_step_from_rotor_speeds,
        rigid_body_state_rk4_step_from_rotor_speeds,
    ],
)
def test_stepper_explicit_zero_environment_matches_omission_exactly(stepper) -> None:
    omitted = _level_hover_step(stepper)
    explicit_zero = _level_hover_step(
        stepper,
        environment={
            "wind_velocity_W": np.zeros(3),
            "quadratic_drag_coefficient_B": np.zeros(3),
        },
    )

    for omitted_array, explicit_array in zip(omitted, explicit_zero, strict=True):
        np.testing.assert_array_equal(explicit_array, omitted_array)


@pytest.mark.parametrize(
    ("time_step", "expected_message"),
    [
        (np.nan, "time_step must be finite"),
        (np.inf, "time_step must be finite"),
        (0.0, "time_step must be positive"),
        (-0.1, "time_step must be positive"),
    ],
)
def test_rigid_body_state_rk4_step_rejects_invalid_time_step(
    time_step: float,
    expected_message: str,
) -> None:
    position_W = np.zeros(3, dtype=np.float64)
    velocity_W = np.zeros(3, dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    rotor_omega = np.zeros(4, dtype=np.float64)
    rotor_positions_B = np.zeros((4, 3), dtype=np.float64)
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )
    mass = 1.0
    inertia_B = np.eye(3, dtype=np.float64)
    gravity_acceleration = 9.81
    thrust_coefficient = 1.0
    moment_coefficient = 1.0

    with pytest.raises(ValueError, match=expected_message):
        rigid_body_state_rk4_step_from_rotor_speeds(
            position_W,
            velocity_W,
            q_WB,
            omega_B,
            rotor_omega,
            rotor_positions_B,
            rotor_spin_directions,
            mass,
            inertia_B,
            gravity_acceleration,
            thrust_coefficient,
            moment_coefficient,
            time_step,
        )


@pytest.mark.parametrize(
    ("time_step", "expected_message"),
    [
        (np.nan, "time_step must be finite"),
        (np.inf, "time_step must be finite"),
        (0.0, "time_step must be positive"),
        (-0.1, "time_step must be positive"),
    ],
)
def test_rigid_body_state_euler_step_rejects_invalid_time_step(
    time_step: float,
    expected_message: str,
) -> None:
    position_W = np.zeros(3, dtype=np.float64)
    velocity_W = np.zeros(3, dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    rotor_omega = np.zeros(4, dtype=np.float64)
    rotor_positions_B = np.zeros((4, 3), dtype=np.float64)
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )
    mass = 1.0
    inertia_B = np.eye(3, dtype=np.float64)
    gravity_acceleration = 9.81
    thrust_coefficient = 1.0
    moment_coefficient = 1.0

    with pytest.raises(ValueError, match=expected_message):
        rigid_body_state_euler_step_from_rotor_speeds(
            position_W,
            velocity_W,
            q_WB,
            omega_B,
            rotor_omega,
            rotor_positions_B,
            rotor_spin_directions,
            mass,
            inertia_B,
            gravity_acceleration,
            thrust_coefficient,
            moment_coefficient,
            time_step,
        )
