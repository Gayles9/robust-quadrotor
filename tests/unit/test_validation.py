import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.simulation import (
    simulate_rigid_body_euler_from_rotor_speeds,
    simulate_rigid_body_rk4_from_rotor_speeds,
)
from quadrotor_math.validation import validate_rigid_body_state_history


def test_validate_rigid_body_state_history_rejects_invalid_time_shape() -> None:
    time_s = np.array(
        [
            [0.0],
            [0.5],
            [1.0],
        ],
        dtype=np.float64,
    )
    position_history_W = np.zeros((3, 3), dtype=np.float64)
    velocity_history_W = np.zeros((3, 3), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 3), dtype=np.float64)

    with pytest.raises(ValueError, match=r"time_s must have shape \(N,\)"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


def test_validate_rigid_body_state_history_rejects_fewer_than_two_samples() -> None:
    time_s = np.array([0.0], dtype=np.float64)
    position_history_W = np.zeros((1, 3), dtype=np.float64)
    velocity_history_W = np.zeros((1, 3), dtype=np.float64)
    q_history_WB = np.array(
        [[1.0, 0.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((1, 3), dtype=np.float64)

    with pytest.raises(ValueError, match="state histories must contain at least two samples"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


def test_validate_rigid_body_state_history_rejects_invalid_position_shape() -> None:
    time_s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    position_history_W = np.zeros((3, 2), dtype=np.float64)
    velocity_history_W = np.zeros((3, 3), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 3), dtype=np.float64)

    with pytest.raises(ValueError, match=r"position_history_W must have shape \(N, 3\)"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


def test_validate_rigid_body_state_history_rejects_invalid_velocity_shape() -> None:
    time_s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    position_history_W = np.zeros((3, 3), dtype=np.float64)
    velocity_history_W = np.zeros((3, 2), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 3), dtype=np.float64)

    with pytest.raises(ValueError, match=r"velocity_history_W must have shape \(N, 3\)"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


def test_validate_rigid_body_state_history_rejects_invalid_quaternion_shape() -> None:
    time_s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    position_history_W = np.zeros((3, 3), dtype=np.float64)
    velocity_history_W = np.zeros((3, 3), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 3), dtype=np.float64)

    with pytest.raises(ValueError, match=r"q_history_WB must have shape \(N, 4\)"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


def test_validate_rigid_body_state_history_rejects_invalid_angular_velocity_shape() -> None:
    time_s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    position_history_W = np.zeros((3, 3), dtype=np.float64)
    velocity_history_W = np.zeros((3, 3), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 2), dtype=np.float64)

    with pytest.raises(ValueError, match=r"omega_history_B must have shape \(N, 3\)"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


@pytest.mark.parametrize(
    ("position_rows", "velocity_rows", "quaternion_rows", "omega_rows"),
    [
        (2, 3, 3, 3),
        (3, 2, 3, 3),
        (3, 3, 2, 3),
        (3, 3, 3, 2),
    ],
)
def test_validate_rigid_body_state_history_rejects_mismatched_sample_counts(
    position_rows: int,
    velocity_rows: int,
    quaternion_rows: int,
    omega_rows: int,
) -> None:
    time_s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    position_history_W = np.zeros((position_rows, 3), dtype=np.float64)
    velocity_history_W = np.zeros((velocity_rows, 3), dtype=np.float64)
    identity_q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    q_history_WB = np.tile(identity_q_WB, (quaternion_rows, 1))
    omega_history_B = np.zeros((omega_rows, 3), dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="state histories must have the same number of samples as time_s",
    ):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


def test_validate_rigid_body_state_history_rejects_nonfinite_time() -> None:
    time_s = np.array([0.0, np.nan, 1.0], dtype=np.float64)
    position_history_W = np.zeros((3, 3), dtype=np.float64)
    velocity_history_W = np.zeros((3, 3), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 3), dtype=np.float64)

    with pytest.raises(ValueError, match="time_s must contain only finite values"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


@pytest.mark.parametrize(
    "history_name",
    [
        "position_history_W",
        "velocity_history_W",
        "q_history_WB",
        "omega_history_B",
    ],
)
def test_validate_rigid_body_state_history_rejects_nonfinite_state_history(
    history_name: str,
) -> None:
    time_s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    position_history_W = np.zeros((3, 3), dtype=np.float64)
    velocity_history_W = np.zeros((3, 3), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 3), dtype=np.float64)

    if history_name == "position_history_W":
        position_history_W[1, 0] = np.nan
    elif history_name == "velocity_history_W":
        velocity_history_W[1, 0] = np.nan
    elif history_name == "q_history_WB":
        q_history_WB[1, 0] = np.nan
    elif history_name == "omega_history_B":
        omega_history_B[1, 0] = np.nan
    else:
        raise AssertionError(f"unexpected history name: {history_name}")

    with pytest.raises(
        ValueError,
        match=f"{history_name} must contain only finite values",
    ):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


@pytest.mark.parametrize(
    "time_s",
    [
        np.array([0.0, 0.5, 0.5], dtype=np.float64),
        np.array([0.0, 0.5, 0.25], dtype=np.float64),
    ],
)
def test_validate_rigid_body_state_history_rejects_nonincreasing_time(
    time_s: NDArray[np.float64],
) -> None:
    position_history_W = np.zeros((3, 3), dtype=np.float64)
    velocity_history_W = np.zeros((3, 3), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 3), dtype=np.float64)

    with pytest.raises(ValueError, match="time_s must be strictly increasing"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


def test_validate_rigid_body_state_history_rejects_nonuniform_time_spacing() -> None:
    time_s = np.array([0.0, 0.5, 1.25], dtype=np.float64)
    position_history_W = np.zeros((3, 3), dtype=np.float64)
    velocity_history_W = np.zeros((3, 3), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 3), dtype=np.float64)

    with pytest.raises(ValueError, match="time_s must be uniformly spaced"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


def test_validate_rigid_body_state_history_rejects_nonunit_quaternion() -> None:
    time_s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    position_history_W = np.zeros((3, 3), dtype=np.float64)
    velocity_history_W = np.zeros((3, 3), dtype=np.float64)
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.zeros((3, 3), dtype=np.float64)

    with pytest.raises(ValueError, match="q_history_WB must contain only unit quaternions"):
        validate_rigid_body_state_history(
            time_s,
            position_history_W,
            velocity_history_W,
            q_history_WB,
            omega_history_B,
        )


def test_validate_rigid_body_state_history_accepts_valid_history() -> None:
    time_s = np.array([0.0, 0.25, 0.5], dtype=np.float64)
    position_history_W = np.array(
        [
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [7.0, 8.0, 9.0],
        ],
        dtype=np.float64,
    )
    velocity_history_W = np.array(
        [
            [0.1, -0.2, 0.3],
            [0.4, -0.5, 0.6],
            [0.7, -0.8, 0.9],
        ],
        dtype=np.float64,
    )
    q_history_WB = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)],
            [-1.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    omega_history_B = np.array(
        [
            [0.01, -0.02, 0.03],
            [0.04, -0.05, 0.06],
            [0.07, -0.08, 0.09],
        ],
        dtype=np.float64,
    )

    result = validate_rigid_body_state_history(
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    )

    assert result is None


@pytest.mark.parametrize(
    "simulate_rigid_body_from_rotor_speeds",
    [
        simulate_rigid_body_euler_from_rotor_speeds,
        simulate_rigid_body_rk4_from_rotor_speeds,
    ],
)
def test_validate_rigid_body_state_history_accepts_simulator_output(
    simulate_rigid_body_from_rotor_speeds,
) -> None:
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
    rotor_spin_directions = np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)
    mass = 1.0
    inertia_B = np.diag(np.array([2.0, 3.0, 4.0], dtype=np.float64))
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5
    time_step = 0.1
    number_of_steps = 2

    (
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    ) = simulate_rigid_body_from_rotor_speeds(
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
        number_of_steps,
    )

    result = validate_rigid_body_state_history(
        time_s,
        position_history_W,
        velocity_history_W,
        q_history_WB,
        omega_history_B,
    )

    assert result is None
