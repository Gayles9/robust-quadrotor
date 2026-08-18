import numpy as np
import pytest
from numpy.typing import NDArray

from quadrotor_math.actuation import (
    reaction_moment_body_from_rotor_speeds,
    rotor_thrusts_from_speeds,
    thrust_force_body_from_rotor_thrusts,
    thrust_moment_body_from_rotor_thrusts,
)


def test_rotor_thrusts_from_speeds_uses_quadratic_law() -> None:
    rotor_omega = np.array(
        [0.0, 100.0, 200.0, 300.0],
        dtype=np.float64,
    )
    thrust_coefficient = 1.0e-5

    rotor_thrusts = rotor_thrusts_from_speeds(
        rotor_omega,
        thrust_coefficient,
    )

    expected_rotor_thrusts = np.array(
        [0.0, 0.1, 0.4, 0.9],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        rotor_thrusts,
        expected_rotor_thrusts,
        atol=1e-12,
    )


def test_reaction_moment_body_from_rotor_speeds_produces_yaw() -> None:
    rotor_omega = np.array([2.0, 1.0, 2.0, 1.0], dtype=np.float64)
    rotor_spin_directions = np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)
    moment_coefficient = 0.5

    reaction_moment_B = reaction_moment_body_from_rotor_speeds(
        rotor_omega,
        rotor_spin_directions,
        moment_coefficient,
    )

    expected_reaction_moment_B = np.array([0.0, 0.0, -3.0], dtype=np.float64)
    np.testing.assert_allclose(
        reaction_moment_B,
        expected_reaction_moment_B,
        atol=1e-12,
    )


def test_balanced_counter_rotating_rotors_cancel_yaw_reaction_moment() -> None:
    rotor_omega = np.full(4, 600.0, dtype=np.float64)
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )
    moment_coefficient = 2.0e-7

    reaction_moment_B = reaction_moment_body_from_rotor_speeds(
        rotor_omega,
        rotor_spin_directions,
        moment_coefficient,
    )

    expected_reaction_moment_B = np.zeros(3, dtype=np.float64)
    np.testing.assert_allclose(
        reaction_moment_B,
        expected_reaction_moment_B,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    "rotor_omega",
    [
        np.zeros(3, dtype=np.float64),
        np.zeros((4, 1), dtype=np.float64),
        np.zeros(5, dtype=np.float64),
    ],
)
def test_reaction_moment_body_from_rotor_speeds_rejects_invalid_speed_shape(
    rotor_omega: NDArray[np.float64],
) -> None:
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    with pytest.raises(ValueError, match=r"rotor_omega must have shape \(4,\)"):
        reaction_moment_body_from_rotor_speeds(
            rotor_omega,
            rotor_spin_directions,
            1.0,
        )


@pytest.mark.parametrize(
    "rotor_spin_directions",
    [
        np.zeros(3, dtype=np.float64),
        np.zeros((4, 1), dtype=np.float64),
        np.zeros(5, dtype=np.float64),
    ],
)
def test_reaction_moment_body_from_rotor_speeds_rejects_invalid_direction_shape(
    rotor_spin_directions: NDArray[np.float64],
) -> None:
    rotor_omega = np.ones(4, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"rotor_spin_directions must have shape \(4,\)",
    ):
        reaction_moment_body_from_rotor_speeds(
            rotor_omega,
            rotor_spin_directions,
            1.0,
        )


@pytest.mark.parametrize(
    "rotor_omega",
    [
        np.array([np.nan, 1.0, 1.0, 1.0], dtype=np.float64),
        np.array([np.inf, 1.0, 1.0, 1.0], dtype=np.float64),
        np.array([-np.inf, 1.0, 1.0, 1.0], dtype=np.float64),
    ],
)
def test_reaction_moment_body_from_rotor_speeds_rejects_non_finite_speed(
    rotor_omega: NDArray[np.float64],
) -> None:
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    with pytest.raises(
        ValueError,
        match="rotor_omega must contain only finite values",
    ):
        reaction_moment_body_from_rotor_speeds(
            rotor_omega,
            rotor_spin_directions,
            1.0,
        )


def test_reaction_moment_body_from_rotor_speeds_rejects_negative_speed() -> None:
    rotor_omega = np.array([1.0, 1.0, -1.0, 1.0], dtype=np.float64)
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    with pytest.raises(ValueError, match="rotor_omega must be nonnegative"):
        reaction_moment_body_from_rotor_speeds(
            rotor_omega,
            rotor_spin_directions,
            1.0,
        )


@pytest.mark.parametrize(
    "rotor_spin_directions",
    [
        np.array([np.nan, -1.0, 1.0, -1.0], dtype=np.float64),
        np.array([np.inf, -1.0, 1.0, -1.0], dtype=np.float64),
        np.array([-np.inf, -1.0, 1.0, -1.0], dtype=np.float64),
    ],
)
def test_reaction_moment_body_from_rotor_speeds_rejects_non_finite_directions(
    rotor_spin_directions: NDArray[np.float64],
) -> None:
    rotor_omega = np.ones(4, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="rotor_spin_directions must contain only finite values",
    ):
        reaction_moment_body_from_rotor_speeds(
            rotor_omega,
            rotor_spin_directions,
            1.0,
        )


@pytest.mark.parametrize(
    "rotor_spin_directions",
    [
        np.array([0.0, -1.0, 1.0, -1.0], dtype=np.float64),
        np.array([2.0, -1.0, 1.0, -1.0], dtype=np.float64),
        np.array([-2.0, -1.0, 1.0, -1.0], dtype=np.float64),
    ],
)
def test_reaction_moment_body_from_rotor_speeds_rejects_invalid_directions(
    rotor_spin_directions: NDArray[np.float64],
) -> None:
    rotor_omega = np.ones(4, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"rotor_spin_directions must contain only -1 or \+1",
    ):
        reaction_moment_body_from_rotor_speeds(
            rotor_omega,
            rotor_spin_directions,
            1.0,
        )


@pytest.mark.parametrize("moment_coefficient", [np.nan, np.inf, -np.inf])
def test_reaction_moment_body_from_rotor_speeds_rejects_non_finite_coefficient(
    moment_coefficient: float,
) -> None:
    rotor_omega = np.ones(4, dtype=np.float64)
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    with pytest.raises(ValueError, match="moment_coefficient must be finite"):
        reaction_moment_body_from_rotor_speeds(
            rotor_omega,
            rotor_spin_directions,
            moment_coefficient,
        )


@pytest.mark.parametrize("moment_coefficient", [0.0, -1.0])
def test_reaction_moment_body_from_rotor_speeds_rejects_non_positive_coefficient(
    moment_coefficient: float,
) -> None:
    rotor_omega = np.ones(4, dtype=np.float64)
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    with pytest.raises(ValueError, match="moment_coefficient must be positive"):
        reaction_moment_body_from_rotor_speeds(
            rotor_omega,
            rotor_spin_directions,
            moment_coefficient,
        )


def test_thrust_force_body_from_rotor_thrusts_sums_upward_force() -> None:
    rotor_thrusts = np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float64)

    thrust_force_B = thrust_force_body_from_rotor_thrusts(rotor_thrusts)

    expected_thrust_force_B = np.array([0.0, 0.0, -10.0], dtype=np.float64)
    np.testing.assert_allclose(
        thrust_force_B,
        expected_thrust_force_B,
        atol=1e-12,
    )


def test_thrust_moment_body_from_rotor_thrusts_uses_rotor_offset() -> None:
    rotor_positions_B = np.array(
        [
            [2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_thrusts = np.array([3.0, 0.0, 0.0, 0.0], dtype=np.float64)

    thrust_moment_B = thrust_moment_body_from_rotor_thrusts(
        rotor_positions_B,
        rotor_thrusts,
    )

    expected_thrust_moment_B = np.array([0.0, 6.0, 0.0], dtype=np.float64)
    np.testing.assert_allclose(
        thrust_moment_B,
        expected_thrust_moment_B,
        atol=1e-12,
    )


def test_thrust_moment_body_from_multiple_rotors_produces_roll_and_pitch() -> None:
    rotor_positions_B = np.array(
        [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [-0.5, 0.0, 0.0],
            [0.0, -0.5, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_thrusts = np.array([4.0, 2.0, 1.0, 3.0], dtype=np.float64)

    moment_B = thrust_moment_body_from_rotor_thrusts(
        rotor_positions_B,
        rotor_thrusts,
    )

    expected_moment_B = np.array([0.5, 1.5, 0.0], dtype=np.float64)
    np.testing.assert_allclose(moment_B, expected_moment_B, atol=1e-12)


@pytest.mark.parametrize(
    "rotor_positions_B",
    [
        np.zeros((3, 3), dtype=np.float64),
        np.zeros((4, 2), dtype=np.float64),
        np.zeros((4, 3, 1), dtype=np.float64),
    ],
)
def test_thrust_moment_body_from_rotor_thrusts_rejects_invalid_position_shape(
    rotor_positions_B: NDArray[np.float64],
) -> None:
    rotor_thrusts = np.zeros(4, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match=r"rotor_positions_B must have shape \(4, 3\)",
    ):
        thrust_moment_body_from_rotor_thrusts(rotor_positions_B, rotor_thrusts)


@pytest.mark.parametrize(
    "rotor_thrusts",
    [
        np.zeros(3, dtype=np.float64),
        np.zeros((4, 1), dtype=np.float64),
        np.zeros(5, dtype=np.float64),
    ],
)
def test_thrust_moment_body_from_rotor_thrusts_rejects_invalid_thrust_shape(
    rotor_thrusts: NDArray[np.float64],
) -> None:
    rotor_positions_B = np.zeros((4, 3), dtype=np.float64)

    with pytest.raises(ValueError, match=r"rotor_thrusts must have shape \(4,\)"):
        thrust_moment_body_from_rotor_thrusts(rotor_positions_B, rotor_thrusts)


@pytest.mark.parametrize(
    "rotor_positions_B",
    [
        np.array(
            [[np.nan, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            dtype=np.float64,
        ),
        np.array(
            [[np.inf, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            dtype=np.float64,
        ),
        np.array(
            [[-np.inf, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            dtype=np.float64,
        ),
    ],
)
def test_thrust_moment_body_from_rotor_thrusts_rejects_non_finite_positions(
    rotor_positions_B: NDArray[np.float64],
) -> None:
    rotor_thrusts = np.zeros(4, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="rotor_positions_B must contain only finite values",
    ):
        thrust_moment_body_from_rotor_thrusts(rotor_positions_B, rotor_thrusts)


@pytest.mark.parametrize(
    "rotor_thrusts",
    [
        np.array([np.nan, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([np.inf, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([-np.inf, 0.0, 0.0, 0.0], dtype=np.float64),
    ],
)
def test_thrust_moment_body_from_rotor_thrusts_rejects_non_finite_thrusts(
    rotor_thrusts: NDArray[np.float64],
) -> None:
    rotor_positions_B = np.zeros((4, 3), dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="rotor_thrusts must contain only finite values",
    ):
        thrust_moment_body_from_rotor_thrusts(rotor_positions_B, rotor_thrusts)


def test_thrust_moment_body_from_rotor_thrusts_rejects_negative_thrust() -> None:
    rotor_positions_B = np.zeros((4, 3), dtype=np.float64)
    rotor_thrusts = np.array([0.0, 0.0, -1.0, 0.0], dtype=np.float64)

    with pytest.raises(ValueError, match="rotor_thrusts must be nonnegative"):
        thrust_moment_body_from_rotor_thrusts(rotor_positions_B, rotor_thrusts)


@pytest.mark.parametrize(
    "rotor_thrusts",
    [
        np.zeros(3, dtype=np.float64),
        np.zeros((4, 1), dtype=np.float64),
        np.zeros(5, dtype=np.float64),
    ],
)
def test_thrust_force_body_from_rotor_thrusts_rejects_invalid_shape(
    rotor_thrusts: NDArray[np.float64],
) -> None:
    with pytest.raises(ValueError, match=r"rotor_thrusts must have shape \(4,\)"):
        thrust_force_body_from_rotor_thrusts(rotor_thrusts)


@pytest.mark.parametrize(
    "rotor_thrusts",
    [
        np.array([np.nan, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([np.inf, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([-np.inf, 0.0, 0.0, 0.0], dtype=np.float64),
    ],
)
def test_thrust_force_body_from_rotor_thrusts_rejects_non_finite_values(
    rotor_thrusts: NDArray[np.float64],
) -> None:
    with pytest.raises(
        ValueError,
        match="rotor_thrusts must contain only finite values",
    ):
        thrust_force_body_from_rotor_thrusts(rotor_thrusts)


def test_thrust_force_body_from_rotor_thrusts_rejects_negative_thrust() -> None:
    rotor_thrusts = np.array([1.0, 2.0, -1.0, 4.0], dtype=np.float64)

    with pytest.raises(ValueError, match="rotor_thrusts must be nonnegative"):
        thrust_force_body_from_rotor_thrusts(rotor_thrusts)


@pytest.mark.parametrize(
    "rotor_omega",
    [
        np.zeros(3, dtype=np.float64),
        np.zeros((4, 1), dtype=np.float64),
        np.zeros(5, dtype=np.float64),
    ],
)
def test_rotor_thrusts_from_speeds_rejects_invalid_speed_shape(
    rotor_omega: NDArray[np.float64],
) -> None:
    with pytest.raises(ValueError, match=r"rotor_omega must have shape \(4,\)"):
        rotor_thrusts_from_speeds(rotor_omega, 1.0e-5)


def test_rotor_thrusts_from_speeds_rejects_negative_speed() -> None:
    rotor_omega = np.array([0.0, 100.0, -1.0, 200.0], dtype=np.float64)

    with pytest.raises(ValueError, match="rotor_omega must be nonnegative"):
        rotor_thrusts_from_speeds(rotor_omega, 1.0e-5)


@pytest.mark.parametrize(
    "rotor_omega",
    [
        np.array([np.nan, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([np.inf, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([-np.inf, 0.0, 0.0, 0.0], dtype=np.float64),
    ],
)
def test_rotor_thrusts_from_speeds_rejects_non_finite_speed(
    rotor_omega: NDArray[np.float64],
) -> None:
    with pytest.raises(
        ValueError,
        match="rotor_omega must contain only finite values",
    ):
        rotor_thrusts_from_speeds(rotor_omega, 1.0e-5)


@pytest.mark.parametrize("thrust_coefficient", [np.nan, np.inf, -np.inf])
def test_rotor_thrusts_from_speeds_rejects_non_finite_thrust_coefficient(
    thrust_coefficient: float,
) -> None:
    rotor_omega = np.zeros(4, dtype=np.float64)

    with pytest.raises(ValueError, match="thrust_coefficient must be finite"):
        rotor_thrusts_from_speeds(rotor_omega, thrust_coefficient)


@pytest.mark.parametrize("thrust_coefficient", [0.0, -1.0e-5])
def test_rotor_thrusts_from_speeds_rejects_non_positive_thrust_coefficient(
    thrust_coefficient: float,
) -> None:
    rotor_omega = np.zeros(4, dtype=np.float64)

    with pytest.raises(ValueError, match="thrust_coefficient must be positive"):
        rotor_thrusts_from_speeds(rotor_omega, thrust_coefficient)
