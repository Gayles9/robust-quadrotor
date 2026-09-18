import re

import numpy as np
import pytest
from numpy.typing import NDArray

import quadrotor_math.actuation as actuation
from quadrotor_math.actuation import (
    force_and_moment_body_from_rotor_speeds,
    reaction_moment_body_from_rotor_speeds,
    rotor_thrusts_from_speeds,
    thrust_force_body_from_rotor_thrusts,
    thrust_moment_body_from_rotor_thrusts,
)


def test_allocate_nominal_hover_to_equal_commanded_rotor_speeds() -> None:
    rotor_positions_B = np.array(
        [
            [0.2, 0.2, 0.0],
            [0.2, -0.2, 0.0],
            [-0.2, -0.2, 0.0],
            [-0.2, 0.2, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    commanded_rotor_omega = actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(
        collective_thrust=14.715,
        moment_B=np.zeros(3, dtype=np.float64),
        rotor_positions_B=rotor_positions_B,
        rotor_spin_directions=rotor_spin_directions,
        thrust_coefficient=1.2e-5,
        moment_coefficient=2.0e-7,
        minimum_rotor_omega=0.0,
        maximum_rotor_omega=1000.0,
    )

    np.testing.assert_allclose(
        commanded_rotor_omega,
        np.full(4, np.sqrt(306562.5), dtype=np.float64),
        rtol=0.0,
        atol=1.0e-12,
    )


def test_allocator_round_trips_feasible_mixed_body_wrench() -> None:
    rotor_positions_B = np.array(
        [
            [0.2, 0.2, 0.0],
            [0.2, -0.2, 0.0],
            [-0.2, -0.2, 0.0],
            [-0.2, 0.2, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)
    thrust_coefficient = 1.2e-5
    moment_coefficient = 2.0e-7
    minimum_rotor_omega = 100.0
    maximum_rotor_omega = 1000.0
    expected_commanded_rotor_omega = np.array([400.0, 500.0, 600.0, 700.0], dtype=np.float64)
    collective_thrust = 15.12
    moment_B = np.array([-0.096, -1.056, 0.044], dtype=np.float64)

    commanded_rotor_omega = actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(
        collective_thrust=collective_thrust,
        moment_B=moment_B,
        rotor_positions_B=rotor_positions_B,
        rotor_spin_directions=rotor_spin_directions,
        thrust_coefficient=thrust_coefficient,
        moment_coefficient=moment_coefficient,
        minimum_rotor_omega=minimum_rotor_omega,
        maximum_rotor_omega=maximum_rotor_omega,
    )
    np.testing.assert_allclose(
        commanded_rotor_omega,
        expected_commanded_rotor_omega,
        rtol=0.0,
        atol=1.0e-12,
    )

    force_B, achieved_moment_B = actuation.force_and_moment_body_from_rotor_speeds(
        rotor_omega=commanded_rotor_omega,
        rotor_positions_B=rotor_positions_B,
        rotor_spin_directions=rotor_spin_directions,
        thrust_coefficient=thrust_coefficient,
        moment_coefficient=moment_coefficient,
    )
    np.testing.assert_allclose(
        force_B,
        np.array([0.0, 0.0, -collective_thrust], dtype=np.float64),
        rtol=0.0,
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        achieved_moment_B,
        moment_B,
        rtol=0.0,
        atol=1.0e-12,
    )


def test_allocated_commands_pass_through_motor_lag_before_forward_actuation() -> None:
    rotor_positions_B = np.array(
        [
            [0.2, 0.2, 0.0],
            [0.2, -0.2, 0.0],
            [-0.2, -0.2, 0.0],
            [-0.2, 0.2, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)
    thrust_coefficient = 1.2e-5
    moment_coefficient = 2.0e-7
    minimum_rotor_omega = 100.0
    maximum_rotor_omega = 1000.0
    collective_thrust = 15.12
    desired_moment_B = np.array([-0.096, -1.056, 0.044], dtype=np.float64)
    initial_actual_rotor_omega = np.full(4, 300.0, dtype=np.float64)
    motor_time_constant_s = 0.2
    time_step_s = 0.05

    commanded_rotor_omega = actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(
        collective_thrust=collective_thrust,
        moment_B=desired_moment_B,
        rotor_positions_B=rotor_positions_B,
        rotor_spin_directions=rotor_spin_directions,
        thrust_coefficient=thrust_coefficient,
        moment_coefficient=moment_coefficient,
        minimum_rotor_omega=minimum_rotor_omega,
        maximum_rotor_omega=maximum_rotor_omega,
    )
    np.testing.assert_allclose(
        commanded_rotor_omega,
        np.array([400.0, 500.0, 600.0, 700.0], dtype=np.float64),
        rtol=0.0,
        atol=1.0e-12,
    )

    actual_rotor_omega = actuation.motor_speed_first_order_step(
        actual_rotor_omega=initial_actual_rotor_omega,
        commanded_rotor_omega=commanded_rotor_omega,
        minimum_rotor_omega=minimum_rotor_omega,
        maximum_rotor_omega=maximum_rotor_omega,
        motor_time_constant_s=motor_time_constant_s,
        time_step_s=time_step_s,
    )
    response_factor = np.exp(-time_step_s / motor_time_constant_s)
    expected_actual_rotor_omega = (
        commanded_rotor_omega
        + (initial_actual_rotor_omega - commanded_rotor_omega) * response_factor
    )
    np.testing.assert_allclose(
        actual_rotor_omega,
        expected_actual_rotor_omega,
        rtol=0.0,
        atol=1.0e-12,
    )
    assert np.all(actual_rotor_omega > initial_actual_rotor_omega)
    assert np.all(actual_rotor_omega < commanded_rotor_omega)

    force_B, achieved_moment_B = actuation.force_and_moment_body_from_rotor_speeds(
        rotor_omega=actual_rotor_omega,
        rotor_positions_B=rotor_positions_B,
        rotor_spin_directions=rotor_spin_directions,
        thrust_coefficient=thrust_coefficient,
        moment_coefficient=moment_coefficient,
    )
    expected_squared_rotor_omega = expected_actual_rotor_omega**2
    expected_rotor_thrusts = thrust_coefficient * expected_squared_rotor_omega
    expected_rotor_forces_B = np.column_stack(
        (
            np.zeros(4, dtype=np.float64),
            np.zeros(4, dtype=np.float64),
            -expected_rotor_thrusts,
        )
    )
    expected_force_B = np.sum(expected_rotor_forces_B, axis=0)
    expected_moment_B = np.sum(np.cross(rotor_positions_B, expected_rotor_forces_B), axis=0)
    expected_moment_B[2] += -moment_coefficient * np.sum(
        rotor_spin_directions * expected_squared_rotor_omega
    )
    np.testing.assert_allclose(force_B, expected_force_B, rtol=0.0, atol=1.0e-12)
    np.testing.assert_allclose(
        achieved_moment_B,
        expected_moment_B,
        rtol=0.0,
        atol=1.0e-12,
    )
    assert not np.allclose(
        force_B,
        np.array([0.0, 0.0, -collective_thrust], dtype=np.float64),
        rtol=0.0,
        atol=1.0e-12,
    )
    assert not np.allclose(
        achieved_moment_B,
        desired_moment_B,
        rtol=0.0,
        atol=1.0e-12,
    )


@pytest.mark.parametrize(
    ("field_name", "invalid_value", "expected_message"),
    [
        pytest.param(
            "collective_thrust", np.nan, "collective_thrust must be finite", id="collective-nan"
        ),
        pytest.param(
            "collective_thrust",
            np.inf,
            "collective_thrust must be finite",
            id="collective-positive-infinity",
        ),
        pytest.param(
            "collective_thrust",
            -np.inf,
            "collective_thrust must be finite",
            id="collective-negative-infinity",
        ),
        pytest.param(
            "collective_thrust",
            -1.0,
            "collective_thrust must be nonnegative",
            id="collective-negative",
        ),
        pytest.param(
            "moment_B",
            np.zeros(2, dtype=np.float64),
            "moment_B must have shape (3,)",
            id="moment-shape-two",
        ),
        pytest.param(
            "moment_B",
            np.zeros((3, 1), dtype=np.float64),
            "moment_B must have shape (3,)",
            id="moment-shape-column",
        ),
        pytest.param(
            "moment_B",
            np.zeros(4, dtype=np.float64),
            "moment_B must have shape (3,)",
            id="moment-shape-four",
        ),
        pytest.param(
            "moment_B",
            np.array([np.nan, 0.0, 0.0], dtype=np.float64),
            "moment_B must contain only finite values",
            id="moment-nan",
        ),
        pytest.param(
            "moment_B",
            np.array([np.inf, 0.0, 0.0], dtype=np.float64),
            "moment_B must contain only finite values",
            id="moment-positive-infinity",
        ),
        pytest.param(
            "moment_B",
            np.array([-np.inf, 0.0, 0.0], dtype=np.float64),
            "moment_B must contain only finite values",
            id="moment-negative-infinity",
        ),
        pytest.param(
            "rotor_positions_B",
            np.zeros((3, 3), dtype=np.float64),
            "rotor_positions_B must have shape (4, 3)",
            id="positions-shape-three-rotors",
        ),
        pytest.param(
            "rotor_positions_B",
            np.zeros((4, 2), dtype=np.float64),
            "rotor_positions_B must have shape (4, 3)",
            id="positions-shape-two-coordinates",
        ),
        pytest.param(
            "rotor_positions_B",
            np.zeros((4, 4), dtype=np.float64),
            "rotor_positions_B must have shape (4, 3)",
            id="positions-shape-four-coordinates",
        ),
        pytest.param(
            "rotor_spin_directions",
            np.zeros(3, dtype=np.float64),
            "rotor_spin_directions must have shape (4,)",
            id="spin-shape-three",
        ),
        pytest.param(
            "rotor_spin_directions",
            np.zeros((4, 1), dtype=np.float64),
            "rotor_spin_directions must have shape (4,)",
            id="spin-shape-column",
        ),
        pytest.param(
            "rotor_spin_directions",
            np.zeros(5, dtype=np.float64),
            "rotor_spin_directions must have shape (4,)",
            id="spin-shape-five",
        ),
        pytest.param(
            "rotor_positions_B",
            np.array(
                [[0.2, 0.2, np.nan], [0.2, -0.2, 0.0], [-0.2, -0.2, 0.0], [-0.2, 0.2, 0.0]],
                dtype=np.float64,
            ),
            "rotor_positions_B must contain only finite values",
            id="positions-z-nan",
        ),
        pytest.param(
            "rotor_positions_B",
            np.array(
                [[0.2, 0.2, np.inf], [0.2, -0.2, 0.0], [-0.2, -0.2, 0.0], [-0.2, 0.2, 0.0]],
                dtype=np.float64,
            ),
            "rotor_positions_B must contain only finite values",
            id="positions-z-positive-infinity",
        ),
        pytest.param(
            "rotor_positions_B",
            np.array(
                [[0.2, 0.2, -np.inf], [0.2, -0.2, 0.0], [-0.2, -0.2, 0.0], [-0.2, 0.2, 0.0]],
                dtype=np.float64,
            ),
            "rotor_positions_B must contain only finite values",
            id="positions-z-negative-infinity",
        ),
        pytest.param(
            "rotor_spin_directions",
            np.array([np.nan, -1.0, 1.0, -1.0], dtype=np.float64),
            "rotor_spin_directions must contain only finite values",
            id="spin-nan",
        ),
        pytest.param(
            "rotor_spin_directions",
            np.array([np.inf, -1.0, 1.0, -1.0], dtype=np.float64),
            "rotor_spin_directions must contain only finite values",
            id="spin-positive-infinity",
        ),
        pytest.param(
            "rotor_spin_directions",
            np.array([-np.inf, -1.0, 1.0, -1.0], dtype=np.float64),
            "rotor_spin_directions must contain only finite values",
            id="spin-negative-infinity",
        ),
    ],
)
def test_allocator_rejects_invalid_demand_or_array_input(
    field_name: str,
    invalid_value: object,
    expected_message: str,
) -> None:
    arguments: dict[str, object] = {
        "collective_thrust": 14.715,
        "moment_B": np.zeros(3, dtype=np.float64),
        "rotor_positions_B": np.array(
            [
                [0.2, 0.2, 0.0],
                [0.2, -0.2, 0.0],
                [-0.2, -0.2, 0.0],
                [-0.2, 0.2, 0.0],
            ],
            dtype=np.float64,
        ),
        "rotor_spin_directions": np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64),
        "thrust_coefficient": 1.2e-5,
        "moment_coefficient": 2.0e-7,
        "minimum_rotor_omega": 0.0,
        "maximum_rotor_omega": 1000.0,
    }
    arguments[field_name] = invalid_value

    with pytest.raises(ValueError, match=f"^{re.escape(expected_message)}$"):
        with np.errstate(all="ignore"):
            actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(**arguments)


@pytest.mark.parametrize(
    ("field_name", "invalid_value", "expected_message"),
    [
        pytest.param(
            "rotor_spin_directions",
            np.array([0.0, -1.0, 1.0, -1.0], dtype=np.float64),
            "rotor_spin_directions must contain only -1 or +1",
            id="spin-zero",
        ),
        pytest.param(
            "rotor_spin_directions",
            np.array([0.5, -1.0, 1.0, -1.0], dtype=np.float64),
            "rotor_spin_directions must contain only -1 or +1",
            id="spin-half",
        ),
        pytest.param(
            "rotor_spin_directions",
            np.array([2.0, -1.0, 1.0, -1.0], dtype=np.float64),
            "rotor_spin_directions must contain only -1 or +1",
            id="spin-positive-two",
        ),
        pytest.param(
            "rotor_spin_directions",
            np.array([-2.0, -1.0, 1.0, -1.0], dtype=np.float64),
            "rotor_spin_directions must contain only -1 or +1",
            id="spin-negative-two",
        ),
        pytest.param(
            "thrust_coefficient",
            np.nan,
            "thrust_coefficient must be finite",
            id="thrust-coefficient-nan",
        ),
        pytest.param(
            "thrust_coefficient",
            np.inf,
            "thrust_coefficient must be finite",
            id="thrust-coefficient-positive-infinity",
        ),
        pytest.param(
            "thrust_coefficient",
            -np.inf,
            "thrust_coefficient must be finite",
            id="thrust-coefficient-negative-infinity",
        ),
        pytest.param(
            "thrust_coefficient",
            0.0,
            "thrust_coefficient must be positive",
            id="thrust-coefficient-zero",
        ),
        pytest.param(
            "thrust_coefficient",
            -1.2e-5,
            "thrust_coefficient must be positive",
            id="thrust-coefficient-negative",
        ),
        pytest.param(
            "moment_coefficient",
            np.nan,
            "moment_coefficient must be finite",
            id="moment-coefficient-nan",
        ),
        pytest.param(
            "moment_coefficient",
            np.inf,
            "moment_coefficient must be finite",
            id="moment-coefficient-positive-infinity",
        ),
        pytest.param(
            "moment_coefficient",
            -np.inf,
            "moment_coefficient must be finite",
            id="moment-coefficient-negative-infinity",
        ),
        pytest.param(
            "moment_coefficient",
            0.0,
            "moment_coefficient must be positive",
            id="moment-coefficient-zero",
        ),
        pytest.param(
            "moment_coefficient",
            -2.0e-7,
            "moment_coefficient must be positive",
            id="moment-coefficient-negative",
        ),
        pytest.param(
            "minimum_rotor_omega",
            np.nan,
            "minimum_rotor_omega must be finite",
            id="minimum-speed-nan",
        ),
        pytest.param(
            "minimum_rotor_omega",
            np.inf,
            "minimum_rotor_omega must be finite",
            id="minimum-speed-positive-infinity",
        ),
        pytest.param(
            "minimum_rotor_omega",
            -np.inf,
            "minimum_rotor_omega must be finite",
            id="minimum-speed-negative-infinity",
        ),
        pytest.param(
            "minimum_rotor_omega",
            -1.0,
            "minimum_rotor_omega must be nonnegative",
            id="minimum-speed-negative",
        ),
        pytest.param(
            "maximum_rotor_omega",
            np.nan,
            "maximum_rotor_omega must be finite",
            id="maximum-speed-nan",
        ),
        pytest.param(
            "maximum_rotor_omega",
            np.inf,
            "maximum_rotor_omega must be finite",
            id="maximum-speed-positive-infinity",
        ),
        pytest.param(
            "maximum_rotor_omega",
            -np.inf,
            "maximum_rotor_omega must be finite",
            id="maximum-speed-negative-infinity",
        ),
        pytest.param(
            "maximum_rotor_omega",
            0.0,
            "maximum_rotor_omega must be greater than minimum_rotor_omega",
            id="maximum-speed-equals-minimum",
        ),
        pytest.param(
            "maximum_rotor_omega",
            -1.0,
            "maximum_rotor_omega must be greater than minimum_rotor_omega",
            id="maximum-speed-below-minimum",
        ),
    ],
)
def test_allocator_rejects_invalid_physical_parameter(
    field_name: str,
    invalid_value: object,
    expected_message: str,
) -> None:
    arguments: dict[str, object] = {
        "collective_thrust": 14.715,
        "moment_B": np.zeros(3, dtype=np.float64),
        "rotor_positions_B": np.array(
            [
                [0.2, 0.2, 0.0],
                [0.2, -0.2, 0.0],
                [-0.2, -0.2, 0.0],
                [-0.2, 0.2, 0.0],
            ],
            dtype=np.float64,
        ),
        "rotor_spin_directions": np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64),
        "thrust_coefficient": 1.2e-5,
        "moment_coefficient": 2.0e-7,
        "minimum_rotor_omega": 0.0,
        "maximum_rotor_omega": 1000.0,
    }
    arguments[field_name] = invalid_value

    with pytest.raises(ValueError, match=f"^{re.escape(expected_message)}$"):
        with np.errstate(all="ignore"):
            actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(**arguments)


@pytest.mark.parametrize(
    "maximum_rotor_omega",
    [
        pytest.param(1.0e200, id="python-float"),
        pytest.param(np.float64(1.0e200), id="numpy-float64"),
    ],
)
def test_allocator_rejects_maximum_speed_too_large_to_square_safely(
    maximum_rotor_omega: float,
) -> None:
    rotor_positions_B = np.array(
        [
            [0.2, 0.2, 0.0],
            [0.2, -0.2, 0.0],
            [-0.2, -0.2, 0.0],
            [-0.2, 0.2, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    with pytest.raises(
        ValueError,
        match="^maximum_rotor_omega must be small enough to square safely$",
    ):
        with np.errstate(over="ignore", invalid="ignore"):
            actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(
                collective_thrust=14.715,
                moment_B=np.zeros(3, dtype=np.float64),
                rotor_positions_B=rotor_positions_B,
                rotor_spin_directions=rotor_spin_directions,
                thrust_coefficient=1.2e-5,
                moment_coefficient=2.0e-7,
                minimum_rotor_omega=0.0,
                maximum_rotor_omega=maximum_rotor_omega,
            )


def test_allocator_accepts_largest_safely_squareable_maximum_without_warning() -> None:
    rotor_positions_B = np.array(
        [
            [0.2, 0.2, 0.0],
            [0.2, -0.2, 0.0],
            [-0.2, -0.2, 0.0],
            [-0.2, 0.2, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )
    maximum_safe_rotor_omega = np.sqrt(np.finfo(np.float64).max)
    expected_commanded_rotor_omega = np.full(
        4,
        np.sqrt(306562.5),
        dtype=np.float64,
    )

    with np.errstate(over="raise", invalid="raise"):
        commanded_rotor_omega = (
            actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(
                collective_thrust=14.715,
                moment_B=np.zeros(3, dtype=np.float64),
                rotor_positions_B=rotor_positions_B,
                rotor_spin_directions=rotor_spin_directions,
                thrust_coefficient=1.2e-5,
                moment_coefficient=2.0e-7,
                minimum_rotor_omega=0.0,
                maximum_rotor_omega=maximum_safe_rotor_omega,
            )
        )

    np.testing.assert_allclose(
        commanded_rotor_omega,
        expected_commanded_rotor_omega,
        rtol=0.0,
        atol=1.0e-12,
    )
    assert np.all(np.isfinite(commanded_rotor_omega))
    assert np.all(commanded_rotor_omega >= 0.0)
    assert np.all(commanded_rotor_omega <= maximum_safe_rotor_omega)


@pytest.mark.parametrize(
    ("rotor_positions_B", "rotor_spin_directions"),
    [
        pytest.param(
            np.array(
                [
                    [0.2, 0.2, 0.0],
                    [0.2, -0.2, 0.0],
                    [-0.2, -0.2, 0.0],
                    [-0.2, 0.2, 0.0],
                ],
                dtype=np.float64,
            ),
            np.ones(4, dtype=np.float64),
            id="no-yaw-authority",
        ),
        pytest.param(
            np.zeros((4, 3), dtype=np.float64),
            np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64),
            id="no-roll-or-pitch-authority",
        ),
    ],
)
def test_allocator_rejects_singular_allocation_matrix(
    rotor_positions_B: NDArray[np.float64],
    rotor_spin_directions: NDArray[np.float64],
) -> None:
    thrust_coefficient = 1.2e-5
    moment_coefficient = 2.0e-7
    allocation_matrix = np.array(
        [
            np.full(4, thrust_coefficient),
            -rotor_positions_B[:, 1] * thrust_coefficient,
            rotor_positions_B[:, 0] * thrust_coefficient,
            -rotor_spin_directions * moment_coefficient,
        ],
        dtype=np.float64,
    )
    assert np.linalg.matrix_rank(allocation_matrix) < 4

    with pytest.raises(ValueError, match="^allocation matrix must be nonsingular$"):
        actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(
            collective_thrust=14.715,
            moment_B=np.zeros(3, dtype=np.float64),
            rotor_positions_B=rotor_positions_B,
            rotor_spin_directions=rotor_spin_directions,
            thrust_coefficient=thrust_coefficient,
            moment_coefficient=moment_coefficient,
            minimum_rotor_omega=0.0,
            maximum_rotor_omega=1000.0,
        )


@pytest.mark.parametrize(
    "expected_squared_rotor_omega",
    [
        pytest.param(
            np.array([-10_000.0, 250_000.0, 640_000.0, 1_000_000.0], dtype=np.float64),
            id="negative-squared-speed",
        ),
        pytest.param(
            np.array([2_500.0, 250_000.0, 640_000.0, 1_000_000.0], dtype=np.float64),
            id="below-positive-minimum",
        ),
        pytest.param(
            np.array([10_000.0, 250_000.0, 640_000.0, 1_210_000.0], dtype=np.float64),
            id="above-maximum",
        ),
    ],
)
def test_allocator_rejects_infeasible_demand(
    expected_squared_rotor_omega: NDArray[np.float64],
) -> None:
    rotor_positions_B = np.array(
        [
            [0.2, 0.2, 0.0],
            [0.2, -0.2, 0.0],
            [-0.2, -0.2, 0.0],
            [-0.2, 0.2, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)
    thrust_coefficient = 1.2e-5
    moment_coefficient = 2.0e-7
    allocation_matrix = np.array(
        [
            np.full(4, thrust_coefficient),
            -rotor_positions_B[:, 1] * thrust_coefficient,
            rotor_positions_B[:, 0] * thrust_coefficient,
            -rotor_spin_directions * moment_coefficient,
        ],
        dtype=np.float64,
    )
    demand = allocation_matrix @ expected_squared_rotor_omega

    with pytest.raises(
        ValueError,
        match=(
            "^requested collective thrust and body moment are infeasible within rotor speed limits$"
        ),
    ):
        with np.errstate(all="ignore"):
            actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(
                collective_thrust=float(demand[0]),
                moment_B=demand[1:],
                rotor_positions_B=rotor_positions_B,
                rotor_spin_directions=rotor_spin_directions,
                thrust_coefficient=thrust_coefficient,
                moment_coefficient=moment_coefficient,
                minimum_rotor_omega=100.0,
                maximum_rotor_omega=1000.0,
            )


def test_allocator_rejects_infeasible_lower_bound_with_extreme_unused_maximum() -> None:
    rotor_positions_B = np.array(
        [
            [0.2, 0.2, 0.0],
            [0.2, -0.2, 0.0],
            [-0.2, -0.2, 0.0],
            [-0.2, 0.2, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)
    thrust_coefficient = 1.2e-5
    moment_coefficient = 2.0e-7
    minimum_rotor_omega = 100.0
    maximum_rotor_omega = np.sqrt(np.finfo(np.float64).max)
    allocation_matrix = np.array(
        [
            np.full(4, thrust_coefficient),
            -rotor_positions_B[:, 1] * thrust_coefficient,
            rotor_positions_B[:, 0] * thrust_coefficient,
            -rotor_spin_directions * moment_coefficient,
        ],
        dtype=np.float64,
    )
    infeasible_squared_rotor_omega = np.array(
        [-10_000.0, 250_000.0, 640_000.0, 1_000_000.0],
        dtype=np.float64,
    )
    demand = allocation_matrix @ infeasible_squared_rotor_omega
    recovered_squared_rotor_omega = np.linalg.solve(allocation_matrix, demand)
    np.testing.assert_allclose(
        recovered_squared_rotor_omega,
        infeasible_squared_rotor_omega,
        rtol=0.0,
        atol=1.0e-9,
    )
    assert recovered_squared_rotor_omega[0] < 0.0
    assert recovered_squared_rotor_omega[0] < minimum_rotor_omega**2

    with pytest.raises(
        ValueError,
        match=(
            "^requested collective thrust and body moment are infeasible within rotor speed limits$"
        ),
    ):
        with np.errstate(over="raise", invalid="raise"):
            actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(
                collective_thrust=float(demand[0]),
                moment_B=demand[1:],
                rotor_positions_B=rotor_positions_B,
                rotor_spin_directions=rotor_spin_directions,
                thrust_coefficient=thrust_coefficient,
                moment_coefficient=moment_coefficient,
                minimum_rotor_omega=minimum_rotor_omega,
                maximum_rotor_omega=maximum_rotor_omega,
            )


@pytest.mark.parametrize(
    ("minimum_rotor_omega", "expected_squared_rotor_omega"),
    [
        pytest.param(
            100.0,
            np.array([10_000.0, 10_000.0, 10_000.0, 10_000.0], dtype=np.float64),
            id="all-minimum",
        ),
        pytest.param(
            100.0,
            np.array([1_000_000.0, 1_000_000.0, 1_000_000.0, 1_000_000.0], dtype=np.float64),
            id="all-maximum",
        ),
        pytest.param(
            100.0,
            np.array([10_000.0, 1_000_000.0, 10_000.0, 1_000_000.0], dtype=np.float64),
            id="mixed-limits",
        ),
        pytest.param(
            100.0,
            np.array([10_000.0, 250_000.0, 640_000.0, 1_000_000.0], dtype=np.float64),
            id="mixed-general",
        ),
        pytest.param(
            0.0,
            np.array([0.0, 640_000.0, 250_000.0, 1_000_000.0], dtype=np.float64),
            id="zero-minimum",
        ),
    ],
)
def test_allocator_preserves_feasible_rotor_speed_boundaries(
    minimum_rotor_omega: float,
    expected_squared_rotor_omega: NDArray[np.float64],
) -> None:
    maximum_rotor_omega = 1000.0
    rotor_positions_B = np.array(
        [
            [0.2, 0.2, 0.0],
            [0.2, -0.2, 0.0],
            [-0.2, -0.2, 0.0],
            [-0.2, 0.2, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array([1.0, -1.0, 1.0, -1.0], dtype=np.float64)
    thrust_coefficient = 1.2e-5
    moment_coefficient = 2.0e-7
    allocation_matrix = np.array(
        [
            np.full(4, thrust_coefficient),
            -rotor_positions_B[:, 1] * thrust_coefficient,
            rotor_positions_B[:, 0] * thrust_coefficient,
            -rotor_spin_directions * moment_coefficient,
        ],
        dtype=np.float64,
    )
    demand = allocation_matrix @ expected_squared_rotor_omega

    with np.errstate(all="ignore"):
        commanded_rotor_omega = (
            actuation.commanded_rotor_speeds_from_collective_thrust_and_body_moment(
                collective_thrust=float(demand[0]),
                moment_B=demand[1:],
                rotor_positions_B=rotor_positions_B,
                rotor_spin_directions=rotor_spin_directions,
                thrust_coefficient=thrust_coefficient,
                moment_coefficient=moment_coefficient,
                minimum_rotor_omega=minimum_rotor_omega,
                maximum_rotor_omega=maximum_rotor_omega,
            )
        )

    assert np.all(np.isfinite(commanded_rotor_omega))
    assert np.all(commanded_rotor_omega >= minimum_rotor_omega)
    assert np.all(commanded_rotor_omega <= maximum_rotor_omega)
    np.testing.assert_allclose(
        commanded_rotor_omega,
        np.sqrt(expected_squared_rotor_omega),
        rtol=0.0,
        atol=1.0e-12,
    )


def test_motor_speed_step_uses_saturated_target_and_preserves_lag() -> None:
    actual_rotor_omega = np.zeros(4, dtype=np.float64)
    commanded_rotor_omega = np.array(
        [200.0, 50.0, 50.0, 50.0],
        dtype=np.float64,
    )
    minimum_rotor_omega = 0.0
    maximum_rotor_omega = 100.0
    motor_time_constant_s = 0.2
    time_step_s = 0.1

    next_actual_rotor_omega = actuation.motor_speed_first_order_step(
        actual_rotor_omega,
        commanded_rotor_omega,
        minimum_rotor_omega,
        maximum_rotor_omega,
        motor_time_constant_s,
        time_step_s,
    )

    response_fraction = 1.0 - np.exp(-time_step_s / motor_time_constant_s)
    expected_next_actual_rotor_omega = (
        np.array(
            [100.0, 50.0, 50.0, 50.0],
            dtype=np.float64,
        )
        * response_fraction
    )

    np.testing.assert_allclose(
        next_actual_rotor_omega,
        expected_next_actual_rotor_omega,
        rtol=0.0,
        atol=1.0e-12,
    )
    assert next_actual_rotor_omega[0] < maximum_rotor_omega
    assert next_actual_rotor_omega[0] < commanded_rotor_omega[0]


def test_motor_speed_first_order_step_zero_time_step_preserves_actual_speed() -> None:
    actual_rotor_omega = np.array(
        [0.0, 0.1, 75.0, 100.0],
        dtype=np.float64,
    )
    commanded_rotor_omega = np.array(
        [1000.0, 100.0, 50.0, 25.0],
        dtype=np.float64,
    )

    next_actual_rotor_omega = actuation.motor_speed_first_order_step(
        actual_rotor_omega,
        commanded_rotor_omega,
        minimum_rotor_omega=0.0,
        maximum_rotor_omega=100.0,
        motor_time_constant_s=0.2,
        time_step_s=0.0,
    )

    np.testing.assert_array_equal(next_actual_rotor_omega, actual_rotor_omega)


@pytest.mark.parametrize(
    ("actual_rotor_omega", "commanded_rotor_omega", "expected_message"),
    [
        pytest.param(
            np.zeros(3, dtype=np.float64),
            np.zeros(4, dtype=np.float64),
            r"actual_rotor_omega must have shape \(4,\)",
            id="actual-three",
        ),
        pytest.param(
            np.zeros((4, 1), dtype=np.float64),
            np.zeros(4, dtype=np.float64),
            r"actual_rotor_omega must have shape \(4,\)",
            id="actual-column",
        ),
        pytest.param(
            np.zeros(5, dtype=np.float64),
            np.zeros(4, dtype=np.float64),
            r"actual_rotor_omega must have shape \(4,\)",
            id="actual-five",
        ),
        pytest.param(
            np.zeros(4, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            r"commanded_rotor_omega must have shape \(4,\)",
            id="commanded-three",
        ),
        pytest.param(
            np.zeros(4, dtype=np.float64),
            np.zeros((4, 1), dtype=np.float64),
            r"commanded_rotor_omega must have shape \(4,\)",
            id="commanded-column",
        ),
        pytest.param(
            np.zeros(4, dtype=np.float64),
            np.zeros(5, dtype=np.float64),
            r"commanded_rotor_omega must have shape \(4,\)",
            id="commanded-five",
        ),
    ],
)
def test_motor_speed_first_order_step_rejects_invalid_speed_shape(
    actual_rotor_omega: NDArray[np.float64],
    commanded_rotor_omega: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        actuation.motor_speed_first_order_step(
            actual_rotor_omega,
            commanded_rotor_omega,
            0.0,
            100.0,
            0.2,
            0.1,
        )


@pytest.mark.parametrize(
    (
        "actual_rotor_omega",
        "commanded_rotor_omega",
        "minimum_rotor_omega",
        "maximum_rotor_omega",
        "expected_message",
    ),
    [
        pytest.param(
            np.array([np.nan, 50.0, 50.0, 50.0], dtype=np.float64),
            np.full(4, 50.0, dtype=np.float64),
            0.0,
            100.0,
            "actual_rotor_omega must contain only finite values",
            id="actual-nan",
        ),
        pytest.param(
            np.array([np.inf, 50.0, 50.0, 50.0], dtype=np.float64),
            np.full(4, 50.0, dtype=np.float64),
            0.0,
            100.0,
            "actual_rotor_omega must contain only finite values",
            id="actual-positive-infinity",
        ),
        pytest.param(
            np.array([-np.inf, 50.0, 50.0, 50.0], dtype=np.float64),
            np.full(4, 50.0, dtype=np.float64),
            0.0,
            100.0,
            "actual_rotor_omega must contain only finite values",
            id="actual-negative-infinity",
        ),
        pytest.param(
            np.full(4, 50.0, dtype=np.float64),
            np.array([np.nan, 50.0, 50.0, 50.0], dtype=np.float64),
            0.0,
            100.0,
            "commanded_rotor_omega must contain only finite values",
            id="commanded-nan",
        ),
        pytest.param(
            np.full(4, 50.0, dtype=np.float64),
            np.array([np.inf, 50.0, 50.0, 50.0], dtype=np.float64),
            0.0,
            100.0,
            "commanded_rotor_omega must contain only finite values",
            id="commanded-positive-infinity",
        ),
        pytest.param(
            np.full(4, 50.0, dtype=np.float64),
            np.array([-np.inf, 50.0, 50.0, 50.0], dtype=np.float64),
            0.0,
            100.0,
            "commanded_rotor_omega must contain only finite values",
            id="commanded-negative-infinity",
        ),
        pytest.param(
            np.array([9.0, 50.0, 50.0, 50.0], dtype=np.float64),
            np.full(4, 50.0, dtype=np.float64),
            10.0,
            100.0,
            "actual_rotor_omega must be within rotor speed limits",
            id="actual-below-minimum",
        ),
        pytest.param(
            np.array([101.0, 50.0, 50.0, 50.0], dtype=np.float64),
            np.full(4, 50.0, dtype=np.float64),
            0.0,
            100.0,
            "actual_rotor_omega must be within rotor speed limits",
            id="actual-above-maximum",
        ),
        pytest.param(
            np.full(4, 50.0, dtype=np.float64),
            np.array([-1.0, 50.0, 50.0, 50.0], dtype=np.float64),
            0.0,
            100.0,
            "commanded_rotor_omega must be nonnegative",
            id="commanded-negative",
        ),
    ],
)
def test_motor_speed_first_order_step_rejects_invalid_speed_values(
    actual_rotor_omega: NDArray[np.float64],
    commanded_rotor_omega: NDArray[np.float64],
    minimum_rotor_omega: float,
    maximum_rotor_omega: float,
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        actuation.motor_speed_first_order_step(
            actual_rotor_omega,
            commanded_rotor_omega,
            minimum_rotor_omega,
            maximum_rotor_omega,
            0.2,
            0.1,
        )


@pytest.mark.parametrize(
    (
        "minimum_rotor_omega",
        "maximum_rotor_omega",
        "motor_time_constant_s",
        "time_step_s",
        "expected_message",
    ),
    [
        pytest.param(
            np.nan, 100.0, 0.2, 0.1, "minimum_rotor_omega must be finite", id="minimum-nan"
        ),
        pytest.param(
            np.inf,
            100.0,
            0.2,
            0.1,
            "minimum_rotor_omega must be finite",
            id="minimum-positive-infinity",
        ),
        pytest.param(
            -np.inf,
            100.0,
            0.2,
            0.1,
            "minimum_rotor_omega must be finite",
            id="minimum-negative-infinity",
        ),
        pytest.param(
            -1.0, 100.0, 0.2, 0.1, "minimum_rotor_omega must be nonnegative", id="minimum-negative"
        ),
        pytest.param(0.0, np.nan, 0.2, 0.1, "maximum_rotor_omega must be finite", id="maximum-nan"),
        pytest.param(
            0.0,
            np.inf,
            0.2,
            0.1,
            "maximum_rotor_omega must be finite",
            id="maximum-positive-infinity",
        ),
        pytest.param(
            0.0,
            -np.inf,
            0.2,
            0.1,
            "maximum_rotor_omega must be finite",
            id="maximum-negative-infinity",
        ),
        pytest.param(
            0.0,
            0.0,
            0.2,
            0.1,
            "maximum_rotor_omega must be greater than minimum_rotor_omega",
            id="maximum-equals-minimum",
        ),
        pytest.param(
            0.0,
            -1.0,
            0.2,
            0.1,
            "maximum_rotor_omega must be greater than minimum_rotor_omega",
            id="maximum-below-minimum",
        ),
        pytest.param(
            0.0, 100.0, np.nan, 0.1, "motor_time_constant_s must be finite", id="time-constant-nan"
        ),
        pytest.param(
            0.0,
            100.0,
            np.inf,
            0.1,
            "motor_time_constant_s must be finite",
            id="time-constant-positive-infinity",
        ),
        pytest.param(
            0.0,
            100.0,
            -np.inf,
            0.1,
            "motor_time_constant_s must be finite",
            id="time-constant-negative-infinity",
        ),
        pytest.param(
            0.0,
            100.0,
            np.float64(0.0),
            0.1,
            "motor_time_constant_s must be positive",
            id="time-constant-zero",
        ),
        pytest.param(
            0.0,
            100.0,
            -0.2,
            0.1,
            "motor_time_constant_s must be positive",
            id="time-constant-negative",
        ),
        pytest.param(0.0, 100.0, 0.2, np.nan, "time_step_s must be finite", id="time-step-nan"),
        pytest.param(
            0.0, 100.0, 0.2, np.inf, "time_step_s must be finite", id="time-step-positive-infinity"
        ),
        pytest.param(
            0.0, 100.0, 0.2, -np.inf, "time_step_s must be finite", id="time-step-negative-infinity"
        ),
        pytest.param(
            0.0, 100.0, 0.2, -0.1, "time_step_s must be nonnegative", id="time-step-negative"
        ),
    ],
)
def test_motor_speed_first_order_step_rejects_invalid_scalar_parameter(
    minimum_rotor_omega: float,
    maximum_rotor_omega: float,
    motor_time_constant_s: float,
    time_step_s: float,
    expected_message: str,
) -> None:
    actual_rotor_omega = np.full(4, 50.0, dtype=np.float64)
    commanded_rotor_omega = np.full(4, 50.0, dtype=np.float64)

    with pytest.raises(ValueError, match=f"^{expected_message}$"):
        actuation.motor_speed_first_order_step(
            actual_rotor_omega,
            commanded_rotor_omega,
            minimum_rotor_omega,
            maximum_rotor_omega,
            motor_time_constant_s,
            time_step_s,
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


def test_force_and_moment_body_from_rotor_speeds_combines_actuation_effects() -> None:
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

    force_B, moment_B = force_and_moment_body_from_rotor_speeds(
        rotor_omega,
        rotor_positions_B,
        rotor_spin_directions,
        thrust_coefficient=0.75,
        moment_coefficient=0.5,
    )

    expected_force_B = np.array([0.0, 0.0, -3.0], dtype=np.float64)
    expected_moment_B = np.array([0.0, 6.0, -2.0], dtype=np.float64)
    np.testing.assert_allclose(force_B, expected_force_B, atol=1e-12)
    np.testing.assert_allclose(moment_B, expected_moment_B, atol=1e-12)


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
