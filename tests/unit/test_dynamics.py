import re
import warnings

import numpy as np
import pytest
from numpy.typing import NDArray

import quadrotor_math.dynamics as dynamics
from quadrotor_math.dynamics import (
    angular_acceleration_body_from_moment,
    rigid_body_state_derivative_from_body_wrench,
    rigid_body_state_derivative_from_rotor_speeds,
    translational_acceleration_world_from_body_force,
)


def _level_hover_rotor_derivative(
    *,
    wind_velocity_W: NDArray[np.float64] | None = None,
    quadratic_drag_coefficient_B: NDArray[np.float64] | None = None,
) -> tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
]:
    environment = {}
    if wind_velocity_W is not None:
        environment["wind_velocity_W"] = wind_velocity_W
    if quadratic_drag_coefficient_B is not None:
        environment["quadratic_drag_coefficient_B"] = quadratic_drag_coefficient_B
    return rigid_body_state_derivative_from_rotor_speeds(
        np.zeros(3, dtype=np.float64),
        np.array([2.0, 0.0, 0.0], dtype=np.float64),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float64),
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
        **environment,
    )


def test_rigid_body_state_derivative_combines_kinematics_and_dynamics() -> None:
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.array([0.0, 0.0, 2.0], dtype=np.float64)

    force_B = np.array([0.0, 0.0, -19.62], dtype=np.float64)
    moment_B = np.array([2.0, 0.0, 0.0], dtype=np.float64)

    mass = 2.0
    inertia_B = np.diag(
        np.array([2.0, 3.0, 4.0], dtype=np.float64),
    )
    gravity_acceleration = 9.81

    (
        position_derivative_W,
        velocity_derivative_W,
        quaternion_derivative_WB,
        angular_velocity_derivative_B,
    ) = rigid_body_state_derivative_from_body_wrench(
        position_W,
        velocity_W,
        q_WB,
        omega_B,
        force_B,
        moment_B,
        mass,
        inertia_B,
        gravity_acceleration,
    )

    expected_position_derivative_W = np.array(
        [1.0, -2.0, 3.0],
        dtype=np.float64,
    )
    expected_velocity_derivative_W = np.zeros(
        3,
        dtype=np.float64,
    )
    expected_quaternion_derivative_WB = np.array(
        [0.0, 0.0, 0.0, 1.0],
        dtype=np.float64,
    )
    expected_angular_velocity_derivative_B = np.array(
        [1.0, 0.0, 0.0],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        position_derivative_W,
        expected_position_derivative_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        velocity_derivative_W,
        expected_velocity_derivative_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        quaternion_derivative_WB,
        expected_quaternion_derivative_WB,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        angular_velocity_derivative_B,
        expected_angular_velocity_derivative_B,
        atol=1e-12,
    )


def test_rigid_body_state_derivative_from_rotor_speeds_combines_actuation_and_dynamics() -> None:
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.array([1.0, -2.0, 3.0], dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)

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

    (
        position_derivative_W,
        velocity_derivative_W,
        quaternion_derivative_WB,
        angular_velocity_derivative_B,
    ) = rigid_body_state_derivative_from_rotor_speeds(
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
    )

    expected_position_derivative_W = np.array(
        [1.0, -2.0, 3.0],
        dtype=np.float64,
    )
    expected_velocity_derivative_W = np.array(
        [0.0, 0.0, 6.81],
        dtype=np.float64,
    )
    expected_quaternion_derivative_WB = np.zeros(4, dtype=np.float64)
    expected_angular_velocity_derivative_B = np.array(
        [0.0, 2.0, -0.5],
        dtype=np.float64,
    )

    np.testing.assert_allclose(
        position_derivative_W,
        expected_position_derivative_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        velocity_derivative_W,
        expected_velocity_derivative_W,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        quaternion_derivative_WB,
        expected_quaternion_derivative_WB,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        angular_velocity_derivative_B,
        expected_angular_velocity_derivative_B,
        atol=1e-12,
    )


def test_rigid_body_state_derivative_from_rotor_speeds_is_zero_at_balanced_hover() -> None:
    position_W = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    velocity_W = np.zeros(3, dtype=np.float64)
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)

    mass = 1.0
    inertia_B = np.diag(np.array([2.0, 3.0, 4.0], dtype=np.float64))
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5

    rotor_positions_B = np.array(
        [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [-0.5, 0.0, 0.0],
            [0.0, -0.5, 0.0],
        ],
        dtype=np.float64,
    )
    rotor_spin_directions = np.array(
        [1.0, -1.0, 1.0, -1.0],
        dtype=np.float64,
    )

    hover_rotor_speed = np.sqrt(mass * gravity_acceleration / (4.0 * thrust_coefficient))
    rotor_omega = np.full(4, hover_rotor_speed, dtype=np.float64)

    (
        position_derivative_W,
        velocity_derivative_W,
        quaternion_derivative_WB,
        angular_velocity_derivative_B,
    ) = rigid_body_state_derivative_from_rotor_speeds(
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
    )

    np.testing.assert_allclose(
        position_derivative_W,
        np.zeros(3, dtype=np.float64),
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        velocity_derivative_W,
        np.zeros(3, dtype=np.float64),
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        quaternion_derivative_WB,
        np.zeros(4, dtype=np.float64),
        rtol=0.0,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        angular_velocity_derivative_B,
        np.zeros(3, dtype=np.float64),
        rtol=0.0,
        atol=1e-12,
    )


def test_rotor_speed_derivative_adds_quadratic_drag_force_without_a_moment() -> None:
    no_drag = _level_hover_rotor_derivative()
    calm_air = _level_hover_rotor_derivative(
        wind_velocity_W=np.zeros(3, dtype=np.float64),
        quadratic_drag_coefficient_B=np.array([0.5, 0.0, 0.0], dtype=np.float64),
    )
    following_wind = _level_hover_rotor_derivative(
        wind_velocity_W=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        quadratic_drag_coefficient_B=np.array([0.5, 0.0, 0.0], dtype=np.float64),
    )

    np.testing.assert_array_equal(calm_air[1], np.array([-2.0, 0.0, 0.0]))
    np.testing.assert_array_equal(following_wind[1], np.array([-0.5, 0.0, 0.0]))
    np.testing.assert_array_equal(calm_air[3], no_drag[3])
    np.testing.assert_array_equal(following_wind[3], no_drag[3])


def test_rotor_speed_derivative_zero_environment_preserves_historical_result_exactly() -> None:
    omitted = _level_hover_rotor_derivative()
    explicit_zero = _level_hover_rotor_derivative(
        wind_velocity_W=np.zeros(3, dtype=np.float64),
        quadratic_drag_coefficient_B=np.zeros(3, dtype=np.float64),
    )

    for omitted_array, explicit_array in zip(omitted, explicit_zero, strict=True):
        np.testing.assert_array_equal(explicit_array, omitted_array)


def test_rotor_speed_derivative_zero_drag_coefficient_validates_wind() -> None:
    omitted = _level_hover_rotor_derivative()
    wind_only = _level_hover_rotor_derivative(
        wind_velocity_W=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        quadratic_drag_coefficient_B=np.zeros(3, dtype=np.float64),
    )
    for omitted_array, wind_only_array in zip(omitted, wind_only, strict=True):
        np.testing.assert_array_equal(wind_only_array, omitted_array)

    with pytest.raises(ValueError, match=r"^wind_velocity_W must have shape \(3,\)$"):
        _level_hover_rotor_derivative(
            wind_velocity_W=np.zeros(2, dtype=np.float64),
            quadratic_drag_coefficient_B=np.zeros(3, dtype=np.float64),
        )


@pytest.mark.parametrize(
    ("environment", "expected_message"),
    [
        (
            {"wind_velocity_W": np.zeros(2, dtype=np.float64)},
            r"wind_velocity_W must have shape \(3,\)",
        ),
        (
            {"wind_velocity_W": np.array([np.nan, 0.0, 0.0])},
            "wind_velocity_W must contain only finite values",
        ),
        (
            {"quadratic_drag_coefficient_B": np.zeros(2, dtype=np.float64)},
            r"quadratic_drag_coefficient_B must have shape \(3,\)",
        ),
        (
            {"quadratic_drag_coefficient_B": np.array([-1.0, 0.0, 0.0])},
            "quadratic_drag_coefficient_B must be nonnegative",
        ),
    ],
)
def test_rotor_speed_derivative_uses_quadratic_drag_input_validation(
    environment: dict[str, NDArray[np.float64]], expected_message: str
) -> None:
    with pytest.raises(ValueError, match=rf"^{expected_message}$"):
        _level_hover_rotor_derivative(**environment)


def test_rotor_speed_derivative_validates_rotors_before_environment() -> None:
    with pytest.raises(ValueError, match=r"^rotor_omega must have shape \(4,\)"):
        rigid_body_state_derivative_from_rotor_speeds(
            np.zeros(3),
            np.zeros(3),
            np.array([1.0, 0.0, 0.0, 0.0]),
            np.zeros(3),
            np.zeros(3),
            np.zeros((4, 3)),
            np.array([1.0, -1.0, 1.0, -1.0]),
            1.0,
            np.eye(3),
            10.0,
            1.0e-5,
            1.0e-6,
            wind_velocity_W=np.zeros(2),
        )


def test_rotor_speed_derivative_rejects_combined_force_overflow_without_warning(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    maximum = np.finfo(np.float64).max
    monkeypatch.setattr(
        dynamics,
        "force_and_moment_body_from_rotor_speeds",
        lambda *_args: (np.full(3, maximum), np.zeros(3)),
    )
    monkeypatch.setattr(
        dynamics,
        "quadratic_drag_force_body",
        lambda *_args: np.full(3, maximum),
    )

    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(
            ValueError,
            match="^combined rotor and quadratic drag force must remain finite$",
        ):
            _level_hover_rotor_derivative(
                wind_velocity_W=np.zeros(3),
                quadratic_drag_coefficient_B=np.ones(3),
            )


@pytest.mark.parametrize(
    ("position_W", "velocity_W", "expected_message"),
    [
        (
            np.zeros(2, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            r"position_W must have shape \(3,\)",
        ),
        (
            np.zeros((3, 1), dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            r"position_W must have shape \(3,\)",
        ),
        (
            np.zeros(4, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            r"position_W must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(2, dtype=np.float64),
            r"velocity_W must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            r"velocity_W must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(4, dtype=np.float64),
            r"velocity_W must have shape \(3,\)",
        ),
    ],
)
def test_rigid_body_state_derivative_rejects_invalid_position_and_velocity_shapes(
    position_W: NDArray[np.float64],
    velocity_W: NDArray[np.float64],
    expected_message: str,
) -> None:
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    force_B = np.zeros(3, dtype=np.float64)
    moment_B = np.zeros(3, dtype=np.float64)
    mass = 1.0
    inertia_B = np.eye(3, dtype=np.float64)
    gravity_acceleration = 9.81

    with pytest.raises(ValueError, match=expected_message):
        rigid_body_state_derivative_from_body_wrench(
            position_W,
            velocity_W,
            q_WB,
            omega_B,
            force_B,
            moment_B,
            mass,
            inertia_B,
            gravity_acceleration,
        )


@pytest.mark.parametrize(
    ("position_W", "velocity_W", "expected_message"),
    [
        (
            np.array([np.nan, 0.0, 0.0], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            "position_W must contain only finite values",
        ),
        (
            np.array([np.inf, 0.0, 0.0], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            "position_W must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.array([np.nan, 0.0, 0.0], dtype=np.float64),
            "velocity_W must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.array([np.inf, 0.0, 0.0], dtype=np.float64),
            "velocity_W must contain only finite values",
        ),
    ],
)
def test_rigid_body_state_derivative_rejects_nonfinite_position_and_velocity(
    position_W: NDArray[np.float64],
    velocity_W: NDArray[np.float64],
    expected_message: str,
) -> None:
    q_WB = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    force_B = np.zeros(3, dtype=np.float64)
    moment_B = np.zeros(3, dtype=np.float64)
    mass = 1.0
    inertia_B = np.eye(3, dtype=np.float64)
    gravity_acceleration = 9.81

    with pytest.raises(ValueError, match=expected_message):
        rigid_body_state_derivative_from_body_wrench(
            position_W,
            velocity_W,
            q_WB,
            omega_B,
            force_B,
            moment_B,
            mass,
            inertia_B,
            gravity_acceleration,
        )


def test_angular_acceleration_body_includes_gyroscopic_coupling() -> None:
    moment_B = np.array([4.0, 5.0, 6.0], dtype=np.float64)
    omega_B = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    inertia_B = np.diag(
        np.array([2.0, 3.0, 4.0], dtype=np.float64),
    )

    angular_acceleration_B = angular_acceleration_body_from_moment(
        moment_B,
        omega_B,
        inertia_B,
    )

    expected_angular_acceleration_B = np.array(
        [-1.0, 11.0 / 3.0, 1.0],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        angular_acceleration_B,
        expected_angular_acceleration_B,
        atol=1e-12,
    )


def test_angular_acceleration_body_from_moment_has_zero_rotational_energy_rate_when_torque_free() -> (  # noqa: E501
    None
):
    moment_B = np.zeros(3, dtype=np.float64)
    omega_B = np.array(
        [0.7, -0.4, 1.1],
        dtype=np.float64,
    )
    inertia_B = np.diag(
        np.array([2.0, 3.0, 4.0], dtype=np.float64),
    )

    omega_dot_B = angular_acceleration_body_from_moment(
        moment_B,
        omega_B,
        inertia_B,
    )

    expected_omega_dot_B = np.array(
        [0.22, 1.54 / 3.0, 0.07],
        dtype=np.float64,
    )
    np.testing.assert_allclose(
        omega_dot_B,
        expected_omega_dot_B,
        rtol=0.0,
        atol=1e-12,
    )

    rotational_energy_rate = omega_B @ inertia_B @ omega_dot_B
    np.testing.assert_allclose(
        rotational_energy_rate,
        0.0,
        rtol=0.0,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    ("moment_B", "omega_B", "inertia_B", "expected_message"),
    [
        (
            np.zeros(2, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"moment_B must have shape \(3,\)",
        ),
        (
            np.zeros((3, 1), dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"moment_B must have shape \(3,\)",
        ),
        (
            np.zeros(4, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"moment_B must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(2, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"omega_B must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"omega_B must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(4, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            r"omega_B must have shape \(3,\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros((2, 2), dtype=np.float64),
            r"inertia_B must have shape \(3, 3\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros((3, 2), dtype=np.float64),
            r"inertia_B must have shape \(3, 3\)",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros((3, 3, 1), dtype=np.float64),
            r"inertia_B must have shape \(3, 3\)",
        ),
    ],
)
def test_angular_acceleration_body_rejects_invalid_shapes(
    moment_B: NDArray[np.float64],
    omega_B: NDArray[np.float64],
    inertia_B: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        angular_acceleration_body_from_moment(
            moment_B,
            omega_B,
            inertia_B,
        )


@pytest.mark.parametrize(
    ("moment_B", "omega_B", "inertia_B", "expected_message"),
    [
        (
            np.array([np.nan, 0.0, 0.0], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "moment_B must contain only finite values",
        ),
        (
            np.array([np.inf, 0.0, 0.0], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "moment_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.array([np.nan, 0.0, 0.0], dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "omega_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.array([np.inf, 0.0, 0.0], dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "omega_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.array(
                [
                    [np.nan, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            "inertia_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.array(
                [
                    [np.inf, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            "inertia_B must contain only finite values",
        ),
    ],
)
def test_angular_acceleration_body_rejects_nonfinite_inputs(
    moment_B: NDArray[np.float64],
    omega_B: NDArray[np.float64],
    inertia_B: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        angular_acceleration_body_from_moment(
            moment_B,
            omega_B,
            inertia_B,
        )


def test_angular_acceleration_body_rejects_nonsymmetric_inertia() -> None:
    moment_B = np.zeros(3, dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)
    inertia_B = np.array(
        [
            [1.0, 0.1, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 3.0],
        ],
        dtype=np.float64,
    )

    with pytest.raises(ValueError, match="inertia_B must be symmetric"):
        angular_acceleration_body_from_moment(
            moment_B,
            omega_B,
            inertia_B,
        )


@pytest.mark.parametrize(
    "inertia_B",
    [
        np.zeros((3, 3), dtype=np.float64),
        np.diag(
            np.array([1.0, 1.0, 0.0], dtype=np.float64),
        ),
        np.diag(
            np.array([1.0, -1.0, 1.0], dtype=np.float64),
        ),
    ],
)
def test_angular_acceleration_body_rejects_non_positive_definite_inertia(
    inertia_B: NDArray[np.float64],
) -> None:
    moment_B = np.zeros(3, dtype=np.float64)
    omega_B = np.zeros(3, dtype=np.float64)

    with pytest.raises(
        ValueError,
        match="inertia_B must be positive definite",
    ):
        angular_acceleration_body_from_moment(
            moment_B,
            omega_B,
            inertia_B,
        )


def test_translational_acceleration_world_rotates_body_force_and_adds_gravity() -> None:
    force_B = np.array([0.0, 0.0, -2.0], dtype=np.float64)
    R_WB = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    mass = 2.0
    gravity_acceleration = 9.81

    acceleration_W = translational_acceleration_world_from_body_force(
        force_B,
        R_WB,
        mass,
        gravity_acceleration,
    )

    expected_acceleration_W = np.array([0.0, 1.0, 9.81], dtype=np.float64)
    np.testing.assert_allclose(
        acceleration_W,
        expected_acceleration_W,
        atol=1e-12,
    )


def test_translational_acceleration_world_rejects_invalid_force_shape() -> None:
    force_B = np.zeros(2, dtype=np.float64)
    R_WB = np.eye(3, dtype=np.float64)

    with pytest.raises(ValueError, match=r"force_B must have shape \(3,\)"):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=1.0,
            gravity_acceleration=9.81,
        )


def test_translational_acceleration_world_rejects_invalid_rotation_shape() -> None:
    force_B = np.zeros(3, dtype=np.float64)
    R_WB = np.zeros((2, 2), dtype=np.float64)

    with pytest.raises(ValueError, match=r"R_WB must have shape \(3, 3\)"):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=1.0,
            gravity_acceleration=9.81,
        )


@pytest.mark.parametrize(
    ("force_B", "R_WB", "expected_message"),
    [
        (
            np.array([np.nan, 0.0, 0.0], dtype=np.float64),
            np.eye(3, dtype=np.float64),
            "force_B must contain only finite values",
        ),
        (
            np.zeros(3, dtype=np.float64),
            np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, np.inf, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            "R_WB must contain only finite values",
        ),
    ],
)
def test_translational_acceleration_world_rejects_nonfinite_array_inputs(
    force_B: NDArray[np.float64],
    R_WB: NDArray[np.float64],
    expected_message: str,
) -> None:
    with pytest.raises(ValueError, match=expected_message):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=1.0,
            gravity_acceleration=9.81,
        )


@pytest.mark.parametrize(
    ("mass", "expected_message"),
    [
        (np.nan, "mass must be finite"),
        (np.inf, "mass must be finite"),
        (0.0, "mass must be positive"),
        (-1.0, "mass must be positive"),
    ],
)
def test_translational_acceleration_world_rejects_invalid_mass(
    mass: float,
    expected_message: str,
) -> None:
    force_B = np.zeros(3, dtype=np.float64)
    R_WB = np.eye(3, dtype=np.float64)

    with pytest.raises(ValueError, match=expected_message):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=mass,
            gravity_acceleration=9.81,
        )


@pytest.mark.parametrize(
    ("gravity_acceleration", "expected_message"),
    [
        (np.nan, "gravity_acceleration must be finite"),
        (np.inf, "gravity_acceleration must be finite"),
        (0.0, "gravity_acceleration must be positive"),
        (-9.81, "gravity_acceleration must be positive"),
    ],
)
def test_translational_acceleration_world_rejects_invalid_gravity(
    gravity_acceleration: float,
    expected_message: str,
) -> None:
    force_B = np.zeros(3, dtype=np.float64)
    R_WB = np.eye(3, dtype=np.float64)

    with pytest.raises(ValueError, match=expected_message):
        translational_acceleration_world_from_body_force(
            force_B,
            R_WB,
            mass=1.0,
            gravity_acceleration=gravity_acceleration,
        )


def test_quadratic_drag_force_body_has_expected_northward_signs() -> None:
    R_WB = np.eye(3, dtype=np.float64)
    coefficient_B = np.array([0.5, 0.0, 0.0], dtype=np.float64)
    zero_W = np.zeros(3, dtype=np.float64)

    np.testing.assert_array_equal(
        dynamics.quadratic_drag_force_body(zero_W, R_WB, zero_W, coefficient_B),
        zero_W,
    )
    np.testing.assert_array_equal(
        dynamics.quadratic_drag_force_body(np.array([2.0, 0.0, 0.0]), R_WB, zero_W, coefficient_B),
        np.array([-2.0, 0.0, 0.0]),
    )
    np.testing.assert_array_equal(
        dynamics.quadratic_drag_force_body(zero_W, R_WB, np.array([1.0, 0.0, 0.0]), coefficient_B),
        np.array([0.5, 0.0, 0.0]),
    )


def test_quadratic_drag_force_body_rotates_air_velocity_before_anisotropic_drag() -> None:
    R_WB = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    velocity_W = np.array([3.0, 2.0, -1.0])
    wind_velocity_W = np.array([1.0, -1.0, 1.0])
    coefficient_B = np.array([0.5, 1.0, 2.0])

    force_drag_B = dynamics.quadratic_drag_force_body(
        velocity_W, R_WB, wind_velocity_W, coefficient_B
    )

    # Relative air velocity is [2, 3, -2] NED and [3, -2, -2] FRD.
    np.testing.assert_array_equal(force_drag_B, np.array([-4.5, 4.0, 8.0]))
    assert force_drag_B @ np.array([3.0, -2.0, -2.0]) <= 0.0


@pytest.mark.parametrize(
    ("velocity_W", "wind_velocity_W", "coefficient_B"),
    [
        (np.array([2.0, -1.0, 3.0]), np.array([0.5, 0.0, -0.5]), np.array([1.0, 2.0, 3.0])),
        (np.array([-2.0, 1.0, 0.0]), np.array([1.0, 2.0, 0.0]), np.array([0.0, 0.5, 0.0])),
        (np.array([1.0, 2.0, 3.0]), np.array([1.0, 2.0, 3.0]), np.array([1.0, 1.0, 1.0])),
    ],
)
def test_quadratic_drag_force_body_never_adds_air_relative_power(
    velocity_W: NDArray[np.float64],
    wind_velocity_W: NDArray[np.float64],
    coefficient_B: NDArray[np.float64],
) -> None:
    R_WB = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    velocity_air_B = R_WB.T @ (velocity_W - wind_velocity_W)
    force_drag_B = dynamics.quadratic_drag_force_body(
        velocity_W, R_WB, wind_velocity_W, coefficient_B
    )
    assert force_drag_B @ velocity_air_B <= 0.0


def test_quadratic_drag_force_body_zero_coefficients_avoid_extreme_arithmetic() -> None:
    maximum = np.finfo(np.float64).max
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        force_drag_B = dynamics.quadratic_drag_force_body(
            np.array([maximum, 0.0, 0.0]),
            np.eye(3),
            np.array([-maximum, 0.0, 0.0]),
            np.zeros(3),
        )
    np.testing.assert_array_equal(force_drag_B, np.zeros(3))


def test_quadratic_drag_force_body_owns_output_and_does_not_mutate_inputs() -> None:
    inputs = (
        np.array([2.0, -1.0, 3.0]),
        np.eye(3),
        np.array([0.5, 0.0, -0.5]),
        np.array([1.0, 2.0, 3.0]),
    )
    snapshots = tuple(values.copy() for values in inputs)

    force_drag_B = dynamics.quadratic_drag_force_body(*inputs)

    assert force_drag_B.shape == (3,)
    assert force_drag_B.dtype == np.float64
    assert force_drag_B.flags.c_contiguous and force_drag_B.flags.owndata
    assert all(not np.shares_memory(force_drag_B, values) for values in inputs)
    for values, snapshot in zip(inputs, snapshots, strict=True):
        np.testing.assert_array_equal(values, snapshot)


@pytest.mark.parametrize(
    ("argument_name", "invalid", "expected_message"),
    [
        ("velocity_W", np.zeros(2), "velocity_W must have shape (3,)"),
        ("R_WB", np.zeros((2, 3)), "R_WB must have shape (3, 3)"),
        ("wind_velocity_W", np.zeros(2), "wind_velocity_W must have shape (3,)"),
        (
            "quadratic_drag_coefficient_B",
            np.zeros(2),
            "quadratic_drag_coefficient_B must have shape (3,)",
        ),
        ("velocity_W", np.array([np.nan, 0.0, 0.0]), "velocity_W must contain only finite values"),
        ("R_WB", np.diag([1.0, np.inf, 1.0]), "R_WB must contain only finite values"),
        (
            "wind_velocity_W",
            np.array([0.0, -np.inf, 0.0]),
            "wind_velocity_W must contain only finite values",
        ),
        (
            "quadratic_drag_coefficient_B",
            np.array([0.0, 0.0, np.nan]),
            "quadratic_drag_coefficient_B must contain only finite values",
        ),
        (
            "quadratic_drag_coefficient_B",
            np.array([0.0, -0.1, 0.0]),
            "quadratic_drag_coefficient_B must be nonnegative",
        ),
    ],
)
def test_quadratic_drag_force_body_rejects_invalid_inputs(
    argument_name: str, invalid: NDArray[np.float64], expected_message: str
) -> None:
    arguments = {
        "velocity_W": np.zeros(3),
        "R_WB": np.eye(3),
        "wind_velocity_W": np.zeros(3),
        "quadratic_drag_coefficient_B": np.ones(3),
    }
    arguments[argument_name] = invalid
    with pytest.raises(ValueError, match=rf"^{re.escape(expected_message)}$"):
        dynamics.quadratic_drag_force_body(**arguments)


def test_quadratic_drag_force_body_checks_all_shapes_before_finiteness() -> None:
    with pytest.raises(ValueError, match=r"^R_WB must have shape \(3, 3\)$"):
        dynamics.quadratic_drag_force_body(
            np.array([np.nan, 0.0, 0.0]),
            np.eye(2),
            np.zeros(3),
            np.ones(3),
        )


@pytest.mark.parametrize(
    ("velocity_W", "wind_velocity_W"),
    [
        (
            np.array([np.finfo(np.float64).max, 0.0, 0.0]),
            np.array([-np.finfo(np.float64).max, 0.0, 0.0]),
        ),
        (np.array([1.0e200, 0.0, 0.0]), np.zeros(3)),
    ],
)
def test_quadratic_drag_force_body_rejects_finite_input_overflow_without_warning(
    velocity_W: NDArray[np.float64], wind_velocity_W: NDArray[np.float64]
) -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="^quadratic drag calculation must remain finite$"):
            dynamics.quadratic_drag_force_body(
                velocity_W,
                np.eye(3),
                wind_velocity_W,
                np.array([1.0, 0.0, 0.0]),
            )
