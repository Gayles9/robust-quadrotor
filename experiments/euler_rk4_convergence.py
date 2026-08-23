"""Measure Euler and projected-RK4 convergence for a rigid quadrotor state.

The experiment propagates one deterministic asymmetric constant-rotor-input
scenario with explicit Euler and projected RK4 over four fixed-step grids. The
world frame is north-east-down (NED), and the body frame is
forward-right-down (FRD). Position and velocity errors are measured in NED,
body angular-velocity errors are measured in FRD, and attitude errors compare
body-to-world quaternions without depending on quaternion sign.

A fine 2560-step RK4 trajectory supplies a numerical reference, not an exact
solution. Nested grids are sampled using integer indices, avoiding
floating-point time matching. The study reports independent physical errors
for position, velocity, attitude, and angular velocity at final time and over
the complete trajectory, plus observed orders between adjacent resolutions.
It is an empirical comparison rather than a formal convergence proof, and it
does not assume or claim that quaternion-projected RK4 must achieve fourth
order.
"""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from quadrotor_math.metrics import (
    euclidean_vector_trajectory_errors,
    observed_convergence_orders,
    quaternion_attitude_trajectory_errors_body_to_world,
)
from quadrotor_math.simulation import (
    simulate_rigid_body_euler_from_rotor_speeds,
    simulate_rigid_body_rk4_from_rotor_speeds,
)


@dataclass(frozen=True)
class _ConvergenceStudyResult:
    """Store deterministic errors and observed orders for both methods.

    Method rows are ordered Euler then RK4. Quantity columns are ordered NED
    position, NED velocity, body-to-world attitude, and FRD angular velocity.
    Error arrays have shape ``(2, 4, 4)``; order arrays have shape
    ``(2, 3, 4)``.
    """

    time_steps_s: NDArray[np.float64]
    step_counts: NDArray[np.int64]
    final_errors: NDArray[np.float64]
    maximum_errors: NDArray[np.float64]
    final_orders: NDArray[np.float64]
    maximum_orders: NDArray[np.float64]


def _reference_sample_indices(
    reference_number_of_steps: int,
    comparison_number_of_steps: int,
) -> NDArray[np.int64]:
    """Return exact nested-grid reference indices for a comparison history.

    Integer strides select all comparison times, including the shared initial
    and final times, without floating-point equality, rounding, interpolation,
    or nearest-neighbour matching.

    Raises:
        ValueError: If the reference step count is not divisible by the
            comparison step count.
    """
    if reference_number_of_steps % comparison_number_of_steps != 0:
        raise ValueError(
            "reference_number_of_steps must be divisible by comparison_number_of_steps"
        )

    stride = reference_number_of_steps // comparison_number_of_steps
    return np.arange(comparison_number_of_steps + 1, dtype=np.int64) * stride


def _state_trajectory_errors(
    position_history_W: NDArray[np.float64],
    velocity_history_W: NDArray[np.float64],
    q_history_WB: NDArray[np.float64],
    omega_history_B: NDArray[np.float64],
    reference_position_history_W: NDArray[np.float64],
    reference_velocity_history_W: NDArray[np.float64],
    reference_q_history_WB: NDArray[np.float64],
    reference_omega_history_B: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Return four independent physical errors for every trajectory sample.

    The output has shape ``(N + 1, 4)``. Its columns contain Euclidean NED
    position error in metres, Euclidean NED velocity error in m/s,
    sign-invariant geodesic body-to-world attitude error in radians, and
    Euclidean FRD angular-velocity error in rad/s. The quantities remain
    separate because a mixed-unit aggregate norm would not be meaningful.
    """
    position_errors = euclidean_vector_trajectory_errors(
        position_history_W,
        reference_position_history_W,
    )
    velocity_errors = euclidean_vector_trajectory_errors(
        velocity_history_W,
        reference_velocity_history_W,
    )
    attitude_errors = quaternion_attitude_trajectory_errors_body_to_world(
        q_history_WB,
        reference_q_history_WB,
    )
    angular_velocity_errors = euclidean_vector_trajectory_errors(
        omega_history_B,
        reference_omega_history_B,
    )
    return np.asarray(
        np.column_stack(
            (
                position_errors,
                velocity_errors,
                attitude_errors,
                angular_velocity_errors,
            )
        ),
        dtype=np.float64,
    )


def _run_convergence_study() -> _ConvergenceStudyResult:
    """Run the nested-grid convergence study and return measured quantities.

    One fine projected-RK4 trajectory is generated as a numerical reference.
    Four Euler and four RK4 histories then use identical initial NED/FRD state,
    constant asymmetric rotor input, vehicle parameters, and total duration.
    Final and maximum trajectory errors are measured independently for each
    physical quantity. Observed orders describe the measured change between
    adjacent grids; they are not theoretical guarantees, and projected RK4 is
    not presumed to exhibit fourth order.
    """
    position_W = np.array(
        [10.0, 20.0, 30.0],
        dtype=np.float64,
    )
    velocity_W = np.array(
        [1.0, -2.0, 3.0],
        dtype=np.float64,
    )
    q_WB = np.array(
        [1.0, 0.0, 0.0, 0.0],
        dtype=np.float64,
    )
    omega_B = np.array(
        [0.0, 0.0, 2.0],
        dtype=np.float64,
    )
    rotor_omega = np.array(
        [2.0, 0.0, 0.0, 0.0],
        dtype=np.float64,
    )
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
    inertia_B = np.diag(np.array([2.0, 3.0, 4.0], dtype=np.float64))
    gravity_acceleration = 9.81
    thrust_coefficient = 0.75
    moment_coefficient = 0.5

    total_duration_s = 1.0
    step_counts = np.array(
        [10, 20, 40, 80],
        dtype=np.int64,
    )
    time_steps_s = total_duration_s / step_counts.astype(np.float64)

    reference_number_of_steps = 2560
    reference_time_step = total_duration_s / reference_number_of_steps
    (
        reference_time_s,
        reference_position_history_W,
        reference_velocity_history_W,
        reference_q_history_WB,
        reference_omega_history_B,
    ) = simulate_rigid_body_rk4_from_rotor_speeds(
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
        reference_time_step,
        reference_number_of_steps,
    )

    final_errors = np.empty((2, 4, 4), dtype=np.float64)
    maximum_errors = np.empty((2, 4, 4), dtype=np.float64)
    time_tolerance_s = 16.0 * np.finfo(np.float64).eps * total_duration_s

    for resolution_index, step_count in enumerate(step_counts):
        number_of_steps = int(step_count)
        time_step = float(time_steps_s[resolution_index])
        reference_indices = _reference_sample_indices(
            reference_number_of_steps,
            number_of_steps,
        )
        sampled_reference_time_s = reference_time_s[reference_indices]
        sampled_reference_position_history_W = reference_position_history_W[reference_indices]
        sampled_reference_velocity_history_W = reference_velocity_history_W[reference_indices]
        sampled_reference_q_history_WB = reference_q_history_WB[reference_indices]
        sampled_reference_omega_history_B = reference_omega_history_B[reference_indices]

        (
            euler_time_s,
            euler_position_history_W,
            euler_velocity_history_W,
            euler_q_history_WB,
            euler_omega_history_B,
        ) = simulate_rigid_body_euler_from_rotor_speeds(
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
        (
            rk4_time_s,
            rk4_position_history_W,
            rk4_velocity_history_W,
            rk4_q_history_WB,
            rk4_omega_history_B,
        ) = simulate_rigid_body_rk4_from_rotor_speeds(
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

        np.testing.assert_allclose(
            euler_time_s,
            sampled_reference_time_s,
            rtol=0.0,
            atol=time_tolerance_s,
        )
        np.testing.assert_allclose(
            rk4_time_s,
            sampled_reference_time_s,
            rtol=0.0,
            atol=time_tolerance_s,
        )

        trial_histories = (
            (
                euler_position_history_W,
                euler_velocity_history_W,
                euler_q_history_WB,
                euler_omega_history_B,
            ),
            (
                rk4_position_history_W,
                rk4_velocity_history_W,
                rk4_q_history_WB,
                rk4_omega_history_B,
            ),
        )
        for method_index, (
            trial_position_history_W,
            trial_velocity_history_W,
            trial_q_history_WB,
            trial_omega_history_B,
        ) in enumerate(trial_histories):
            trajectory_errors = _state_trajectory_errors(
                trial_position_history_W,
                trial_velocity_history_W,
                trial_q_history_WB,
                trial_omega_history_B,
                sampled_reference_position_history_W,
                sampled_reference_velocity_history_W,
                sampled_reference_q_history_WB,
                sampled_reference_omega_history_B,
            )
            final_errors[method_index, resolution_index] = trajectory_errors[-1]
            maximum_errors[method_index, resolution_index] = np.max(
                trajectory_errors,
                axis=0,
            )

    maximum_reference_position_norm = float(
        np.max(np.linalg.norm(reference_position_history_W, axis=1))
    )
    maximum_reference_velocity_norm = float(
        np.max(np.linalg.norm(reference_velocity_history_W, axis=1))
    )
    maximum_reference_angular_velocity_norm = float(
        np.max(np.linalg.norm(reference_omega_history_B, axis=1))
    )
    reference_scales = np.array(
        [
            max(1.0, maximum_reference_position_norm),
            max(1.0, maximum_reference_velocity_norm),
            1.0,
            max(1.0, maximum_reference_angular_velocity_norm),
        ],
        dtype=np.float64,
    )
    error_floors = 128.0 * np.finfo(np.float64).eps * reference_scales

    final_orders = np.empty((2, 3, 4), dtype=np.float64)
    maximum_orders = np.empty((2, 3, 4), dtype=np.float64)
    for method_index in range(2):
        for quantity_index in range(4):
            error_floor = float(error_floors[quantity_index])
            final_orders[method_index, :, quantity_index] = observed_convergence_orders(
                time_steps_s,
                final_errors[method_index, :, quantity_index],
                error_floor=error_floor,
            )
            maximum_orders[method_index, :, quantity_index] = observed_convergence_orders(
                time_steps_s,
                maximum_errors[method_index, :, quantity_index],
                error_floor=error_floor,
            )

    return _ConvergenceStudyResult(
        time_steps_s=time_steps_s,
        step_counts=step_counts,
        final_errors=final_errors,
        maximum_errors=maximum_errors,
        final_orders=final_orders,
        maximum_orders=maximum_orders,
    )


def _format_results(result: _ConvergenceStudyResult) -> str:
    """Format deterministic error and observed-order tables as Markdown.

    Error tables retain the independent NED position and velocity, geodesic
    attitude, and FRD angular-velocity units. Order tables report measurements
    between adjacent grid resolutions, including negative values and explicit
    ``nan`` entries. The fine projected-RK4 trajectory remains a numerical
    reference rather than an exact solution or a convergence proof.
    """
    lines = [
        "# Euler-versus-RK4 Convergence Study",
        "",
        ("The 2560-step projected-RK4 trajectory is a numerical reference, not an exact solution."),
        "",
    ]
    method_names = ("Euler", "RK4")
    error_header = (
        "| Method | Time step (s) | Steps | Position (m) | Velocity (m/s) | "
        "Attitude (rad) | Angular velocity (rad/s) |"
    )
    error_separator = "|---|---:|---:|---:|---:|---:|---:|"

    lines.extend(["## Final-time errors", "", error_header, error_separator])
    for method_index, method_name in enumerate(method_names):
        for resolution_index in range(4):
            errors = result.final_errors[method_index, resolution_index]
            lines.append(
                f"| {method_name} | {float(result.time_steps_s[resolution_index]):.10e} | "
                f"{int(result.step_counts[resolution_index])} | {float(errors[0]):.10e} | "
                f"{float(errors[1]):.10e} | {float(errors[2]):.10e} | "
                f"{float(errors[3]):.10e} |"
            )

    lines.extend(["", "## Maximum-trajectory errors", "", error_header, error_separator])
    for method_index, method_name in enumerate(method_names):
        for resolution_index in range(4):
            errors = result.maximum_errors[method_index, resolution_index]
            lines.append(
                f"| {method_name} | {float(result.time_steps_s[resolution_index]):.10e} | "
                f"{int(result.step_counts[resolution_index])} | {float(errors[0]):.10e} | "
                f"{float(errors[1]):.10e} | {float(errors[2]):.10e} | "
                f"{float(errors[3]):.10e} |"
            )

    order_header = (
        "| Method | Time-step pair (s) | Position (m) order | Velocity (m/s) order | "
        "Attitude (rad) order | Angular velocity (rad/s) order |"
    )
    order_separator = "|---|---|---:|---:|---:|---:|"
    lines.extend(["", "## Final-time observed orders", "", order_header, order_separator])
    for method_index, method_name in enumerate(method_names):
        for order_index in range(3):
            orders = result.final_orders[method_index, order_index]
            order_values = [
                "nan" if np.isnan(value) else f"{float(value):.10e}" for value in orders
            ]
            lines.append(
                f"| {method_name} | {float(result.time_steps_s[order_index]):.10e} → "
                f"{float(result.time_steps_s[order_index + 1]):.10e} | "
                f"{' | '.join(order_values)} |"
            )

    lines.extend(["", "## Maximum-error observed orders", "", order_header, order_separator])
    for method_index, method_name in enumerate(method_names):
        for order_index in range(3):
            orders = result.maximum_orders[method_index, order_index]
            order_values = [
                "nan" if np.isnan(value) else f"{float(value):.10e}" for value in orders
            ]
            lines.append(
                f"| {method_name} | {float(result.time_steps_s[order_index]):.10e} → "
                f"{float(result.time_steps_s[order_index + 1]):.10e} | "
                f"{' | '.join(order_values)} |"
            )

    return "\n".join(lines)


def main() -> None:
    """Run the deterministic convergence study and print its Markdown tables."""
    result = _run_convergence_study()
    print(_format_results(result))


if __name__ == "__main__":
    main()
