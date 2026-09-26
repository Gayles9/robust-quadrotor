"""Opt-in geometric attitude control in NED/FRD, with analytic rotation jets.

Lee/Leok/McClamroch CDC 2010 moment law. The mission adapter supplies causal
approximate reference derivatives; no continuous-time flight guarantee follows.
"""

from dataclasses import dataclass, replace
from math import cos, hypot, sin

import numpy as np
from numpy.typing import NDArray

from .attitude_control import (
    AttitudeControlCommand,
    AttitudeControllerParameters,
    _array,
    _quaternion,
    _scalar,
    allocate_limited_body_moment,
    attitude_error_body,
)
from .position_control import _finite_scalar, _quaternion_from_axes
from .rotations import rotation_matrix_body_to_world

Vector = NDArray[np.float64]
Jet = tuple[Vector, Vector, Vector]


class GeometricDomainError(ValueError):
    """A finite command leaves the declared smooth control domain; abort safely."""


@dataclass(frozen=True, slots=True)
class GeometricControllerParameters:
    """Positive scalar stiffness [N m], damping [N m s], filter pole [rad/s]."""

    attitude_stiffness: float = 0.64
    rate_damping: float = 0.32
    filter_pole_rad_s: float = 30.0
    rebase_estimator_corrections: bool = False

    def __post_init__(self) -> None:
        for name in ("attitude_stiffness", "rate_damping", "filter_pole_rad_s"):
            object.__setattr__(self, name, _scalar(name, getattr(self, name), positive=True))
        if type(self.rebase_estimator_corrections) is not bool:
            raise ValueError("rebase_estimator_corrections must be bool")


@dataclass(frozen=True, slots=True, eq=False)
class RotationReference:
    """Owned Hamilton q_WB and desired-frame angular rate/acceleration (SI).

    omega_reference_D and alpha_reference_D are expressed in the desired FRD
    body frame, not the current body frame. Quaternion sign is immaterial.
    """

    q_reference_WB: Vector
    omega_reference_D: Vector
    alpha_reference_D: Vector

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "q_reference_WB", _quaternion("q_reference_WB", self.q_reference_WB)
        )
        for name in ("omega_reference_D", "alpha_reference_D"):
            object.__setattr__(self, name, _array(name, getattr(self, name), (3,)))


def _vee_skew(matrix: Vector) -> Vector:
    return (
        np.array(
            [
                matrix[2, 1] - matrix[1, 2],
                matrix[0, 2] - matrix[2, 0],
                matrix[1, 0] - matrix[0, 1],
            ]
        )
        / 2
    )


def _unit_jet(jet: Jet, name: str) -> Jet:
    value, first, second = jet
    length = hypot(*value)
    if not np.isfinite(length):
        raise ValueError("rotation jet norm must remain finite")
    if length <= 1e-12:
        raise GeometricDomainError(f"{name} direction is singular")
    axis = value / length
    length_dot = float(axis @ first)
    axis_dot = (first - axis * length_dot) / length
    length_ddot = float(axis @ second + axis_dot @ first)
    axis_ddot = (second - axis * length_ddot - 2 * axis_dot * length_dot) / length
    return axis, axis_dot, axis_ddot


def _cross_jet(a: Jet, b: Jet) -> Jet:
    return (
        np.cross(a[0], b[0]),
        np.cross(a[1], b[0]) + np.cross(a[0], b[1]),
        np.cross(a[2], b[0]) + 2 * np.cross(a[1], b[1]) + np.cross(a[0], b[2]),
    )


def force_rotation_reference(
    lift_W: Vector,
    lift_rate_W: Vector,
    lift_acceleration_W: Vector,
    yaw_rad: float,
) -> RotationReference:
    """Twice differentiate normalized lift and fixed-heading axes analytically.

    Lift is the desired positive body-down force vector [N], not the negative
    applied thrust. Its first/second derivatives are N/s and N/s². Input arrays
    have shape (3,); singular force or heading is an explicit domain failure.
    """
    jet = tuple(
        _array(name, value, (3,))
        for name, value in (
            ("lift_W", lift_W),
            ("lift_rate_W", lift_rate_W),
            ("lift_acceleration_W", lift_acceleration_W),
        )
    )
    yaw = _finite_scalar("yaw_rad", yaw_rad)
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            down = _unit_jet((jet[0], jet[1], jet[2]), "force")
            heading = (np.array([-sin(yaw), cos(yaw), 0.0]), np.zeros(3), np.zeros(3))
            forward = _unit_jet(_cross_jet(heading, down), "heading")
            right = _cross_jet(down, forward)
            R, dR, ddR = (np.column_stack((forward[i], right[i], down[i])) for i in range(3))
            skew = R.T @ dR
            omega = _vee_skew(skew)
            alpha = _vee_skew(R.T @ ddR - skew @ skew)
            return RotationReference(_quaternion_from_axes(R), omega, alpha)
    except FloatingPointError:
        raise ValueError("force-to-rotation arithmetic must remain finite") from None


def compute_geometric_control(
    q_WB: Vector,
    omega_B: Vector,
    reference: RotationReference,
    collective_thrust: float,
    actuator_parameters: AttitudeControllerParameters,
    parameters: GeometricControllerParameters,
) -> AttitudeControlCommand:
    """Geometric FRD moment, transported moving-reference feedforward, allocation.

    Current rate [rad/s], full nominal SPD inertia [kg m²], collective [N].
    Error diagnostic is e_R (sine-axis), desired_omega_B is transported Omega_d.
    Local attitude/reference-rate violations abort rather than clipping jets.
    Moment clipping and moment-allocation scaling remain explicit diagnostics.
    """
    if (
        not isinstance(reference, RotationReference)
        or not isinstance(parameters, GeometricControllerParameters)
        or not isinstance(actuator_parameters, AttitudeControllerParameters)
    ):
        raise TypeError("geometric inputs must use their declared dataclasses")
    model, gains, target = replace(actuator_parameters), replace(parameters), replace(reference)
    current = _quaternion("q_WB", q_WB)
    omega = _array("omega_B", omega_B, (3,))
    thrust = _scalar("collective_thrust", collective_thrust, positive=True)
    if (
        hypot(*attitude_error_body(current, target.q_reference_WB))
        > model.maximum_attitude_error_rad
    ):
        raise GeometricDomainError("attitude_domain")
    R, Rd = (
        rotation_matrix_body_to_world(current),
        rotation_matrix_body_to_world(target.q_reference_WB),
    )
    J = model.nominal_inertia_B
    try:
        with np.errstate(over="raise", invalid="raise"):
            A = R.T @ Rd
            desired_rate = A @ target.omega_reference_D
            if np.any(np.abs(desired_rate) > model.maximum_body_rate_B):
                raise GeometricDomainError("reference_rate_domain")
            er = _vee_skew(Rd.T @ R - A) / 2
            ew = omega - desired_rate
            moment = (
                -gains.attitude_stiffness * er
                - gains.rate_damping * ew
                + np.cross(omega, J @ omega)
                - J @ (np.cross(omega, desired_rate) - A @ target.alpha_reference_D)
            )
    except FloatingPointError:
        raise ValueError("geometric moment must remain finite") from None
    moment = _array("moment_requested_B", moment, (3,))
    limited = np.clip(moment, -model.maximum_moment_B, model.maximum_moment_B)
    return AttitudeControlCommand(
        er,
        desired_rate,
        moment,
        limited,
        False,
        bool(np.any(limited != moment)),
        allocate_limited_body_moment(thrust, limited, model.nominal_rotors),
    )
