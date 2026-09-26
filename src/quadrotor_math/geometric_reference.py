"""Causal feedback-force jets with analytic planned jerk/snap and fixed yaw."""

from dataclasses import dataclass, replace

import numpy as np
from numpy.typing import NDArray

from .attitude_control import _array, _scalar
from .geometric_control import GeometricDomainError, RotationReference, force_rotation_reference
from .geometric_filter import FeedbackDerivativeFilter
from .missions import MinimumSnapMissionSegment, MissionPlan, ReferenceKind
from .position_control import (
    PositionControlCommand,
    PositionControllerParameters,
    PositionReference,
    compute_position_control,
)
from .rotations import rotation_matrix_body_to_world


@dataclass(frozen=True, slots=True, eq=False)
class GeometricPositionReference:
    """Held outer command: positive desired lift [N], rotation jets, PD diagnostics."""

    lift_W: NDArray[np.float64]
    rotation: RotationReference
    position_command: PositionControlCommand

    def __post_init__(self) -> None:
        if not isinstance(self.rotation, RotationReference) or not isinstance(
            self.position_command, PositionControlCommand
        ):
            raise TypeError("geometric reference requires typed rotation and position commands")
        object.__setattr__(self, "lift_W", _array("lift_W", self.lift_W, (3,)))
        object.__setattr__(self, "rotation", replace(self.rotation))
        object.__setattr__(self, "position_command", replace(self.position_command))


def reference_jerk_snap(
    plan: MissionPlan,
    time_s: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Right-hand derivatives at segment boundaries; no derivative of step impulses.

    Fixed-yaw HOLD, quintic SMOOTH and seventh-degree minimum-snap references.
    The final held target has zero jerk and snap. Piecewise jerk/snap jumps are
    explicit one-sided values; this does not make a quintic join C3/C4.
    """
    if not isinstance(plan, MissionPlan):
        raise TypeError("plan must be MissionPlan")
    time = _scalar("time_s", time_s)
    start = 0.0
    for segment in plan.segments:
        end = start + segment.duration_s
        if time < end:
            elapsed = time - start
            if isinstance(segment, MinimumSnapMissionSegment):
                return segment.trajectory.evaluate(elapsed, 3), segment.trajectory.evaluate(
                    elapsed, 4
                )
            if segment.kind == ReferenceKind.HOLD:
                return np.zeros(3), np.zeros(3)
            if segment.kind != ReferenceKind.SMOOTH:
                raise ValueError(
                    "geometric control requires HOLD, SMOOTH or minimum-snap references"
                )
            T = segment.duration_s
            s = elapsed / T
            try:
                with np.errstate(over="raise", divide="raise", invalid="raise"):
                    delta = segment.end_position_W - segment.start_position_W
                    jerk = ((60 - 360 * s + 360 * s * s) / T / T / T) * delta
                    snap = ((-360 + 720 * s) / T / T / T / T) * delta
            except (FloatingPointError, OverflowError, ZeroDivisionError):
                raise ValueError("planned derivatives must remain finite") from None
            return _array("jerk_W", jerk, (3,)), _array("snap_W", snap, (3,))
        start = end
    return np.zeros(3), np.zeros(3)


def build_geometric_reference(
    position_W: NDArray[np.float64],
    velocity_W: NDArray[np.float64],
    reference: PositionReference,
    jerk_W: NDArray[np.float64],
    snap_W: NDArray[np.float64],
    parameters: PositionControllerParameters,
    memory: FeedbackDerivativeFilter,
    sample_index: int,
) -> tuple[GeometricPositionReference, FeedbackDerivativeFilter]:
    """PD force plus filtered correction derivatives, without hidden plant inputs.

    Reject outer limiting: a clipped force would require different derivative
    equations. The cascade default continues to use its original clipping path.
    Returning new memory only after success makes an outer update transactional.
    """
    if not isinstance(memory, FeedbackDerivativeFilter):
        raise TypeError("memory must be FeedbackDerivativeFilter")
    position = _array("position_W", position_W, (3,))
    velocity = _array("velocity_W", velocity_W, (3,))
    jerk, snap = _array("jerk_W", jerk_W, (3,)), _array("snap_W", snap_W, (3,))
    held = compute_position_control(position, velocity, reference, parameters)
    if held.acceleration_limited or held.tilt_limited or held.thrust_limited:
        raise GeometricDomainError("outer_reference_domain")
    try:
        with np.errstate(over="raise", invalid="raise"):
            m = parameters.nominal_mass
            correction = m * (
                parameters.position_gain_W * (position - reference.position_W)
                + parameters.velocity_gain_W * (velocity - reference.velocity_W)
            )
            first, second, new_memory = memory.step(correction, sample_index)
            lift = (
                m
                * (
                    np.array([0.0, 0, parameters.nominal_gravity_acceleration])
                    - reference.acceleration_W
                )
                + correction
            )
            rotation = force_rotation_reference(
                lift, -m * jerk + first, -m * snap + second, reference.yaw_rad
            )
    except FloatingPointError:
        raise ValueError("geometric force arithmetic must remain finite") from None
    held = replace(held, q_reference_WB=rotation.q_reference_WB)
    return GeometricPositionReference(lift, rotation, held), new_memory


def projected_collective_thrust(
    reference: GeometricPositionReference,
    q_WB: NDArray[np.float64],
    parameters: PositionControllerParameters,
) -> float:
    """Project held desired lift onto current/estimated body-down axis; units N."""
    if not isinstance(reference, GeometricPositionReference) or not isinstance(
        parameters, PositionControllerParameters
    ):
        raise TypeError("reference and parameters must use their dataclasses")
    R = rotation_matrix_body_to_world(q_WB)
    with np.errstate(over="ignore", invalid="ignore"):
        thrust = float(reference.lift_W @ R[:, 2])
    if not np.isfinite(thrust):
        raise ValueError("projected collective must remain finite")
    if not parameters.minimum_collective_thrust <= thrust <= parameters.maximum_collective_thrust:
        raise GeometricDomainError("projected_thrust_domain")
    return thrust
