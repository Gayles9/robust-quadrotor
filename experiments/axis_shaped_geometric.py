"""ADR 0042: coherent axis-dependent feedback-force shaping, experiment only."""

from dataclasses import replace

import numpy as np
from numpy.typing import NDArray

from quadrotor_math.attitude_control import _array, _scalar
from quadrotor_math.cascade_analysis import hover_axis_zero_order_hold
from quadrotor_math.geometric_control import GeometricDomainError, force_rotation_reference
from quadrotor_math.geometric_filter import FeedbackDerivativeFilter
from quadrotor_math.geometric_reference import GeometricPositionReference
from quadrotor_math.position_control import (
    PositionControllerParameters,
    PositionReference,
    compute_position_control,
)

Array = NDArray[np.float64]


def force_step(
    correction_W: Array,
    memory: FeedbackDerivativeFilter,
    sample_index: int,
    vertical_pole_rad_s: float,
) -> tuple[Array, Array, FeedbackDerivativeFilter]:
    """Three independent bilinear cascades; immutable memory, constant prehistory.

    The memory's scalar pole records the common horizontal pole. The vertical
    pole is an explicit, flight-constant experiment parameter. Sections retain
    the original (section, NED-axis) storage and SI units of force [N].
    """
    if not isinstance(memory, FeedbackDerivativeFilter):
        raise TypeError("memory must be FeedbackDerivativeFilter")
    if type(sample_index) is not int or sample_index != memory.next_index:
        raise ValueError("sample index must equal next_index")
    vertical = _scalar("vertical_pole_rad_s", vertical_pole_rad_s, positive=True)
    if not 0 < vertical * memory.period_s <= 1 or not np.isfinite(vertical * vertical):
        raise ValueError("vertical pole must satisfy w*h <= 1")
    correction = _array("correction_W", correction_W, (3,))
    w = np.array([memory.pole_rad_s, memory.pole_rad_s, vertical])
    if sample_index == 0:
        sections = np.tile(correction, (3, 1))
        first, second = np.zeros(3), np.zeros(3)
    else:
        try:
            with np.errstate(over="raise", invalid="raise"):
                a = w * memory.period_s / 2
                b = a / (1 + a)
                sections = np.empty((3, 3))
                current, previous = correction, memory.previous
                for i, old in enumerate(memory.sections):
                    sections[i] = old + b * ((current - old) + (previous - old))
                    current, previous = sections[i], old
                first = w * (sections[1] - sections[2])
                second = w**2 * (sections[0] - 2 * sections[1] + sections[2])
        except FloatingPointError:
            raise ValueError("axis force arithmetic must remain finite") from None
    return (
        _array("force_rate_W", first, (3,)),
        _array("force_acceleration_W", second, (3,)),
        FeedbackDerivativeFilter(
            memory.period_s, memory.pole_rad_s, sample_index + 1, correction, sections
        ),
    )


def axis_shaped_reference(
    position_W: Array,
    velocity_W: Array,
    reference: PositionReference,
    jerk_W: Array,
    snap_W: Array,
    parameters: PositionControllerParameters,
    memory: FeedbackDerivativeFilter,
    sample_index: int,
    *,
    vertical_pole_rad_s: float = 30.0,
    estimated_acceleration_W: Array | None = None,
) -> tuple[GeometricPositionReference, FeedbackDerivativeFilter]:
    """Use Hc, sHc and s²Hc together with analytic planned derivatives.

    Raw and shaped commands must both lie in the existing smooth force domain.
    No hidden plant input, mutable closure, rebase or measured derivative exists.
    Returning memory only after all checks keeps a failed update transactional.
    """
    if estimated_acceleration_W is not None:
        raise ValueError("axis shaping requires the feedback correction channel")
    position = _array("position_W", position_W, (3,))
    velocity = _array("velocity_W", velocity_W, (3,))
    jerk, snap = _array("jerk_W", jerk_W, (3,)), _array("snap_W", snap_W, (3,))
    raw = compute_position_control(position, velocity, reference, parameters)
    if raw.acceleration_limited or raw.tilt_limited or raw.thrust_limited:
        raise GeometricDomainError("outer_reference_domain")
    try:
        with np.errstate(over="raise", invalid="raise"):
            m = parameters.nominal_mass
            correction = m * (
                parameters.position_gain_W * (position - reference.position_W)
                + parameters.velocity_gain_W * (velocity - reference.velocity_W)
            )
            first, second, updated = force_step(
                correction, memory, sample_index, vertical_pole_rad_s
            )
            acceleration = reference.acceleration_W - updated.sections[2] / m
            shaped = compute_position_control(
                np.zeros(3),
                np.zeros(3),
                PositionReference(np.zeros(3), np.zeros(3), acceleration, reference.yaw_rad),
                parameters,
            )
            if shaped.acceleration_limited or shaped.tilt_limited or shaped.thrust_limited:
                raise GeometricDomainError("shaped_reference_domain")
            lift = m * (
                np.array([0.0, 0.0, parameters.nominal_gravity_acceleration]) - acceleration
            )
            rotation = force_rotation_reference(
                lift, -m * jerk + first, -m * snap + second, reference.yaw_rad
            )
    except FloatingPointError:
        raise ValueError("axis reference arithmetic must remain finite") from None
    command = replace(
        raw,
        feasible_acceleration_W=shaped.feasible_acceleration_W,
        collective_thrust=shaped.collective_thrust,
        q_reference_WB=rotation.q_reference_WB,
    )
    return GeometricPositionReference(lift, rotation, command), updated


def horizontal_transition(pole: float, frequency: float, inertia: float) -> Array:
    """Local [p,v,eta,r,a,previous,s1,s2,s3] map, 20 ms outer / 10 ms inner.

    Filter states have signed tilt units. Estimator errors are external inputs;
    eigenvalues do not certify stochastic nonlinear closed-loop stability.
    """
    w, f, j = (
        _scalar(n, v, positive=True)
        for n, v in (("pole", pole), ("frequency", frequency), ("inertia", inertia))
    )
    if w * 0.02 > 1:
        raise ValueError("horizontal pole must satisfy w*h <= 1")
    phi, gamma = hover_axis_zero_order_hold(9.81, 0.025, 0.01)

    def step(x: Array) -> Array:
        physical = x[:5].copy()
        desired = -(f**2 * x[0] + 1.8 * f * x[1]) / 9.81
        current, previous, sections = desired, x[5], np.empty(3)
        b = w * 0.01 / (1 + w * 0.01)
        for i, old in enumerate(x[6:]):
            sections[i] = old + b * ((current - old) + (previous - old))
            current, previous = sections[i], old
        first = w * (sections[1] - sections[2])
        second = w**2 * (sections[0] - 2 * sections[1] + sections[2])
        for _ in range(2):
            u = (0.64 * (sections[2] - physical[2]) + 0.32 * (first - physical[3])) / j
            physical = phi @ physical + gamma * (u + second)
        return np.concatenate((physical, [desired], sections))

    return np.column_stack([step(x) for x in np.eye(9)])


def vertical_transition(pole: float = 30.0) -> Array:
    """Local [p,v,a,previous,s1,s2,s3] map with collective motor lag and PD.

    In hover tau*a'=u-a, p'=v, v'=a. Exact ZOH is the last three coordinates
    of the existing chain-integrator map; the angular names there are incidental.
    """
    w = _scalar("pole", pole, positive=True)
    if w * 0.02 > 1:
        raise ValueError("vertical pole must satisfy w*h <= 1")
    full_phi, full_gamma = hover_axis_zero_order_hold(9.81, 0.025, 0.02)
    phi, gamma = full_phi[2:, 2:], full_gamma[2:]

    def step(x: Array) -> Array:
        desired = -(2.25 * x[0] + 3 * x[1])
        current, previous, sections = desired, x[3], np.empty(3)
        b = w * 0.01 / (1 + w * 0.01)
        for i, old in enumerate(x[4:]):
            sections[i] = old + b * ((current - old) + (previous - old))
            current, previous = sections[i], old
        return np.concatenate((phi @ x[:3] + gamma * sections[2], [desired], sections))

    return np.column_stack([step(x) for x in np.eye(7)])
