"""Unfitted sampled response models for ADR 0032; never flight controllers."""

import numpy as np
from numpy.typing import NDArray

from experiments.robustness_evidence import ensure

Array = NDArray[np.float64]
ComplexArray = NDArray[np.complex128]


def pd_response(
    time: Array,
    outer_indices: NDArray[np.int64],
    kp: Array,
    kv: Array,
    acceleration: Array,
    position_increment: Array,
    velocity_increment: Array,
    initial_position: Array,
    initial_velocity: Array,
) -> tuple[Array, Array]:
    """Exact superposition under sample-held PD, with additive interval increments.

    acceleration has (N-1,C,3) rows, increments the same shape; initial states
    are (C,3). Each channel gets its own held feedback. This attributes a fixed
    trajectory; removing a channel does not simulate changed nonlinear feedback.
    """
    n = len(time)
    ensure(n >= 2 and np.all(np.diff(time) > 0), "response clock")
    ensure(
        acceleration.ndim == 3 and acceleration.shape[0] == n - 1 and acceleration.shape[2] == 3,
        "response forcing shape",
    )
    shape = acceleration.shape
    ensure(position_increment.shape == velocity_increment.shape == shape, "increment shape")
    ensure(initial_position.shape == initial_velocity.shape == shape[1:], "initial shape")
    ensure(kp.shape == kv.shape == (3,) and np.all(kp > 0) and np.all(kv > 0), "PD gains")
    ensure(
        outer_indices.shape == (n - 1,)
        and np.issubdtype(outer_indices.dtype, np.integer)
        and outer_indices[0] == 0
        and np.all(np.diff(outer_indices) >= 0)
        and np.all(outer_indices <= np.arange(n - 1)),
        "causal outer hold",
    )
    ensure(
        all(
            np.all(np.isfinite(x))
            for x in (
                time,
                acceleration,
                position_increment,
                velocity_increment,
                initial_position,
                initial_velocity,
                kp,
                kv,
            )
        ),
        "finite response inputs",
    )
    p, v = np.zeros((n, *shape[1:])), np.zeros((n, *shape[1:]))
    p[0], v[0] = initial_position, initial_velocity
    for k, h in enumerate(np.diff(time)):
        j = outer_indices[k]
        a = -kp * p[j] - kv * v[j] + acceleration[k]
        p[k + 1] = p[k] + h * v[k] + h * h / 2 * a + position_increment[k]
        v[k + 1] = v[k] + h * a + velocity_increment[k]
    return p, v


def cascade_transition(h: float, tau: float) -> Array:
    """Exact linear flow for [p,v,u,w,alpha,held_u,held_alpha].

    u is horizontal acceleration from inclination, w=du/dt, alpha=dw/dt.
    Linearized rotor moment has the existing first-order motor time constant.
    A convergent matrix Taylor series handles the repeated zero eigenvalues.
    """
    ensure(np.isfinite(h) and np.isfinite(tau) and 0 < h <= tau, "cascade time scales")
    a = np.zeros((7, 7))
    a[0, 1] = a[1, 2] = a[2, 3] = a[3, 4] = 1
    a[4, 4], a[4, 6] = -1 / tau, 1 / tau
    result, term = np.eye(7), np.eye(7)
    for k in range(1, 41):
        term = term @ (h * a) / k
        result += term
    ensure(np.max(np.abs(term)) < 1e-16, "matrix series convergence")
    return result


def cascade_updates(kp: float, kv: float, ka: float, kr: float) -> tuple[Array, Array]:
    """Linear outer/inner update matrices; positive SI gains, no fitted coefficient."""
    ensure(all(np.isfinite(x) and x > 0 for x in (kp, kv, ka, kr)), "cascade gains")
    outer, inner = np.eye(7), np.eye(7)
    outer[5] = 0
    outer[5, :2] = -kp, -kv
    inner[6] = 0
    inner[6, [2, 3, 5]] = -kr * ka, -kr, kr * ka
    return outer, inner


def cascade_model(
    time: Array,
    initial: Array,
    navigation_p: Array,
    navigation_v: Array,
    inclination_error: Array,
    inclination_rate_error: Array,
    *,
    kp: float,
    kv: float,
    ka: float,
    kr: float,
    tau: float,
    outer_stride: int,
    inner_stride: int,
) -> tuple[Array, ComplexArray]:
    """Horizontal near-level cascade driven by saved estimation-error histories.

    Constant hover thrust, matched inertia, no drag, saturation or reference
    motion. Returns all states and eigenvalues of one outer-period map.
    It is an unfitted local explanatory model, not a flight or a stability proof.
    """
    n = len(time)
    ensure(n >= 2 and time[0] == 0 and np.all(np.diff(time) > 0), "cascade clock")
    h = float(time[1] - time[0])
    ensure(np.allclose(np.diff(time), h, atol=1e-14, rtol=0), "uniform cascade clock")
    ensure(
        type(outer_stride) is int
        and type(inner_stride) is int
        and outer_stride >= inner_stride > 0
        and outer_stride % inner_stride == 0,
        "nested cascade clocks",
    )
    errors = (navigation_p, navigation_v, inclination_error, inclination_rate_error)
    ensure(
        initial.shape == (5, 2) and all(x.shape == (n, 2) for x in errors), "horizontal model shape"
    )
    ensure(all(np.all(np.isfinite(x)) for x in (initial, *errors)), "finite cascade state")
    flow = cascade_transition(h, tau)
    outer, inner = cascade_updates(kp, kv, ka, kr)
    states = np.zeros((n, 7, 2))
    states[0, :5] = initial
    period = np.eye(7)
    for k in range(outer_stride):
        if k % outer_stride == 0:
            period = outer @ period
        if k % inner_stride == 0:
            period = inner @ period
        period = flow @ period
    for k in range(n - 1):
        state = states[k].copy()
        if k % outer_stride == 0:
            state = outer @ state
            state[5] -= kp * navigation_p[k] + kv * navigation_v[k]
        if k % inner_stride == 0:
            state = inner @ state
            state[6] -= kr * (ka * inclination_error[k] + inclination_rate_error[k])
        states[k + 1] = flow @ state
    return states, np.asarray(np.linalg.eigvals(period[:5, :5]), dtype=np.complex128)
