"""Rejected fixed motor-damping candidate; an experiment, not a runtime option.

Only already observed seeds are supported. Nominal rotor motion is reconstructed
from prior commands and explicit nominal hover initialization, never true speeds.
The original mission ends only because this diagnostic stops after its 10-s
endpoint observation. Short-prefix success cannot qualify the full mission.
"""

import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from datetime import UTC, datetime
from functools import partial
from pathlib import Path
from typing import Any, cast
from unittest.mock import patch

import numpy as np

from experiments.attitude_control_validation import source_sha256
from experiments.feedback_bandwidth_validation import make_configuration
from quadrotor_math import estimated_mission, mission_simulation
from quadrotor_math.attitude_control import allocate_limited_body_moment
from quadrotor_math.attitude_simulation import _motor, _wrench
from quadrotor_math.cascade_analysis import hover_axis_zero_order_hold

SEEDS = (30, 91001, 93003, 93012, 8200, 8201)


class _StopPrefix(Exception):
    """Explicit diagnostic termination, not mission completion."""


def local_model() -> dict[str, Any]:
    """Matched local model: x=[p,v,eta,r,a], eta=-pitch north, roll east.

    Motor acceleration feedback gain .5 changes only the s^4 coefficient to
    1.5, permitting .025*(s+12)^5. The observer's free error modes decay with
    the nominal motor lag; the five poles here assume that error is zero.
    """
    phi, gamma = hover_axis_zero_order_hold(9.81, 0.025, 0.01)
    inner = np.eye(6)
    inner[:5, :5] = phi
    inner[:5] += np.outer(gamma, [0, 0, -432, -36, -0.5, 432])
    reset = np.eye(6)
    reset[5] = [-14.4 / 9.81, -6 / 9.81, 0, 0, 0, 0]
    lifted = (inner @ inner @ reset)[:5, :5]
    return {
        "characteristic_coefficients": [0.025, 1.5, 36, 432, 2592, 6220.8],
        "lifted_transition": lifted,
        "equivalent_poles": np.log(np.linalg.eigvals(lifted).astype(complex)) / 0.02,
    }


def configuration(seed: int) -> dict[str, Any]:
    if type(seed) is not int or seed not in SEEDS:
        raise ValueError("only declared observed seeds are supported")
    kwargs = make_configuration(
        {"case": "hover", "seed": seed, "noiseless": False}, design_version=2
    )
    kwargs["position_controller"] = replace(
        kwargs["position_controller"],
        position_gain_W=np.array([14.4, 14.4, 2.25]),
        velocity_gain_W=np.array([6.0, 6.0, 3.0]),
    )
    kwargs["attitude_controller"] = replace(
        kwargs["attitude_controller"],
        attitude_gain_B=np.array([12.0, 12.0, 2.0]),
        rate_gain_B=np.array([36.0, 36.0, 8.0]),
    )
    return kwargs


def run(seed: int, output: Path, *, horizon_s: float = 10.0) -> dict[str, Any]:
    """One process-local probe. A shorter grid-aligned horizon is only for tests."""
    kwargs = configuration(seed)
    if isinstance(horizon_s, (bool, np.bool_)) or not np.isfinite(horizon_s):
        raise ValueError("horizon_s must be finite")
    steps = round(horizon_s / 0.0025)
    if not 1 <= steps <= 4000 or not np.isclose(steps * 0.0025, horizon_s, atol=1e-12, rtol=0):
        raise ValueError("horizon_s must lie on the plant grid in (0, 10]")
    model, outer = kwargs["attitude_controller"], kwargs["position_controller"]
    speeds = np.full(
        4,
        np.sqrt(
            outer.nominal_mass
            * outer.nominal_gravity_acceleration
            / (4 * model.nominal_rotors.thrust_coefficient)
        ),
    )
    last_command = None
    rows: list[Any] = []
    flags: list[bool] = []
    moments: list[Any] = []
    nominal: list[Any] = []
    original = cast(Any, mission_simulation).compute_attitude_control

    def inner(q_WB: Any, omega_B: Any, q_reference_WB: Any, thrust: float, parameters: Any) -> Any:
        nonlocal speeds, last_command
        if last_command is not None:
            speeds = _motor(speeds, last_command, parameters.nominal_rotors, 0.01)
        result = original(q_WB, omega_B, q_reference_WB, thrust, parameters)
        inertia = parameters.nominal_inertia_B
        acceleration = np.linalg.solve(
            inertia,
            _wrench(speeds, parameters.nominal_rotors)[1] - np.cross(omega_B, inertia @ omega_B),
        )
        requested = result.moment_requested_B - inertia @ (np.array([0.5, 0.5, 0]) * acceleration)
        limited = np.clip(requested, -parameters.maximum_moment_B, parameters.maximum_moment_B)
        allocation = allocate_limited_body_moment(thrust, limited, parameters.nominal_rotors)
        result = replace(
            result,
            moment_requested_B=requested,
            moment_limited_B=limited,
            moment_limited=bool(np.any(limited != requested)),
            allocation=allocation,
        )
        last_command = result.allocation.commanded_rotor_omega
        flags.append(result.rate_limited or result.moment_limited or allocation.moment_scale < 1)
        moments.append(requested)
        nominal.append(speeds.copy())
        return result

    original_sim = cast(Any, estimated_mission)._simulate_mission

    def simulate(*args: Any) -> Any:
        observer = args[-1]

        def observe(k: int, time: float, truth: Any, actual: Any) -> Any:
            estimate = observer(k, time, truth, actual)
            rows.append([time, *truth[0], *estimate[0], *estimate[1]])
            if k == steps:
                raise _StopPrefix
            return estimate

        return original_sim(*args[:-1], observe)

    with (
        patch.object(mission_simulation, "compute_attitude_control", inner),
        patch.object(estimated_mission, "_simulate_mission", simulate),
    ):
        try:
            estimated_mission.simulate_estimated_mission(**kwargs)
        except _StopPrefix:
            pass
        else:
            raise RuntimeError("mission terminated before the diagnostic horizon")
    trace = np.array(rows)
    selected = trace[:, 0] >= 5
    peak = (
        float(np.linalg.norm(trace[selected, 1:4] - [0, 0, -1], axis=1).max())
        if np.any(selected)
        else None
    )
    file = output / f"probe-{seed}.npz"
    with file.open("xb") as stream:
        np.savez_compressed(
            stream,
            trace=trace,
            moments=np.array(moments),
            limited=np.array(flags),
            nominal_rotors=np.array(nominal),
        )
    longest = current = 0
    for flag in flags:
        current = current + 1 if flag else 0
        longest = max(longest, current)
    return {
        "seed": seed,
        "peak_m": peak,
        "longest_limiting_s": longest * 0.01,
        "requested_moment_peak_Nm": np.max(abs(np.array(moments)), axis=0).tolist(),
        "file": file.name,
        "sha256": hashlib.sha256(file.read_bytes()).hexdigest(),
        "passes_prefix_screen": bool(
            horizon_s == 10 and peak is not None and peak < 0.08 and longest * 0.01 <= 0.5
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, choices=range(1, 5), default=3)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(partial(run, output=args.output), SEEDS))
    record = {
        "date_utc": datetime.now(UTC).isoformat(),
        "source_sha256": source_sha256(Path(__file__).resolve().parents[1]),
        "semantics": "Observed-case prefix screening, not full mission qualification. "
        "One fixed .5 motor-damping design; a failed screen is not promoted.",
        "results": results,
        "passes_prefix_screen": all(r["passes_prefix_screen"] for r in results),
    }
    (args.output / "report.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record))
    return 0 if record["passes_prefix_screen"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
