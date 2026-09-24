"""Observed-seed oracle substitutions to diagnose startup, never flight qualification.

The ESKF and sensor generator still run causally on each newly generated plant
trajectory. Only the state returned to the controller/supervisor is substituted.
Truth-assisted modes are deliberately non-deployable diagnostic counterfactuals.
No plant, gain, noise, prior, reference, clock or acceptance limit is changed.
"""

import argparse
import hashlib
import io
import json
from concurrent.futures import ProcessPoolExecutor
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, cast
from unittest.mock import patch

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.feedback_bandwidth_validation import make_configuration
from quadrotor_math import estimated_mission
from quadrotor_math.attitude_simulation import State
from quadrotor_math.missions import mission_reference

SEEDS = (93003, 93012)  # Already observed failures; no fresh validation seeds.
MODES = {
    "estimated": (),
    "truth_position": (0,),
    "truth_velocity": (1,),
    "truth_position_velocity": (0, 1),
    "truth_attitude": (2,),
    "truth_rate": (3,),
    "truth_attitude_rate": (2, 3),
    "truth_all": (0, 1, 2, 3),
}
FIELDS = ("position_W", "velocity_W", "q_WB", "omega_B")


class _HorizonReached(Exception):
    """End a diagnostic prefix explicitly, without fabricating a mission result."""


def select_feedback(truth: State, estimate: State, mode: str) -> State:
    """Select complete vectors in their existing frames; never mutate either input."""
    if mode not in MODES:
        raise ValueError("unknown diagnostic mode")
    chosen = [truth[j] if j in MODES[mode] else estimate[j] for j in range(4)]
    return chosen[0], chosen[1], chosen[2], chosen[3]


def run_prefix(seed: int, mode: str, horizon_s: float = 10.0) -> dict[str, NDArray[np.float64]]:
    """Record an exact 400-Hz prefix; use separate processes, never threads.

    The temporary private integration hook is restored on success or any error.
    The original mission remains intact. Stopping happens after the endpoint
    observation and before control at horizon_s, not by changing the hold window.
    """
    if type(seed) is not int or seed not in SEEDS or mode not in MODES:
        raise ValueError("only declared observed seeds and diagnostic modes are supported")
    if isinstance(horizon_s, bool) or not np.isfinite(horizon_s) or not 0 < horizon_s <= 10:
        raise ValueError("horizon_s must be finite in (0, 10]")
    kwargs = make_configuration(
        {"case": "hover", "seed": seed, "noiseless": False}, design_version=2
    )
    dt = kwargs["numerics"].time_step_s
    steps = round(horizon_s / dt)
    if steps < 1 or not np.isclose(steps * dt, horizon_s, rtol=0, atol=1e-12):
        raise ValueError("horizon_s must be on the plant grid")
    rows: dict[str, list[Any]] = {"time_s": [], "reference_position_W": []}
    for prefix in ("truth_", "estimate_", "feedback_"):
        rows.update({prefix + name: [] for name in FIELDS})
    # Dynamic access is confined to this private, process-local diagnostic hook.
    original = cast(Any, estimated_mission)._simulate_mission

    def intercepted(*args: Any) -> Any:
        observe = args[-1]

        def diagnostic(k: int, time: float, truth: State, actual: NDArray[np.float64]) -> State:
            estimate = observe(k, time, truth, actual)
            feedback = select_feedback(truth, estimate, mode)
            rows["time_s"].append(time)
            rows["reference_position_W"].append(
                mission_reference(kwargs["plan"], time)[0].position_W
            )
            for prefix, state in (
                ("truth_", truth),
                ("estimate_", estimate),
                ("feedback_", feedback),
            ):
                for name, value in zip(FIELDS, state, strict=True):
                    rows[prefix + name].append(value.copy())
            if k == steps:
                raise _HorizonReached
            return feedback

        return original(*args[:-1], diagnostic)

    with patch.object(estimated_mission, "_simulate_mission", intercepted):
        try:
            estimated_mission.simulate_estimated_mission(**kwargs)
        except _HorizonReached:
            pass
        else:
            raise RuntimeError("mission terminated before the diagnostic horizon")
    arrays = {key: np.asarray(value, dtype=np.float64) for key, value in rows.items()}
    if any(len(value) != steps + 1 or not np.all(np.isfinite(value)) for value in arrays.values()):
        raise ValueError("diagnostic prefix must be complete and finite")
    return arrays


def authenticate_original_prefix(
    arrays: dict[str, NDArray[np.float64]], archive: dict[str, NDArray[np.float64]]
) -> None:
    """Require exact agreement with the original unmodified full campaign prefix."""
    mapping = {
        "time_s": "mission_time_s",
        "reference_position_W": "mission_reference_position_W",
        **{"truth_" + name: "mission_" + name for name in FIELDS},
        **{"estimate_" + name: "estimate_" + name for name in FIELDS[:3]},
        "estimate_omega_B": "angular_velocity_estimate_B",
    }
    for key, original in mapping.items():
        if not np.array_equal(arrays[key], archive[original][: len(arrays[key])]):
            raise ValueError(f"original prefix mismatch: {key}")


def _execute(job: tuple[int, str]) -> tuple[int, str, dict[str, NDArray[np.float64]]]:
    return *job, run_prefix(*job)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--original", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--workers", type=int, default=4, choices=range(1, 5))
    args = parser.parse_args(argv)
    report = json.loads((args.original / "report.json").read_bytes())
    originals = {}
    for row in report["trials"]:
        seed = row["job"]["seed"]
        if seed in SEEDS:
            part = row["history_parts"][0]
            raw = (args.original / part["file"]).read_bytes()
            if hashlib.sha256(raw).hexdigest() != part["sha256"]:
                raise ValueError("original archive digest mismatch")
            with np.load(io.BytesIO(raw), allow_pickle=False) as archive:
                originals[seed] = {key: archive[key] for key in archive.files}
    if set(originals) != set(SEEDS):
        raise ValueError("both original failed trials are required")
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    jobs = [(seed, mode) for seed in SEEDS for mode in MODES]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for seed, mode, arrays in pool.map(_execute, jobs):
            if mode == "estimated":
                authenticate_original_prefix(arrays, originals[seed])
            selected = (arrays["time_s"] >= 5) & (arrays["time_s"] <= 10)
            error = np.linalg.norm(
                arrays["truth_position_W"] - arrays["reference_position_W"], axis=1
            )
            peak_index = np.flatnonzero(selected)[np.argmax(error[selected])]
            file = args.output / f"hover-{seed}-{mode}.npz"
            with file.open("xb") as stream:
                np.savez_compressed(stream, allow_pickle=False, **arrays)
            result = {
                "seed": seed,
                "mode": mode,
                "startup_peak_m": float(error[peak_index]),
                "peak_time_s": float(arrays["time_s"][peak_index]),
                "file": file.name,
                "sha256": hashlib.sha256(file.read_bytes()).hexdigest(),
            }
            results.append(result)
            print(json.dumps(result), flush=True)
    record = {
        "date_utc": datetime.now(UTC).isoformat(),
        "source_sha256": source_sha256(Path(__file__).resolve().parents[1]),
        "original_report_sha256": hashlib.sha256(
            (args.original / "report.json").read_bytes()
        ).hexdigest(),
        "semantics": "Diagnostic only: truth substitutions are not implementable feedback. "
        "Two observed seeds, original 0..10 s prefix, descriptive 5..10 s peak, "
        "no full-hold qualification. Nonlinear interventions are not additive contributions.",
        "original_estimated_prefixes_exact": True,
        "results": results,
    }
    (args.output / "diagnostic.json").write_bytes(canonical_json(record) + b"\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
