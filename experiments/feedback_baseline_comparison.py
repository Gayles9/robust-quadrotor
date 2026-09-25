"""Compare two frozen profiles on six observed full hovers; no fresh qualification.

Run with --output NEW_DIR. Keeps compact full-rate truth/reference/moment traces;
the earlier campaigns own complete covariance/replay evidence. No gain fitting.
"""

import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.feedback_bandwidth_validation import make_configuration
from experiments.position_control_validation import score_case
from quadrotor_math.estimated_mission import simulate_estimated_mission

OBSERVED_SEEDS = (30, 91001, 93003, 93012, 8200, 8201)


def compare_case(job: tuple[int, int, str]) -> dict[str, Any]:
    version, seed, directory = job
    if type(version) is not int or version not in (1, 2) or seed not in OBSERVED_SEEDS:
        raise ValueError("only the two existing profiles and six observed seeds are permitted")
    configuration = make_configuration(
        {"case": "hover", "seed": seed, "noiseless": False}, design_version=version
    )
    result = simulate_estimated_mission(**configuration)
    mission = result.mission
    metrics = score_case({"case": "hover", "seed": seed, "refinement": 1}, mission)
    hold = (mission.time_s >= 5) & (mission.time_s <= 65)
    if mission.time_s[-1] < 65 or not np.any(hold):
        raise RuntimeError("comparison requires the complete original hold")
    error = np.linalg.norm(mission.position_W - mission.reference_position_W, axis=1)
    peak = float(np.max(error[hold]))
    effort = float(np.trapezoid(np.sum(mission.actual_moment_B**2, axis=1), mission.time_s))
    path = Path(directory) / f"v{version}-hover-{seed}.npz"
    with path.open("xb") as stream:
        np.savez_compressed(
            stream,
            time_s=mission.time_s,
            position_W=mission.position_W,
            reference_position_W=mission.reference_position_W,
            actual_moment_B=mission.actual_moment_B,
            inner_limit_flags=mission.inner_limit_flags,
            outer_limit_flags=mission.outer_limit_flags,
        )
    row = {
        "version": version,
        "seed": seed,
        "hold_peak_m": peak,
        "squared_moment_integral_N2_m2_s": effort,
        "metrics": metrics,
        "file": path.name,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }
    print(json.dumps(row), flush=True)
    return row


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=2, choices=range(1, 5))
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    jobs = [(version, seed, str(args.output)) for seed in OBSERVED_SEEDS for version in (1, 2)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(compare_case, jobs))
    report = {
        "semantics": "Observed-case full-hold comparison, not fresh qualification. "
        "Original 5..65-s scoring and both frozen profiles retained. "
        "Prefer lower worst and average hold peak, considering control effort. "
        "No performance failure is reclassified.",
        "source_sha256": source_sha256(Path(__file__).resolve().parents[1]),
        "rows": rows,
    }
    (args.output / "comparison.json").write_bytes(canonical_json(report) + b"\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
