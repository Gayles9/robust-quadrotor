"""One frozen coherent-force experiment (ADR 0020), never a production default."""

import argparse
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any
from unittest.mock import patch

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import canonical_json, source_sha256
from experiments.estimated_feedback_evidence import plain
from experiments.geometric_correction_validation import (
    planned_jobs,
    study_summary,
    verify_payloads,
)
from experiments.geometric_reimplementation_validation import configuration, execute
from quadrotor_math.geometric_control import GeometricDomainError, force_rotation_reference
from quadrotor_math.geometric_filter import FeedbackDerivativeFilter
from quadrotor_math.geometric_reference import GeometricPositionReference, build_geometric_reference
from quadrotor_math.position_control import (
    PositionControllerParameters,
    PositionReference,
    compute_position_control,
)

POLE_RAD_S = 10.0


def shaped_reference(
    position_W: NDArray[np.float64],
    velocity_W: NDArray[np.float64],
    reference: PositionReference,
    jerk_W: NDArray[np.float64],
    snap_W: NDArray[np.float64],
    parameters: PositionControllerParameters,
    memory: FeedbackDerivativeFilter,
    sample_index: int,
    *,
    estimated_acceleration_W: NDArray[np.float64] | None = None,
) -> tuple[GeometricPositionReference, FeedbackDerivativeFilter]:
    """Use Hc, sHc, s²Hc together; retain raw and shaped domain checks.

    Constant prehistory and immutable continuation are inherited from the filter.
    This experiment cannot be combined with measured derivatives or rebasing.
    Requested acceleration remains the raw PD request; feasible acceleration is
    the shaped command's nominal acceleration, with all limits still enforced.
    """
    if estimated_acceleration_W is not None:
        raise ValueError("coherent shaping requires the correction-force channel")
    raw, updated = build_geometric_reference(
        position_W, velocity_W, reference, jerk_W, snap_W, parameters, memory, sample_index
    )
    w, sections = updated.pole_rad_s, updated.sections
    m = parameters.nominal_mass
    gravity = np.array([0.0, 0.0, parameters.nominal_gravity_acceleration])
    with np.errstate(over="raise", invalid="raise"):
        acceleration = reference.acceleration_W - sections[2] / m
        shaped = compute_position_control(
            np.zeros(3),
            np.zeros(3),
            PositionReference(np.zeros(3), np.zeros(3), acceleration, reference.yaw_rad),
            parameters,
        )
        if shaped.acceleration_limited or shaped.tilt_limited or shaped.thrust_limited:
            raise GeometricDomainError("shaped_reference_domain")
        lift = m * (gravity - acceleration)
        rotation = force_rotation_reference(
            lift,
            -m * jerk_W + w * (sections[1] - sections[2]),
            -m * snap_W + w**2 * (sections[0] - 2 * sections[1] + sections[2]),
            reference.yaw_rad,
        )
    command = replace(
        raw.position_command,
        feasible_acceleration_W=shaped.feasible_acceleration_W,
        collective_thrust=shaped.collective_thrust,
        q_reference_WB=rotation.q_reference_WB,
    )
    return GeometricPositionReference(lift, rotation, command), updated


def candidate_configuration(job: dict[str, Any]) -> dict[str, Any]:
    result = configuration(job)
    if job["controller"] == "geometric":
        result["geometric_controller"] = replace(
            result["geometric_controller"], filter_pole_rad_s=POLE_RAD_S
        )
    return result


def run_item(item: tuple[int, dict[str, Any], str]) -> dict[str, Any]:
    """Patch only in a worker process, restoring the adapter on every exit."""
    config = candidate_configuration(item[1])
    with patch("quadrotor_math.mission_simulation.build_geometric_reference", shaped_reference):
        return execute(item, config_override=config)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, choices=range(1, 5), default=3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    fingerprint = source_sha256(root)
    selected = planned_jobs("development")
    protocol = dict(
        decision="ADR 0020; one candidate; stop on any development failure",
        mechanism="coherent third-section feedback force and its bilinear derivative jets",
        pole_rad_s=POLE_RAD_S,
        source_sha256=fingerprint,
        jobs=selected,
        configurations=[
            plain(
                {
                    k: asdict(v) if hasattr(v, "__dataclass_fields__") else v
                    for k, v in candidate_configuration(j).items()
                }
            )
            for j in selected
        ],
        hover_window_s=[5.0, 65.0],
        hover_peak_limit_m=0.08,
        maximum_rmse_ratio=1.0,
        maximum_effort_ratio=2.0,
        reserved_seeds_opened=False,
    )
    raw = canonical_json(protocol)
    (args.output / "protocol.json").write_bytes(raw + b"\n")
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(run_item, [(i, j, str(args.output)) for i, j in enumerate(selected)]))
    verification = verify_payloads(rows, args.output)
    summary = study_summary(rows, "development", args.output)
    report = dict(
        protocol_sha256=hashlib.sha256(raw).hexdigest(),
        source_sha256=fingerprint,
        source_unchanged=source_sha256(root) == fingerprint,
        rows=rows,
        payload_verification=verification,
        summary=summary,
    )
    (args.output / "report.json").write_bytes(canonical_json(report) + b"\n")
    print(json.dumps(summary, indent=2), flush=True)
    return (
        0
        if (
            report["source_unchanged"]
            and verification["passed"]
            and summary["candidate_tracking_and_effort_passed"]
        )
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
