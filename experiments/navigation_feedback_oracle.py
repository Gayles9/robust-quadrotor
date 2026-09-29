"""ADR0033: one supported-hover outer-navigation oracle, never qualification."""

import argparse
import hashlib
import io
import json
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import asdict
from pathlib import Path
from typing import Any, cast
from unittest.mock import patch

import numpy as np

from experiments import robustness_evidence
from experiments.attitude_control_validation import source_sha256
from experiments.early_flight_diagnostic import authenticated
from experiments.estimated_feedback_evidence import (
    load_history,
    pack_history,
    save_history,
    unpack_history,
)
from experiments.residual_hover_diagnostic import INPUT_SHA, authenticate
from experiments.robustness_evidence import ensure, equal_json, load_json, save_json
from experiments.supported_start import (
    PreparedStart,
    audit,
    configuration_for,
    fly,
    plain_configuration,
)
from experiments.supported_start_validation import score
from experiments.supported_validation_protocol import Job, prepare_job
from quadrotor_math import estimated_mission, mission_simulation
from quadrotor_math.attitude_simulation import State
from quadrotor_math.dynamics import quadratic_drag_force_body
from quadrotor_math.estimated_mission import EstimatedMissionResult
from quadrotor_math.position_control import compute_position_control
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0033-navigation-feedback-isolation.md"
JOB = Job("nominal_hover", 47001)
INTERVENTION = "outer controller position and velocity only; simultaneous truth"


@contextmanager
def navigation_oracle() -> Iterator[list[dict[str, Any]]]:
    """Replace only outer p/v; guards, completion and inner feedback keep estimates.

    The original observer tuple is returned unchanged. Process-local patches
    restore on all exits; never run concurrent mission threads in this context.
    """
    original = cast(Any, estimated_mission)._simulate_mission
    outer = cast(Any, mission_simulation).compute_position_control
    latest: dict[str, Any] = {}
    trace: list[dict[str, Any]] = []

    def intercept(*args: Any, **kwargs: Any) -> Any:
        observe = args[10]

        def observe_current(k: int, time: float, truth: State, actual: Any) -> State:
            feedback: State = observe(k, time, truth, actual)
            latest.update(
                index=k,
                time_s=time,
                true_p=truth[0].copy(),
                true_v=truth[1].copy(),
                estimated_p=feedback[0].copy(),
                estimated_v=feedback[1].copy(),
            )
            return feedback

        return original(*args[:10], observe_current, **kwargs)

    def control(position: Any, velocity: Any, reference: Any, parameters: Any) -> Any:
        ensure(
            latest
            and np.array_equal(position, latest["estimated_p"])
            and np.array_equal(velocity, latest["estimated_v"]),
            "oracle expected same-epoch estimated navigation",
        )
        trace.append(
            dict(
                index=latest["index"],
                time_s=latest["time_s"],
                estimated_position_W=position.tolist(),
                estimated_velocity_W=velocity.tolist(),
                feedback_position_W=latest["true_p"].tolist(),
                feedback_velocity_W=latest["true_v"].tolist(),
            )
        )
        return outer(latest["true_p"], latest["true_v"], reference, parameters)

    with (
        patch.object(estimated_mission, "_simulate_mission", intercept),
        patch.object(mission_simulation, "compute_position_control", control),
    ):
        yield trace


def audit_oracle(
    prepared: PreparedStart,
    result: EstimatedMissionResult,
    diagnostic: dict[str, Any],
    trace: list[dict[str, Any]],
) -> dict[str, float]:
    """Replay the full estimator/guard pipeline and only the changed outer dataflow."""
    m = result.mission
    indices = np.searchsorted(m.time_s, m.position_control_time_s)
    ensure(len(trace) == len(indices), "oracle trace length")
    for row, k in zip(trace, indices, strict=True):
        estimate = result.estimates.states[k]
        equal_json(
            row,
            dict(
                index=int(k),
                time_s=float(m.time_s[k]),
                estimated_position_W=estimate.position_W.tolist(),
                estimated_velocity_W=estimate.velocity_W.tolist(),
                feedback_position_W=m.position_W[k].tolist(),
                feedback_velocity_W=m.velocity_W[k].tolist(),
            ),
            "oracle trace",
        )
    sequence = iter(indices)

    def reconstruct(position: Any, velocity: Any, reference: Any, parameters: Any) -> Any:
        k = next(sequence)
        ensure(
            np.array_equal(position, result.estimates.states[k].position_W)
            and np.array_equal(velocity, result.estimates.states[k].velocity_W),
            "oracle reconstruction estimated input",
        )
        return compute_position_control(m.position_W[k], m.velocity_W[k], reference, parameters)

    with patch.object(robustness_evidence, "compute_position_control", reconstruct):
        checks = audit(prepared, "aligned", result, diagnostic)
    ensure(next(sequence, None) is None, "oracle reconstruction count")
    ensure(max(checks.values()) <= 1e-12, "oracle numerical reconstruction")
    return checks


def noise_pairing(a: dict[str, Any], b: dict[str, Any], args: dict[str, Any]) -> dict[str, float]:
    """Compare actual common-prefix draws after removing each physical signal.

    Each flight's independent named-RNG reconstruction is additionally required
    by audit(). The first acceleration measurement is mechanically supported.
    """
    n = min(len(a["mission_time_s"]), len(b["mission_time_s"]))
    ensure(np.array_equal(a["mission_time_s"][:n], b["mission_time_s"][:n]), "paired clocks")
    residuals = {}
    for name, field, bias in (
        ("accelerometer", "specific_force_measurements_B", "true_accelerometer_bias_B"),
        ("gyroscope", "angular_velocity_measurements_B", "true_gyroscope_bias_B"),
    ):
        ensure(np.array_equal(a[bias][:n], b[bias][:n]), "paired bias draws")
        draws = []
        for history in (a, b):
            physical = history["mission_omega_B"][:n].copy()
            if name == "accelerometer":
                for k in range(n):
                    rotation = rotation_matrix_body_to_world(history["mission_q_WB"][k])
                    if k == 0:
                        physical[k] = -rotation.T @ np.array(
                            [0.0, 0.0, args["truth_world"].gravity_acceleration]
                        )
                    else:
                        drag = quadratic_drag_force_body(
                            history["mission_velocity_W"][k],
                            rotation,
                            args["truth_world"].wind_velocity_W,
                            args["truth_body"].quadratic_drag_coefficient_B,
                        )
                        thrust = args["truth_rotors"].thrust_coefficient * np.sum(
                            history["mission_actual_rotor_omega"][k] ** 2
                        )
                        physical[k] = (drag + np.array([0.0, 0.0, -thrust])) / args[
                            "truth_body"
                        ].mass
            draws.append(history[field][:n] - physical - history[bias][:n])
        residuals[name + "_maximum_draw_difference"] = float(np.max(np.abs(draws[0] - draws[1])))
    identifiers, slow = [], []
    for history in (a, b):
        ids, draws = [], []
        for event in json.loads(history["metadata_json"].tobytes())["events"]:
            obs = event["observation"]
            k = obs["acquisition_index"]
            if k >= n:
                continue
            ids.append((obs["kind"], obs["observation_index"], k, obs["delivery_index"]))
            measured = np.array(obs["measurement"])
            p = history["mission_position_W"][k]
            draws.append(measured - p if measured.size == 3 else measured + p[2])
        identifiers.append(ids)
        slow.append(np.concatenate(draws) if draws else np.zeros(1))
    ensure(identifiers[0] == identifiers[1], "paired slow observation clocks")
    residuals["slow_sensor_maximum_draw_difference"] = float(np.max(np.abs(slow[0] - slow[1])))
    ensure(max(residuals.values()) <= 1e-12, "paired random draws")
    return residuals


def compare(baseline: dict[str, Any], oracle: dict[str, Any]) -> dict[str, Any]:
    """Headroom requires every frozen condition; truth never qualifies a controller."""
    conditions = dict(
        original_flight_conditions=oracle["flight_passed"],
        hover_improves=oracle["hover_peak_m"] is not None
        and oracle["hover_peak_m"] < baseline["hover_peak_m"],
        rmse_no_regression=oracle["tracking_rmse_m"] <= baseline["tracking_rmse_m"] + 1e-12,
    )
    return dict(
        conditions=conditions, useful_headroom=all(conditions.values()), qualification=False
    )


def load_baseline(campaign: Path) -> tuple[PreparedStart, dict[str, Any], dict[str, Any]]:
    """Authenticate the complete campaign and fully reconstruct the selected baseline."""
    parent = authenticate(campaign)
    row = next(x for x in parent["cases"] if x["name"] == JOB.name)
    item = row["modes"]["aligned"]
    target = campaign / JOB.name
    prepared = prepare_job(JOB)
    ensure(prepared.release is not None, "baseline support rejected")
    assert prepared.release is not None
    record = row["support"]
    with np.load(
        io.BytesIO(authenticated(target / record["file"], record["sha256"])), allow_pickle=False
    ) as saved:
        ensure(set(saved.files) == set(prepared.evidence), "baseline support fields")
        for key, value in prepared.evidence.items():
            ensure(np.array_equal(value, saved[key]), "baseline support reconstruction")
    equal_json(
        load_json(target, "release.json", row["release"]),
        asdict(prepared.release),
        "baseline release",
    )
    args = configuration_for(prepared, "aligned")
    equal_json(
        load_json(target, "configuration-aligned.json", item["configuration"]),
        {k: plain_configuration(v) for k, v in args.items()},
        "baseline configuration",
    )
    arrays = load_history(target, 1, item["history"])
    result, _ = unpack_history(arrays)
    diagnostic = load_json(target, "diagnostic-aligned.json", item["diagnostic"])
    equal_json(
        audit(prepared, "aligned", result, diagnostic), item["audit"], "baseline full replay"
    )
    equal_json(
        score(arrays, JOB.case),
        {k: v for k, v in item["metrics"].items() if k != "abort_reason"},
        "baseline scores",
    )
    ensure(result.mission.abort_reason == item["metrics"]["abort_reason"], "baseline termination")
    return prepared, arrays, row


def definition(prepared: PreparedStart, row: dict[str, Any]) -> dict[str, Any]:
    return dict(
        intervention=INTERVENTION,
        baseline_report_sha256=INPUT_SHA,
        baseline_case=row,
        configuration={
            k: plain_configuration(v) for k, v in configuration_for(prepared, "aligned").items()
        },
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        new_scientific_flights=1,
        qualification=False,
        software=asdict(capture_software_provenance(ROOT)),
    )


def verify(directory: Path, campaign: Path) -> dict[str, Any]:
    """Authenticate saved evidence and replay without executing another flight."""
    prepared, baseline, row = load_baseline(campaign)
    report: dict[str, Any] = json.loads((directory / "report.json").read_bytes())
    frozen = json.loads((directory / "protocol.json").read_bytes())
    expected = definition(prepared, row)
    ensure(set(frozen) == set(expected), "oracle protocol fields")
    for key, value in expected.items():
        if key != "software":
            equal_json(frozen[key], value, "oracle protocol " + key)
    equal_json(report["protocol"], frozen, "oracle report protocol")
    arrays = load_history(directory, 0, report["history"])
    result, _ = unpack_history(arrays)
    diagnostic = load_json(directory, "diagnostic.json", report["diagnostic"])
    trace = load_json(directory, "outer-inputs.json", report["outer_inputs"])
    equal_json(
        audit_oracle(prepared, result, diagnostic, trace), report["audit"], "oracle saved replay"
    )
    equal_json(
        noise_pairing(baseline, arrays, configuration_for(prepared, "aligned")),
        report["noise_pairing"],
        "oracle saved noise pairing",
    )
    equal_json(score(baseline, JOB.case), report["baseline_metrics"], "oracle baseline metrics")
    equal_json(score(arrays, JOB.case), report["oracle_metrics"], "oracle full metrics")
    equal_json(
        compare(report["baseline_metrics"], report["oracle_metrics"]),
        report["comparison"],
        "oracle decision",
    )
    ensure(report["abort_reason"] == result.mission.abort_reason, "oracle termination")
    return report


def run(campaign: Path, output: Path) -> dict[str, Any]:
    print("Authenticating campaign and reconstructing saved aligned baseline", flush=True)
    prepared, baseline, row = load_baseline(campaign)
    output.mkdir(parents=True, exist_ok=False)
    frozen = definition(prepared, row)
    save_json(output, "protocol.json", frozen)
    try:
        print("Executing the single frozen outer-navigation oracle", flush=True)
        with navigation_oracle() as trace:
            result, diagnostic = fly(prepared, "aligned")
        arrays = pack_history(result, result.mission)
        history = save_history(output, 0, arrays)
        diagnostics = save_json(output, "diagnostic.json", diagnostic)
        inputs = save_json(output, "outer-inputs.json", trace)
        print("Authenticating and reconstructing the complete saved oracle", flush=True)
        # Authenticate/decode the written bytes before auditing or scoring.
        restored = load_history(output, 0, history)
        decoded, _ = unpack_history(restored)
        checks = audit_oracle(prepared, decoded, diagnostic, trace)
        baseline_metrics, oracle_metrics = score(baseline, JOB.case), score(restored, JOB.case)
        report = dict(
            protocol=frozen,
            history=history,
            diagnostic=diagnostics,
            outer_inputs=inputs,
            audit=checks,
            baseline_metrics=baseline_metrics,
            oracle_metrics=oracle_metrics,
            abort_reason=decoded.mission.abort_reason,
            noise_pairing=noise_pairing(baseline, restored, configuration_for(prepared, "aligned")),
            comparison=compare(baseline_metrics, oracle_metrics),
        )
        ensure(
            source_sha256(ROOT) == frozen["source_sha256"], "oracle source changed during execution"
        )
        save_json(output, "report.json", report)
    except (ValueError, RuntimeError, FloatingPointError, np.linalg.LinAlgError) as error:
        save_json(
            output, "failure.json", dict(protocol=frozen, error=f"{type(error).__name__}: {error}")
        )
        raise
    print(
        json.dumps(
            {k: report[k] for k in ("baseline_metrics", "oracle_metrics", "comparison")}, indent=2
        ),
        flush=True,
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", required=True, type=Path)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--output", type=Path)
    action.add_argument("--verify", type=Path)
    args = parser.parse_args()
    report = verify(args.verify, args.campaign) if args.verify else run(args.campaign, args.output)
    if args.verify:
        print("Full saved navigation-oracle replay PASS", flush=True)
    return 0 if report["comparison"]["useful_headroom"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
