"""One attitude-only hover counterfactual; ADR 0025, never qualification."""

import argparse
import hashlib
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
from experiments.early_flight_diagnostic import (
    REPORT_DIGESTS,
    ROOT,
    authenticate_campaign,
    describe,
)
from experiments.estimated_feedback_evidence import (
    load_history,
    pack_history,
    save_history,
    unpack_history,
)
from experiments.robustness_evidence import ensure, equal_json, load_json, save_json
from experiments.robustness_protocol import configuration, policies, protocol
from quadrotor_math import estimated_mission, mission_simulation
from quadrotor_math.attitude_control import compute_attitude_control
from quadrotor_math.attitude_simulation import State
from quadrotor_math.eskf_live_faults import EskfLiveObservationFaults
from quadrotor_math.observation_health import ObservationHealthMonitor
from quadrotor_math.observation_supervision import ObservationSupervisor
from quadrotor_math.run_manifest import capture_software_provenance


@contextmanager
def attitude_oracle() -> Iterator[list[dict[str, Any]]]:
    """Process-local adapter. Only inner-control q changes; all guards keep ESKF q.

    ESKF inputs/state and returned observer tuple are untouched. Never use this
    alongside another thread running a mission. Patches restore on every exit.
    """
    original = cast(Any, estimated_mission)._simulate_mission
    inner_control = cast(Any, mission_simulation).compute_attitude_control
    trace: list[dict[str, Any]] = []
    latest: dict[str, Any] = {}

    def intercept(*args: Any, **kwargs: Any) -> Any:
        observe = args[10]

        def observe_current(k: int, time: float, truth: State, actual: Any) -> State:
            feedback: State = observe(k, time, truth, actual)
            latest.update(time_s=time, truth=truth[2].copy(), estimate=feedback[2].copy())
            return feedback

        return original(*args[:10], observe_current, **kwargs)

    def control(q_WB: Any, rate: Any, target: Any, thrust: Any, parameters: Any) -> Any:
        ensure(np.array_equal(q_WB, latest["estimate"]), "oracle expected same-epoch estimate")
        trace.append(
            {
                "time_s": latest["time_s"],
                "estimate_q_WB": q_WB.tolist(),
                "feedback_q_WB": latest["truth"].tolist(),
            }
        )
        return inner_control(latest["truth"], rate, target, thrust, parameters)

    with (
        patch.object(estimated_mission, "_simulate_mission", intercept),
        patch.object(mission_simulation, "compute_attitude_control", control),
    ):
        yield trace


def noise_pairing(baseline: dict[str, Any], oracle: dict[str, Any]) -> dict[str, float]:
    """Remove each run's changed physical signals; compare the actual random draws."""
    n = min(len(baseline["mission_time_s"]), len(oracle["mission_time_s"]))
    residuals: dict[str, float] = {}
    for name, field, bias in (
        ("gyro", "angular_velocity_measurements_B", "true_gyroscope_bias_B"),
        ("accelerometer", "specific_force_measurements_B", "true_accelerometer_bias_B"),
    ):
        noise = []
        for a in (baseline, oracle):
            physical = a["mission_omega_B"][:n].copy() if name == "gyro" else np.zeros((n, 3))
            if name == "accelerometer":
                physical[:, 2] = -1e-5 * np.sum(a["mission_actual_rotor_omega"][:n] ** 2, axis=1)
            noise.append(a[field][:n] - physical - a[bias][:n])
        residuals[name + "_max_abs_draw_difference"] = float(np.max(np.abs(noise[0] - noise[1])))
        ensure(np.array_equal(baseline[bias][:n], oracle[bias][:n]), "bias random draws differ")
    sources = []
    for a in (baseline, oracle):
        metadata = json.loads(a["metadata_json"].tobytes())
        values = []
        for event in metadata["events"]:
            obs = event["observation"]
            k = obs["acquisition_index"]
            if k >= n:
                continue
            measured = np.array(obs["measurement"])
            truth = a["mission_position_W"][k]
            values.append(measured - truth if measured.size == 3 else measured + truth[2])
        sources.append(np.concatenate(values))
    residuals["slow_sensor_max_abs_draw_difference"] = float(
        np.max(np.abs(sources[0] - sources[1]))
    )
    ensure(max(residuals.values()) < 1e-12, "paired sensor random draws differ")
    return residuals


def baseline_hover(directory: Path) -> dict[str, Any]:
    report = authenticate_campaign(directory, "baseline")
    record = next(r for r in report["cases"] if r["name"] == "nominal_hover")["modes"]["on"]
    return load_history(directory / "nominal_hover", 1, record["history"])


def verify(directory: Path, baseline: Path) -> dict[str, Any]:
    """Authenticate and replay ESKF/guards/outer commands and oracle inner commands."""
    report: dict[str, Any] = json.loads((directory / "report.json").read_bytes())
    ensure(report["source_sha256"] == source_sha256(ROOT), "oracle verification source mismatch")
    ensure(
        report["counterfactual"] == "inner attitude quaternion only; instantaneous truth",
        "oracle identity mismatch",
    )
    ensure(
        report["qualification"] is False and report["new_flights"] == 1,
        "oracle is not qualification",
    )
    ensure(report["baseline_report_sha256"] == REPORT_DIGESTS["baseline"], "oracle baseline")
    equal_json(report["configuration"], protocol("campaign")["cases"][0], "oracle fixture")
    definition = json.loads((directory / "protocol.json").read_bytes())
    equal_json(
        definition,
        {
            k: report[k]
            for k in (
                "counterfactual",
                "configuration",
                "baseline_report_sha256",
                "source_sha256",
                "qualification",
                "new_flights",
                "software",
            )
        },
        "oracle frozen protocol",
    )
    arrays = load_history(directory, 0, report["history"])
    result, _ = unpack_history(arrays)
    diagnostic = load_json(directory, "diagnostic.json", report["diagnostic"])
    trace = load_json(directory, "oracle-inputs.json", report["oracle_inputs"])
    m = result.mission
    ensure(len(trace) == len(m.control_time_s), "oracle trace length")
    indices = np.searchsorted(m.time_s, m.control_time_s)
    for record, k in zip(trace, indices, strict=True):
        ensure(
            record
            == {
                "time_s": float(m.time_s[k]),
                "estimate_q_WB": result.estimates.states[k].q_WB.tolist(),
                "feedback_q_WB": m.q_WB[k].tolist(),
            },
            "oracle channel/time mismatch",
        )
    sequence = iter(indices)

    def reconstruct(q_WB: Any, rate: Any, target: Any, thrust: Any, parameters: Any) -> Any:
        k = next(sequence)
        ensure(np.array_equal(q_WB, result.estimates.states[k].q_WB), "oracle replay estimate")
        return compute_attitude_control(m.q_WB[k], rate, target, thrust, parameters)

    with patch.object(robustness_evidence, "compute_attitude_control", reconstruct):
        robustness_evidence.audit_result("nominal_hover", "campaign", "on", result, diagnostic)
    ensure(next(sequence, None) is None, "oracle replay command count")
    described, _ = describe(arrays, "nominal_hover", diagnostic)
    equal_json(described, report["diagnosis"], "oracle derived claims mismatch")
    equal_json(
        noise_pairing(baseline_hover(baseline), arrays),
        report["noise_pairing"],
        "oracle random-draw pairing mismatch",
    )
    return report


def run(baseline: Path, output: Path) -> dict[str, Any]:
    baseline_report = authenticate_campaign(baseline, "baseline")
    output.mkdir(parents=True, exist_ok=False)
    source = source_sha256(ROOT)
    args = configuration("nominal_hover", "campaign")
    h, response = policies(args, "campaign")
    health = ObservationHealthMonitor(h)
    supervisor = ObservationSupervisor(health, response)
    channel = EskfLiveObservationFaults(())
    definition = {
        "counterfactual": "inner attitude quaternion only; instantaneous truth",
        "configuration": baseline_report["protocol"]["cases"][0],
        "baseline_report_sha256": hashlib.sha256(
            (baseline / "report.json").read_bytes()
        ).hexdigest(),
        "source_sha256": source,
        "qualification": False,
        "new_flights": 1,
        "software": asdict(capture_software_provenance(ROOT)),
    }
    save_json(output, "protocol.json", definition)
    try:
        with attitude_oracle() as trace:
            result = estimated_mission.simulate_estimated_mission(
                **args,
                observation_health=health,
                observation_supervision=supervisor,
                observation_faults=channel,
            )
    except (ValueError, RuntimeError, FloatingPointError, np.linalg.LinAlgError) as error:
        save_json(
            output, "failure.json", {**definition, "error": f"{type(error).__name__}: {error}"}
        )
        raise
    arrays = pack_history(result, result.mission)
    assert channel.injection is not None
    diagnostic = {
        "fault_records": [asdict(r) for r in channel.injection.records],
        "health": [asdict(s) for s in health.history],
        "supervision": [asdict(s) for s in supervisor.history],
    }
    diagnosis, signals = describe(arrays, "nominal_hover", diagnostic)
    report = {
        **definition,
        "history": save_history(output, 0, arrays),
        "diagnostic": save_json(output, "diagnostic.json", diagnostic),
        "oracle_inputs": save_json(output, "oracle-inputs.json", trace),
        "diagnosis": diagnosis,
        "noise_pairing": noise_pairing(baseline_hover(baseline), arrays),
    }
    with (output / "signals.npz").open("xb") as stream:
        np.savez_compressed(stream, **signals)
    ensure(source_sha256(ROOT) == source, "oracle source changed during execution")
    save_json(output, "report.json", report)
    verify(output, baseline)
    print(json.dumps(diagnosis["metrics"], indent=2), flush=True)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--output", type=Path)
    actions.add_argument("--verify", type=Path)
    args = parser.parse_args()
    if args.verify:
        verify(args.verify, args.baseline)
    else:
        ensure(args.baseline is not None, "baseline required")
        run(args.baseline, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
