"""ADR0038 authenticated saved-data accounting; no scientific flight execution."""

import argparse
import hashlib
import json
from collections.abc import Iterator
from concurrent.futures import ProcessPoolExecutor
from contextlib import contextmanager, nullcontext
from functools import partial
from pathlib import Path
from typing import Any
from unittest.mock import patch

import numpy as np

from experiments.attitude_control_validation import source_sha256
from experiments.attitude_startup_audit import independent_condition
from experiments.combined_supported_prior import BOUNDARY_SHA, combined_prediction, condition_start
from experiments.durable_release_evidence import save_arrays, save_json
from experiments.early_flight_diagnostic import authenticated
from experiments.estimated_feedback_evidence import load_history, unpack_history
from experiments.navigation_feedback_oracle import noise_pairing
from experiments.nonlinear_release import nonlinear_release_prediction
from experiments.nonlinear_release_validation import load_inputs
from experiments.residual_hover_diagnostic import (
    CHANNELS,
    INPUT_SHA,
    command_residual,
    local_cascade,
    response_budget,
)
from experiments.robustness_evidence import ensure, equal_json, load_json
from experiments.supported_start import audit, configuration_for, plain_configuration
from experiments.supported_start_validation import score
from experiments.supported_validation_protocol import CLEAN, Job, jobs
from quadrotor_math import eskf_replay
from quadrotor_math.eskf_replay import EskfObservationKind, EskfReplayStatus
from quadrotor_math.prearm_uncertainty import Array

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0038-combined-prior-tradeoff-diagnosis.md"
REPORTS = dict(
    original=INPUT_SHA,
    boundary=BOUNDARY_SHA,
    combined="6d44ab913156fafe3846a630065cdaad9eee2dc9c64c1ea50234b82fbfa314fa",
)
PRECEDING_SOURCE = "0073d6024a5fea509fe3aaf2dca06e3c5b835b708eb637136c0043833926a12b"
COMMAND_CHANNELS = ("reference", "true_feedback", "navigation_position", "navigation_velocity")
BLOCKS = dict(
    position=slice(0, 3),
    velocity=slice(3, 6),
    attitude=slice(6, 9),
    accelerometer_bias=slice(9, 12),
    gyroscope_bias=slice(12, 15),
)


def gaussian_split(C: Array, H: Array, R: Array, noise: Array, prediction: Array) -> dict[str, Any]:
    """Independent joint conditioning split into noise and prediction-error terms."""
    ensure(noise.shape == prediction.shape and np.all(np.isfinite(noise)), "innovation components")
    innovation = noise + prediction
    correction, covariance = independent_condition(C, H, R, innovation)
    S = H @ C @ H.T + R
    gain = np.linalg.solve(S, H @ C).T
    return dict(
        gain=gain,
        innovation=innovation,
        correction=correction,
        covariance=covariance,
        noise=gain @ noise,
        prediction=gain @ prediction,
        nis=float(innovation @ np.linalg.solve(S, innovation)),
    )


def correction_change(control: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Array]:
    """Ordered algebraic difference, with zero effective gains for rejected events."""
    K0, K1 = (np.asarray(x["gain"], dtype=float) for x in (control, candidate))
    n0, n1 = (np.asarray(x["innovation"], dtype=float) for x in (control, candidate))
    ensure(
        K0.ndim == 2 and K0.shape == K1.shape and n0.shape == n1.shape == (K0.shape[1],),
        "paired gain shape",
    )
    ensure(all(np.all(np.isfinite(x)) for x in (K0, K1, n0, n1)), "finite paired gain")
    gain, innovation = (K1 - K0) @ n0, K1 @ (n1 - n0)
    total = K1 @ n1 - K0 @ n0
    ensure(np.max(np.abs(gain + innovation - total)) <= 1e-12, "correction change closure")
    return dict(gain=gain, innovation=innovation, total=total)


def signed_mse(error: Array, channels: Array) -> Array:
    """Signed projection shares; correlated cancellation is retained, not normalized."""
    ensure(error.ndim == 2 and error.shape[1] == 3 and len(error) > 0, "error shape")
    ensure(
        channels.ndim == 3 and channels.shape[0] == len(error) and channels.shape[2] == 3,
        "channel shape",
    )
    ensure(np.all(np.isfinite(error)) and np.all(np.isfinite(channels)), "finite MSE inputs")
    ensure(np.max(np.abs(channels.sum(axis=1) - error)) <= 1e-10, "response sum")
    shares = np.mean(np.einsum("ni,nci->nc", error, channels), axis=0)
    ensure(
        abs(float(shares.sum()) - float(np.mean(np.sum(error**2, axis=1)))) <= 1e-12, "MSE closure"
    )
    return np.asarray(shares, dtype=np.float64)


def command_terms(a: dict[str, Any], args: dict[str, Any]) -> Array:
    """Requested acceleration identity at actual outer-loop epochs, in NED SI units."""
    t, clock = a["mission_time_s"], a["mission_position_control_time_s"]
    indices = np.searchsorted(t, clock)
    ensure(
        len(clock) > 0 and np.all(np.diff(clock) > 0) and np.all(indices < len(t)), "outer clock"
    )
    ensure(np.array_equal(t[indices], clock), "outer clock")
    kp, kv = (
        args["position_controller"].position_gain_W,
        args["position_controller"].velocity_gain_W,
    )
    p, v = a["mission_position_W"][indices], a["mission_velocity_W"][indices]
    terms = np.stack(
        (
            (
                kp * a["mission_reference_position_W"]
                + kv * a["mission_reference_velocity_W"]
                + a["mission_reference_acceleration_W"]
            )[indices],
            -kp * p - kv * v,
            -kp * (a["estimate_position_W"][indices] - p),
            -kv * (a["estimate_velocity_W"][indices] - v),
        ),
        axis=1,
    )
    ensure(
        np.max(np.abs(terms.sum(axis=1) - a["mission_requested_acceleration_W"])) <= 1e-12,
        "requested acceleration closure",
    )
    return np.asarray(terms, dtype=np.float64)


@contextmanager
def capture_updates(a: dict[str, Any], args: dict[str, Any]) -> Iterator[dict[str, Any]]:
    """Observe ordered replay updates without changing any prediction or measurement."""
    original = eskf_replay._correct_eskf_epoch
    trace: dict[str, Any] = dict(
        events=[], maximum=dict(correction=0.0, covariance=0.0, noise_mean=0.0, nis=0.0)
    )

    def corrected(
        state: Any,
        covariance: Array,
        endpoint: Any,
        observations: Any,
        index: int,
        configuration: Any,
    ) -> Any:
        events: list[Any] = []
        for observation in observations:
            ensure(
                endpoint is not None and observation.acquisition_index == index,
                "fresh joint observation",
            )
            before = endpoint
            position = observation.kind is EskfObservationKind.LOCAL_POSITION
            H = np.zeros((3 if position else 1, 21))
            truth = a["mission_position_W"][index]
            if position:
                H[:, :3] = np.eye(3)
                R = configuration.local_position_noise_covariance_W
                predicted = state.position_W + configuration.local_position_bias_W
                physical = truth + configuration.local_position_bias_W
            else:
                H[0, 2] = -1
                R = np.array([[configuration.barometric_altitude_noise_variance]])
                offset = (
                    configuration.barometric_reference_altitude
                    + configuration.barometric_altitude_bias
                )
                predicted = np.array([offset - state.position_W[2]])
                physical = np.array([offset - truth[2]])
            sensor_noise, prediction_error = (
                observation.measurement - physical,
                physical - predicted,
            )
            split = gaussian_split(before.joint_covariance, H, R, sensor_noise, prediction_error)
            state, covariance, endpoint, one = original(
                state, covariance, endpoint, (observation,), index, configuration
            )
            event = one[0]
            ensure(
                event.status in (EskfReplayStatus.FUSED, EskfReplayStatus.REJECTED),
                "clean update disposition",
            )
            fused = event.status is EskfReplayStatus.FUSED
            assert endpoint is not None and event.innovation is not None
            maximum = trace["maximum"]
            maximum["nis"] = max(
                maximum["nis"], abs(split["nis"] - event.innovation.normalized_innovation_squared)
            )
            if fused:
                assert event.update is not None
                for key, residual in (
                    ("correction", split["correction"][:15] - event.update.error_state_correction),
                    ("covariance", split["covariance"] - endpoint.joint_covariance),
                    (
                        "noise_mean",
                        before.imu_noise_mean_B
                        + split["correction"][15:]
                        - endpoint.imu_noise_mean_B,
                    ),
                ):
                    maximum[key] = max(maximum[key], float(np.max(np.abs(residual))))
            else:
                ensure(
                    np.array_equal(before.joint_covariance, endpoint.joint_covariance),
                    "rejected covariance",
                )
            ensure(
                maximum["nis"] <= 1e-9
                and all(v <= 1e-10 for k, v in maximum.items() if k != "nis"),
                "independent update check",
            )
            trace["events"].append(
                dict(
                    index=index,
                    time_s=float(a["mission_time_s"][index]),
                    sensor="position" if position else "altitude",
                    observation_index=observation.observation_index,
                    fused=fused,
                    nis=split["nis"],
                    gain=(split["gain"] if fused else np.zeros_like(split["gain"])).tolist(),
                    innovation=split["innovation"].tolist(),
                    sensor_noise=sensor_noise.tolist(),
                    prediction_error=prediction_error.tolist(),
                    correction=(split["correction"] if fused else np.zeros(21)).tolist(),
                    noise_correction=(split["noise"] if fused else np.zeros(21)).tolist(),
                    prediction_correction=(split["prediction"] if fused else np.zeros(21)).tolist(),
                    prior_variance=np.diag(before.joint_covariance).tolist(),
                    posterior_variance=np.diag(endpoint.joint_covariance).tolist(),
                )
            )
            events.extend(one)
        return state, covariance, endpoint, tuple(events)

    with patch.object(eskf_replay, "_correct_eskf_epoch", corrected):
        yield trace


def preceding_source_sha256() -> str:
    digest = hashlib.sha256()
    paths = [
        *ROOT.glob("src/quadrotor_math/*.py"),
        *ROOT.glob("experiments/*.py"),
        ROOT / "pyproject.toml",
        ROOT / "uv.lock",
    ]
    for path in sorted(p for p in paths if p != Path(__file__).resolve()):
        data = path.read_bytes()
        digest.update(path.relative_to(ROOT).as_posix().encode() + b"\0")
        digest.update(str(len(data)).encode() + b"\0" + data)
    return digest.hexdigest()


def authenticate_inputs(directories: dict[str, Path]) -> dict[str, Any]:
    """Authenticate every original outcome; no filtering precedes integrity/scoring."""
    reports = {}
    for label, expected in REPORTS.items():
        directory = directories[label]
        report = json.loads(authenticated(directory / "report.json", expected))
        equal_json(
            json.loads((directory / "protocol.json").read_bytes()),
            report["protocol"],
            "input protocol",
        )
        ensure([r["name"] for r in report["cases"]] == [j.name for j in jobs()], "input ledger")
        count = 0
        for row in report["cases"]:
            target = directory / row["name"]
            equal_json(json.loads((target / "case.json").read_bytes()), row, "input case")
            for key in ("support", "release"):
                if key in row:
                    authenticated(target / row[key]["file"], row[key]["sha256"])
            for item in row["modes"].values():
                for record in (item["configuration"], item["diagnostic"], *item["history"]):
                    authenticated(target / record["file"], record["sha256"])
                with np.load(target / item["history"][0]["file"], allow_pickle=False) as arrays:
                    metric = score(dict(arrays), row["case"])
                saved = item["metrics" if label == "original" else "candidate_metrics"]
                equal_json(
                    metric, {k: v for k, v in saved.items() if k != "abort_reason"}, "saved score"
                )
                count += 1
        ensure(count == (34 if label == "original" else 25), "full input flight count")
        reports[label] = report
    return reports


def rms(a: Array) -> float:
    return float(np.sqrt(np.mean(np.sum(a**2, axis=-1))))


def compare_events(
    control: dict[str, Any], candidate: dict[str, Any], end_time: float
) -> dict[str, Any]:
    left = [e for e in control["events"] if e["time_s"] <= end_time]
    right = [e for e in candidate["events"] if e["time_s"] <= end_time]
    ensure(len(left) == len(right) > 0, "paired event ledger")
    rows = []
    for a, b in zip(left, right, strict=True):
        ensure(
            all(a[k] == b[k] for k in ("index", "sensor", "observation_index")),
            "paired event identity",
        )
        ensure(
            np.max(np.abs(np.array(a["sensor_noise"]) - b["sensor_noise"])) <= 1e-12,
            "paired observation noise",
        )
        change = correction_change(a, b)
        rows.append(
            dict(
                time_s=a["time_s"],
                sensor=a["sensor"],
                disposition_changed=a["fused"] != b["fused"],
                **{k: v.tolist() for k, v in change.items()},
            )
        )
    windows = {}
    for name, lo, hi in (
        ("release", -1, 0.5),
        ("takeoff", 0.5, 5),
        ("active", 5, 11),
        ("full", -1, end_time),
    ):
        selected = [r for r in rows if lo < r["time_s"] <= hi]
        windows[name] = {
            block: {
                term: rms(np.array([r[term][indices] for r in selected]))
                for term in ("gain", "innovation", "total")
            }
            for block, indices in BLOCKS.items()
        }
    first = next(
        (
            dict(control=a, candidate=b)
            for a, b in zip(left, right, strict=True)
            if a["time_s"] > 0 and a["sensor"] == "position"
        ),
        None,
    )
    return dict(
        paired_events=len(rows),
        end_time_s=end_time,
        disposition_changes=sum(r["disposition_changed"] for r in rows),
        windows=windows,
        first_position=first,
        events=rows,
    )


def diagnose_job(
    job: Job, *, directories: dict[str, Path], reports: dict[str, Any], output: Path, verify: bool
) -> dict[str, Any]:
    source_rows = {
        label: next(r for r in report["cases"] if r["name"] == job.name)
        for label, report in reports.items()
    }
    prepared, _, _ = load_inputs(directories["original"], job, source_rows["original"])
    candidate = condition_start(prepared)
    target = output / job.name
    if not verify:
        target.mkdir()
    arms, arrays_by_arm, traces, budgets, command_by_arm, models, results = (
        {},
        {},
        {},
        {},
        {},
        {},
        {},
    )
    for label in REPORTS:
        source = directories[label] / job.name
        item = source_rows[label]["modes"]["aligned"]
        current = candidate if label == "combined" else prepared
        args = configuration_for(current, "aligned")
        equal_json(
            load_json(source, "configuration-aligned.json", item["configuration"]),
            {k: plain_configuration(v) for k, v in args.items()},
            "clean configuration",
        )
        arrays = load_history(source, 1 if label == "original" else 0, item["history"])
        result, _ = unpack_history(arrays)
        diagnostic = load_json(source, "diagnostic-aligned.json", item["diagnostic"])
        context = (
            combined_prediction(prepared, candidate, "replay")
            if label == "combined"
            else nonlinear_release_prediction(prepared, "replay")
            if label == "boundary"
            else nullcontext(None)
        )
        with context as release_trace, capture_updates(arrays, args) as trace:
            reconstruction = audit(
                current,
                "aligned",
                result,
                {k: v for k, v in diagnostic.items() if k != "release_prediction"},
            )
        equal_json(reconstruction, item["audit"], "saved complete replay record")
        if release_trace is not None:
            equal_json(release_trace, diagnostic["release_prediction"], "first prediction trace")
        command = command_residual(arrays, args)
        terms, budget = command_terms(arrays, args), response_budget(arrays, args)
        error = arrays["mission_position_W"] - arrays["mission_reference_position_W"]
        shares = signed_mse(error, budget["response_error_W"])
        metric = score(arrays, job.case)
        model, model_summary = (
            local_cascade(arrays, args) if job.case == "nominal_hover" else ({}, None)
        )
        if model_summary is not None:
            model_summary["agrees_within_2mm_rms"] = (
                model_summary["horizontal_rms_discrepancy_m"] <= 0.002
            )
        payload = dict(
            time_s=arrays["mission_time_s"],
            true_error_W=error,
            command_time_s=arrays["mission_position_control_time_s"],
            command_terms_W=terms,
            **budget,
            **model,
        )
        event_file, array_file = label + "-updates.json", label + "-response.npz"
        if verify:
            equal_json(
                json.loads((target / event_file).read_bytes()),
                trace,
                "saved observation accounting",
            )
            with np.load(target / array_file, allow_pickle=False) as saved_arrays:
                ensure(set(saved_arrays.files) == set(payload), "saved response schema")
                for key, value in payload.items():
                    ensure(np.array_equal(saved_arrays[key], value), "saved response payload")
        else:
            save_json(target, event_file, trace)
            save_arrays(target / array_file, payload)
        arms[label] = dict(
            metrics=metric,
            audit=reconstruction,
            independent_update=trace["maximum"],
            command_residual=command,
            acceleration_closure=float(budget["acceleration_closure"]),
            response_closure=float(budget["response_closure"]),
            mse_shares_m2=dict(zip(CHANNELS, shares.tolist(), strict=True)),
            hover_model=model_summary,
            events=len(trace["events"]),
        )
        (
            arrays_by_arm[label],
            traces[label],
            budgets[label],
            command_by_arm[label],
            models[label],
            results[label],
        ) = arrays, trace, budget, terms, model, result
    comparisons = {}
    for label in ("original", "boundary"):
        a, b = arrays_by_arm[label], arrays_by_arm["combined"]
        n = min(len(a["mission_time_s"]), len(b["mission_time_s"]))
        ensure(
            np.array_equal(a["mission_time_s"][:n], b["mission_time_s"][:n]),
            "paired physical clock",
        )
        end_time = float(a["mission_time_s"][n - 1])
        correction = compare_events(traces[label], traces["combined"], end_time)
        m = min(len(command_by_arm[label]), len(command_by_arm["combined"]))
        ensure(
            np.array_equal(
                a["mission_position_control_time_s"][:m], b["mission_position_control_time_s"][:m]
            ),
            "paired command clock",
        )
        delta_command = command_by_arm["combined"][:m] - command_by_arm[label][:m]
        delta_response = (
            budgets["combined"]["response_error_W"][:n] - budgets[label]["response_error_W"][:n]
        )
        delta_actual = (b["mission_position_W"] - b["mission_reference_position_W"])[:n] - (
            a["mission_position_W"] - a["mission_reference_position_W"]
        )[:n]
        ensure(
            np.max(np.abs(delta_response.sum(axis=1) - delta_actual)) <= 1e-10,
            "paired response closure",
        )
        mse_delta = {
            k: arms["combined"]["mse_shares_m2"][k] - arms[label]["mse_shares_m2"][k]
            for k in CHANNELS
        }
        squared_score_delta = (
            arms["combined"]["metrics"]["tracking_rmse_m"] ** 2
            - arms[label]["metrics"]["tracking_rmse_m"] ** 2
        )
        ensure(
            abs(sum(mse_delta.values()) - squared_score_delta) <= 1e-12,
            "full-duration score change",
        )
        model_delta = None
        if job.case == "nominal_hover":
            model_delta = rms(
                models["combined"]["cascade_states"][:n, 0]
                - models[label]["cascade_states"][:n, 0]
                - delta_actual[:, :2]
            )
        comparisons[label] = dict(
            noise_pairing=noise_pairing(
                a,
                b,
                configuration_for(prepared, "aligned"),
            ),
            common_time_s=end_time,
            durations_s={k: arms[k]["metrics"]["terminal_time_s"] for k in (label, "combined")},
            correction={k: v for k, v in correction.items() if k != "events"},
            mse_change_m2=mse_delta,
            squared_rmse_change_m2=squared_score_delta,
            command_change_rms_m_s2={
                k: rms(delta_command[:, i]) for i, k in enumerate(COMMAND_CHANNELS)
            },
            hover_model_delta_rms_m=model_delta,
        )
        if not verify:
            save_json(target, label + "-correction-change.json", correction)
        else:
            equal_json(
                json.loads((target / (label + "-correction-change.json")).read_bytes()),
                correction,
                "paired correction accounting",
            )
    row = dict(name=job.name, case=job.case, seed=job.seed, arms=arms, comparisons=comparisons)
    if verify:
        equal_json(json.loads((target / "case.json").read_bytes()), row, "diagnostic case")
    else:
        save_json(target, "case.json", row)
    print(job.name + " all three saved histories reconstructed", flush=True)
    return row


def run(
    directories: dict[str, Path], output: Path, workers: int, *, verify: bool = False
) -> dict[str, Any]:
    ensure(type(workers) is int and workers in (1, 2), "one or two worker processes")
    ensure(preceding_source_sha256() == PRECEDING_SOURCE, "preceding source identity")
    reports = authenticate_inputs(directories)
    frozen = dict(
        design="ADR0038 saved combined-prior tradeoff",
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        inputs=REPORTS,
        jobs=[j.name for j in jobs() if j.case in CLEAN],
        new_scientific_flights=0,
    )
    if verify:
        equal_json(
            json.loads((output / "protocol.json").read_bytes()), frozen, "diagnostic protocol"
        )
    else:
        output.mkdir(parents=True, exist_ok=False)
        save_json(output, "protocol.json", frozen)
    operation = partial(
        diagnose_job, directories=directories, reports=reports, output=output, verify=verify
    )
    planned = [j for j in jobs() if j.case in CLEAN]
    if workers == 1:
        rows = [operation(j) for j in planned]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(operation, planned))
    ensure(
        source_sha256(ROOT) == frozen["source_sha256"], "diagnostic source changed during execution"
    )
    ensure(
        [r["name"] for r in rows] == frozen["jobs"] and sum(len(r["arms"]) for r in rows) == 27,
        "complete diagnostic ledger",
    )
    summary = dict(
        authenticated_flights=84,
        reconstructed_clean_histories=27,
        new_scientific_flights=0,
        exact_diagnostic_passed=True,
        unfitted_hover_model_passed=all(
            a["hover_model"]["agrees_within_2mm_rms"]
            for r in rows
            if r["case"] == "nominal_hover"
            for a in r["arms"].values()
        ),
        prior_frozen_comparison_passed=False,
        adopted_as_default=False,
        fresh_validation_complete=False,
    )
    report = dict(protocol=frozen, cases=rows, summary=summary)
    if verify:
        equal_json(json.loads((output / "report.json").read_bytes()), report, "diagnostic report")
    else:
        save_json(output, "report.json", report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for label in REPORTS:
        parser.add_argument("--" + label, type=Path, required=True)
    parser.add_argument("--workers", type=int, default=2)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output", type=Path)
    group.add_argument("--verify", type=Path)
    args = parser.parse_args()
    report = run(
        {k: getattr(args, k) for k in REPORTS},
        args.verify or args.output,
        args.workers,
        verify=args.verify is not None,
    )
    print(json.dumps(report["summary"], indent=2), flush=True)


if __name__ == "__main__":
    main()
