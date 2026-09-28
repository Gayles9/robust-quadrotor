"""ADR 0026: authenticated measurement-only reconstruction, never a flight trial."""

import argparse
import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from experiments.attitude_control_validation import source_sha256
from experiments.early_flight_diagnostic import authenticate_campaign, authenticated, down_axis
from experiments.estimated_feedback_evidence import load_history, plain, unpack_history
from experiments.robustness_evidence import ensure, save_json
from experiments.robustness_protocol import configuration
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.eskf_consistency import eskf_right_local_error
from quadrotor_math.eskf_endpoint import initialize_eskf_endpoint, predict_eskf_endpoint
from quadrotor_math.eskf_replay import EskfObservationKind, EskfReplayStatus, _correct_eskf_epoch
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ORACLE_REPORT = "9e9a62e8f8765677ceaca3b990f60afef353aba76cd6ec629d94859ccce66172"
Array = NDArray[np.float64]


def right_jacobian(phi_B: Array) -> Array:
    """Independent integral of Exp(-s*[phi]x), using Gauss-Legendre quadrature."""
    phi = np.asarray(phi_B, dtype=float)
    ensure(phi.shape == (3,) and np.all(np.isfinite(phi)), "finite rotation vector")
    x, y, z = phi
    cross = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    angle = float(np.linalg.norm(phi))
    # Quadrature is independent of the production right-Jacobian closed form.
    nodes, weights = np.polynomial.legendre.leggauss(12)
    out = np.zeros((3, 3))
    for s, weight in zip((nodes + 1) / 2, weights / 2, strict=True):
        E = np.eye(3) - s * np.sinc(s * angle / np.pi) * cross
        E += 0.5 * s**2 * np.sinc(s * angle / (2 * np.pi)) ** 2 * (cross @ cross)
        out += weight * E
    return out


def independent_condition(C: Array, H: Array, R: Array, residual: Array) -> tuple[Array, Array]:
    """Full 21-state Gaussian conditioning and independent right-local reset."""
    ensure(C.shape == (21, 21), "joint covariance shape")
    ensure(H.ndim == 2 and H.shape[1] == 21, "joint observation shape")
    size = H.shape[0]
    ensure(R.shape == (size, size) and residual.shape == (size,), "measurement shape")
    ensure(all(np.all(np.isfinite(v)) for v in (C, H, R, residual)), "finite conditioning")
    S = H @ C @ H.T + R
    K = np.linalg.solve(S, H @ C).T
    correction = K @ residual
    reset = np.eye(21)
    reset[6:9, 6:9] = right_jacobian(correction[6:9])
    A = np.eye(21) - K @ H
    posterior = reset @ (A @ C @ A.T + K @ R @ K.T) @ reset.T
    return correction, posterior


def tilt_deg(estimate_q_WB: Array, true_q_WB: Array) -> float:
    """Stable angle between body-down axes, insensitive to quaternion sign."""
    a, b = down_axis(np.vstack((estimate_q_WB, true_q_WB)))
    return float(np.rad2deg(np.arctan2(np.linalg.norm(np.cross(a, b)), a @ b)))


def level_hover_information(gravity: float = 9.81) -> dict[str, Any]:
    """Local constant-level-thrust model; not a nonlinear flight observability claim."""
    ensure(np.isfinite(gravity) and gravity > 0, "positive finite gravity")
    F = np.zeros((15, 15))
    F[:3, 3:6] = np.eye(3)
    F[3, 7], F[4, 6] = -gravity, gravity
    F[3:6, 9:12] = -np.eye(3)
    F[6:9, 12:15] = -np.eye(3)
    H = np.zeros((3, 15))
    H[:, :3] = np.eye(3)
    observability = np.vstack([H @ np.linalg.matrix_power(F, k) for k in range(15)])
    null = np.zeros((15, 4))
    null[6, 0], null[10, 0] = 1, gravity
    null[7, 1], null[9, 1] = 1, -gravity
    null[8, 2], null[14, 3] = 1, 1
    ensure(np.max(np.abs(observability @ null)) == 0, "analytic hover nullspace")
    dt, sigma = 0.2, 0.02
    acceleration_std = np.sqrt(6) * sigma / dt**2
    return {
        "model": "level, constant specific force [0,0,-g], zero angular rate; position output",
        "rank": int(np.linalg.matrix_rank(observability)),
        "state_dimension": 15,
        "null_vectors_columns": null.tolist(),
        "null_residual_max": float(np.max(np.abs(observability @ null))),
        "three_position_acceleration_noise_std_m_s2": float(acceleration_std),
        "equivalent_small_angle_noise_std_deg": float(np.rad2deg(acceleration_std / gravity)),
        "warning": "Three-point differentiation is illustrative, not a proposed estimator.",
    }


def prior_audit() -> dict[str, Any]:
    """Compare explicit legacy assumptions with declared uniform truth moments."""
    args = configuration("nominal_hover", "campaign")
    P = args["estimator_configuration"].initial_covariance
    half_width = np.array([0.02] * 6 + [np.deg2rad(2)] * 3 + [0.02] * 3 + [0.003] * 3)
    moments = half_width**2 / 3
    return {
        "declared_prior_variance": np.diag(P).tolist(),
        "uniform_truth_variance": moments.tolist(),
        "variance_ratio_prior_to_truth": (np.diag(P) / moments).tolist(),
        "interpretation": "Legacy prior is deliberately broader, not moment-matched; unchanged.",
    }


def audit_history(arrays: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Array]]:
    """Reconstruct every epoch using only stored measurements; truth scores afterward."""
    result, _ = unpack_history(arrays)
    config = configuration("nominal_hover", "campaign")["estimator_configuration"]
    noise = config.sampled_imu_noise
    ensure(noise is not None, "endpoint fixture")
    current = initialize_eskf_endpoint(config.initial_state, config.initial_covariance, noise)
    data, saved = result.measurements, result.estimates
    n = len(data.time_s)
    events_by_epoch: dict[int, list[Any]] = {}
    for event in saved.events:
        ensure(event.observation.acquisition_index == event.observation.delivery_index, "no delay")
        events_by_epoch.setdefault(event.observation.delivery_index, []).append(event)
    errors, sigma = np.empty((n, 15)), np.empty((n, 15))
    tilt, predicted_tilt, prediction_delta = (np.zeros(n) for _ in range(3))
    nees = np.zeros(n)
    records = []
    maximum = {"correction": 0.0, "joint_covariance": 0.0, "noise_mean": 0.0, "nis": 0.0}
    min_eigenvalue = float("inf")
    prior_tilt = tilt_deg(config.initial_state.q_WB, arrays["mission_q_WB"][0])
    for k, time in enumerate(data.time_s):
        if k:
            current = predict_eskf_endpoint(
                current,
                data.specific_force_measurements_B[k - 1],
                data.angular_velocity_measurements_B[k - 1],
                data.specific_force_measurements_B[k],
                data.angular_velocity_measurements_B[k],
                config.gravity_acceleration,
                noise,
                float(time - data.time_s[k - 1]),
            )
        predicted_tilt[k] = tilt_deg(current.nominal_state.q_WB, arrays["mission_q_WB"][k])
        prediction_delta[k] = predicted_tilt[k] - (tilt[k - 1] if k else prior_tilt)
        for saved_event in events_by_epoch.get(k, []):
            obs = saved_event.observation
            before = current
            position = obs.kind is EskfObservationKind.LOCAL_POSITION
            H = np.zeros((3 if position else 1, 21))
            if position:
                H[:, :3] = np.eye(3)
                R = config.local_position_noise_covariance_W
                predicted = before.nominal_state.position_W + config.local_position_bias_W
                physical = arrays["mission_position_W"][k] + config.local_position_bias_W
            else:
                H[0, 2] = -1
                R = np.array([[config.barometric_altitude_noise_variance]])
                offset = config.barometric_reference_altitude + config.barometric_altitude_bias
                predicted = np.array([offset - before.nominal_state.position_W[2]])
                physical = np.array([offset - arrays["mission_position_W"][k, 2]])
            residual = obs.measurement - predicted
            S = H @ before.joint_covariance @ H.T + R
            nis = float(residual @ np.linalg.solve(S, residual))
            correction, covariance = independent_condition(before.joint_covariance, H, R, residual)
            _, _, endpoint, events = _correct_eskf_epoch(
                before.nominal_state, before.joint_covariance[:15, :15], before, (obs,), k, config
            )
            assert endpoint is not None
            current = endpoint
            event = events[0]
            ensure(plain(asdict(event)) == plain(asdict(saved_event)), "exact event reconstruction")
            assert event.innovation is not None
            maximum["nis"] = max(
                maximum["nis"], abs(nis - event.innovation.normalized_innovation_squared)
            )
            if event.status is EskfReplayStatus.FUSED:
                assert event.update is not None
                maximum["correction"] = max(
                    maximum["correction"],
                    float(np.max(np.abs(correction[:15] - event.update.error_state_correction))),
                )
                maximum["joint_covariance"] = max(
                    maximum["joint_covariance"],
                    float(np.max(np.abs(covariance - current.joint_covariance))),
                )
                maximum["noise_mean"] = max(
                    maximum["noise_mean"],
                    float(
                        np.max(
                            np.abs(
                                before.imu_noise_mean_B + correction[15:] - current.imu_noise_mean_B
                            )
                        )
                    ),
                )
            else:
                ensure(event.status is EskfReplayStatus.REJECTED, "expected rejection")
                ensure(
                    np.array_equal(current.joint_covariance, before.joint_covariance),
                    "rejected covariance unchanged",
                )
                correction = np.zeros(21)
            before_tilt = tilt_deg(before.nominal_state.q_WB, arrays["mission_q_WB"][k])
            after_tilt = tilt_deg(current.nominal_state.q_WB, arrays["mission_q_WB"][k])
            K = np.linalg.solve(S, H @ before.joint_covariance).T
            records.append(
                {
                    "time_s": float(time),
                    "index": k,
                    "sensor": "position" if position else "altitude",
                    "status": event.status.name,
                    "nis": nis,
                    "nis_threshold": event.nis_threshold,
                    "innovation": residual.tolist(),
                    "sensor_noise": (obs.measurement - physical).tolist(),
                    "prediction_error_component": (physical - predicted).tolist(),
                    "attitude_gain": K[6:9].tolist(),
                    "correction_15": correction[:15].tolist(),
                    "attitude_correction_from_sensor_noise_deg": np.rad2deg(
                        K[6:9] @ (obs.measurement - physical)
                    ).tolist(),
                    "attitude_correction_from_prediction_error_deg": np.rad2deg(
                        K[6:9] @ (physical - predicted)
                    ).tolist(),
                    "tilt_before_deg": before_tilt,
                    "tilt_after_deg": after_tilt,
                    "tilt_change_deg": after_tilt - before_tilt,
                    "attitude_std_before_deg": np.rad2deg(
                        np.sqrt(np.diag(before.joint_covariance)[6:9])
                    ).tolist(),
                    "attitude_std_after_deg": np.rad2deg(
                        np.sqrt(np.diag(current.joint_covariance)[6:9])
                    ).tolist(),
                }
            )
        state = current.nominal_state
        for field in state.__dataclass_fields__:
            ensure(
                np.array_equal(getattr(state, field), getattr(saved.states[k], field)),
                "exact state reconstruction",
            )
        ensure(
            np.array_equal(current.joint_covariance[:15, :15], saved.covariances[k]),
            "exact covariance reconstruction",
        )
        ensure(
            np.array_equal(current.imu_noise_mean_B, arrays["imu_noise_mean_B"][k]),
            "exact sample noise reconstruction",
        )
        reference = EskfNominalState(
            arrays["mission_position_W"][k],
            arrays["mission_velocity_W"][k],
            arrays["mission_q_WB"][k],
            arrays["true_accelerometer_bias_B"][k],
            arrays["true_gyroscope_bias_B"][k],
        )
        errors[k] = eskf_right_local_error(state, reference)
        P = current.joint_covariance[:15, :15]
        sigma[k] = np.sqrt(np.diag(P))
        nees[k] = float(errors[k] @ np.linalg.solve(P, errors[k]))
        min_eigenvalue = min(min_eigenvalue, float(np.linalg.eigvalsh(P)[0]))
        tilt[k] = tilt_deg(state.q_WB, reference.q_WB)
    ensure(max(maximum.values()) < 1e-11, "independent correction/covariance/innovation agreement")
    time = data.time_s
    norm = np.linalg.norm(
        arrays["mission_position_W"] - arrays["mission_reference_position_W"], axis=1
    )
    window = np.flatnonzero((time >= 5) & (time <= 11))
    peak = window[np.argmax(norm[window])]
    samples = []
    for requested in (
        0,
        0.04,
        0.08,
        0.12,
        0.16,
        0.1975,
        0.2,
        0.4,
        0.6,
        0.8,
        1,
        2,
        5,
        float(time[peak]),
    ):
        k = int(np.argmin(np.abs(time - requested)))
        samples.append(
            {
                "time_s": float(time[k]),
                "tilt_deg": float(tilt[k]),
                "error_15": errors[k].tolist(),
                "std_15": sigma[k].tolist(),
                "nees": float(nees[k]),
            }
        )
    budgets = []
    for end in (0.2, 1, 5, float(time[peak])):
        mask = time <= end + 1e-12
        selected = [r for r in records if r["time_s"] <= end + 1e-12]
        terms = {
            kind: sum(r["tilt_change_deg"] for r in selected if r["sensor"] == kind)
            for kind in ("position", "altitude")
        }
        prediction = float(np.sum(prediction_delta[mask]))
        final = float(tilt[np.flatnonzero(mask)[-1]])
        ensure(
            abs(prior_tilt + prediction + sum(terms.values()) - final) < 1e-11,
            "telescoping tilt budget",
        )
        budgets.append(
            {
                "end_s": end,
                "initial_deg": prior_tilt,
                "prediction_deg": prediction,
                **terms,
                "final_deg": final,
            }
        )
    return {
        "epochs": n,
        "measurement_events": len(records),
        "exact_saved_reconstruction": True,
        "independent_max_abs_residual": maximum,
        "minimum_physical_covariance_eigenvalue": min_eigenvalue,
        "samples": samples,
        "tilt_change_budgets_deg": budgets,
        "events": records,
        "first_second_nees_range": [float(np.min(nees[time <= 1])), float(np.max(nees[time <= 1]))],
        "first_second_max_component_standardized_error": float(
            np.max(np.abs(errors[time <= 1] / sigma[time <= 1]))
        ),
        "note": "Single correlated trajectory; no population-calibration or flight-pass claim.",
    }, {
        "time_s": time,
        "error_15": errors,
        "std_15": sigma,
        "tilt_deg": tilt,
        "predicted_tilt_deg": predicted_tilt,
        "prediction_delta_deg": prediction_delta,
        "nees": nees,
    }


def run(baseline: Path, oracle: Path, output: Path) -> dict[str, Any]:
    report = authenticate_campaign(baseline, "baseline")
    prior = json.loads(authenticated(oracle / "report.json", ORACLE_REPORT))
    ensure(prior["qualification"] is False, "oracle is not qualification")
    original = next(r for r in report["cases"] if r["name"] == "nominal_hover")["modes"]["on"]
    source = source_sha256(ROOT)
    output.mkdir(parents=True, exist_ok=False)
    definition = {
        "audit_source_sha256": source,
        "oracle_report_sha256": ORACLE_REPORT,
        "original_report_sha256": hashlib.sha256(
            (baseline / "report.json").read_bytes()
        ).hexdigest(),
        "adr_sha256": hashlib.sha256(
            (ROOT / "docs/decisions/0026-attitude-startup-audit.md").read_bytes()
        ).hexdigest(),
        "software": asdict(capture_software_provenance(ROOT)),
        "qualification": False,
        "new_experimental_conditions": 0,
        "truth_usage": (
            "evaluation after prediction/update only; never feedback or reconstruction input"
        ),
    }
    save_json(output, "protocol.json", definition)
    result: dict[str, Any] = {
        **definition,
        "prior_audit": prior_audit(),
        "local_information": level_hover_information(),
        "cases": {},
    }
    for name, directory, index, history in (
        ("original", baseline / "nominal_hover", 1, original["history"]),
        ("oracle", oracle, 0, prior["history"]),
    ):
        summary, signals = audit_history(load_history(directory, index, history))
        with (output / f"{name}.npz").open("xb") as stream:
            serializable: dict[str, Any] = dict(signals)
            np.savez_compressed(stream, **serializable)
        result["cases"][name] = summary
        print(
            json.dumps(
                {
                    "case": name,
                    "epochs": summary["epochs"],
                    "budget": summary["tilt_change_budgets_deg"][0],
                }
            ),
            flush=True,
        )
    ensure(source_sha256(ROOT) == source, "audit source changed during execution")
    save_json(output, "report.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--oracle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.baseline, args.oracle, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
