"""ADR0035 fixed release checks and paired local Gaussian uncertainty; no new flights."""

import argparse
import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import source_sha256
from experiments.release_prediction import predict_release_endpoint, release_endpoint_map
from experiments.release_uncertainty import finite_errors, yaw_noise_counterexample
from experiments.residual_hover_diagnostic import INPUT_SHA
from experiments.robustness_evidence import ensure, equal_json, save_json
from experiments.supported_start import configuration_for, plain_configuration
from experiments.supported_validation_protocol import jobs, prepare_job
from experiments.supported_velocity_prior import (
    RADIUS_99,
    SupportedVelocityConditioner,
    fixture_velocity_support,
)
from experiments.supported_velocity_prior import (
    evaluate as preceding_evaluation,
)
from quadrotor_math.eskf_endpoint import EskfEndpointState, EskfSampledImuNoise
from quadrotor_math.prearm_uncertainty import GRAVITY, SAMPLE_PERIOD_S, Array
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0035-release-aware-prediction.md"
TRIALS = 5000


def ballistic(endpoint: EskfEndpointState, noise: EskfSampledImuNoise) -> dict[str, Any]:
    state, h = endpoint.nominal_state, SAMPLE_PERIOD_S
    R = rotation_matrix_body_to_world(state.q_WB)
    gravity = np.array([0.0, 0, GRAVITY])
    args = (
        state.accelerometer_bias_B - R.T @ gravity,
        state.gyroscope_bias_B,
        state.accelerometer_bias_B,
        state.gyroscope_bias_B,
    )
    out = predict_release_endpoint(endpoint, *args, GRAVITY, noise, h)
    _, A, B = release_endpoint_map(state, *args, endpoint.imu_noise_mean_B, GRAVITY, h)
    ep = out.nominal_state.position_W - (
        state.position_W + h * state.velocity_W + h * h / 2 * gravity
    )
    ev = out.nominal_state.velocity_W - (state.velocity_W + h * gravity)
    radius = RADIUS_99 * float(np.sqrt(out.joint_covariance[5, 5]))
    # In the ballistic tangent map H*e1 equals the initial velocity error.
    H = np.zeros((3, 21))
    H[:, 3:6] = np.eye(3)
    H[:, 9:12] = h * R
    H[:, 15:18] = h * R
    return dict(
        initial=plain_configuration(endpoint),
        predicted=plain_configuration(out),
        A=A.tolist(),
        B=B.tolist(),
        tangent_relation=H.tolist(),
        tangent_relation_covariance=(H @ out.joint_covariance @ H.T).tolist(),
        initial_rank=int(np.linalg.matrix_rank(endpoint.joint_covariance)),
        predicted_rank=int(np.linalg.matrix_rank(out.joint_covariance)),
        position_error_W=ep.tolist(),
        velocity_error_W=ev.tolist(),
        down_standard_deviation_m_s=float(np.sqrt(out.joint_covariance[5, 5])),
        down_radius_99_m_s=radius,
        bias_inside_99_radius=bool(abs(ev[2]) <= radius),
    )


def gaussian_case(
    endpoint: EskfEndpointState, noise: EskfSampledImuNoise, standard: Array, result: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Array]]:
    C = endpoint.joint_covariance
    factor = np.zeros((33, 33))
    if np.all(C[3:6] == 0):
        keep = np.r_[0:3, 6:21]
        factor[np.ix_(keep, keep)] = np.linalg.cholesky(C[np.ix_(keep, keep)])
    else:
        factor[:21, :21] = np.linalg.cholesky(C)
    factor[21:27, 21:27] = np.linalg.cholesky(noise.sample_covariance_B)
    factor[27:, 27:] = np.linalg.cholesky(SAMPLE_PERIOD_S * noise.bias_walk_spectral_density_B)
    d = standard @ factor.T
    state = endpoint.nominal_state
    nonlinear = finite_errors(
        state,
        state.gyroscope_bias_B,
        state.accelerometer_bias_B,
        state.gyroscope_bias_B,
        endpoint.imu_noise_mean_B,
        GRAVITY,
        SAMPLE_PERIOD_S,
        d,
    )
    linear = d @ np.c_[np.array(result["A"]), np.array(result["B"])].T
    H = np.array(result["tangent_relation"])
    summaries = {}
    reported = np.array(result["predicted"]["joint_covariance"])
    for name, error in (("linear", linear), ("nonlinear", nonlinear)):
        residual = error @ H.T
        summaries[name] = dict(
            mean=error.mean(axis=0).tolist(),
            covariance=np.cov(error, rowvar=False).tolist(),
            velocity_variance_ratio=(
                error[:, 3:6].var(axis=0, ddof=1) / np.diag(reported)[3:6]
            ).tolist(),
            velocity_standardized_mean=(
                error[:, 3:6].mean(axis=0) / np.sqrt(np.diag(reported)[3:6])
            ).tolist(),
            tangent_relation_mean_m_s=residual.mean(axis=0).tolist(),
            tangent_relation_standard_deviation_m_s=residual.std(axis=0, ddof=1).tolist(),
            tangent_relation_maximum_abs_m_s=float(np.abs(residual).max()),
        )
    return summaries, dict(perturbations=d, linear_errors=linear, nonlinear_errors=nonlinear)


def definition() -> dict[str, Any]:
    return dict(
        design="ADR0035 one-sided first release prediction",
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        input_report_sha256=INPUT_SHA,
        jobs=[job.name for job in jobs()],
        trials_per_case_per_prior=TRIALS,
        gaussian_entropy=[0x52454C35, "case_index"],
        software=asdict(capture_software_provenance(ROOT)),
    )


def evaluate(campaign: Path, output: Path, *, verify: bool) -> dict[str, Any]:
    preceding = preceding_evaluation(campaign)
    rows = []
    for index, (job, previous) in enumerate(zip(jobs(), preceding["cases"], strict=True)):
        prepared = prepare_job(job)
        release = prepared.release
        assert release is not None
        noise = configuration_for(prepared, "aligned")["estimator_configuration"].sampled_imu_noise
        conditioned = SupportedVelocityConditioner().condition(
            release, fixture_velocity_support(prepared), now_s=release.fresh_sample.time_s
        )
        equal_json(
            plain_configuration(conditioned),
            previous["candidate_release"],
            "preceding conditioned release",
        )
        rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([0x52454C35, index])))
        standard = rng.standard_normal((TRIALS, 33))
        arms = {}
        arrays: dict[str, Any] = {}
        for mode, endpoint in (
            ("original_prior", release.endpoint),
            ("zero_velocity_prior", conditioned.endpoint),
        ):
            result = ballistic(endpoint, noise)
            statistics, raw = gaussian_case(endpoint, noise, standard, result)
            arms[mode] = dict(boundary=result, gaussian=statistics)
            arrays.update({mode + "_" + key: value for key, value in raw.items()})
        name = job.name + ".npz"
        target = output / name
        if verify:
            with np.load(target, allow_pickle=False) as saved:
                ensure(set(saved.files) == set(arrays), "Gaussian fields")
                for key, value in arrays.items():
                    ensure(np.array_equal(saved[key], value), "Gaussian reconstruction " + key)
        else:
            np.savez_compressed(target, **arrays)
        rows.append(
            dict(
                name=job.name,
                noise=plain_configuration(noise),
                old_boundary=previous,
                arms=arms,
                gaussian_payload=dict(
                    file=name, sha256=hashlib.sha256(target.read_bytes()).hexdigest()
                ),
            )
        )
        print(job.name + " reconstructed; first-interval checks recorded", flush=True)
    analytic = yaw_noise_counterexample()
    return dict(
        cases=rows,
        analytic_counterexample=analytic,
        decision=dict(
            corrected_ballistic_passes=sum(
                all(arm["boundary"]["bias_inside_99_radius"] for arm in row["arms"].values())
                for row in rows
            ),
            releases=len(rows),
            component_scope="experiment-only; software checks recorded separately",
            zero_velocity_prior_accepted=analytic["exact_zero_variance_claim_valid"],
            scientific_flights_executed=0,
            flight_qualification=False,
            next_step=(
                "derive nonlinear joint release uncertainty before the frozen "
                "boundary-only flight comparison"
            ),
        ),
    )


def run(campaign: Path, output: Path, *, verify: bool = False) -> dict[str, Any]:
    expected = definition()
    if verify:
        frozen = json.loads((output / "protocol.json").read_bytes())
        for key in expected:
            if key != "software":
                equal_json(frozen[key], expected[key], "protocol " + key)
    else:
        output.mkdir(parents=True, exist_ok=False)
        frozen = expected
        save_json(output, "protocol.json", frozen)
    report = dict(protocol=frozen, **evaluate(campaign, output, verify=verify))
    ensure(source_sha256(ROOT) == frozen["source_sha256"], "source changed during evaluation")
    if verify:
        equal_json(json.loads((output / "report.json").read_bytes()), report, "full release report")
    else:
        save_json(output, "report.json", report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, required=True)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--output", type=Path)
    action.add_argument("--verify", type=Path)
    args = parser.parse_args()
    report = run(args.campaign, args.verify or args.output, verify=args.verify is not None)
    print(json.dumps(report["decision"], indent=2), flush=True)


if __name__ == "__main__":
    main()
