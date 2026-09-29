"""ADR0036 authenticated release inputs and fixed nonlinear uncertainty evaluation."""

import argparse
import hashlib
import io
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import source_sha256
from experiments.durable_release_evidence import save_arrays, save_json
from experiments.early_flight_diagnostic import authenticated
from experiments.nonlinear_release import predict_nonlinear_release_endpoint, structural_factor
from experiments.prearm_nonlinear_uncertainty import local_logs, quaternion_product
from experiments.release_prediction import release_endpoint_map
from experiments.release_uncertainty import exp_batch, finite_errors
from experiments.residual_hover_diagnostic import INPUT_SHA, authenticate
from experiments.robustness_evidence import ensure, equal_json, load_json, observation
from experiments.supported_start import PreparedStart, configuration_for, plain_configuration
from experiments.supported_validation_protocol import CLEAN, Job, jobs, prepare_job
from experiments.supported_velocity_prior import (
    SupportedVelocityConditioner,
    fixture_velocity_support,
)
from quadrotor_math.eskf_endpoint import EskfEndpointState
from quadrotor_math.eskf_replay import _correct_eskf_epoch
from quadrotor_math.prearm_uncertainty import Array
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0036-nonlinear-release-flight-comparison.md"
TRIALS = 20000


def load_inputs(
    campaign: Path, job: Job, row: dict[str, Any]
) -> tuple[PreparedStart, dict[str, Any], dict[str, Any]]:
    """Reconstruct support/configuration and retain the authenticated first two samples."""
    prepared = prepare_job(job)
    assert prepared.release is not None
    target = campaign / job.name
    with np.load(
        io.BytesIO(authenticated(target / row["support"]["file"], row["support"]["sha256"])),
        allow_pickle=False,
    ) as saved:
        ensure(set(saved.files) == set(prepared.evidence), "support fields")
        for key, value in prepared.evidence.items():
            ensure(np.array_equal(saved[key], value), "support reconstruction")
    equal_json(
        load_json(target, "release.json", row["release"]), asdict(prepared.release), "release"
    )
    mode = "aligned" if job.case in CLEAN else "on"
    item = row["modes"][mode]
    args = configuration_for(prepared, "aligned")
    equal_json(
        load_json(target, f"configuration-{mode}.json", item["configuration"]),
        {key: plain_configuration(value) for key, value in args.items()},
        "configuration",
    )
    record = item["history"][0]
    with np.load(
        io.BytesIO(authenticated(target / record["file"], record["sha256"])), allow_pickle=False
    ) as data:
        raw = dict(data)
    observations = tuple(
        observation(e["observation"])
        for e in json.loads(raw["metadata_json"].tobytes())["events"]
        if e["observation"]["delivery_index"] == 0
    )
    inputs = dict(
        measurements=[
            raw["specific_force_measurements_B"][0],
            raw["angular_velocity_measurements_B"][0],
            raw["specific_force_measurements_B"][1],
            raw["angular_velocity_measurements_B"][1],
        ],
        observations=observations,
        h=float(raw["mission_time_s"][1] - raw["mission_time_s"][0]),
        baseline_history=record,
        initial_estimate={
            key: raw["estimate_" + key][0]
            for key in prepared.release.endpoint.nominal_state.__dataclass_fields__
        },
    )
    return prepared, args, inputs


def initial_posterior(
    prepared: PreparedStart, args: dict[str, Any], inputs: dict[str, Any], conditioned: bool
) -> EskfEndpointState:
    release = prepared.release
    assert release is not None
    if conditioned:
        release = SupportedVelocityConditioner().condition(
            release, fixture_velocity_support(prepared), now_s=release.fresh_sample.time_s
        )
    endpoint = release.endpoint
    _, _, posterior, _ = _correct_eskf_epoch(
        endpoint.nominal_state,
        endpoint.joint_covariance[:15, :15],
        endpoint,
        inputs["observations"],
        0,
        args["estimator_configuration"],
    )
    assert posterior is not None
    equal_json(
        asdict(posterior.nominal_state), inputs["initial_estimate"], "causal initial posterior"
    )
    return posterior


def accuracy(candidate: EskfEndpointState, reference: EskfEndpointState) -> dict[str, Any]:
    L = np.linalg.cholesky(reference.joint_covariance)
    delta = candidate.joint_covariance - reference.joint_covariance
    whitened = np.linalg.solve(L, np.linalg.solve(L, delta).T).T
    error = np.r_[
        candidate.nominal_state.position_W - reference.nominal_state.position_W,
        candidate.nominal_state.velocity_W - reference.nominal_state.velocity_W,
        np.zeros(3),
        candidate.nominal_state.accelerometer_bias_B - reference.nominal_state.accelerometer_bias_B,
        candidate.nominal_state.gyroscope_bias_B - reference.nominal_state.gyroscope_bias_B,
        candidate.imu_noise_mean_B - reference.imu_noise_mean_B,
    ]
    mean_error = float(np.max(np.abs(error) / np.sqrt(np.diag(reference.joint_covariance))))
    attitude = float(
        np.linalg.norm(
            local_logs(reference.nominal_state.q_WB, candidate.nominal_state.q_WB[None, :])[0]
        )
    )
    covariance_error = float(np.max(np.abs(np.linalg.eigvalsh((whitened + whitened.T) / 2))))
    return dict(
        covariance_error=covariance_error,
        standardized_mean_error=mean_error,
        attitude_mean_error_rad=attitude,
        passed=covariance_error <= 0.001 and mean_error <= 1e-6 and attitude <= 1e-8,
    )


def gaussian_errors(
    current: EskfEndpointState,
    measurements: list[Array],
    g: float,
    noise: Any,
    h: float,
    output: EskfEndpointState,
    standard: Array,
) -> Array:
    factor = np.zeros((33, 33))
    root, active = structural_factor(current.joint_covariance)
    factor[np.ix_(np.arange(21), active)] = root
    root, active = structural_factor(noise.sample_covariance_B)
    factor[np.ix_(np.arange(21, 27), 21 + active)] = root
    root, active = structural_factor(h * noise.bias_walk_spectral_density_B)
    factor[np.ix_(np.arange(27, 33), 27 + active)] = root
    d = standard @ factor.T
    state = current.nominal_state
    f0, w0, f1, w1 = measurements
    base, _, _ = release_endpoint_map(state, f0, w0, f1, w1, current.imu_noise_mean_B, g, h)
    error = finite_errors(
        state, measurements[1], measurements[2], measurements[3], current.imu_noise_mean_B, g, h, d
    )
    true_q_WB = quaternion_product(base.q_WB, exp_batch(error[:, 6:9]))
    error[:, :3] -= output.nominal_state.position_W - base.position_W
    error[:, 3:6] -= output.nominal_state.velocity_W - base.velocity_W
    error[:, 6:9] = local_logs(output.nominal_state.q_WB, true_q_WB)
    return error


def calibration(errors: Array, covariance: Array) -> dict[str, Any]:
    whitened = np.linalg.solve(np.linalg.cholesky(covariance), errors.T).T
    mean = whitened.mean(axis=0)
    eigenvalues = np.linalg.eigvalsh(np.cov(whitened, rowvar=False))
    return dict(
        trials=len(errors),
        mean=errors.mean(axis=0).tolist(),
        empirical_covariance=np.cov(errors, rowvar=False).tolist(),
        whitened_mean=mean.tolist(),
        whitened_covariance_eigenvalues=eigenvalues.tolist(),
        minimum_variance=float(eigenvalues[0]),
        maximum_variance=float(eigenvalues[-1]),
        maximum_abs_mean=float(np.max(np.abs(mean))),
        passed=bool(
            eigenvalues[0] >= 0.90 and eigenvalues[-1] <= 1.10 and np.max(np.abs(mean)) <= 0.05
        ),
    )


def definition() -> dict[str, Any]:
    return dict(
        design="ADR0036 nonlinear release and boundary-only flight comparison",
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        input_report_sha256=INPUT_SHA,
        trials=TRIALS,
        jobs=[j.name for j in jobs()],
        gaussian_entropy=[0x52454C36, "case_index"],
        candidate_order=5,
        reference_order=7,
    )


def evaluate(campaign: Path, output: Path, verify: bool) -> dict[str, Any]:
    parent = authenticate(campaign)
    rows = []
    for index, (job, row) in enumerate(zip(jobs(), parent["cases"], strict=True)):
        prepared, args, inputs = load_inputs(campaign, job, row)
        config = args["estimator_configuration"]
        noise, g, h = config.sampled_imu_noise, config.gravity_acceleration, inputs["h"]
        measurements = inputs["measurements"]
        standard = np.random.Generator(
            np.random.PCG64(np.random.SeedSequence([0x52454C36, index]))
        ).standard_normal((TRIALS, 33))
        arms = {}
        raw: dict[str, Any] = {}
        for mode, conditioned in (("original_prior", False), ("zero_velocity_prior", True)):
            current = initial_posterior(prepared, args, inputs, conditioned)
            f0, w0, f1, w1 = measurements
            candidate = predict_nonlinear_release_endpoint(current, f0, w0, f1, w1, g, noise, h)
            reference = predict_nonlinear_release_endpoint(
                current, f0, w0, f1, w1, g, noise, h, order=7
            )
            errors = gaussian_errors(current, measurements, g, noise, h, candidate, standard)
            s = current.nominal_state
            R = rotation_matrix_body_to_world(s.q_WB)
            ballistic_measurements = (
                s.accelerometer_bias_B - R.T @ np.array([0.0, 0, g]),
                s.gyroscope_bias_B,
                s.accelerometer_bias_B,
                s.gyroscope_bias_B,
            )
            ballistic = predict_nonlinear_release_endpoint(
                current, *ballistic_measurements, g, noise, h
            )
            ballistic_reference = predict_nonlinear_release_endpoint(
                current, *ballistic_measurements, g, noise, h, order=7
            )
            arms[mode] = dict(
                input=plain_configuration(current),
                output=plain_configuration(candidate),
                accuracy=accuracy(candidate, reference),
                calibration=calibration(errors, candidate.joint_covariance),
                rank=int(np.linalg.matrix_rank(candidate.joint_covariance)),
                ballistic_output=plain_configuration(ballistic),
                ballistic_accuracy=accuracy(ballistic, ballistic_reference),
            )
            raw[mode + "_errors"] = errors
        filename = job.name + ".npz"
        path = output / filename
        if verify:
            with np.load(path, allow_pickle=False) as stored:
                ensure(set(stored.files) == set(raw), "Gaussian fields")
                for key, value in raw.items():
                    ensure(
                        np.array_equal(stored[key], value), "Gaussian saved error reconstruction"
                    )
        else:
            save_arrays(path, raw)
        rows.append(
            dict(
                name=job.name,
                arms=arms,
                noise=plain_configuration(noise),
                measurements=[x.tolist() for x in measurements],
                gravity=g,
                h=h,
                observations=[plain_configuration(x) for x in inputs["observations"]],
                baseline_history=inputs["baseline_history"],
                gaussian_payload=dict(
                    file=filename, sha256=hashlib.sha256(path.read_bytes()).hexdigest()
                ),
            )
        )
        print(
            job.name,
            {
                mode: {
                    key: arms[mode][key]["passed"]
                    for key in ("accuracy", "calibration", "ballistic_accuracy")
                }
                for mode in arms
            },
            flush=True,
        )
    passed = {
        mode: all(
            row["arms"][mode]["rank"] == 21
            and all(
                row["arms"][mode][key]["passed"]
                for key in ("accuracy", "calibration", "ballistic_accuracy")
            )
            for row in rows
        )
        for mode in ("original_prior", "zero_velocity_prior")
    }
    return dict(
        cases=rows,
        summary=dict(
            releases=17,
            trials_per_prior=17 * TRIALS,
            original_prior_passed=passed["original_prior"],
            exact_prior_mathematics_passed=passed["zero_velocity_prior"],
            boundary_flight_eligible=passed["original_prior"],
            zero_velocity_flight_use=False,
        ),
    )


def run(campaign: Path, output: Path, *, verify: bool = False) -> dict[str, Any]:
    frozen = definition()
    if verify:
        saved = json.loads((output / "report.json").read_bytes())
        equal_json(saved["protocol"], frozen, "uncertainty protocol")
        equal_json(json.loads((output / "protocol.json").read_bytes()), frozen, "protocol file")
        software = saved["software"]
    else:
        output.mkdir(parents=True, exist_ok=False)
        save_json(output, "protocol.json", frozen)
        software = asdict(capture_software_provenance(ROOT))
    report = dict(protocol=frozen, software=software, **evaluate(campaign, output, verify))
    equal_json(definition(), frozen, "source/protocol unchanged")
    if verify:
        equal_json(saved, report, "complete uncertainty reconstruction")
    else:
        save_json(output, "report.json", report)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", required=True, type=Path)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output", type=Path)
    group.add_argument("--verify", type=Path)
    args = parser.parse_args()
    result = run(args.campaign, args.verify or args.output, verify=args.verify is not None)
    print(json.dumps(result["summary"], indent=2), flush=True)
    return 0 if result["summary"]["boundary_flight_eligible"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
