"""ADR 0029 fixed reference equivalence and 200 complete standalone sessions."""

import argparse
import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from experiments import prearm_alignment_feasibility as base
from experiments import prearm_nonlinear_uncertainty as reference
from experiments.attitude_control_validation import source_sha256
from quadrotor_math import prearm_alignment as component
from quadrotor_math.eskf import EskfNominalState
from quadrotor_math.prearm_uncertainty import Array, _moments
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
PRIOR_REPORT = "a3b5cc87716dab48e7e7541684ba14216c2d507726536eb0ee861d765a17dfa4"
PRIOR_TRIALS = "7bcecd011586659495d6abd326233597c333f447f5e498ed6c498871b4ee60a2"


def authenticate(path: Path) -> dict[str, Array]:
    for name, expected in (("report.json", PRIOR_REPORT), ("trials.npz", PRIOR_TRIALS)):
        if hashlib.sha256((path / name).read_bytes()).hexdigest() != expected:
            raise ValueError("ADR 0028 evidence digest mismatch")
    with np.load(path / "trials.npz", allow_pickle=False) as values:
        return {k: values[k] for k in values.files}


def comparison(prior: dict[str, Array], label: str) -> tuple[dict[str, Any], dict[str, Array]]:
    records: dict[str, list[Any]] = {
        k: []
        for k in (
            "q_WB",
            "covariance",
            "errors",
            "whitened",
            "rotation_difference_rad",
            "covariance_relative_error",
            "error_difference_rad",
            "rejected",
            "modeled_mean_B",
        )
    }
    for k, s in enumerate(prior[label + "_mean_accel"]):
        result = _moments(s)
        R = rotation_matrix_body_to_world(result.q_WB)
        R0 = base.inclination(s)[0]
        expected_R = R0 @ base.rotation_vector_matrix(prior[label + "_mean_shift_B"][k])
        truth_R = R0 @ base.rotation_vector_matrix(prior[label + "_first_order_errors"][k, :3])
        error = prior[label + "_errors"][k].copy()
        error[:3] = base.rotation_log(R.T @ truth_R)
        P = result.joint_covariance
        ref_P = prior[label + "_covariance"][k]
        L = np.linalg.cholesky(ref_P)
        delta = np.linalg.solve(L, np.linalg.solve(L, P - ref_P).T).T
        radius = np.rad2deg(np.sqrt(-2 * np.log(0.01) * np.linalg.eigvalsh(P[:2, :2])[-1]))
        tilt = np.rad2deg(np.arccos(np.clip(R0[2, 2], -1, 1)))
        d = np.array([3, 3, 1200])
        thresholds = d + 2 * np.sqrt(d * np.log(1000)) + 2 * np.log(1000)
        rejected = bool(
            np.any(prior[label + "_statistics"][k] > thresholds) or radius > 0.75 or tilt > 15
        )
        values = (
            result.q_WB,
            P,
            error,
            np.linalg.solve(np.linalg.cholesky(P), error),
            np.linalg.norm(base.rotation_log(R.T @ expected_R)),
            np.max(np.abs(np.linalg.eigvalsh(delta))),
            np.linalg.norm(error[:3] - prior[label + "_errors"][k, :3]),
            rejected,
            result.centered_mean_B,
        )
        for key, value in zip(records, values, strict=True):
            records[key].append(value)
    arrays = {key: np.asarray(value) for key, value in records.items()}
    summary = dict(
        trials=len(arrays["errors"]),
        maximum_rotation_difference_rad=float(arrays["rotation_difference_rad"].max()),
        maximum_error_difference_rad=float(arrays["error_difference_rad"].max()),
        maximum_covariance_relative_error=float(arrays["covariance_relative_error"].max()),
        identical_rejections=bool(np.array_equal(arrays["rejected"], prior[label + "_rejected"])),
        rejected=int(arrays["rejected"].sum()),
        maximum_centered_mean_rad=float(np.linalg.norm(arrays["modeled_mean_B"], axis=1).max()),
        whitened_mean=arrays["whitened"].mean(axis=0).tolist(),
        whitened_covariance_eigenvalues=np.linalg.eigvalsh(
            np.cov(arrays["whitened"], rowvar=False)
        ).tolist(),
    )
    summary["equivalent"] = (
        summary["maximum_rotation_difference_rad"] <= 2e-14
        and summary["maximum_error_difference_rad"] <= 2e-14
        and summary["maximum_covariance_relative_error"] <= 1e-9
        and summary["identical_rejections"]
    )
    return summary, arrays


def complete_sessions(prior: dict[str, Array]) -> tuple[dict[str, Any], dict[str, Array]]:
    state = EskfNominalState(
        np.array([1.0, 2, -3]),
        np.array([0.01, 0, 0]),
        np.array([1.0, 0, 0, 0]),
        np.zeros(3),
        np.zeros(3),
    )
    P = np.diag(
        [0.05**2] * 3 + [0.03**2] * 3 + [np.deg2rad(3) ** 2] * 3 + [0.03**2] * 3 + [0.005**2] * 3
    )
    P[0, 3] = P[3, 0] = 0.0001
    initial = component.PrearmPrior(state, P, navigation_independent=True)
    records: dict[str, list[Any]] = {
        k: []
        for k in (
            "partition",
            "trial",
            "released",
            "endpoint_covariance",
            "fresh_force_B",
            "fresh_rate_B",
            "fresh_truth_ba",
            "fresh_truth_bg",
            "covariance_difference",
            "statistics_difference",
            "alignment_covariance_difference",
            "position_W",
            "velocity_W",
            "sample_ids",
            "ownership_passed",
        )
    }
    for partition, label in enumerate(("nominal", "gaussian")):
        for trial in range(100):
            truth = reference.draw_trial(partition, trial, 201)
            identity = component.PrearmSessionIdentity(
                f"{label}-{trial}",
                "imu",
                "relative",
                f"support-{label}-{trial}",
                "simulated-fixture",
                0,
            )
            session = component.PrearmAlignmentSession(identity, initial)

            def support(
                t: float, identity: component.PrearmSessionIdentity = identity
            ) -> component.StationarySupportEvidence:
                return component.StationarySupportEvidence(
                    identity.acquisition_id,
                    identity.support_id,
                    identity.source_id,
                    identity.clock_id,
                    0.0,
                    t,
                    True,
                    True,
                    True,
                    True,
                    False,
                )

            for k in range(201):
                t = k * 0.0025
                sample = component.PrearmImuSample(
                    "imu", "relative", k, t, truth["accel"][k], truth["gyro"][k]
                )
                session.consume(sample, support(t), now_s=t)
            estimate = session.snapshot.estimate
            if estimate is None or session.status is not component.PrearmStatus.READY:
                raise ValueError(
                    f"unexpected rejected full session {label}/{trial}: {session.snapshot.reasons}"
                )
            rng = np.random.Generator(
                np.random.PCG64(np.random.SeedSequence([0x50524541, 3, partition, trial]))
            )
            ba = truth["ba_terminal"] + rng.normal(0, 0.0002 * np.sqrt(0.0025), 3)
            bg = truth["bg_terminal"] + rng.normal(0, 0.00002 * np.sqrt(0.0025), 3)
            force, rate = truth["h"] + ba + rng.normal(0, 0.04, 3), bg + rng.normal(0, 0.002, 3)
            fresh = component.PrearmImuSample("imu", "relative", 201, 0.5025, force, rate)
            released = session.release(fresh, support(0.5025), now_s=0.5025)
            if released is None:
                raise ValueError(f"unexpected failed release {label}/{trial}")
            expected = np.zeros((21, 21))
            expected[:6, :6] = P[:6, :6]
            expected[6:15, 6:15] = prior[label + "_covariance"][trial]
            expected[9:12, 9:12] += 0.0002**2 * 0.0025 * np.eye(3)
            expected[12:15, 12:15] += 0.00002**2 * 0.0025 * np.eye(3)
            expected[15:, 15:] = np.diag([0.04**2] * 3 + [0.002**2] * 3)
            fresh.specific_force_B.flags.writeable = True
            fresh.specific_force_B[:] = np.nan
            owned = bool(np.array_equal(released.fresh_sample.specific_force_B, force))
            end = released.endpoint
            np.testing.assert_array_equal(end.imu_noise_mean_B, 0)
            np.testing.assert_array_equal(end.nominal_state.position_W, state.position_W)
            np.testing.assert_array_equal(end.nominal_state.velocity_W, state.velocity_W)
            np.testing.assert_allclose(
                estimate.statistics, prior[label + "_statistics"][trial], rtol=0, atol=2e-10
            )
            values = (
                partition,
                trial,
                session.snapshot.status is component.PrearmStatus.RELEASED,
                end.joint_covariance,
                force,
                rate,
                ba,
                bg,
                np.max(np.abs(end.joint_covariance - expected)),
                np.max(np.abs(estimate.statistics - prior[label + "_statistics"][trial])),
                np.max(np.abs(estimate.joint_covariance - prior[label + "_covariance"][trial])),
                end.nominal_state.position_W,
                end.nominal_state.velocity_W,
                (*released.aligned_sample_ids, released.fresh_sample.sample_id),
                owned,
            )
            for key, value in zip(records, values, strict=True):
                records[key].append(value)
    arrays = {key: np.asarray(value) for key, value in records.items()}
    return dict(
        sessions=200,
        released=int(arrays["released"].sum()),
        maximum_endpoint_covariance_difference=float(arrays["covariance_difference"].max()),
        maximum_statistics_difference=float(arrays["statistics_difference"].max()),
        passed=bool(
            np.all(arrays["released"])
            and np.all(arrays["ownership_passed"])
            and arrays["covariance_difference"].max() <= 1e-16
        ),
    ), arrays


def run(prior_path: Path, output: Path) -> dict[str, Any]:
    prior = authenticate(prior_path)
    if output.exists():
        raise ValueError("new output directory required; preserve preceding evidence")
    output.mkdir(parents=True)
    reports: dict[str, Any] = {}
    arrays: dict[str, Any] = {}
    for label in ("regression", "nominal", "gaussian"):
        reports[label], values = comparison(prior, label)
        arrays.update({label + "_" + k: v for k, v in values.items()})
    reports["sessions"], values = complete_sessions(prior)
    arrays.update({"sessions_" + k: v for k, v in values.items()})
    np.savez_compressed(output / "trials.npz", **arrays)
    checks = dict(
        moment_equivalence=all(
            reports[x]["equivalent"] for x in ("regression", "nominal", "gaussian")
        ),
        original_regression=reports["regression"]["whitened_covariance_eigenvalues"][-1] <= 1.10,
        fresh_gaussian_covariance=reports["gaussian"]["whitened_covariance_eigenvalues"][-1]
        <= 1.10,
        fresh_gaussian_mean=max(abs(x) for x in reports["gaussian"]["whitened_mean"]) <= 0.05,
        nominal_rejection=reports["nominal"]["rejected"] / 5000 <= 0.01,
        centered_mean=all(
            reports[x]["maximum_centered_mean_rad"] <= 1e-13
            for x in ("regression", "nominal", "gaussian")
        ),
        complete_sessions=reports["sessions"]["passed"],
    )
    report = dict(
        design="ADR 0029 standalone component; no flight integration",
        **reports,
        software=asdict(capture_software_provenance(ROOT)),
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(
            (ROOT / "docs/decisions/0029-standalone-prearm-alignment.md").read_bytes()
        ).hexdigest(),
        prior_report_sha256=PRIOR_REPORT,
        prior_trials_sha256=PRIOR_TRIALS,
        acceptance=checks,
        component_go=all(checks.values()),
        flight_qualified=False,
    )
    (output / "report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prior-evidence", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = run(args.prior_evidence, args.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    if not report["component_go"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
