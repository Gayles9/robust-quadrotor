"""ADR0034: exact supported velocity conditioning and a pre-flight release screen.

This experiment does not fly, arm, change estimator equations or insert a truth
update. A failed necessary boundary screen rejects the candidate before flights.
"""

import argparse
import hashlib
import io
import json
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import source_sha256
from experiments.early_flight_diagnostic import authenticated
from experiments.residual_hover_diagnostic import INPUT_SHA, authenticate
from experiments.robustness_evidence import ensure, equal_json, save_json
from experiments.supported_start import PreparedStart, configuration_for, plain_configuration
from experiments.supported_validation_protocol import CLEAN, jobs, prepare_job
from quadrotor_math.eskf_endpoint import (
    EskfSampledImuNoise,
    _nonnegative_scalar,
    predict_eskf_endpoint,
)
from quadrotor_math.prearm_alignment import PrearmRelease, StationarySupportEvidence
from quadrotor_math.prearm_uncertainty import (
    CLOCK_TOLERANCE_S,
    GRAVITY,
    PROFILE_ID,
    SAMPLE_COUNT,
    SAMPLE_PERIOD_S,
)
from quadrotor_math.rotations import rotation_matrix_body_to_world
from quadrotor_math.run_manifest import capture_software_provenance

ROOT = Path(__file__).resolve().parents[1]
ADR = ROOT / "docs/decisions/0034-supported-velocity-prior.md"
RADIUS_99 = 2.5758293035489004


@dataclass(frozen=True)
class VelocitySupport:
    """External world-stationarity assertion, additional to acceleration support.

    The caller owns physical authentication and unique acquisition allocation;
    no IMU-only test can authenticate this assertion or a global one-shot use.
    """

    support: StationarySupportEvidence
    zero_world_velocity: bool


class SupportedVelocityConditioner:
    """One local, fail-closed attempt for the independent zero-mean prior profile."""

    def __init__(self) -> None:
        self._consumed = False

    def condition(
        self, release: PrearmRelease, support: VelocitySupport | None, *, now_s: float
    ) -> PrearmRelease:
        ensure(not self._consumed, "velocity conditioner already consumed")
        self._consumed = True
        ensure(isinstance(release, PrearmRelease), "typed pre-arm release required")
        ensure(isinstance(support, VelocitySupport), "world-velocity support required")
        assert support is not None
        ensure(support.zero_world_velocity is True, "world-zero-velocity assertion required")
        now = _nonnegative_scalar("now_s", now_s)
        s, fresh, identity = release.support, release.fresh_sample, release.identity
        ensure(isinstance(support.support, StationarySupportEvidence), "typed support required")
        ensure(support.support == s, "velocity assertion and release support differ")
        ensure(
            all(
                getattr(s, key) == getattr(identity, key)
                for key in ("acquisition_id", "support_id", "source_id", "clock_id")
            )
            and s.mechanically_supported
            and s.motors_off
            and s.zero_world_acceleration
            and s.zero_angular_velocity
            and not s.revoked,
            "invalid external support",
        )
        ensure(
            abs(s.covered_from_s) <= CLOCK_TOLERANCE_S
            and abs(s.observed_through_s - now) <= CLOCK_TOLERANCE_S
            and abs(fresh.time_s - now) <= CLOCK_TOLERANCE_S
            and abs(now - SAMPLE_COUNT * SAMPLE_PERIOD_S) <= CLOCK_TOLERANCE_S,
            "stale or invalid release epoch",
        )
        ensure(
            fresh.sample_id == identity.first_sample_id + SAMPLE_COUNT
            and release.aligned_sample_ids
            == (identity.first_sample_id, identity.first_sample_id + SAMPLE_COUNT - 1)
            and fresh.stream_id == identity.stream_id
            and fresh.clock_id == identity.clock_id
            and fresh.frame == "FRD"
            and fresh.units == "SI"
            and fresh.profile_id == identity.profile_id == PROFILE_ID,
            "release sample identity/profile/ownership mismatch",
        )
        endpoint = release.endpoint
        C = endpoint.joint_covariance
        other = np.r_[0:3, 6:21]
        ensure(
            np.all(endpoint.nominal_state.velocity_W == 0)
            and np.all(C[3:6, other] == 0)
            and np.all(C[other, 3:6] == 0)
            and np.all(C[:15, 15:] == 0)
            and np.all(C[15:, :15] == 0)
            and np.all(endpoint.imu_noise_mean_B == 0),
            "unsupported velocity or fresh-sample prior",
        )
        H = np.eye(21)[3:6]
        S = H @ C @ H.T
        try:
            np.linalg.cholesky(S)
            K = np.linalg.solve(S, H @ C).T
        except np.linalg.LinAlgError:
            raise ValueError("positive definite unconditioned velocity block required") from None
        M = np.eye(21) - K @ H
        covariance = M @ C @ M.T
        return replace(release, endpoint=replace(endpoint, joint_covariance=covariance))


def fixture_velocity_support(prepared: PreparedStart) -> VelocitySupport:
    """Substantiate the boundary assertion from this explicit mechanical fixture."""
    ensure(prepared.release is not None, "released support required")
    assert prepared.release is not None
    a = prepared.evidence
    valid = bool(
        np.all(a["velocity_W"] == 0)
        and np.all(a["position_W"] == a["position_W"][0])
        and np.all(a["q_WB"] == a["q_WB"][0])
        and np.all(a["omega_B"] == 0)
        and np.all(a["actual_rotor_omega"] == 0)
        and np.max(np.abs(a["equilibrium_residual_W"])) <= 1e-12
    )
    return VelocitySupport(prepared.release.support, valid)


def boundary_screen(release: PrearmRelease, noise: EskfSampledImuNoise) -> dict[str, Any]:
    """Ballistic release counterexample, retaining full reported joint uncertainty.

    Mean-consistent IMU endpoints have supported force at 0- and zero force at h.
    Their map crosses a force step. The exact h>0 dynamics are ballistic, not a
    linear ramp from the supported acceleration. This is not a mission flight.
    """
    endpoint, h, g = release.endpoint, SAMPLE_PERIOD_S, GRAVITY
    state, C = endpoint.nominal_state, endpoint.joint_covariance
    R = rotation_matrix_body_to_world(state.q_WB)
    gravity = np.array([0.0, 0.0, g])
    predicted = predict_eskf_endpoint(
        endpoint,
        state.accelerometer_bias_B - R.T @ gravity,
        state.gyroscope_bias_B,
        state.accelerometer_bias_B,
        state.gyroscope_bias_B,
        g,
        noise,
        h,
    )
    exact_position = state.position_W + h * state.velocity_W + h * h / 2 * gravity
    exact_velocity = state.velocity_W + h * gravity
    error = predicted.nominal_state.velocity_W - exact_velocity
    # Down projection kills the first-order tilt sensitivity: e_D^T R [R^T g]_x=0.
    # Velocity/bias and fresh-sample cross terms vanish for the accepted profile.
    r = R[2]
    independent = float(
        C[5, 5]
        + h * h * (r @ C[9:12, 9:12] @ r)
        + h * h / 4 * (r @ (C[15:18, 15:18] + noise.sample_covariance_B[:3, :3]) @ r)
        + h**3 / 4 * (r @ noise.bias_walk_spectral_density_B[:3, :3] @ r)
    )
    variance = float(predicted.joint_covariance[5, 5])
    ensure(abs(independent - variance) <= 1e-12, "independent down variance mismatch")
    sigma = float(np.sqrt(variance))
    radius = RADIUS_99 * sigma
    return dict(
        initial_joint_covariance=C.tolist(),
        initial_rank=int(np.linalg.matrix_rank(C)),
        predicted=plain_configuration(predicted),
        exact_position_W=exact_position.tolist(),
        exact_velocity_W=exact_velocity.tolist(),
        position_error_W=(predicted.nominal_state.position_W - exact_position).tolist(),
        velocity_error_W=error.tolist(),
        down_variance=variance,
        independent_down_variance=independent,
        down_standard_deviation_m_s=sigma,
        down_radius_99_m_s=radius,
        down_bias_sigma_ratio=abs(float(error[2])) / sigma if sigma else None,
        bias_inside_99_radius=bool(abs(error[2]) <= radius),
    )


def evaluate(campaign: Path) -> dict[str, Any]:
    """Authenticate all original flights; evaluate each reconstructed release once."""
    parent = authenticate(campaign)
    results = []
    for job, row in zip(jobs(), parent["cases"], strict=True):
        prepared = prepare_job(job)
        release = prepared.release
        ensure(release is not None, "baseline release rejected")
        assert release is not None
        target = campaign / job.name
        with np.load(
            io.BytesIO(authenticated(target / row["support"]["file"], row["support"]["sha256"])),
            allow_pickle=False,
        ) as saved:
            ensure(set(saved.files) == set(prepared.evidence), "support fields differ")
            for key, value in prepared.evidence.items():
                ensure(np.array_equal(saved[key], value), "support reconstruction differs")
        equal_json(
            json.loads(authenticated(target / row["release"]["file"], row["release"]["sha256"])),
            asdict(release),
            "release reconstruction",
        )
        mode = "aligned" if job.case in CLEAN else "on"
        config_record = row["modes"][mode]["configuration"]
        args = configuration_for(prepared, "aligned")
        equal_json(
            json.loads(authenticated(target / config_record["file"], config_record["sha256"])),
            {k: plain_configuration(v) for k, v in args.items()},
            "configuration reconstruction",
        )
        candidate = SupportedVelocityConditioner().condition(
            release, fixture_velocity_support(prepared), now_s=release.fresh_sample.time_s
        )
        noise = args["estimator_configuration"].sampled_imu_noise
        results.append(
            dict(
                name=job.name,
                support=row["support"],
                release=row["release"],
                configuration=config_record,
                candidate_release=plain_configuration(candidate),
                baseline=boundary_screen(release, noise),
                candidate=boundary_screen(candidate, noise),
            )
        )
    eligible = all(row["candidate"]["bias_inside_99_radius"] for row in results)
    return dict(
        cases=results,
        decision=dict(
            releases=len(results),
            candidate_boundary_passes=sum(
                row["candidate"]["bias_inside_99_radius"] for row in results
            ),
            regression_eligible=eligible,
            candidate_accepted=False,
            qualification=False,
            scientific_flights_executed=0,
            conditional_regression_flights=25,
            next_stage="regression_required" if eligible else "release_integration_design",
        ),
    )


def definition() -> dict[str, Any]:
    return dict(
        design="ADR0034 supported zero-velocity prior release screen",
        source_sha256=source_sha256(ROOT),
        protocol_sha256=hashlib.sha256(ADR.read_bytes()).hexdigest(),
        input_report_sha256=INPUT_SHA,
        jobs=[job.name for job in jobs()],
        gravity_m_s2=GRAVITY,
        interval_s=SAMPLE_PERIOD_S,
        radius_99_multiplier=RADIUS_99,
        software=asdict(capture_software_provenance(ROOT)),
    )


def run(campaign: Path, output: Path) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=False)
    frozen = definition()
    save_json(output, "protocol.json", frozen)
    report = dict(protocol=frozen, **evaluate(campaign))
    ensure(source_sha256(ROOT) == frozen["source_sha256"], "source changed during evaluation")
    save_json(output, "report.json", report)
    return report


def verify(campaign: Path, output: Path) -> dict[str, Any]:
    saved: dict[str, Any] = json.loads((output / "report.json").read_bytes())
    frozen = json.loads((output / "protocol.json").read_bytes())
    expected = definition()
    ensure(set(frozen) == set(expected), "protocol fields")
    for key in expected:
        if key != "software":
            equal_json(frozen[key], expected[key], "protocol " + key)
    equal_json(saved, dict(protocol=frozen, **evaluate(campaign)), "saved release evaluation")
    print("All 17 saved release screens independently rederived; no flights executed.", flush=True)
    return saved


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", required=True, type=Path)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--output", type=Path)
    action.add_argument("--verify", type=Path)
    args = parser.parse_args()
    report = verify(args.campaign, args.verify) if args.verify else run(args.campaign, args.output)
    print(json.dumps(report["decision"], indent=2), flush=True)
    return 0 if report["decision"]["regression_eligible"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
