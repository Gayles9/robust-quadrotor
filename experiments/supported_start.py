"""ADR 0030 fixture, one-shot pre-arm handoff and experiment-only boundary adapter."""

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import asdict, dataclass, is_dataclass, replace
from typing import Any, cast
from unittest.mock import patch

import numpy as np

from experiments import robustness_evidence
from experiments.early_flight_diagnostic import check_plant
from experiments.estimated_feedback_evidence import pack_history, plain
from experiments.robustness_evidence import ensure
from experiments.robustness_protocol import configuration, policies
from quadrotor_math import estimated_mission
from quadrotor_math.dynamics import quadratic_drag_force_body
from quadrotor_math.eskf_endpoint import initialize_eskf_endpoint
from quadrotor_math.eskf_faults import EskfObservationFault
from quadrotor_math.eskf_live_faults import EskfLiveObservationFaults
from quadrotor_math.eskf_replay import EskfObservationKind, EskfReplayObservation
from quadrotor_math.estimated_mission import EstimatedMissionResult
from quadrotor_math.imu import (
    accelerometer_specific_force_measurement_body,
    gyroscope_angular_velocity_measurement_body,
)
from quadrotor_math.observation_health import ObservationHealthMonitor
from quadrotor_math.observation_supervision import ObservationSupervisor
from quadrotor_math.prearm_alignment import (
    PrearmAlignmentSession,
    PrearmImuSample,
    PrearmPrior,
    PrearmRelease,
    PrearmSessionIdentity,
    PrearmStatus,
    StationarySupportEvidence,
)
from quadrotor_math.prearm_uncertainty import Array
from quadrotor_math.randomness import create_run_random_streams
from quadrotor_math.rotations import rotation_matrix_body_to_world

CASES = ("nominal_hover", "nominal_tracking", "wind_tracking", "mass_tracking")
MODES = ("unaligned", "aligned")


def plain_configuration(value: Any) -> Any:
    return plain(asdict(value) if is_dataclass(value) and not isinstance(value, type) else value)


@dataclass
class PreparedStart:
    case: str
    partition: str
    original: dict[str, Any]
    evidence: dict[str, Array]
    status: PrearmStatus
    reasons: tuple[str, ...]
    release: PrearmRelease | None


def support_evidence(
    identity: PrearmSessionIdentity, evidence: dict[str, Array], k: int
) -> StationarySupportEvidence:
    """Assert only the explicit equilibrium fixture; never infer support from IMU."""
    prefix = slice(0, k + 1)
    stationary = bool(
        np.all(evidence["velocity_W"][prefix] == 0)
        and np.all(evidence["q_WB"][prefix] == evidence["q_WB"][0])
        and np.all(evidence["position_W"][prefix] == evidence["position_W"][0])
    )
    balanced = bool(np.max(np.abs(evidence["equilibrium_residual_W"][prefix])) < 1e-12)
    return StationarySupportEvidence(
        identity.acquisition_id,
        identity.support_id,
        identity.source_id,
        identity.clock_id,
        0.0,
        float(evidence["time_s"][k]),
        stationary,
        bool(np.all(evidence["actual_rotor_omega"][prefix] == 0)),
        balanced,
        bool(np.all(evidence["omega_B"][prefix] == 0)),
        False,
    )


def prepare(case: str, partition: str = "campaign") -> PreparedStart:
    if case not in CASES:
        raise ValueError("unknown frozen supported-start case")
    return prepare_configured(case, configuration(case, partition), partition)


def prepare_configured(
    case: str, args: dict[str, Any], partition: str = "campaign"
) -> PreparedStart:
    """Use the unchanged physical fixture with an explicitly supplied experiment."""
    initial, body, world, sensors = (
        args[k] for k in ("initial_state", "truth_body", "truth_world", "sensors")
    )
    R = rotation_matrix_body_to_world(initial.q_WB)
    gravity = np.array([0.0, 0, world.gravity_acceleration])
    drag = quadratic_drag_force_body(
        np.zeros(3), R, world.wind_velocity_W, body.quadratic_drag_coefficient_B
    )
    reaction = -body.mass * gravity - R @ drag
    ideal = -R.T @ gravity
    n, dt = 202, 0.0025
    evidence = dict(
        time_s=np.arange(n) * dt,
        sample_id=np.arange(n, dtype=np.float64),
        position_W=np.tile(initial.position_W, (n, 1)),
        q_WB=np.tile(initial.q_WB, (n, 1)),
        velocity_W=np.zeros((n, 3)),
        omega_B=np.zeros((n, 3)),
        actual_rotor_omega=np.zeros((n, 4)),
        support_force_W=np.tile(reaction, (n, 1)),
        equilibrium_residual_W=np.tile(reaction + R @ drag + body.mass * gravity, (n, 1)),
        ideal_specific_force_B=np.tile(ideal, (n, 1)),
    )
    seed = sensors.root_seed
    generators = [
        np.random.Generator(np.random.PCG64(np.random.SeedSequence([0x50524541, 4, seed, i])))
        for i in (1, 2, 3, 4)
    ]
    flight = create_run_random_streams(seed)
    imu = sensors.imu
    ba, bg = imu.initial_accelerometer_bias_B.copy(), imu.initial_gyroscope_bias_B.copy()
    force, rate, bias_a, bias_g = [], [], [], []
    for k in range(n):
        if k:
            ba = ba + imu.accelerometer_bias_random_walk_density_B * np.sqrt(dt) * generators[
                2
            ].standard_normal(3)
            bg = bg + imu.gyroscope_bias_random_walk_density_B * np.sqrt(dt) * generators[
                3
            ].standard_normal(3)
        a_rng = generators[0] if k < 201 else flight.accelerometer_measurement_noise
        g_rng = generators[1] if k < 201 else flight.gyroscope_measurement_noise
        force.append(
            accelerometer_specific_force_measurement_body(
                ideal, ba, imu.accelerometer_noise_standard_deviation_B, a_rng
            )
        )
        rate.append(
            gyroscope_angular_velocity_measurement_body(
                np.zeros(3), bg, imu.gyroscope_noise_standard_deviation_B, g_rng
            )
        )
        bias_a.append(ba.copy())
        bias_g.append(bg.copy())
    evidence.update(
        specific_force_B=np.array(force),
        angular_velocity_B=np.array(rate),
        true_accelerometer_bias_B=np.array(bias_a),
        true_gyroscope_bias_B=np.array(bias_g),
    )
    config = args["estimator_configuration"]
    identity = PrearmSessionIdentity(
        f"supported-{case}",
        "imu-400hz",
        "relative-support",
        f"fixture-{case}",
        "equilibrium-fixture",
        0,
    )
    session = PrearmAlignmentSession(
        identity,
        PrearmPrior(config.initial_state, config.initial_covariance, True),
        noise=config.sampled_imu_noise,
        gravity_acceleration=world.gravity_acceleration,
        sample_period_s=dt,
    )
    release = None
    for k in range(n):
        sample = PrearmImuSample(
            identity.stream_id,
            identity.clock_id,
            k,
            float(evidence["time_s"][k]),
            force[k],
            rate[k],
        )
        supported = support_evidence(identity, evidence, k)
        if k < 201:
            session.consume(sample, supported, now_s=sample.time_s)
        else:
            release = session.release(sample, supported, now_s=sample.time_s)
        if session.status is PrearmStatus.REJECTED:
            break
    return PreparedStart(
        case, partition, args, evidence, session.status, tuple(session.snapshot.reasons), release
    )


def configuration_for(prepared: PreparedStart, mode: str) -> dict[str, Any]:
    if mode not in MODES:
        raise ValueError("unknown supported-start mode")
    ensure(prepared.release is not None, "supported release required before flight")
    assert prepared.release is not None
    args = prepared.original.copy()
    args["initial_state"] = replace(
        args["initial_state"], velocity_W=np.zeros(3), omega_B=np.zeros(3)
    )
    args["initial_actual_rotor_omega"] = np.zeros(4)
    args["sensors"] = replace(
        args["sensors"],
        imu=replace(
            args["sensors"].imu,
            initial_accelerometer_bias_B=prepared.evidence["true_accelerometer_bias_B"][-1],
            initial_gyroscope_bias_B=prepared.evidence["true_gyroscope_bias_B"][-1],
        ),
    )
    config = args["estimator_configuration"]
    if mode == "aligned":
        endpoint = prepared.release.endpoint
        config = replace(
            config,
            initial_state=endpoint.nominal_state,
            initial_covariance=endpoint.joint_covariance[:15, :15],
        )
        restored = initialize_eskf_endpoint(
            config.initial_state, config.initial_covariance, config.sampled_imu_noise
        )
        ensure(
            np.array_equal(restored.joint_covariance, endpoint.joint_covariance)
            and np.array_equal(restored.imu_noise_mean_B, endpoint.imu_noise_mean_B),
            "complete endpoint reconstruction",
        )
    else:
        covariance = config.initial_covariance.copy()
        covariance[9:15, 9:15] += config.sampled_imu_noise.bias_walk_spectral_density_B * 0.5025
        config = replace(config, initial_covariance=covariance)
    args["estimator_configuration"] = config
    return args


@contextmanager
def release_measurement(prepared: PreparedStart) -> Iterator[dict[str, int]]:
    """Only replace the first sensor's physical force by its supported left limit.

    The actual original RNG is consumed once at every sample. This process-local
    patch restores on exceptions; do not share it with concurrent mission threads.
    """
    ensure(prepared.release is not None, "supported release required")
    assert prepared.release is not None
    trace = {"samples": 0, "supported_samples": 0}
    original = cast(Any, estimated_mission).accelerometer_specific_force_measurement_body

    def measured(ideal: Array, bias: Array, sigma: Array, rng: np.random.Generator) -> Array:
        first = trace["samples"] == 0
        result: Array = original(
            prepared.evidence["ideal_specific_force_B"][-1] if first else ideal, bias, sigma, rng
        )
        if first:
            assert prepared.release is not None
            ensure(
                np.array_equal(result, prepared.release.fresh_sample.specific_force_B),
                "fresh release sample mismatch",
            )
            trace["supported_samples"] += 1
        trace["samples"] += 1
        return result

    with patch.object(estimated_mission, "accelerometer_specific_force_measurement_body", measured):
        yield trace


def fly(
    prepared: PreparedStart,
    mode: str,
    *,
    faults: tuple[EskfObservationFault, ...] = (),
    supervised: bool = True,
) -> tuple[EstimatedMissionResult, dict[str, Any]]:
    args = configuration_for(prepared, mode)
    health_config, response = policies(args, prepared.partition)
    health = ObservationHealthMonitor(health_config)
    supervisor = ObservationSupervisor(health, response)
    channel = EskfLiveObservationFaults(faults)
    with release_measurement(prepared) as trace:
        result = estimated_mission.simulate_estimated_mission(
            **args,
            observation_health=health,
            observation_supervision=supervisor if supervised else None,
            observation_faults=channel,
        )
    assert channel.injection is not None and prepared.release is not None
    ensure(
        np.array_equal(
            result.measurements.angular_velocity_measurements_B[0],
            prepared.release.fresh_sample.angular_velocity_B,
        ),
        "fresh gyro mismatch",
    )
    diagnostic = dict(
        fault_records=[asdict(r) for r in channel.injection.records],
        health=[asdict(s) for s in health.history],
        supervision=[asdict(s) for s in supervisor.history],
        boundary=trace,
    )
    return result, diagnostic


def verify_noise(
    result: EstimatedMissionResult,
    args: dict[str, Any],
    *,
    supported: bool,
    original_observations: tuple[EskfReplayObservation, ...] | None = None,
) -> dict[str, float]:
    """Reconstruct every actual named-stream draw independently of mission sampling."""
    m, imu, pos = result.mission, args["sensors"].imu, args["sensors"].position
    rng = create_run_random_streams(args["sensors"].root_seed)
    n = len(m.time_s)
    force = np.zeros((n, 3))
    for k in range(n):
        R = rotation_matrix_body_to_world(m.q_WB[k])
        drag = quadratic_drag_force_body(
            m.velocity_W[k],
            R,
            args["truth_world"].wind_velocity_W,
            args["truth_body"].quadratic_drag_coefficient_B,
        )
        force[k] = (
            drag
            + np.array(
                [
                    0.0,
                    0,
                    -args["truth_rotors"].thrust_coefficient * np.sum(m.actual_rotor_omega[k] ** 2),
                ]
            )
        ) / args["truth_body"].mass
        if supported and k == 0:
            force[k] = -R.T @ np.array([0.0, 0, args["truth_world"].gravity_acceleration])
    errors = []
    for measurements, physical, bias, sigma, generator in (
        (
            result.measurements.specific_force_measurements_B,
            force,
            result.true_accelerometer_bias_B,
            imu.accelerometer_noise_standard_deviation_B,
            rng.accelerometer_measurement_noise,
        ),
        (
            result.measurements.angular_velocity_measurements_B,
            m.omega_B,
            result.true_gyroscope_bias_B,
            imu.gyroscope_noise_standard_deviation_B,
            rng.gyroscope_measurement_noise,
        ),
    ):
        expected = sigma * generator.standard_normal((n, 3))
        errors.append(float(np.max(np.abs(measurements - physical - bias - expected))))
    for initial, density, history, generator in (
        (
            imu.initial_accelerometer_bias_B,
            imu.accelerometer_bias_random_walk_density_B,
            result.true_accelerometer_bias_B,
            rng.accelerometer_bias_random_walk,
        ),
        (
            imu.initial_gyroscope_bias_B,
            imu.gyroscope_bias_random_walk_density_B,
            result.true_gyroscope_bias_B,
            rng.gyroscope_bias_random_walk,
        ),
    ):
        expected = initial.copy()
        ensure(np.array_equal(expected, history[0]), "initial true bias mismatch")
        for k in range(1, n):
            expected = expected + density * np.sqrt(
                args["numerics"].time_step_s
            ) * generator.standard_normal(3)
            ensure(np.array_equal(expected, history[k]), "flight bias walk changed")
    sources = (
        result.measurements.observations if original_observations is None else original_observations
    )
    for kind in EskfObservationKind:
        observations = sorted(
            (o for o in sources if o.kind is kind),
            key=lambda o: o.observation_index,
        )
        for o in observations:
            p = m.position_W[o.acquisition_index]
            if kind is EskfObservationKind.LOCAL_POSITION:
                expected = (
                    p
                    + pos.local_position_bias_W
                    + pos.local_position_noise_standard_deviation_W
                    * rng.local_position_measurement_noise.standard_normal(3)
                )
            else:
                expected = np.array(
                    [
                        pos.barometric_reference_altitude
                        - p[2]
                        + pos.barometric_altitude_bias
                        + pos.barometric_altitude_noise_standard_deviation
                        * rng.barometric_altitude_measurement_noise.standard_normal()
                    ]
                )
            errors.append(float(np.max(np.abs(o.measurement - expected))))
    maximum = max(errors)
    ensure(maximum < 1e-12, "flight random draw mismatch")
    return {"maximum_draw_error": maximum}


def audit(
    prepared: PreparedStart,
    mode: str,
    result: EstimatedMissionResult,
    diagnostic: dict[str, Any],
    *,
    supervision: str = "on",
) -> dict[str, float]:
    args = configuration_for(prepared, mode)
    # The existing verifier replays ESKF, commands, guards and supervision against
    # the explicitly changed configuration. No plant/controller law is patched.
    with patch.object(robustness_evidence, "configuration", lambda _case, _partition: args):
        robustness_evidence.audit_result(
            prepared.case,
            prepared.partition,
            supervision,
            result,
            {k: v for k, v in diagnostic.items() if k != "boundary"},
        )
    for field in ("position_W", "velocity_W", "q_WB", "omega_B", "actual_rotor_omega"):
        ensure(
            np.array_equal(getattr(result.mission, field)[0], prepared.evidence[field][-1]),
            "release state/motor discontinuity",
        )
    ensure(
        diagnostic["boundary"] == {"samples": len(result.mission.time_s), "supported_samples": 1},
        "support continued into flight",
    )
    ensure(
        np.array_equal(
            result.measurements.specific_force_measurements_B[0],
            prepared.evidence["specific_force_B"][-1],
        )
        and np.array_equal(
            result.measurements.angular_velocity_measurements_B[0],
            prepared.evidence["angular_velocity_B"][-1],
        ),
        "fresh sample reuse mismatch",
    )
    # Authenticate original draws before causal faults, including dropped sources.
    # The replay above independently verifies the complete source -> output mapping.
    original_observations = tuple(
        robustness_evidence.observation(r["source"]) for r in diagnostic["fault_records"]
    )
    return {
        **verify_noise(result, args, supported=True, original_observations=original_observations),
        **check_plant(pack_history(result, result.mission), args),
    }
