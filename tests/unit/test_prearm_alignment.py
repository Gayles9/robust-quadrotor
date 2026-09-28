"""Standalone contract: physical assertions, timing, rejection and endpoint ownership."""

from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from quadrotor_math import prearm_alignment as p
from quadrotor_math.eskf import EskfNominalState


def prior():
    state = EskfNominalState(
        np.array([1.0, 2, -3]),
        np.array([0.01, 0, 0]),
        np.array([1.0, 0, 0, 0]),
        np.zeros(3),
        np.zeros(3),
    )
    covariance = np.diag(
        [0.05**2] * 3 + [0.03**2] * 3 + [np.deg2rad(3) ** 2] * 3 + [0.03**2] * 3 + [0.005**2] * 3
    )
    covariance[0, 3] = covariance[3, 0] = 0.0001
    return p.PrearmPrior(state, covariance, navigation_independent=True)


def identity():
    return p.PrearmSessionIdentity("acq-1", "imu-1", "clock-1", "support-1", "fixture-owner", 100)


def support(t=0.0, **changes):
    value = p.StationarySupportEvidence(
        "acq-1", "support-1", "fixture-owner", "clock-1", 0.0, t, True, True, True, True, False
    )
    return replace(value, **changes)


def sample(k, **changes):
    value = p.PrearmImuSample(
        "imu-1", "clock-1", 100 + k, k * 0.0025, np.array([0.0, 0, -9.81]), np.zeros(3)
    )
    return replace(value, **changes)


def session():
    return p.PrearmAlignmentSession(identity(), prior())


def ready():
    current = session()
    for k in range(201):
        current.consume(sample(k), support(k * 0.0025), now_s=k * 0.0025)
    assert current.status is p.PrearmStatus.READY
    return current


def test_no_ready_until_complete_window_and_no_automatic_release():
    current = session()
    assert current.snapshot.accepted_samples == 0
    for k in range(200):
        snapshot = current.consume(sample(k), support(k * 0.0025), now_s=k * 0.0025)
        assert snapshot.status is p.PrearmStatus.COLLECTING
        assert snapshot.estimate is None
    snapshot = current.consume(sample(200), support(0.5), now_s=0.5)
    assert snapshot.status is p.PrearmStatus.READY
    assert snapshot.accepted_samples == 201 and snapshot.last_sample_id == 300
    assert snapshot.estimate is not None
    assert snapshot.estimate.radius_99_deg < 0.75
    assert current.check(support(0.5), now_s=0.5).status is p.PrearmStatus.READY


@pytest.mark.parametrize(
    "changes",
    [
        {"acquisition_id": "other"},
        {"support_id": "other"},
        {"source_id": "other"},
        {"clock_id": "other"},
        {"covered_from_s": 0.001, "observed_through_s": 0.001},
        {"observed_through_s": 0.001},
        {"mechanically_supported": False},
        {"motors_off": False},
        {"zero_world_acceleration": False},
        {"zero_angular_velocity": False},
        {"revoked": True},
    ],
)
def test_invalid_or_missing_support_latches_without_consuming(changes):
    current = session()
    result = current.consume(sample(0), support(**changes), now_s=0)
    assert result.status is p.PrearmStatus.REJECTED
    assert result.accepted_samples == 0 and result.reasons
    with pytest.raises(ValueError, match="terminal"):
        current.consume(sample(0), support(), now_s=0)


def test_support_cannot_be_inferred_from_quiet_samples():
    current = session()
    current.consume(sample(0), None, now_s=0)
    assert current.status is p.PrearmStatus.REJECTED
    assert current.snapshot.reasons == (p.PrearmRejection.MISSING_SUPPORT,)


@pytest.mark.parametrize(
    "changes",
    [
        {"sample_id": 101},
        {"stream_id": "other"},
        {"clock_id": "other"},
        {"time_s": 0.0025},
        {"frame": "NED"},
        {"units": "g;deg/s"},
        {"profile_id": "other"},
    ],
)
def test_mismatched_sample_metadata_latches(changes):
    current = session()
    current.consume(sample(0, **changes), support(), now_s=0)
    assert current.status is p.PrearmStatus.REJECTED
    assert current.snapshot.accepted_samples == 0


@pytest.mark.parametrize("k", [0, 2])
def test_duplicate_or_skipped_sample_cannot_be_recovered_in_same_acquisition(k):
    current = session()
    current.consume(sample(0), support(), now_s=0)
    current.consume(sample(k), support(k * 0.0025), now_s=k * 0.0025)
    assert current.status is p.PrearmStatus.REJECTED
    assert current.snapshot.accepted_samples == 1


@pytest.mark.parametrize("now", [True, np.nan, np.inf, -0.1, "0"])
def test_invalid_call_clock_is_fail_closed(now):
    current = session()
    result = current.consume(sample(0), support(), now_s=now)
    assert result.status is p.PrearmStatus.REJECTED


def test_call_clock_cannot_run_backwards_or_reuse_stale_support():
    current = session()
    current.consume(sample(0), support(), now_s=0)
    current.check(support(0.001), now_s=0.001)
    assert current.check(support(0.0005), now_s=0.0005).status is p.PrearmStatus.REJECTED
    current = session()
    current.consume(sample(0), support(), now_s=0)
    current.consume(sample(1), support(), now_s=0.0025)
    assert current.status is p.PrearmStatus.REJECTED


def test_time_check_detects_missed_sample_and_expired_release():
    current = session()
    current.consume(sample(0), support(), now_s=0)
    assert current.check(support(0.0025), now_s=0.0025).status is p.PrearmStatus.COLLECTING
    assert (
        current.check(support(0.0025 + 2e-12), now_s=0.0025 + 2e-12).status
        is p.PrearmStatus.REJECTED
    )
    current = ready()
    assert (
        current.check(support(0.5025 + 2e-12), now_s=0.5025 + 2e-12).status
        is p.PrearmStatus.REJECTED
    )


@pytest.mark.parametrize("phase", [0, 100, 201])
def test_explicit_revocation_latches_at_every_phase(phase):
    current = session()
    for k in range(phase):
        current.consume(sample(k), support(k * 0.0025), now_s=k * 0.0025)
    t = max(0, phase - 1) * 0.0025
    assert current.check(support(t, revoked=True), now_s=t).status is p.PrearmStatus.REJECTED


def test_release_builds_exact_joint_prior_and_preserves_all_cross_blocks():
    current = ready()
    estimate = current.snapshot.estimate
    initial = prior()
    release = current.release(sample(201), support(0.5025), now_s=0.5025)
    assert release is not None
    assert current.status is p.PrearmStatus.RELEASED
    assert release.fresh_sample.sample_id == 301
    assert release.aligned_sample_ids == (100, 300)
    C = release.endpoint.joint_covariance
    np.testing.assert_array_equal(C[:6, :6], initial.covariance[:6, :6])
    np.testing.assert_array_equal(C[:6, 6:15], 0)
    np.testing.assert_array_equal(C[6:9, 9:12], estimate.joint_covariance[:3, 3:6])
    expected = estimate.joint_covariance.copy()
    expected[3:6, 3:6] += 0.0002**2 * 0.0025 * np.eye(3)
    expected[6:, 6:] += 0.00002**2 * 0.0025 * np.eye(3)
    np.testing.assert_array_equal(C[6:15, 6:15], expected)
    np.testing.assert_array_equal(C[:15, 15:], 0)
    np.testing.assert_array_equal(C[15:, 15:], np.diag([0.04**2] * 3 + [0.002**2] * 3))
    np.testing.assert_array_equal(release.endpoint.imu_noise_mean_B, 0)
    np.testing.assert_array_equal(
        release.endpoint.nominal_state.position_W, initial.nominal_state.position_W
    )
    np.testing.assert_array_equal(
        release.endpoint.nominal_state.velocity_W, initial.nominal_state.velocity_W
    )
    with pytest.raises(ValueError, match="terminal"):
        current.release(sample(201), support(0.5025), now_s=0.5025)


@pytest.mark.parametrize("k", [200, 202])
def test_release_rejects_overlap_and_gap(k):
    current = ready()
    assert current.release(sample(k), support(k * 0.0025), now_s=k * 0.0025) is None
    assert current.status is p.PrearmStatus.REJECTED


def test_release_rechecks_support_and_never_arms():
    current = ready()
    assert current.release(sample(201), support(0.5025, motors_off=False), now_s=0.5025) is None
    assert current.status is p.PrearmStatus.REJECTED
    assert not hasattr(current, "arm") and not hasattr(current, "reset")


def test_incomplete_release_and_extra_acquisition_sample_reject():
    current = session()
    assert current.release(sample(0), support(), now_s=0) is None
    assert current.status is p.PrearmStatus.REJECTED
    current = ready()
    current.consume(sample(201), support(0.5025), now_s=0.5025)
    assert current.status is p.PrearmStatus.REJECTED


def test_input_and_output_ownership_and_tamper_revalidation():
    current = session()
    first = sample(0)
    current.consume(first, support(), now_s=0)
    first.specific_force_B.flags.writeable = True
    first.specific_force_B[:] = np.nan
    for k in range(1, 201):
        current.consume(sample(k), support(k * 0.0025), now_s=k * 0.0025)
    assert current.status is p.PrearmStatus.READY
    snapshot = current.snapshot
    saved = snapshot.estimate.joint_covariance.copy()
    snapshot.estimate.joint_covariance.flags.writeable = True
    snapshot.estimate.joint_covariance[:] = 0
    np.testing.assert_array_equal(current.snapshot.estimate.joint_covariance, saved)
    with pytest.raises(FrozenInstanceError):
        snapshot.accepted_samples = 0
    fresh = sample(201)
    fresh.specific_force_B.flags.writeable = True
    fresh.specific_force_B[:] = np.nan
    assert current.release(fresh, support(0.5025), now_s=0.5025) is None


@pytest.mark.parametrize(
    "bad",
    [np.zeros(2), np.array([np.nan, 0, 0]), np.array([np.inf, 0, 0]), np.array(["x", "y", "z"])],
)
def test_invalid_vector_cannot_form_a_sample(bad):
    with pytest.raises(ValueError):
        sample(0, specific_force_B=bad)


@pytest.mark.parametrize("bad", [True, -1, 1.5, "0"])
def test_invalid_sample_ids_and_identity_ids_are_rejected(bad):
    with pytest.raises(ValueError):
        sample(0, sample_id=bad)
    with pytest.raises(ValueError):
        replace(identity(), first_sample_id=bad)


def test_unsupported_priors_profiles_and_correlations_are_rejected():
    initial = prior()
    with pytest.raises(ValueError, match="independence"):
        replace(initial, navigation_independent=False)
    for i, j in ((0, 6), (6, 9), (9, 12)):
        C = initial.covariance.copy()
        C[i, j] = C[j, i] = 1e-6
        with pytest.raises(ValueError):
            replace(initial, covariance=C)
    C = initial.covariance.copy()
    C[8, 8] *= 1.01
    with pytest.raises(ValueError):
        replace(initial, covariance=C)
    for state in (
        replace(initial.nominal_state, accelerometer_bias_B=np.ones(3) * 0.001),
        replace(initial.nominal_state, gyroscope_bias_B=np.ones(3) * 0.001),
        replace(initial.nominal_state, q_WB=np.array([np.cos(0.1), 0, 0, np.sin(0.1)])),
    ):
        with pytest.raises(ValueError):
            replace(initial, nominal_state=state)
    for kwargs in ({"gravity_acceleration": 9.8}, {"sample_period_s": 0.005}):
        with pytest.raises(ValueError):
            p.PrearmAlignmentSession(identity(), initial, **kwargs)
    noise = p.prearm_imu_noise()
    changed = noise.sample_covariance_B.copy() * 1.01
    with pytest.raises(ValueError):
        p.PrearmAlignmentSession(
            identity(), initial, noise=replace(noise, sample_covariance_B=changed)
        )
    with pytest.raises(ValueError):
        p.PrearmAlignmentSession(replace(identity(), profile_id="unknown"), initial)


def test_sign_equivalent_identity_and_semidefinite_navigation_prior_are_allowed():
    initial = prior()
    C = initial.covariance.copy()
    C[:6, :6] = 0
    state = replace(initial.nominal_state, q_WB=np.array([-1.0, 0, 0, 0]))
    p.PrearmAlignmentSession(identity(), replace(initial, covariance=C, nominal_state=state))


@pytest.mark.parametrize(
    "mode",
    [
        "impulse",
        "vibration",
        "changing_gravity",
        "mean_rate",
        "gravity_magnitude",
        "uncertainty",
        "upside_down",
        "degenerate",
        "overflow",
    ],
)
def test_numeric_rejections_are_latched_with_no_endpoint(mode):
    rng = np.random.default_rng(293001)
    accel = np.tile([0.0, 0, -9.81], (201, 1)) + rng.normal(0, 0.04, (201, 3))
    gyro = rng.normal(0, 0.002, (201, 3))
    time = np.arange(201) * 0.0025
    if mode == "impulse":
        accel[100, 0] += 1
    if mode == "vibration":
        accel[:, 0] += 0.2 * np.sin(2 * np.pi * 40 * time)
    if mode == "changing_gravity":
        accel[:, 1] += np.linspace(0, 0.5, 201)
    if mode == "mean_rate":
        gyro[:, 2] += 0.05
    if mode == "gravity_magnitude":
        accel[:, 2] -= 1
    if mode == "uncertainty":
        accel[:] = [0, -9.81 * np.sin(0.1), -9.81 * np.cos(0.1)]
    if mode == "upside_down":
        accel[:] = [0, 0, 9.81]
    if mode == "degenerate":
        accel[:] = 0
    if mode == "overflow":
        accel[:] = 1e308
    current = session()
    for k in range(201):
        current.consume(
            sample(k, specific_force_B=accel[k], angular_velocity_B=gyro[k]),
            support(time[k]),
            now_s=time[k],
        )
    assert current.status is p.PrearmStatus.REJECTED
    assert current.snapshot.reasons


def test_constant_acceleration_ambiguity_is_preserved_not_claimed_as_stationarity():
    # Moving level vehicle: f=a-g. Supported one-degree pitch gives the same f.
    force = np.array([9.81 * np.sin(np.deg2rad(1)), 0, -9.81 * np.cos(np.deg2rad(1))])
    assert np.linalg.norm(force + [0, 0, 9.81]) > 0.17
    current = session()
    for k in range(201):
        current.consume(sample(k, specific_force_B=force), support(k * 0.0025), now_s=k * 0.0025)
    assert current.status is p.PrearmStatus.READY  # Only under the external assertion.
    missing = session()
    missing.consume(sample(0, specific_force_B=force), None, now_s=0)
    assert missing.status is p.PrearmStatus.REJECTED


def test_support_gap_even_with_contemporaneous_timestamp_rejects():
    current = session()
    current.consume(sample(0), support(), now_s=0)
    result = current.consume(sample(1), support(0.0025, covered_from_s=0.001), now_s=0.0025)
    assert result.status is p.PrearmStatus.REJECTED
    assert result.accepted_samples == 1


@pytest.mark.parametrize("operation", ["consume", "check", "release"])
def test_wrong_support_type_latches_for_each_operation(operation):
    current = ready() if operation == "release" else session()
    kwargs = {"support": "trusted", "now_s": 0.5025 if operation == "release" else 0}
    if operation != "check":
        kwargs["sample"] = sample(201 if operation == "release" else 0)
    getattr(current, operation)(**kwargs)
    assert current.status is p.PrearmStatus.REJECTED


@pytest.mark.parametrize("bad", [None, np.zeros(3), "sample"])
def test_wrong_sample_type_latches(bad):
    current = session()
    current.consume(bad, support(), now_s=0)
    assert current.status is p.PrearmStatus.REJECTED


@pytest.mark.parametrize(
    "changes",
    [
        {"motors_off": 1},
        {"revoked": None},
        {"mechanically_supported": np.bool_(True)},
        {"observed_through_s": np.nan},
        {"covered_from_s": -1},
        {"covered_from_s": 0.1},
        {"source_id": " "},
    ],
)
def test_structurally_invalid_support_raises_before_session_call(changes):
    current = session()
    with pytest.raises(ValueError):
        support(**changes)
    assert current.status is p.PrearmStatus.COLLECTING


def test_all_numeric_failures_and_diagnostics_are_retained():
    current = session()
    for k in range(201):
        force = np.array([0.0, 0, -11.0])
        force[0] = 1.0 if k % 2 else -1.0
        current.consume(
            sample(k, specific_force_B=force, angular_velocity_B=np.array([0.1, 0, 0])),
            support(k * 0.0025),
            now_s=k * 0.0025,
        )
    snapshot = current.snapshot
    assert snapshot.estimate is not None
    assert set(snapshot.reasons) == {
        p.PrearmRejection.RATE_COMPATIBILITY,
        p.PrearmRejection.GRAVITY_COMPATIBILITY,
        p.PrearmRejection.TEMPORAL_COMPATIBILITY,
    }
    with pytest.raises(ValueError, match="terminal"):
        current.check(support(0.5), now_s=0.5)
    assert current.snapshot.reasons == snapshot.reasons


def test_fresh_sample_does_not_recondition_alignment_or_endpoint():
    first, second = ready(), ready()
    normal = first.release(sample(201), support(0.5025), now_s=0.5025)
    different = second.release(
        sample(201, specific_force_B=np.ones(3) * 100, angular_velocity_B=np.ones(3)),
        support(0.5025),
        now_s=0.5025,
    )
    assert normal is not None and different is not None
    np.testing.assert_array_equal(
        normal.endpoint.joint_covariance, different.endpoint.joint_covariance
    )
    np.testing.assert_array_equal(
        normal.endpoint.nominal_state.q_WB, different.endpoint.nominal_state.q_WB
    )
    np.testing.assert_array_equal(
        normal.endpoint.nominal_state.gyroscope_bias_B,
        different.endpoint.nominal_state.gyroscope_bias_B,
    )


def test_prior_is_owned_and_revalidated_at_session_construction():
    initial = prior()
    current = p.PrearmAlignmentSession(identity(), initial)
    initial.covariance.flags.writeable = True
    initial.covariance[:] = np.nan
    with pytest.raises(ValueError):
        p.PrearmAlignmentSession(identity(), initial)
    for k in range(201):
        current.consume(sample(k), support(k * 0.0025), now_s=k * 0.0025)
    released = current.release(sample(201), support(0.5025), now_s=0.5025)
    np.testing.assert_array_equal(
        released.endpoint.joint_covariance[:6, :6], prior().covariance[:6, :6]
    )


@pytest.mark.parametrize(
    "offset, accepted", [(-0.5e-12, True), (0.5e-12, True), (-2e-12, False), (2e-12, False)]
)
def test_sample_clock_tolerance_boundary(offset, accepted):
    current = session()
    current.consume(sample(0), support(), now_s=0)
    t = 0.0025 + offset
    result = current.consume(sample(1, time_s=t), support(t), now_s=t)
    assert (result.status is p.PrearmStatus.COLLECTING) is accepted


def test_numpy_integer_ids_cannot_overflow_sequence_arithmetic():
    first = np.uint64(np.iinfo(np.uint64).max)
    current = p.PrearmAlignmentSession(replace(identity(), first_sample_id=first), prior())
    current.consume(sample(0, sample_id=first), support(), now_s=0)
    result = current.consume(sample(1, sample_id=int(first) + 1), support(0.0025), now_s=0.0025)
    assert result.status is p.PrearmStatus.COLLECTING and result.accepted_samples == 2
