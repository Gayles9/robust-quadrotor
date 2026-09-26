"""Physical force-jet identities and explicit measurement-only mode boundaries."""

from dataclasses import replace

import numpy as np
import pytest

from experiments.estimated_feedback_validation import make_configuration
from quadrotor_math import geometric_reference as module
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.geometric_control import GeometricControllerParameters
from quadrotor_math.geometric_filter import FeedbackDerivativeFilter
from quadrotor_math.mission_simulation import simulate_mission
from quadrotor_math.position_control import PositionReference


def configuration():
    return make_configuration(dict(case="smoke", seed=None, noiseless=True))


def test_measured_force_jets_follow_physical_kinematics(monkeypatch):
    outer = replace(configuration()["position_controller"], nominal_mass=1.2)
    p, v = np.array([0.03, -0.02, 0.01]), np.array([0.04, 0.02, -0.01])
    ref = PositionReference(
        np.zeros(3), np.array([0.01, -0.01, 0]), np.array([0.02, -0.03, 0.01]), 0.2
    )
    acceleration = np.array([0.05, -0.04, 0.02])
    jerk, snap = np.array([0.01, 0.02, -0.03]), np.array([0.03, -0.02, 0.01])
    captured = []
    actual = module.force_rotation_reference

    def record(*args):
        captured.append(args)
        return actual(*args)

    monkeypatch.setattr(module, "force_rotation_reference", record)
    _, memory = module.build_geometric_reference(
        p,
        v,
        ref,
        jerk,
        snap,
        outer,
        FeedbackDerivativeFilter(0.02),
        0,
        estimated_acceleration_W=acceleration,
    )
    m, kp, kv = outer.nominal_mass, outer.position_gain_W, outer.velocity_gain_W
    np.testing.assert_allclose(
        captured[0][0],
        m * (np.array([0, 0, 9.81]) - ref.acceleration_W + kp * p + kv * (v - ref.velocity_W)),
    )
    np.testing.assert_allclose(
        captured[0][1],
        m * (-jerk + kp * (v - ref.velocity_W) + kv * (acceleration - ref.acceleration_W)),
    )
    # Constant acceleration prehistory initializes filtered physical jerk to zero.
    np.testing.assert_allclose(
        captured[0][2], m * (-snap + kp * (acceleration - ref.acceleration_W) - kv * jerk)
    )
    np.testing.assert_array_equal(memory.previous, m * acceleration)


def test_constant_position_error_does_not_invent_measured_feedforward():
    outer = configuration()["position_controller"]
    ref = PositionReference(np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    memory = FeedbackDerivativeFilter(0.02)
    for index in range(5):
        target, memory = module.build_geometric_reference(
            np.array([0.05, -0.03, 0]),
            np.zeros(3),
            ref,
            np.zeros(3),
            np.zeros(3),
            outer,
            memory,
            index,
            estimated_acceleration_W=np.zeros(3),
        )
        np.testing.assert_array_equal(target.rotation.omega_reference_D, 0)
        np.testing.assert_array_equal(target.rotation.alpha_reference_D, 0)


def test_measured_mode_cannot_silently_use_true_acceleration():
    args = {
        k: v for k, v in configuration().items() if k not in ("sensors", "estimator_configuration")
    }
    with pytest.raises(ValueError, match="estimated feedback"):
        simulate_mission(
            **args,
            geometric_controller=GeometricControllerParameters(use_measured_acceleration=True),
        )
    with pytest.raises(ValueError, match="mutually exclusive"):
        GeometricControllerParameters(
            use_measured_acceleration=True, rebase_estimator_corrections=True
        )


def test_noiseless_measured_hover_has_zero_acceleration_feedforward(monkeypatch):
    from quadrotor_math import estimated_mission

    actual = estimated_mission._simulate_mission
    accelerations = []

    def record(*args, **kwargs):
        original = kwargs["estimated_acceleration"]

        def checked():
            value = original()
            accelerations.append(value.copy())
            return value

        kwargs["estimated_acceleration"] = checked
        return actual(*args, **kwargs)

    monkeypatch.setattr(estimated_mission, "_simulate_mission", record)
    result = simulate_estimated_mission(
        **configuration(),
        geometric_controller=GeometricControllerParameters(use_measured_acceleration=True),
    )
    # Check the physical acceleration itself: vertical acceleration errors can
    # vanish when a force direction is normalized and escape moment-only tests.
    assert accelerations
    np.testing.assert_allclose(accelerations, 0, atol=1e-13)
    np.testing.assert_allclose(result.mission.position_W, 0, atol=1e-13)
    np.testing.assert_allclose(result.mission.moment_requested_B, 0, atol=1e-13)
    np.testing.assert_allclose(result.mission.collective_thrust, 9.81, atol=1e-13)


@pytest.mark.parametrize("value", [1, 0, None, "yes", np.bool_(True)])
def test_measured_mode_boolean_is_explicit(value):
    with pytest.raises(ValueError, match="use_measured_acceleration"):
        GeometricControllerParameters(use_measured_acceleration=value)
