"""An aborted hover must remain a serializable failed trial, including t=0."""

from dataclasses import replace

import numpy as np
import pytest
from test_mission_simulation import simulate

from experiments.attitude_control_validation import canonical_json
from experiments.position_control_validation import baseline_mission, make_case, score_case


@pytest.mark.parametrize("after_first_step", [False, True])
def test_aborted_hover_has_json_boolean_conditions(after_first_step, monkeypatch):
    initial = make_case({"case": "hover", "seed": None, "refinement": 1})[0]
    if after_first_step:
        from quadrotor_math.attitude_simulation import _plant_step

        def crossing(*args):
            state, motors = _plant_step(*args)
            return (np.array([4.0, 0, 0]), *state[1:]), motors

        monkeypatch.setattr("quadrotor_math.mission_simulation._plant_step", crossing)
    else:
        initial = replace(initial, position_W=np.array([4.0, 0, 0]))
    result = simulate(initial_state=initial, plan=baseline_mission("hover"))
    assert result.abort_reason == "geofence"
    assert result.time_s[-1] == (0.0025 if after_first_step else 0.0)
    metrics = score_case({"case": "hover", "seed": None, "refinement": 1}, result)
    assert all(type(value) is bool for value in metrics["conditions"].values())
    assert metrics["conditions"]["60_second_hover"] is False
    assert metrics["passed"] is False
    assert b'"60_second_hover":false' in canonical_json(metrics)
