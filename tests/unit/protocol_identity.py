"""Golden protocol regression identity with a narrow derived-attitude boundary.

This helper is test-only. Production report authentication remains byte exact.
BLAS norm reductions can change initial quaternion components by a few ULPs.
No other field is normalized, rounded, removed or substituted for digest checks.
"""

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

import numpy as np

from experiments.attitude_control_validation import canonical_json


def frozen_protocol_identity(protocol: dict[str, Any]) -> bytes:
    fixture = json.loads(
        (
            Path(__file__).parents[1] / "fixtures/feedback_protocol_initial_attitudes.json"
        ).read_bytes()
    )["q_WB_by_seed"]
    result = deepcopy(protocol)

    def visit(value: Any) -> None:
        if isinstance(value, dict):
            if "configurations" in value and "jobs" in value:
                for job, config in zip(value["jobs"], value["configurations"], strict=True):
                    if "initial_state" in config:
                        expected = np.asarray(fixture[str(job["seed"])], dtype=np.float64)
                        actual = np.asarray(config["initial_state"]["q_WB"], dtype=np.float64)
                        assert actual.shape == expected.shape == (4,)
                        assert np.all(np.isfinite(actual))
                        np.testing.assert_array_max_ulp(actual, expected, maxulp=4)
                        config["initial_state"]["q_WB"] = expected.tolist()
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(result)
    return canonical_json(result)
