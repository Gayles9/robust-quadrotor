"""A numerical test boundary must not weaken production evidence authentication."""

import hashlib
import json

import numpy as np
import pytest
from protocol_identity import frozen_protocol_identity

from experiments.attitude_control_validation import canonical_json
from experiments.feedback_bandwidth_validation import validate_report, validation_protocol


@pytest.mark.parametrize(
    "partition, expected",
    [
        ("fixed", "08fed2f9bded2462ae06991333e4f1449376a8dcbb77262d7c83e2f8db6864ec"),
        ("development", "e55ed6f3cef35f4e9baa1e062c489661f558393be7c83d580ba8c4d719a7b7fb"),
        ("validation", "7c4800d34d12b89d4476cbd9ca46d9e39df8ac2900789c148f37ca1ec76b7230"),
    ],
)
def test_version_two_golden_definitions(partition, expected):
    protocol = validation_protocol(partition, design_version=2)
    before = canonical_json(protocol)
    assert hashlib.sha256(frozen_protocol_identity(protocol)).hexdigest() == expected
    assert canonical_json(protocol) == before


@pytest.mark.parametrize("units", [1, 4, 5])
def test_only_bounded_attitude_roundoff_is_normalized(units):
    original = frozen_protocol_identity(validation_protocol("fixed"))
    changed = json.loads(original)
    attitude = changed["configurations"][1]["initial_state"]["q_WB"]
    for _ in range(units):
        attitude[2] = float(np.nextafter(attitude[2], np.inf))
    if units <= 4:
        assert frozen_protocol_identity(changed) == original
    else:
        with pytest.raises(AssertionError):
            frozen_protocol_identity(changed)


def test_non_attitude_changes_are_still_exactly_visible():
    original = frozen_protocol_identity(validation_protocol("fixed"))
    changed = json.loads(original)
    state = changed["configurations"][1]["initial_state"]
    state["position_W"][0] = float(np.nextafter(state["position_W"][0], np.inf))
    assert frozen_protocol_identity(changed) != original


def test_production_protocol_validation_rejects_even_one_attitude_unit():
    protocol = validation_protocol("fixed")
    attitude = protocol["configurations"][1]["initial_state"]["q_WB"]
    attitude[2] = float(np.nextafter(attitude[2], np.inf))
    report = {
        "protocol": protocol,
        "protocol_sha256": hashlib.sha256(canonical_json(protocol)).hexdigest(),
    }
    with pytest.raises(ValueError, match="protocol differs"):
        validate_report(report)
