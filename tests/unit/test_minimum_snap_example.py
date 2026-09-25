import json

import numpy as np
import pytest

from experiments.minimum_snap_example import main


def test_example_independently_verifies_and_preserves_existing_output(tmp_path):
    output = tmp_path / "new-example"
    assert main(["--output", str(output)]) == 0
    report = json.loads((output / "report.json").read_text())
    assert 0 < report["minimum_snap_cost_m2_s7"] < report["stopped_reference_cost_m2_s7"]
    assert report["maximum_waypoint_residual_m"] < 1e-9
    assert max(report["maximum_knot_discontinuity_by_order"]) < 1e-8
    with np.load(output / "trajectory.npz", allow_pickle=False) as archive:
        assert archive["derivatives_W"].shape == (5, 401, 3)
        assert np.all(np.isfinite(archive["derivatives_W"]))
    assert (output / "trajectory.png").stat().st_size > 1000
    before = (output / "report.json").read_bytes()
    with pytest.raises(FileExistsError):
        main(["--output", str(output)])
    assert (output / "report.json").read_bytes() == before
