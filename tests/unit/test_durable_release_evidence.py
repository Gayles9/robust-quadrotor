"""Evidence durability must not change bytes or overwrite a prior outcome."""

import hashlib

import numpy as np
import pytest

from experiments.durable_release_evidence import save_arrays, save_json, staged_directory
from experiments.robustness_evidence import save_json as ordinary_json


def test_durable_arrays_roundtrip_and_no_clobber(tmp_path):
    path = tmp_path / "errors.npz"
    values = {"errors": np.arange(42).reshape(2, 21)}
    save_arrays(path, values)
    original = path.read_bytes()
    with np.load(path, allow_pickle=False) as archive:
        np.testing.assert_array_equal(archive["errors"], values["errors"])
    with pytest.raises(FileExistsError):
        save_arrays(path, {"errors": np.zeros((2, 21))})
    assert path.read_bytes() == original
    assert list(tmp_path.iterdir()) == [path]


def test_json_bytes_unchanged(tmp_path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    first.mkdir()
    second.mkdir()
    value = {"a": [1, 2.0, False]}
    assert save_json(first, "record.json", value) == ordinary_json(second, "record.json", value)
    assert (first / "record.json").read_bytes() == (second / "record.json").read_bytes()
    record = save_json(first, "other.json", value)
    assert record["sha256"] == hashlib.sha256((first / "other.json").read_bytes()).hexdigest()


def test_failed_writer_publishes_nothing(tmp_path):
    with pytest.raises(RuntimeError, match="interrupted"):
        with staged_directory(tmp_path) as staging:
            (staging / "partial").write_bytes(b"incomplete")
            raise RuntimeError("interrupted")
    assert list(tmp_path.iterdir()) == []
