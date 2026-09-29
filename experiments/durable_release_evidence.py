"""Atomic, durable, no-clobber output for the ADR0036 evidence campaign."""

import os
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

import numpy as np

from experiments.estimated_feedback_evidence import save_history as _save_history
from experiments.robustness_evidence import save_json as _save_json


@contextmanager
def staged_directory(directory: Path) -> Iterator[Path]:
    """Publish only closed, fsynced files; never replace an existing evidence file."""
    with TemporaryDirectory(prefix="evidence-", dir=directory) as temporary:
        staging = Path(temporary)
        yield staging
        paths = sorted(staging.iterdir())
        for path in paths:
            if (directory / path.name).exists():
                raise FileExistsError(directory / path.name)
            with path.open("rb") as stream:
                os.fsync(stream.fileno())
        for path in paths:
            os.link(path, directory / path.name)
        descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def save_arrays(path: Path, arrays: dict[str, Any]) -> None:
    """Save one losslessly compressed Gaussian payload."""
    with staged_directory(path.parent) as staging:
        with (staging / path.name).open("xb") as stream:
            np.savez_compressed(stream, **arrays)


def save_history(directory: Path, index: int, arrays: dict[str, Any]) -> list[dict[str, str]]:
    """Retain the canonical history schema and digest records without partial files."""
    with staged_directory(directory) as staging:
        records = _save_history(staging, index, arrays)
    return records


def save_json(directory: Path, name: str, value: Any) -> dict[str, str]:
    """Retain the canonical JSON schema and digest record without a partial file."""
    with staged_directory(directory) as staging:
        record = _save_json(staging, name, value)
    return record
