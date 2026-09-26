"""Small repository fixtures exercise documentation failures seen in practice."""

from pathlib import Path

import pytest

from scripts.check_docs import check_documents


def _repository(tmp_path: Path, readme: str, guide: str = "# Guide\n") -> Path:
    (tmp_path / "README.md").write_text(readme, encoding="utf-8")
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs" / "guide.md").write_text(guide, encoding="utf-8")
    return tmp_path


def test_valid_relative_links_fragments_images_and_reference_definitions(tmp_path: Path) -> None:
    root = _repository(
        tmp_path,
        "# Home\n[guide](docs/guide.md#design--choices)\n"
        "[repeat](docs/guide.md#design--choices-1)\n![plot](docs/plot.svg)\n"
        "[source][ref]\n[ref]: docs/guide.md\n[web](https://example.com/absent)\n",
        "# Guide\n## Design & choices\n## Design & choices\n[home](../README.md#home)\n",
    )
    (root / "docs" / "plot.svg").write_text("<svg/>", encoding="utf-8")
    errors, files, links, examples = check_documents(root)
    assert errors == []
    assert (files, links, examples) == (2, 5, 0)


@pytest.mark.parametrize(
    ("content", "expected"),
    [
        ("[broken](docs/absent.md)\n", "missing link target"),
        ("[broken](docs/guide.md#absent)\n", "missing heading anchor"),
        ("```python\ndef invalid(\n```\n", "invalid python example"),
        ("```python\nreturn 1\n```\n", "invalid python example"),
        ('```json\n{"bad": }\n```\n', "invalid json example"),
        ("```python\nx = 1\n", "unclosed code fence"),
        ("~~~python\nx = 1\n```\n", "unclosed code fence"),
    ],
)
def test_errors_include_the_source_location(tmp_path: Path, content: str, expected: str) -> None:
    root = _repository(tmp_path, content)
    errors, _, _, _ = check_documents(root)
    assert any(error.startswith("README.md:1:") and expected in error for error in errors)


def test_examples_are_checked_without_executing_or_resolving_their_text(tmp_path: Path) -> None:
    root = _repository(
        tmp_path,
        "# Home\n```python\nraise RuntimeError('must not execute')\n```\n"
        '~~~json\n{"ok": true}\n~~~\n'
        "````markdown\n# Pretend heading\n[absent](not-real.md)\n```\n````\n"
        "`[example](not-real.md)`\n[heading](#home)\n",
    )
    errors, _, links, examples = check_documents(root)
    assert errors == []
    assert links == 1
    assert examples == 2


def test_a_heading_in_a_code_example_is_not_a_section(tmp_path: Path) -> None:
    root = _repository(tmp_path, "```text\n# Example\n```\n[bad](#example)\n")
    errors, _, _, _ = check_documents(root)
    assert errors == ["README.md:4: missing heading anchor: #example"]


def test_nested_and_encoded_paths(tmp_path: Path) -> None:
    root = _repository(tmp_path, "[guide](docs/nested/other%20guide.md#café)\n")
    nested = root / "docs" / "nested"
    nested.mkdir()
    (nested / "other guide.md").write_text("# Café\n[up](../guide.md#guide)\n", encoding="utf-8")
    assert check_documents(root)[0] == []
