"""Check the Markdown conventions used in this repository without extra dependencies.

Check relative inline links, images and reference definitions against files and
GitHub-style ATX heading anchors. Check fenced Python/JSON syntax without running
examples. External URL availability and mathematical correctness require review.
This intentionally handles the repository's Markdown subset, not all CommonMark.
"""

from __future__ import annotations

import html
import json
import re
import sys
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import unquote, urlsplit

_FENCE = re.compile(r"^ {0,3}(`{3,}|~{3,})(.*)$")
_HEADING = re.compile(r"^ {0,3}#{1,6}\s+(.+?)(?:\s+#+\s*)?$")
_INLINE_CODE = re.compile(r"(`+).*?\1")
_LINK = re.compile(r"!?\[[^\]\n]*\]\(\s*(<[^>\n]+>|[^\s)]+)(?:\s+[\"\'][^\n]*?[\"\'])?\s*\)")
_REFERENCE = re.compile(r"^ {0,3}\[[^\]\n]+\]:\s*(<[^>\n]+>|\S+)")


@dataclass(frozen=True)
class Document:
    """Parsed prose, anchors and syntax errors with original line numbers."""

    lines: tuple[str, ...]
    anchors: frozenset[str]
    errors: tuple[str, ...]
    examples: int


def _slug(heading: str) -> str:
    heading = re.sub(r"!?\[([^\]]+)\]\([^)]*\)", r"\1", heading)
    heading = html.unescape(re.sub(r"<[^>]+>", "", heading)).lower().strip()
    return "".join(
        "-" if character == " " else character
        for character in heading
        if character in " -_" or unicodedata.category(character)[0] in "LN"
    )


def parse_document(content: str) -> Document:
    """Extract visible ATX headings and check closed, syntax-labelled fences."""
    prose: list[str] = []
    anchors: set[str] = set()
    errors: list[str] = []
    fence = ""
    language = ""
    first_line = 0
    body: list[str] = []
    examples = 0
    for number, line in enumerate(content.splitlines(), start=1):
        match = _FENCE.match(line)
        if fence:
            prose.append("")
            if (
                match
                and match[1][0] == fence[0]
                and len(match[1]) >= len(fence)
                and not match[2].strip()
            ):
                if language in {"python", "json"}:
                    examples += 1
                    source = "\n".join(body)
                    try:
                        if language == "python":
                            compile(source, "<documentation example>", "exec")
                        else:
                            json.loads(source)
                    except (SyntaxError, json.JSONDecodeError) as error:
                        errors.append(f"{first_line}: invalid {language} example: {error}")
                fence = ""
                body = []
            else:
                body.append(line)
            continue
        if match:
            fence = match[1]
            language = match[2].strip().split(maxsplit=1)[0] if match[2].strip() else ""
            first_line = number
            prose.append("")
            continue
        prose.append(line)
        if heading := _HEADING.match(line):
            slug = _slug(heading[1])
            anchor = slug
            suffix = 0
            while anchor in anchors:
                suffix += 1
                anchor = f"{slug}-{suffix}"
            anchors.add(anchor)
    if fence:
        errors.append(f"{first_line}: unclosed code fence")
    return Document(tuple(prose), frozenset(anchors), tuple(errors), examples)


def check_documents(root: Path) -> tuple[list[str], int, int, int]:
    """Return errors and counts for root Markdown files and all guides/records."""
    root = root.resolve()
    paths = sorted(set(root.glob("*.md")) | set((root / "docs").rglob("*.md")))
    documents = {path: parse_document(path.read_text(encoding="utf-8")) for path in paths}
    errors = [
        f"{path.relative_to(root)}:{error}"
        for path, document in documents.items()
        for error in document.errors
    ]
    links = 0
    for path in paths:
        document = documents[path]
        for number, line in enumerate(document.lines, start=1):
            # Inline code often describes example link syntax; it is not a link.
            prose = _INLINE_CODE.sub("", line)
            targets = [match[1].strip("<>") for match in _LINK.finditer(prose)]
            if reference := _REFERENCE.match(prose):
                targets.append(reference[1].strip("<>"))
            for destination in targets:
                parts = urlsplit(destination)
                if parts.scheme or parts.netloc:
                    continue
                links += 1
                target = (path.parent / unquote(parts.path)).resolve() if parts.path else path
                prefix = f"{path.relative_to(root)}:{number}"
                if not target.exists():
                    errors.append(f"{prefix}: missing link target: {destination}")
                    continue
                if parts.fragment and target.suffix.lower() == ".md":
                    if target not in documents:
                        documents[target] = parse_document(target.read_text(encoding="utf-8"))
                    if unquote(parts.fragment) not in documents[target].anchors:
                        errors.append(f"{prefix}: missing heading anchor: {destination}")
    return errors, len(paths), links, sum(document.examples for document in documents.values())


def main() -> int:
    """Print actionable source locations and fail the gate on documentation errors."""
    root = Path(__file__).resolve().parents[1]
    errors, files, links, examples = check_documents(root)
    for error in errors:
        print(error, file=sys.stderr)
    print(f"Documentation: {files} files, {links} local links, {examples} Python/JSON examples")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
