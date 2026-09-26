.PHONY: sync test lint format typecheck docs-check check

sync:
	uv sync

test:
	uv run pytest

lint:
	uv run ruff check .

format:
	uv run ruff format .

typecheck:
	uv run mypy src experiments scripts

docs-check:
	uv run python scripts/check_docs.py

check: docs-check
	uv run ruff check .
	uv run ruff format --check .
	uv run mypy src experiments scripts
	uv run pytest
