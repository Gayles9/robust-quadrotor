.PHONY: sync test lint format typecheck check

sync:
	uv sync

test:
	uv run pytest

lint:
	uv run ruff check .

format:
	uv run ruff format .

typecheck:
	uv run mypy src

check:
	uv run ruff check .
	uv run ruff format --check .
	uv run mypy src
	uv run pytest
