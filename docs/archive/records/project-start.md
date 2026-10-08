# 2026-08-12: Week 1, Session 1

## Objective

Establish and verify the development environment and reproducible Python workflow.

## Environment Decisions

- Standardize on Python 3.12 and uv 0.12.3.
- Use `pyproject.toml`, a committed `uv.lock`, and a project-local `.venv`.
- Use a `src` layout with core algorithms isolated in `quadrotor_math`.

## Concepts Learned

- A `src` layout verifies imports against the installed package.
- A lockfile makes dependency resolution reproducible.
- RED/GREEN test-driven development exposes a missing interface before adding its minimal implementation.

## Files and Workflow Created

- Added Python project metadata, dependency locking, and Make targets for synchronization and quality checks.
- Added the `quadrotor_math` package and its initial deterministic `squared_norm` vector function.
- Added the first focused unit test.

## Verification Evidence

- RED: `uv run pytest tests/unit/test_vectors.py` failed with `ModuleNotFoundError: No module named 'quadrotor_math.vectors'` before `vectors.py` existed.
- GREEN: the focused vector test passed after the minimal implementation was added.
- Quality gate: `make check` passed Ruff linting, Ruff formatting verification, strict mypy checking, and one pytest test.
- Initial commit: `06a9558`.

## Blockers

None for the Python core. WSL graphics and memory compatibility remain untested.

## Next Exact Action

Review and commit the Python workflow, then begin Session 2 with repository structure completion and deterministic-seed foundations.
