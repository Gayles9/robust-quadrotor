# 0001: Python Workflow

- Date: 2026-08-12
- Status: Accepted

## Context

The project needs a reproducible Python workflow before mathematical or integration work begins.

## Decision

- Use Python 3.12 for a stable, single-version development target.
- Use `pyproject.toml` as the standard source of project metadata, dependencies, and tool configuration.
- Use uv 0.12.3 for deterministic environment and dependency management.
- Commit the generated `uv.lock` so all environments resolve the same dependency versions.
- Let uv maintain a project-local `.venv` to isolate project tools and dependencies.
- Use a `src` layout so imports exercise the installed package rather than the repository directory.

## Reason

These choices provide fast setup, explicit configuration, reproducible resolution, environment isolation, and reliable package-import behavior while keeping the workflow small.

## Consequences/Risks

Contributors must use Python 3.12 and the pinned uv version. Dependency changes require regenerating and reviewing `uv.lock`, and local environments must be synchronized before running tools.

## Revisit Trigger

Revisit when Python 3.12 or uv 0.12.3 no longer supports required tooling, or when packaging and deployment requirements materially change.
