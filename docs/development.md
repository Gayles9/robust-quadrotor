# Development and validation practices

The core algorithms live in `src/quadrotor_math`; `experiments` connects them to
fixed studies, saved evidence and plots. Tests belong in `tests/unit`, with small
reproducibility inputs in `tests/fixtures`. The source layout, clocks, random
streams and acceptance thresholds are part of the experiment contract.

## Routine checks

Use the [locked Linux environment](guides/getting-started.md):

```bash
uv sync --locked
PYTEST_ADDOPTS='-W error' make check
```

The complete gate checks documentation, lint, formatting, strict typing and
tests. For a focused change, the existing component checks are also available:

```bash
make docs-check
uv run ruff check .
uv run ruff format --check .
uv run mypy src experiments scripts
uv run pytest -q -W error tests/unit/test_geometric_control.py
```

Run `uv build` when checking packaging. The wheel contains `quadrotor_math`;
experiment modules run from a repository checkout. Markdown renders directly on
GitHub. `scripts/check_docs.py` checks local destinations, heading anchors and
fenced Python/JSON syntax. It does not execute examples, validate equations or
check external websites.

A software test pass and a flight performance pass answer different questions.
The [results guide](results/README.md) separates analytical checks, true-state
control, estimator calibration and noisy closed-loop experiments.

## Reproducibility and preservation

I use explicit configurations and random seeds. Experiments bind saved payloads
with SHA-256 and retain failures. Loading and independent rescoring check both
data integrity and the conclusions drawn from the data. Source changes during
a campaign invalidate its recorded run. Use a new output path for every run.

Exact historical bytes can also depend on the numerical backend. Different BLAS
kernels can change a derived floating-point value even with the same NumPy
version and seed. The [feedback-design reproduction contract](design/feedback-design.md#reproduction)
explains the existing narrow protocol-fixture boundary. Production artifact
hashes and archive checks remain strict. A new numerical realization must not
be described as an exact replay of earlier bytes.

Some Markdown is an authenticated experiment input: decision records 0026–0043
and `docs/progress/2026-09-24-feedback-codesign.md` retain their original paths
and bytes. Moving or editing these files would change a frozen protocol. The
remaining historical records live in [the archive](archive/README.md), where
dates, seed assignments, source identities and scientific outcomes remain intact.

New behavioral work should define its scope and acceptance criteria before
implementation, add relevant tests, and retain failures as evidence. Keep core
algorithms independent of middleware, use the [frame contract](architecture/frame-contract.md),
and fix numerical or sign errors at their source. Planned integration and
qualification work is listed in [next steps](next-steps.md).

## Development tooling

I use AI assistance for bounded, repetitive development tasks, including this
repository's documentation organization and reference review. An example prompt
is: “Check these moved paths against their callers and documentation links, run
the existing relevant checks, and report failures without changing equations
or acceptance criteria.” Explicit inputs and acceptance criteria make that work
useful because the proposed changes can be checked against concrete evidence.
This describes the assistance workflow; the equations, tests and retained
simulation evidence support the project's technical conclusions.
