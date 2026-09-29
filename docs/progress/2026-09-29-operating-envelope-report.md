# Original-sensor operating envelope and report consolidation

## Scope and acceptance fixed before editing

I audited merged PR #39 at
`67499e9eb2c9a0acd72eb45d7be690f1bb2e3a2b`. The remote main branch and local
checkout agree. The preceding inclination contract is mathematically checked
but cannot justify a component under the original sensor assumptions. Startup
tuning and the sensor extension remain closed; all failed flight requirements
and current defaults are retained.

The fresh warning-strict preceding-milestone audit passed 42 tests in 0.23 s:

```bash
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_inclination_feasibility.py tests/unit/test_documentation.py
.venv/bin/python scripts/check_docs.py
```

Documentation checks passed for 151 files, 681 local links and 23 Python/JSON
examples. The preceding full local/CI result of 3,949 passing tests remains
separately dated evidence, not a newly executed full suite for this report.

The existing editable report is revision 2.0 at source
`92256d62722081f43f6ff0952888dcd3a7444d0d`. The source ZIP authenticates to its
published hash. The separately stored PDF has a different byte hash from the
PDF in that ZIP, but all 41 pages have identical extracted text and rendered
pixels. Both remain prior artifacts; this does not silently rewrite the old
build's byte identity.

Acceptance for revision 3.0 requires:

- One source-indexed capability/requirement table, separating default behavior,
  research components, passing evidence, failed or conditional requirements,
  and unimplemented integration.
- The supported initialization, exact velocity conditioning, release boundary,
  nonlinear joint moments, full-flight accounting and inclination stopping
  decision explained with their equations and assumptions.
- Existing numerical evidence authenticated, tabulated claims independently
  checked and original scoring windows/failed comparisons preserved.
- A self-contained LaTeX build with no unresolved references or layout defects,
  inspected PDF pages, exact source identities and reproducible audit commands.
- Updated existing report artifacts with their identity/history preserved,
  and publication of the repository documentation after verification.

No flight execution, controller/estimator behavior, gain, threshold, seed,
dependency, sensor assumption, default or middleware changes belong to this
step. Documentation completion does not close estimated-flight gate G2. Any
subsequent integration work needs a separately frozen simulation-only contract.

## Outcome and engineering decision

The consolidation meets its documentation/report acceptance. Revision 3.0 is a
51-page complete LaTeX report with a 13-row source-indexed capability/requirement
ledger. The report preserves the original mathematical foundation and adds
supported initialization, exact velocity conditioning, the force discontinuity,
nonlinear joint release moments, matched outcomes, signed whole-flight budgets
and the independent-inclination stopping decision. The README now leads to the
ledger and detailed guides rather than repeating the entire study chronology.

The combined candidate passes all nine tested clean absolute flight conditions
but fails five strict comparisons against one or both controls. The report keeps
the original hover, mass and geometric failures visible, distinguishes reused
development seeds from fresh qualification, and retains the original cascade
and ordinary mission initialization. No production Python or experiment code
changes are included. G2 remains open.

Next: a separately frozen simulation-only ROS 2/PX4 interface and acceptance
design. Select one initial boundary, specify frames/clocks/authority/resources
and offline parity fixtures, and record a go/no-go before adapters, dependency
changes or new campaigns. No controller promotion or hardware action follows
from report completion.

## Fresh verification

The independent report checker authenticates nine exact summary reports (two
historical and seven current). It authenticates 436 payload references and
recomputes full-grid RMSE, terminal position/speed/time and applicable inclusive
hover maxima from all 84 saved executions. It imports no flight implementation
and executes no scientific flight. It checks 144 saved Gram windows, including
signed cross terms and nonzero means, and the exact generated table contents.
Full ESKF/command replay and the large nonlinear calibration populations remain
separately dated preceding results.

The new checker also rejects an altered report byte and an altered typeset
number in isolated temporary copies. All 91 LaTeX repository paths resolve;
the new operating-envelope guide is the documentation handoff, while the
mathematical implementation stays pinned to PR #39.

Repository verification after edits:

```bash
.venv/bin/python scripts/check_docs.py
.venv/bin/python -m ruff check .
.venv/bin/python -m ruff format --check .
.venv/bin/python -m mypy src experiments scripts
OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='-W error' .venv/bin/python -m pytest -q tests/unit/test_documentation.py
git diff --check
```

Documentation passes for 153 files, 687 local links and 23 Python/JSON examples.
Ruff passes with 345 Python files formatted; mypy passes 101 source files.
The 11 documentation tests pass in 0.04 s and overlap the earlier 42-test audit;
these are not 53 distinct tests. The full 3,949-test result from PR #39 is
preceding evidence, not a newly executed local suite for this documentation step.

The report's `BUILD.md` supplies portable independent audit commands. The new
audit script also passes Ruff E/F/I/B checks with line length 100. The final
clean source-only LaTeX build needs neither repository access nor raw payloads.
It has no unresolved references, warnings, overfull or underfull boxes. All pages
were rasterized and visually reviewed, including detailed equation/table/figure
checks. Text bounds and replacement-glyph checks pass.

## Corrections and artifact integrity

The first independent report audit exposed a schema distinction: the budget
omits the null clean-flight `abort_reason` field. The checker now requires that
field to be null in source scores and compares every other field exactly.
No numerical tolerance or result was changed.

One intermediate review contact sheet was truncated and regenerated from intact
page rasters. A later PDF and bookmark auxiliary file were truncated despite a
successful compiler log; the cause was not established. A separate clean
source-only build, compiler byte/page-count checks, atomic verified copy and
fresh page review recovered the output. No flight payload was modified.

The 66-file source package passes ZIP CRC checks and independent SHA-256 checks
for every internal manifest entry. Its embedded PDF is byte-identical to the
standalone compiled build. Existing report filenames and artifact identities
are retained; the revision inside the document is authoritative.

| Compiled deliverable | SHA-256 |
| --- | --- |
| `Robust_Quadrotor_Technical_Report_v1.pdf` | `ce3a902164f39db2437b55d197c39bc011e56311a7464d6fc59ca775f9178f9e` |
| `Robust_Quadrotor_Report_v1.zip` | `76393742fb095578e3ffe32bb9b9e48063d8257793d6fce3cb16668cb3c727ff` |

These identify the compiled build, not a claim that future metadata-bearing
copies preserve identical bytes. The previous revision's byte identities and
review records remain in the source package. Publication uses documentation
only, with hosted checks required before merge.
