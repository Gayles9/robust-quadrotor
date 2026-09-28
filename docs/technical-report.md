# Technical report and operating boundary

Revision 2.0, dated 2026-09-28, updates the original prediction-only technical
report to source baseline
`92256d62722081f43f6ff0952888dcd3a7444d0d`.

The separate PDF and complete editable LaTeX package retain the existing
filenames `Robust_Quadrotor_Technical_Report_v1.pdf` and
`Robust_Quadrotor_Report_v1.zip` for artifact continuity. The revision stated
inside the report is authoritative. They are separate deliverables, not
generated files committed to this repository.

## What it covers

- Frame conventions, plant, motors, numerical integration, sensors and timing.
- ESKF prediction, correction/reset, endpoint noise memory and causal replay.
- Minimum-snap planning, nominal reference bounds and bounded retiming.
- Cascade and geometric control, derivative tradeoffs and bounded vertical compensation.
- Observation availability, timed aborts, complete campaign results and retained failures.
- Reproduction commands, exact source/report identities, current operating
  limitations and a separately scoped future ROS 2/PX4 interface.

The default remains the original cascade. Geometric control and vertical
compensation remain explicit research options. The heavier-mass candidate now
completes and reduces final error to 6.02 cm, but full-flight RMSE is 17.57 cm
against 15 cm; hover remains above 8 cm. Software correctness, response
acceptance and flight qualification are different decisions.

## Build and evidence

From the source package's `report/` directory:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
```

The package includes exact campaign summary JSON, numerical tables, bibliography,
figures, an independent claim/payload audit, an authenticated plotting script,
build instructions and current verification/traceability notes. Raw flight
payloads remain in the two preserved campaign archives.

| Identity | SHA-256 |
| --- | --- |
| Candidate execution source | `008470bec6b8b1137f7cb90db2760925e24329a475fcda5c2ef579b6618fceac` |
| Original campaign report | `ec85a0d6b55acc278acb18b1c7af7b02cb56c6e985dc48eaf66fbc7fd520bb09` |
| Candidate campaign report | `46a506cc25881cb86a22fd7c4a3aeb85ff1f6e28244d51b492305922dae8b4bc` |
| Candidate protocol | `6bab1e089730485310a9d69a3d49361618c73982663b75f2a9efe8fc9fa9635e` |

The candidate ran before publication with a recorded dirty working tree;
the execution fingerprint binds the subsequently published code. The report
does not relabel that execution as a clean post-merge run.

Historical convergence results remain explicitly dated 2026-09-23. Campaign
pass fractions are not an overall project-completion percentage. Virtual
landing is not touchdown/disarming, and numerical abort is not physical
fallback flight.

The [closeout](progress/2026-09-28-technical-report.md) records acceptance.
The report's proposed [early-flight diagnosis](early-flight-diagnosis.md) is
now complete in a subsequent, separately recorded milestone. The current
[startup audit](attitude-startup-audit.md) is also complete and finds no
demonstrated filter defect. The separately scoped
[pre-arm alignment contract](prearm-alignment.md) is now defined and evaluated;
its first-order joint covariance misses the frozen calibration limit. The
[nonlinear uncertainty derivation](prearm-nonlinear-uncertainty.md) subsequently
passes that gate. The [next step](next-steps.md) is the standalone component.
These later diagnostic results are not part of report revision 2.0.
