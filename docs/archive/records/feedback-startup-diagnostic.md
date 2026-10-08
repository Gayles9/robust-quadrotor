# Bounded startup feedback diagnosis — 2026-09-24

## Scope and audited baseline

Audited draft commit: `772dcb58b103313bc2f0047a1201f70d842495c9`.
Main remains at `f45f610776ec1c6877e896d7f63ec17c87f1d322`.
This follow-up diagnoses the two already observed version-2 hover failures,
93003 and 93012. It does not change the ESKF, controller, plant, initial prior,
sensors, reference, clocks, limits or qualification criteria. No new validation
seed is opened. No gain search or additional controller design is included.

The acceptance criteria for this diagnostic are: reproduce both original failed
prefixes exactly; substitute only declared feedback channels; preserve causal
sensor/estimator execution; verify the all-truth counterfactual against the
independent original true-state baseline; and report the finding without treating
oracle results as deployable performance.

## Experiment and mathematical interpretation

`experiments.feedback_startup_diagnostic` runs the original mission from zero
through 10 s. It records the endpoint observation and then explicitly ends the
diagnostic prefix before issuing control at 10 s. It neither moves the 5-s hold
start nor manufactures a completed `MissionResult`. The reported quantity is
the maximum three-dimensional position error over 5..10 s, not full 60-s
qualification. Both historical full-hold maxima lie inside this interval.

Eight modes select estimated or true complete state vectors at the existing
observer-to-feedback boundary: position, velocity, attitude and angular rate.
The ESKF still receives newly generated sensor measurements on each modified
physical trajectory. Its state and covariance are not replaced by truth. The
same seed preserves the independent sensor-noise draws, but measurements change
with the changed trajectory. The temporary hook is confined to this experiment,
restored even on failure, and runs in separate processes rather than threads.
Feedback guards and the supervisor receive the selected vectors too; neither
terminal completion nor abort is encountered within these prefixes.

The unsaturated outer-loop request is exactly

```text
a_request = a_reference + Kp (p_reference - p_hat)
                           + Kv (v_reference - v_hat).
delta_a_request = -Kp e_p - Kv e_v,
e_p = p_hat - p_true,       e_v = v_hat - v_true.
```

All vectors are three-dimensional NED world quantities. Position is in metres,
velocity in m/s, and acceleration in m/s². `Kp` and `Kv` denote diagonal gain
matrices; their horizontal entries are 6.4 s^-2 and 4 s^-1. The second equation
compares estimated and true feedback at the **same** state/reference epoch.
It is not an additive decomposition of separate nonlinear flight trajectories.
For example, at 5 s in seed 93012, horizontal position/velocity errors produce
an acceleration-request difference of approximately `[0.4863, -0.4414]` m/s².
The controller faithfully reacts to erroneous position and velocity information.

## Measured result

Each table entry is the descriptive 5..10-s peak position error in centimetres.
Truth-assisted rows are diagnostic counterfactuals, not candidate controllers.

| Feedback substitution | Seed 93003 | Seed 93012 |
| --- | ---: | ---: |
| None: actual estimated feedback | 9.14027 | 8.86862 |
| True position only | 4.67112 | 4.45054 |
| True velocity only | 5.33846 | 5.11529 |
| True position and velocity | 0.75132 | 0.65256 |
| True attitude only | 8.52872 | 8.38535 |
| True angular rate only | 9.12236 | 8.84082 |
| True attitude and angular rate | 8.51098 | 8.35813 |
| All four true state vectors | 0.16448 | 0.15992 |

The unchanged trajectories, references and estimated position, velocity, attitude
and rate match the authenticated original archives exactly through 10 s. The
all-truth trajectories match the independent saved true-state baseline exactly
for all four state vectors. These checks establish that the diagnostic actually
reproduces the two execution paths it is intended to compare.

For these two failures, position/velocity feedback error is the dominant
intervention target. Removing both errors leaves less than 1 cm peak error;
removing both inner-loop feedback errors still leaves more than 8 cm. This
supports prioritizing translational estimation/control coupling over another
attitude/rate gain sweep. It does not prove a defect in the ESKF equations,
estimator inconsistency across the population, or a globally optimal controller.
Separate nonlinear interventions are not additive percentages of causation.

Both original traces reject a position observation at 5 s: NIS 11.54091 and
13.09126 versus the configured 11.345 threshold. The gate therefore follows its
declared rule. This correlation has not been established as the cause of either
peak and does not justify disabling the gate or increasing its threshold.

## Verification and reproduction

The new tests check each channel selection without mutation, rejection of
unobserved seeds and invalid horizons, deterministic no-op execution, restoration
after injected errors, separation of truth-assisted feedback from the ESKF state,
and detection of a one-ULP changed historical state. Existing estimated-mission
and local-cascade tests are rerun alongside these tests. Runtime files under
`src/quadrotor_math` are unchanged.

Verified on 2026-09-24: all 112 focused tests pass (49.56 s), including 19 new
diagnostic tests; Ruff lint and format checks pass; mypy passes all 50 source
files. The final instrumented execution source digest is
`27596beb14cf70b806ddcf0b28d7c5f61f24aab90590478c94769a7738fd4f56`.
The 16 traces were rerun after a typing-only hook-access refinement; every NPZ
digest and reported peak exactly matches the initial run. This repeat is a
reproducibility check of the same observed cases, not additional validation data.

```bash
uv run ruff check .
uv run ruff format --check .
uv run mypy src experiments
uv run pytest -q -W error tests/unit/test_feedback_startup_diagnostic.py \
  tests/unit/test_estimated_mission.py tests/unit/test_cascade_analysis.py
OPENBLAS_NUM_THREADS=1 uv run python -m experiments.feedback_startup_diagnostic \
  --original /path/to/design-validation-v2 --output /path/to/NEW_DIR --workers 4
```

Use the recorded numerical backend for exact historical-prefix comparison. The
experiment authenticates the two input NPZ byte digests before decoding. Generated
traces, their SHA-256 digests, source identity and exact-prefix verification stay
outside Git, with the previous campaign evidence. The README links this record;
no failed campaign is reclassified as passing.

## Decision and next bounded requirement

No production fix is supported by this diagnosis alone, so production behavior
is unchanged and PR #10 remains draft. The existing 28/30 qualification result
and progress estimates are unchanged.

The next design question is how to reduce the acceleration command driven by
correlated position/velocity estimation error while preserving tracking response.
First examine the translational error/covariance and innovation histories around
the missed peaks and quantify the error-to-command transfer. Any proposed change
must be causal, use available measurements only, preserve the existing sensor and
physical contracts, and be justified beyond these two observed seeds. Regressions
on observed cases precede source freeze and untouched full-hold validation.
Ground-contact alignment, additional sensors, truth resets, weaker targets and
minimum-snap planning are outside this diagnostic step.
