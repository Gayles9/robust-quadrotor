# Integrated observation robustness

This campaign evaluates how the existing cascade, online ESKF and observation
supervisor behave together during virtual flight. It covers nominal hover and
translation, separate position/altitude loss, rejection and delay, recovery,
landing loss, wind and mass mismatch. The
[frozen protocol](../decisions/0023-integrated-robustness-evaluation.md) defines
all 12 cases and their acceptance criteria before execution. The
[verification record](../archive/records/integrated-robustness.md) retains the
measured results and limitations.

## Information boundary

`EskfLiveObservationFaults` receives only observations that the existing sensor
scheduler has nominally delivered. It can remove an observation, add a declared
measurement offset or delay it by an integer number of plant epochs. The ESKF
receives only the resulting measurements. Neither the ESKF nor the health
monitor receives fault labels, dropped values or simulation truth.

Surviving observations are numbered in acquisition order per stream. Delivery
can occur out of order after a delay. The adapter emits only current-epoch
arrivals in canonical order; it does not rewind the estimator. Existing ESKF
stale/rejection handling therefore remains part of the measured response.
Invalid adapter calls leave its state unchanged. Reset is required before reuse.

After flight, the original acquisition ledger is reconciled exactly against
the pure offline fault injector. That ledger includes dropped and pending
measurements. A planned fault after an early termination is unobserved, not an
invented sample. The mission archive contains surviving inputs; a separate
diagnostic sidecar contains the exhaustive original ledger and health/response
history. The existing mission-result and history formats are unchanged.

## Pairing and interpretation

Every case executes twice with identical faults, random streams and original
cascade parameters: supervision off and supervision on. The health monitor is
active in both. The original parameter factory is used explicitly; the separate
version-2 cascade and geometric controller are not silently selected.

Response acceptance and flight acceptance are separate. A correctly timed
numerical abort can satisfy a persistent-loss response case while ending the
mission. It does not demonstrate a physical fallback maneuver. Nominal,
recovery, wind and mass cases must also complete and meet the declared flight
limits. An earlier safety abort or an unreached fault cannot count as a
successful observation response. Reduced error caused by a shorter aborted
run is not evidence of better tracking, so paired errors also use the common
time horizon.

No gain, timeout, seed or acceptance limit is selected from these outcomes.
These are previously inspected seeds, not fresh qualification. The bounded
hover window does not replace the original 60-second hover requirement, and
the virtual landing has no ground-contact model.

## Running and verifying

From the repository root, using the locked environment:

```bash
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.robustness_validation --partition campaign --workers 2 --output results/robustness-campaign
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.robustness_validation --verify results/robustness-campaign
```

Use a new output directory for each run. At most two independent process workers
are supported. `--partition smoke` selects two short stationary fixtures for
software checks; it does not qualify maneuver performance. A valid campaign
that misses any declared criterion is saved and returns exit status 1.
An evidence or implementation error raises an exception instead.

The protocol is saved before execution; `report.json` is published only after
every planned case has been retained and audited. Each mode has its own full
sensor, truth, estimate, covariance, controller and event histories. Numerical
exceptions are retained with the available diagnostic prefix; no partial flight
is fabricated as a complete result.

The reused archive format also has `baseline_` fields. Here those fields are an
exact companion copy of that mode's mission, checked on reload. The actual
comparison is the separately executed and saved `off`/`on` pair; the companion
copy is not an additional true-state flight or an independent comparator.

Verification checks byte hashes before decoding, requires the recorded
execution-source fingerprint and frozen protocol, replays every ESKF state and
covariance, reconstructs health transitions, supervisor decisions, references,
mission guards and controller commands, and recomputes all reported metrics.
It rejects missing cases and altered claims. The checks establish internal
consistency and reproducibility of the declared simulation; they are not a
cryptographic signature from an independent flight recorder. The auditor does
not independently regenerate plant integration or sensor random draws.

## Source map

| Responsibility | Source |
| --- | --- |
| Causal delivery and offline reconciliation | [eskf_live_faults.py](../../src/quadrotor_math/eskf_live_faults.py) |
| Optional mission integration | [estimated_mission.py](../../src/quadrotor_math/estimated_mission.py) |
| Frozen cases, parameters and limits | [robustness_protocol.py](../../experiments/robustness_protocol.py) |
| Evidence reconstruction | [robustness_evidence.py](../../experiments/robustness_evidence.py) |
| Paired execution, scoring and report verification | [robustness_validation.py](../../experiments/robustness_validation.py) |
| Boundary and evidence tests | [live fault tests](../../tests/unit/test_eskf_live_faults.py), [campaign tests](../../tests/unit/test_robustness_validation.py) |
