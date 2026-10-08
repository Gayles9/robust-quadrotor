# ADR 0023: bounded integrated robustness evaluation

## Purpose and scope

This paired campaign asks whether observation supervision responds as specified
when measurements disappear, are rejected or arrive too late. Each case runs
with supervision disabled and enabled, sharing the same random inputs. Separate
criteria measure the response to faults and the quality of the flight.

Live observation faults connect the existing injector to the online ESKF.
Controller and estimator equations, the health policy and the supervisor law
remain unchanged. The campaign uses previously inspected seeds, so it is a
bounded engineering evaluation rather than new controller qualification or a
hardware fallback demonstration.

## Live delivery boundary

Reuse `EskfObservationFault`: source sensor/index identities select dropout,
finite additive measurement offsets, or an integer number of delayed plant
epochs. Intercept only nominally delivered observations before the online ESKF.
Dropouts remove the input; survivors receive consecutive per-sensor identities
in acquisition order. Queue delayed arrivals and emit only current-epoch data,
in canonical order. Apply offsets only to owned measured values. The boundary
receives no truth, commands, estimate or future sensor samples.

Retain the full original acquisition ledger separately. After execution,
reconcile the live outputs against the existing pure offline fault injector,
including pending inputs and dropped sources. Planned faults beyond an early
termination remain explicitly unobserved. Labels and original dropped values
never enter the ESKF, health monitor or supervisor. Existing mission result
and history archive schemas remain unchanged; separate authenticated evidence
holds the original source ledger, fault configuration and diagnostic traces.
Empty fault plans must preserve every existing saved payload byte.

## Campaign definition

Use the unchanged cascade parameters and prior from
`experiments.estimated_feedback_validation.make_configuration`. State the exact
serialized configuration in the protocol. This is the original cascade fixture,
not an implicit selection of the separately available version-2 gains.
Use noisy seed 30 for hover and noisy seed 31 elsewhere. Plant/IMU is 400 Hz,
attitude control 100 Hz, position control 50 Hz, position sensing 5 Hz and
altitude sensing 25 Hz, with zero nominal delivery delay and original NIS gates.

Hover: 1 s initialization, 4 s smooth ascent to NED z=-1 m, 6 s hold, 4 s
smooth virtual landing. Tracking: 1 s initialization, 4 s ascent, 1 s hold,
6 s smooth translation to [0.75, 0, -1] m, 1 s hold, then 4 s smooth landing
to [0.75, 0, 0] m. Preserve original completion tolerances (0.08 m and
0.08 m/s), 0.5 s dwell, 8 s timeout, actuator bounds and safety limits.

Both streams are required. Health counts are warning=2, lost=4, rejection=3,
recovery=2 and evidence-window=5. Position's response budget is 0.6 s and
altitude's is 0.2 s. These are explicit simulation budgets, not vehicle safety
limits. They are fixed before observing campaign results.

| Case | Maneuver and change |
| --- | --- |
| nominal_hover | Hover, no injected fault |
| nominal_tracking | Tracking, no injected fault |
| position_dropout | Tracking, remove position acquisitions in [6, 8) s |
| position_rejection | Tracking, add [5, 0, 0] m to those position acquisitions |
| position_delay | Tracking, delay those position acquisitions by 100 plant epochs (0.25 s) |
| altitude_dropout | Tracking, remove altitude acquisitions in [6, 8) s |
| altitude_rejection | Tracking, add 5 m to those positive-up altitude acquisitions |
| altitude_delay | Tracking, delay those altitude acquisitions by 100 epochs |
| position_recovery | Tracking, remove position acquisitions in [6, 6.4) s |
| landing_position_dropout | Tracking, remove position acquisitions in [13.2, 15.2) s during LAND |
| wind_tracking | Tracking, truth wind [0.5, -0.3, 0] m/s and the existing drag coefficients [0.1, 0.1, 0.15]; nominal controller unchanged |
| mass_tracking | Tracking, truth mass 1.1 kg against nominal 1.0 kg; initial motor speeds and all other nominal settings unchanged |

Every case runs with supervision off and on, with the same fault plan and
random streams. Twelve cases mean 24 planned executions. Two independent
process workers may be used with single-threaded BLAS. The case set, gains,
timeouts and failure definitions are fixed for this comparison.
A separate short stationary smoke partition exercises persistence in CI; it
does not substitute for this maneuver campaign.

Canonical serialized protocol SHA-256 values, anchored in tests before the
maneuver campaign:

- Campaign: `7a686a341305a6a58a0afcf5d20cb4929844165165f7aa70bc11679d99843c30`.
- Smoke: `10897d014c60935ade47dcabc5115efb01ed1fb434d07bad66256e80255bbbd9`.

## Acceptance and interpretation

Software/evidence acceptance requires validated live ordering, atomic invalid
calls, reset, zero-fault byte parity, delayed out-of-order arrivals, pending and
dropped accounting, exact offline/live fault reconciliation, and no future
fault influence on delivered prefixes. Save both full feedback histories,
authenticate before decoding, replay the ESKF and reconstruct health,
supervisor, phases and commands. Reject corrupted evidence, altered protocol,
missing trials and metrics that disagree with their histories. Preserve every
numerical failure with its diagnostic; never synthesize a successful result.

Response acceptance requires no observation abort in nominal, recovery, wind
or mass cases; nominal supervised/off payloads must be identical. Persistent
fault cases require established health before the fault, actual fault exposure,
the appropriate observation abort and no command at or beyond that epoch.
The abort must be the first sampled epoch at/after the recorded unhealthy
origin plus its budget. From fault onset the conservative ceilings are
`warning_age + budget + 2*h`: 1.0075 s for position, 0.2875 s for altitude.
An earlier truth/estimate guard, numerical error or termination before exposure
is retained and does not count as a successful observation response. The brief
dropout must exhibit degradation and confirmed recovery without observation
abort. Preserve exact common-horizon state and command parity in every pair.

Flight-performance acceptance applies to the two nominal cases, recovery,
wind and mass: COMPLETE, full-run position RMSE <=0.15 m, final true position
error <=0.15 m and final true speed <=0.15 m/s. The nominal hover additionally
requires peak error <=0.08 m over its entire 5..11 s hold. This shorter declared
hover does not replace or close the original 60 s requirement. Persistent-fault
cases target numerical abort, and their completion status remains explicit.

Report response and flight outcomes separately. Record detection/recovery and
response times, accepted-data ages, command cutoff, event/fault counts, all
terminal reasons and both tracking/estimation errors over a common horizon.
Lower full-run error caused only by earlier termination is not a tracking
improvement. Campaign acceptance requires all declared response and applicable
flight criteria, with no numerical failures. Completing a valid evaluation can
therefore succeed while the evaluated robustness policy fails its campaign.

The [robustness results](../results/integrated-robustness.md) report the
response and flight outcomes. Controller tuning, additional sensing, middleware
and hardware behavior require separate evidence.
