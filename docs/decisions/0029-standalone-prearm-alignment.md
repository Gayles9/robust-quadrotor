# ADR 0029: standalone supported pre-arm alignment component

Status: interface, scope and acceptance frozen before implementation, 2026-09-28.

## Preceding audit

Audit main `81e28ddbd3c61d01a4383803277b70f4611a9724` (PR 27). Main matches
GitHub and the worktree is clean. Fresh warning-strict pre-arm/documentation
tests pass 53 tests in 0.50 s. Six archived payloads authenticate and independently
reconstruct all 15,000 prior outcomes. Nonlinear maximum normalized variance is
1.061394 on the old failure and 1.082222 on the fresh Gaussian set, below 1.10.
No preceding defect was demonstrated. Preserve the accepted mathematics.

## Bounded interface

Implement in `src/quadrotor_math`, with no imports from experiments. Keep the
offline ADR 0028 implementation as an independent reference. Do not change
controller, mission or ESKF algorithms, dependency pins, scores or flight seeds.

Provide a fixed-profile `PrearmAlignmentSession`, immutable owned input/output
records and explicit COLLECTING, READY, REJECTED and RELEASED states. Bind an
acquisition ID, IMU stream ID, common clock ID, support ID, support source and
first sample ID at construction. IDs are nonempty strings; sample IDs are
nonnegative integers, never booleans. The component has no reset or retry method.

Use a normalized acquisition clock: start is t=0, paired samples 0..200 occur
at k*0.0025 s, and the fresh release sample is at 0.5025 s. Sample IDs are
consecutive from the bound first ID. Require FRD/SI profile identification.
Keep the original 1e-12 s clock tolerance; do not pretend it qualifies physical
hardware clocks. Absolute-clock conversion belongs to a later adapter.

Each consume/check/release call receives contemporaneous external support
evidence: bound IDs/source, coverage starting at t=0 through the supplied current
time, and explicit mechanical support, motors-off, zero-world-acceleration,
zero-angular-rate and revocation fields. Coverage through the call time is a
claim about the observed interval, not a promise of future stationarity. Missing,
expired, future-dated, mismatched, revoked or false attestations reject. The
component checks the assertion's structure/continuity, not physical authenticity.
Quiet IMU data cannot create the assertion. The owner must issue unique support
and acquisition IDs and allocate disjoint windows across separate instances;
this standalone object is not a global identity registry or hardware interlock.

Expose sample consumption, a support/time check for periods without samples,
and one-time release. Partial windows cannot become ready. Consuming extra data
after READY rejects. The time check detects a missed next-sample deadline or
expired release. A returned READY snapshot is only valid at its recorded epoch;
release revalidates current time and support. No background clock is assumed.

Structural value-object construction errors raise before a session is called.
Invalid data/evidence received by an active session latch a reason and cannot
produce an endpoint. REJECTED/RELEASED instances cannot accept further actions;
repeated release raises without producing another output. Numeric window gates
retain every failed reason and diagnostics. Output snapshots must not alias
internal state or caller-owned arrays.

## Fixed priors, uncertainty and handoff

Accept only the ADR 0028 400 Hz noise/bias-walk/gravity profile, identity attitude
prior (quaternion sign equivalent), zero bias means, original 3-degree attitude
sigma, 0.03 m/s² accel-bias sigma and 0.005 rad/s gyro-bias sigma, with the original
independent alignment covariance. Position/velocity mean and a valid 6x6 PSD
covariance may be supplied separately within the initial 15-state prior.
Require explicit navigation/alignment independence and zero cross blocks;
reject other priors/profile parameters rather than silently adapting them.

Port the single order-five positive quadrature model, eight-correction/1e-13
rotation mean, full terminal-bias cross-covariance, three alpha=0.001 compatibility
gates, <=0.75-degree local axis radius, <=15-degree inclination domain and all
existing shape/finiteness/conditioning checks. No fitted factors or retuning.

On release, keep the alignment mean, add exactly one interval of bias-walk
covariance, and preserve all cross blocks. Assemble the initial 15-state prior
with the declared independent p/v block. Return an `EskfEndpointState` plus the
owned fresh IMU sample and acquisition provenance. Its sample-noise mean and
cross block start at zero because the fresh sample is disjoint, not because
state uncertainty vanished. The component issues no arm command, runs no
mission and does not propagate an airborne state.

## Frozen acceptance

1. Independent tests of immutable ownership, input restrictions, every state
   transition, support absence/revocation/source continuity/motor status,
   sample IDs/clock gaps/reordering/duplicates, window and deadline boundaries,
   numeric rejection, all retained covariance blocks and one-time release.
2. Match all 15,000 archived nonlinear moment results: rotation/error agreement
   <=2e-14 rad and maximum covariance difference whitened by reference covariance
   <=1e-9. Authenticate the exact ADR 0028 report and trial payload before using
   them; reproduce all previous calibration acceptance without selecting trials.
3. Execute complete sessions for the first 100 fresh nominal and first 100 fresh
   Gaussian trials from ADR 0028. Recreate the same paired acquisition data;
   use independent release-walk/noise streams
   SeedSequence([0x50524541,3,partition,trial]). Verify the full release matrix,
   fresh sample ownership and unchanged initial p/v. Preserve all outcomes.
4. Preserve the required undetectable-acceleration case; it must not be presented
   as proof of stationarity. Exercise the existing motion/invalid-data fixtures.
5. Pass relevant and full repository software gates. Keep generated evidence
   outside Git and publish an explicit component go/no-go with limitations.

Stop on a mathematical/reference discrepancy or failed acceptance. Fix an
implementation defect at its source; do not alter the accepted model or budgets.
If successful, next scope a separately frozen, physically supported-start flight
evaluation. That later work must retain common flight noise, original controller,
complete flight scoring and unchanged performance/no-regression thresholds.
