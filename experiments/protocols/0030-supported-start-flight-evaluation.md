# ADR 0030: bounded physically supported-start flight comparison

Status: scope, physical boundary and acceptance frozen before implementation
or flight execution, 2026-09-28.

## Preceding audit

Audit merged main `f72cb3521d64a8a6d5726fb0c89dcca3ced48166` (PR 28), matching
GitHub with a clean worktree. Fresh warning-strict component, inherited pre-arm
and documentation tests pass 133 tests in 1.09 s. Seven evidence payloads
authenticate and independently reconstruct all 15,000 moment comparisons and
200 handoffs. No preceding defect is demonstrated. Keep the component and
approved uncertainty model intact.

## One comparison, three clearly separated conditions

Use four existing ADR 0023 campaign cases: nominal_hover (known seed 30),
nominal_tracking, wind_tracking and mass_tracking (known seed 31). Authenticate
the original report and all its payloads and independently reproduce its scores.
Those archived supervised flights remain the original freely flying baseline.
Do not regenerate them with changed conditions or relabel their failures.

Run exactly two new supervised flights per case, eight total: supported release
with the unaligned prior, and the same supported release with the approved
pre-arm estimate. Both share the entire physical startup, true states, acquired
support IMU window, terminal true biases and flight random draws. Their only
difference is the estimator's initial alignment mean/covariance. The unaligned
arm carries its original independent attitude/bias means and adds the same
supported elapsed bias-walk covariance, without using the stationary IMU data.
Initial p/v means/covariance and the heading prior stay unchanged in both arms.

No new controller, correction law, noise scale, duration search, prior tuning,
qualification seed or repeated candidate trial belongs to this scope. This
limited maneuver comparison does not rerun or qualify the whole fault campaign.

## Actual support and release physics

The original random true position and attitude are retained. An ideal external
fixture holds them constant with zero true velocity/angular rate and zero actual
rotor speed. These differ explicitly from the original nonzero random velocities,
rates and initially spinning motors. No state is teleported at release.

For the entire 0..0.5025 s supported interval, the fixture supplies the external
force needed to balance gravity and the existing wind drag:

\[
 F_{support,W}=-mg_W-R_{WB}F_{drag,B},\qquad
 a_W=0,\quad\omega_B=0,\quad f_B=-R_{WB}^Tg_W.
\]

Rotors are off; no rotor torque or drag moment is modeled. The saved equilibrium
residual, constant pose, zero motion and motor histories substantiate the
simulated support assertion. The fixture is an explicit changed operating
condition, not a stationary interval inferred during flight or a hardware claim.

At pre-arm t=0.5025 s, acquire the fresh sample while still supported and hand
it to the ESKF at flight t=0. Then remove the fixture and let the unchanged
motor/plant dynamics respond to the original controller from zero rotor speed.
The state and actual motor speeds are continuous. External force has a finite
step immediately after the boundary; there is no impulse or ground-contact
model. The first IMU endpoint is the supported left-limit sample. The existing
trapezoidal ESKF therefore sees a discontinuous first interval; retain that
startup integration error and its entire flight effect without special tuning
or discarded scores. This is an ideal fixture release, not a ground takeoff
or physical landing/disarming qualification.

## Sample ownership, randomness and handoff

Pre-arm samples 0..200 are used only for alignment; sample 201 is the fresh
release/flight-zero endpoint. Flight index k maps to global sample 201+k.
No sample is replayed twice. Use the existing component's complete endpoint
state, preserving all right-local attitude/bias cross blocks. The existing
online initializer can reconstruct this endpoint from its physical prior and
fixed independent sample noise; require equality of all 21x21 entries, means
and the actual fresh sample before proceeding.

Generate support white noise and 201 bias-walk increments using independent
PCG64 SeedSequence([0x50524541,4,seed,stream]), streams 1/2 for accelerometer/gyro
white noise and 3/4 for bias walks. Start with each original case's true bias.
Draw the fresh sample's white noise from the original flight stream at index 0;
support acquisition must not advance any original flight stream. All subsequent
flight measurement noise, bias increments and slow-sensor draws retain the
original named-stream seed/version and schedule. Verify every realized draw
and paired common prefix after removing each run's physical signal and bias.

Truth is used only by the fixture/sensor generator and evaluation. The alignment
receives its ordinary measurement records, declared prior and support evidence;
the controller receives only the normal ESKF state and rate. Experiment-only
adapters must restore on every exit and run serially within each process.
Independent cases may use at most two worker processes; no concurrent threads
may share an adapter. Production source and defaults remain unchanged.

## Frozen acceptance and stopping rule

1. Independent tests verify equilibrium (including wind/mass), supported sample
   signs, identity/time ownership, fresh sample/noise continuity, prior isolation,
   complete handoff covariance, zero-speed motor continuity, adapter restoration,
   no truth injection into feedback and rejection before flight.
2. Reconstruct saved physical trajectories with the original plant, every ESKF
   estimate/covariance/event, both controllers, mission guards and supervision.
   Authenticate all saved bytes and independently rescore every full history.
3. Keep the original reference, full flight duration, inclusive 5..11 s hover
   window, completion rules and 0.15 m RMSE/final-position and 0.15 m/s final-speed
   limits. The hover limit remains 0.08 m. Extra pre-arm time is reported
   separately; no portion of the original flight scoring is removed.
4. Alignment is go for further supported-start validation only if all eight
   executions have valid evidence, the aligned hover passes every original flight
   limit and its peak is no worse than either original or supported-unaligned
   hover (1e-12 m comparison tolerance). Across all four cases, preserve every
   flight condition passed by either comparator. Record every remaining failure,
   including the separate mass-case limitation. Report all-four-case flight
   qualification separately; a successful startup comparison cannot close a
   failing mass or other flight requirement.
5. Preserve all outcomes outside Git, pass targeted/full software gates and
   hosted CI, and publish an explicit result and next bounded step. Stop after
   these eight flights; a valid performance failure is a scientific result,
   not permission to change the candidate or acceptance. Implementation/evidence
   defects must be diagnosed at source and any invalid run preserved.
