# 0011: Bounded Cascaded Attitude and Body-Rate Baseline

- Date: 2026-09-24
- Baseline: 8fef0ce6c811b64dc2e2ab4558591111254e5628
- Status: Accepted; implemented and verified in the linked record

See the [control guide](../design/control.md) and
[verification record](../archive/records/baseline-attitude-control.md).

## Scope

Complete the master plan's Week 7 inner-loop milestone. Add a pure quaternion
attitude P loop, body-rate P loop, explicit actuator feasibility handling, and
deterministic true-state closed-loop tests/experiments. Do not implement the
Week 8 position controller/mission, geometric tracking, or estimated-state feedback.
Existing estimators, plant equations, sensor/artifact schemas and CI remain unchanged.
No new dependencies. A level force-balanced initial condition is not an altitude
controller: tilted tests may drift in position/height, and that is reported.

## Law and local domain

R_WB maps FRD body to NED world; all moments/rates/errors below are in current
body coordinates. For a piecewise-constant reference q_reference_WB:

    e_B = Log(R_WB.T R_reference_WB)
    omega_desired_B = clip(K_att * e_B, -omega_max, omega_max)
    alpha_desired_B = K_rate * (omega_desired_B - omega_B)
    moment_requested_B = I_nominal alpha_desired_B + omega_B × (I_nominal omega_B)
    moment_limited_B = clip(moment_requested_B, -moment_max, moment_max)

Use the shortest Hamilton relative quaternion and a stable rotation-vector map,
invariant to either quaternion sign. Reject errors above the caller's declared
maximum angle (strictly below pi); no global-stability or moving-reference
feedforward claim. Gains are positive per-axis in 1/s, rate limits rad/s,
moment limits N m, inertia kg m². Input quaternions must already be unit length.

No integral term: the local matched unsaturated zero-lag equation per principal
axis is theta_ddot + K_rate theta_dot + K_rate K_att theta = 0.
This gives a transparent damped baseline without windup. Constant external torque
has nonzero equilibrium attitude error; disturbance acceptance is recovery after
a finite pulse, not asymptotic rejection of arbitrary persistent torque.

## Allocation

Preserve feasible requested collective thrust T >= 0. Require a feasible zero-
moment anchor using the existing strict allocator. In squared rotor speeds,
s0 = A^-1[T,0,0,0] and d = A^-1[0,moment_limited_B].
Choose the largest alpha in [0,1] for which lower <= s0+alpha*d <= upper.
Only reduce moments along that ray; do not independently clip infeasible rotor
solutions. Apply a documented roundoff backoff only when alpha < 1 and pass the
scaled moment to the unchanged strict allocator. An infeasible collective/anchor
is an error, not silent thrust modification. Record requested/clipped/allocated
moments, scale, commands and limit flags. These are nominal allocation results;
truth motor lag and mismatches can produce different actual moments.

## Execution and ownership

New simulation takes independent truth body/rotor/world parameters and controller
nominal parameters, plus an explicit initial state/actual rotor speeds. Reference
quaternion and collective arrays are supplied at controller epochs; body disturbance
moment arrays at plant interval starts. All commands are zero-order held. Plant
interval is positive, controller stride is a positive integer; no clock inference.
At each controller epoch, observe current q/rate, compute and allocate, then advance
plant intervals. There is no terminal controller update after the last interval.

For each projected RK4 stage, obtain rotor speeds from the existing exact motor
response at the stage's elapsed time, evaluate existing rotor forces, drag and
rigid-body body-wrench derivatives, and add the held supplied disturbance moment.
This avoids freezing a changing motor speed throughout the new closed-loop step.
It does not alter the historical run generator's operator splitting. Compare the
constant-speed zero-disturbance limit with the existing RK4 integrator.

Return aligned truth state/actual-speed histories plus per-control-interval commands
and diagnostics with explicit times. Copy arrays, make them read-only, validate
shape/type/finiteness/inertia/quaternion/limits before execution. Fail atomically;
no RNG or truth/fault-history input in the pure controller.

## Verification fixed before implementation

- Independent Rodrigues/relative-matrix error checks; all sign/axis/quaternion
  sign combinations; zero/near-zero/domain-boundary error; overflow rejection.
- Exact rate acceleration after gyroscopic cancellation, including off-diagonal
  inertia; independent scalar local linearization and analytic continuous response.
- Feasible command identity; independent interval/ray optimum; preserved collective
  and torque direction under saturation; minimum/maximum/invalid rotor boundaries.
- Ownership, malformed arrays, non-real/nonfinite/scalar Boolean rejection, local
  symmetry/SPD checks, inputs unchanged on success and failure.
- Initial/final/single-interval timing, command hold, future-reference causality,
  motor exactness, hover equilibrium, old integrator limit, stage/refinement checks.
- Fixed unit regressions for principal-axis recovery, rate damping, reference
  steps, finite moment disturbance, commanded saturation and subsequent recovery.
- Frozen engineering experiment below; failures retained, unique complete cases,
  finite JSON, provenance and source/protocol hashes, no overwrite, plot inspection.

## Experiment protocol v1

Illustrative model, not calibrated hardware: mass 1 kg; diagonal inertia
[.02,.025,.04] kg m²; X geometry at (±.15,±.15,0) m; spin [+1,-1,+1,-1];
kf=1e-5 N/(rad/s)², km=2e-7 N m/(rad/s)²; motor range 0..900 rad/s;
time constant .025 s; gravity 9.81 m/s²; calm air, zero drag.
Nominal equals truth unless a case explicitly declares mismatch.
Gains K_att=[3,3,2], K_rate=[12,12,8] per second; rate limits [2,2,1.5]
rad/s; moment limits [.8,.8,.3] N m; domain pi/2 radians.
Choose gains from the critically damped zero-lag linearization, not held-out tuning.

Plant 400 Hz; control 100 Hz. Fixed cases: 4 s level hover; ±20-degree single-axis
reference steps at .5 s; 30-degree coupled recovery with initial rates; .2 s body
torque pulse [.12,-.1,.06] N m beginning at 1 s; a large within-domain reference
with deliberately small moment limits then return to level, documenting saturation.
The frozen executable protocol specifies a 30-degree roll reference on [.5,1.5) s,
moment limits [.05,.05,.02] N m, and return to the level reference afterward.
Report a persistent-torque diagnostic separately: +.04 N m about x from .5 s,
checking its predicted offset to within .1 degree and final rate below .02 rad/s.
Unit tests and short smoke cases do not count as held-out validation.

Development seeds 1000..1004 and held-out seeds 60000..60029 independently choose
rotation axes, initial attitude angles 5..30 degrees, initial rates ±.3 rad/s.
All use the frozen 4 s recovery experiment. Each case must end below .5 degree
attitude error and .02 rad/s rate norm, remain below the declared attitude domain,
and have no moment limiting in its final second. Fixed steps/pulse must recover
inside the same final bounds, with settling into a 1-degree/.05 rad/s band
within 2 s after the final reference change or end of pulse. Hover state errors
must remain within numerical roundoff. The saturation stress case must exercise
limiting and recover after return; it is not a no-saturation performance claim.

Record settling time (last departure from the simultaneous error/rate band),
axis-step 10–90% rise time/overshoot where meaningful, RMSE, peak/final errors,
command-limited duration, rotor-bound duration, and integral squared actual body
moment. Never infer global stability from a finite case set. Refinement halves
plant dt at fixed control period; acceptance requires the case's reported final
attitude/rate to change by <.02 degree/.002 rad/s, respectively.

Any design correction motivated by development evidence must be documented before
held-out execution. Do not adjust gains or acceptance targets on held-out results.
G2 remains open until the later position/mission milestone.
