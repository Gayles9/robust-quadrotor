# 0011: Bounded Cascaded Attitude and Body-Rate Baseline

See the [control guide](../design/control.md) and
[verification record (ZIP)](../../evidence/development-records.zip).

## Scope

The cascade separates two jobs: an attitude loop chooses a desired rotation
rate, and a body-rate loop requests the moment needed to reach it. Explicit
allocation limits convert that request into feasible rotor commands.

This decision describes the inner loops and their true-state simulation checks.
[Position control](0012-position-control-and-missions.md) supplies attitude and
thrust references, while [estimated feedback](0013-estimated-state-mission-feedback.md)
replaces true-state measurements with estimator outputs. A level, force-balanced
initial condition alone is not an altitude controller: the isolated inner-loop
tests can drift in position and height when tilted.

## Law and local domain

$`R_{WB}`$ maps FRD body to NED world; all moments/rates/errors below are in current
body coordinates. For a piecewise-constant reference $`q_{r,WB}`$:

```math
\begin{aligned}
\mathbf e_B&=\mathrm{Log}(R_{WB}^{T}R_{r,WB}),\\
\boldsymbol\omega_{d,B}&=\mathrm{clip}(K_a\odot\mathbf e_B,-\boldsymbol\omega_{\max},\boldsymbol\omega_{\max}),\\
\boldsymbol\alpha_{d,B}&=K_r\odot(\boldsymbol\omega_{d,B}-\boldsymbol\omega_B),\\
\boldsymbol\tau_{\mathrm{req},B}&=J_n\boldsymbol\alpha_{d,B}+\boldsymbol\omega_B\times(J_n\boldsymbol\omega_B),\\
\boldsymbol\tau_{\mathrm{lim},B}&=\mathrm{clip}(\boldsymbol\tau_{\mathrm{req},B},-\boldsymbol\tau_{\max},\boldsymbol\tau_{\max}).
\end{aligned}
```

Here $`R_{r,WB}`$ is reference orientation, $`J_n`$ nominal inertia, and $`K_a,K_r`$ the per-axis gains (`K_att,K_rate`). The symbol $`\odot`$ means elementwise multiplication. In this equation, $`\mathrm{Log}`$ returns the rotation vector directly.

Use the shortest Hamilton relative quaternion and a stable rotation-vector map,
invariant to either quaternion sign. Reject errors above the caller's declared
maximum angle (strictly below pi); no global-stability or moving-reference
feedforward claim. Gains are positive per-axis in 1/s, rate limits rad/s,
moment limits N m, inertia kg m². Input quaternions must already be unit length.

No integral term: the local matched unsaturated zero-lag equation per principal
axis is $`\ddot\theta+K_r\dot\theta+K_rK_a\theta=0`$.
This gives a transparent damped baseline without windup. Constant external torque
has nonzero equilibrium attitude error; disturbance acceptance is recovery after
a finite pulse, not asymptotic rejection of arbitrary persistent torque.

## Allocation

Preserve feasible requested collective thrust $`T\geq0`$. Require a feasible
zero-moment anchor using the existing strict allocator. For allocation matrix
$`A`$, define squared rotor speeds $`\mathbf s_0`$ and direction $`\mathbf d`$:

```math
\mathbf s_0=A^{-1}[T,0,0,0]^T,\qquad
\mathbf d=A^{-1}[0,\boldsymbol\tau_{\mathrm{lim},B}^T]^T.
```

Choose the largest $`\alpha\in[0,1]`$ satisfying the componentwise speed bounds
$`\mathbf s_{\min}\leq\mathbf s_0+\alpha\mathbf d\leq\mathbf s_{\max}`$.
Only reduce moments along that ray; do not independently clip infeasible rotor
solutions. Apply a documented roundoff backoff only when $`\alpha<1`$ and pass the
scaled moment to the unchanged strict allocator. An infeasible collective/anchor
is an error, not silent thrust modification. Record requested/clipped/allocated
moments, scale, commands and limit flags. These are nominal allocation results;
truth motor lag and mismatches can produce different actual moments.

## Execution and ownership

The simulation takes independent truth body/rotor/world parameters and controller
nominal parameters, plus an explicit initial state/actual rotor speeds. Reference
quaternion and collective arrays are supplied at controller epochs; body disturbance
moment arrays at plant interval starts. All commands are zero-order held. Plant
interval is positive, controller stride is a positive integer; no clock inference.
At each controller epoch, observe current q/rate, compute and allocate, then advance
plant intervals. There is no terminal controller update after the last interval.

For each projected RK4 stage, obtain rotor speeds from the existing exact motor
response at the stage's elapsed time, evaluate existing rotor forces, drag and
rigid-body body-wrench derivatives, and add the held supplied disturbance moment.
This avoids freezing a changing motor speed throughout the closed-loop step.
It does not alter the historical run generator's operator splitting. Compare the
constant-speed zero-disturbance limit with the existing RK4 integrator.

The simulation returns aligned truth-state and actual-speed histories, plus
per-control-interval commands and diagnostics with explicit times. Arrays are
copied and exposed read-only. Shape, type, finiteness, inertia, quaternion and
limit validation occurs before execution; failure leaves caller state unchanged.
The pure controller has no RNG or truth/fault-history input.

## Verification

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
The outer-loop mission evidence is described in
[ADR 0012](0012-position-control-and-missions.md).
