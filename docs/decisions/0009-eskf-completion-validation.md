# 0009: Estimator Completion Scope and Validation Protocol

- Evidence: [completion verification record (ZIP)](../../evidence/development-records.zip)

## Evaluation scope

The validation campaign tests the known-prior, fixed-gravity, 15-state ESKF
described in ADRs 0004–0008. It covers prediction, position/altitude correction,
bias estimation, Joseph covariance updates, right-local injection/reset and NIS
gating. The question is whether those components work together under stated
motion, noise and fault distributions. Finite tests cannot establish correctness
everywhere or recover a state that the observations do not make observable.

The filter rejects stale observations instead of rewinding its history. Live
transport, asynchronous IMU and unknown-pose initialization are outside this
campaign; estimator-only evidence does not qualify closed-loop flight.

## Evidence

1. Independent continuous analytic rigid-body motion supplies exact position
   derivatives, Euler-composed body-to-world quaternions and differentiated body
   rates. Numerical differentiation and step refinement check these fixtures.
   They are estimator references, not rotor-feasible flight trajectories.
2. Deterministic position/altitude faults cover dropout, isolated and burst
   outliers, and delayed delivery. Original identities and fault labels remain
   outside the estimator; tests check ordering, stale/pending behavior and ownership.
3. A Monte Carlo protocol uses distinct development and validation seeds to
   measure errors, NIS/NEES, bias convergence, fault detection, uncertainty
   growth/recovery and Q/R sensitivity, retaining every failure.
4. Headless plots and scalar/position-velocity Kalman examples make the measured
   behavior inspectable. Matplotlib is a development dependency; the mathematical
   package depends only on NumPy.

## First-order evaluation protocol (version 1)

- Independent PCG64 streams use `SeedSequence([0x45534B43,1,stream_id,seed],pool_size=4)`.
  Stream IDs: motion 0, prior 1, initial biases 2, accelerometer white 3, gyro white 4,
  accelerometer bias walk 5, gyro bias walk 6, position white 7, altitude white 8, faults 9.
- Continuous motion families: stationary; constant translation/yaw; and excited sinusoidal
  position and roll/pitch/yaw, with seed-randomized amplitudes, frequencies and phases.
  Excited position amplitudes uniform `[1,2]` m, angular frequencies `[.5,1]` rad/s;
  roll/pitch amplitudes `[.15,.3]` rad, yaw amplitude `[.3,.6]` rad, Euler angular
  frequencies `[.4,.8]` rad/s; phases uniform `[-pi,pi]`. No Euler-angle singularity
  is approached. All dimensions are independent draws in the documented stream order.
- Duration 30 s; IMU 100 Hz; position 10 Hz; altitude 5 Hz. Gravity 9.81 m/s²;
  positive-up altitude datum 100 m. Initial epoch is exactly zero, before any bias walk.
- White sample standard deviations: acceleration .08 m/s², gyro .004 rad/s, position
  .10 m, altitude .08 m. Bias-walk densities .0005 m/s²/√s and .00005 rad/s/√s.
- Prior standard deviations per axis: position .2 m, velocity .1 m/s, right-local
  attitude .015 rad, accelerometer bias .08 m/s², gyro bias .008 rad/s. Initial true
  biases are independent zero-mean draws with those bias standard deviations; nominal
  biases start at zero. Pose/velocity priors are independent errors about the declared
  analytic initial condition. No truth history is supplied to replay.
- Match nominal `Q_c` using ADR 0008's sampled-noise conversion. First-order propagation
  remains the baseline until an independently demonstrated defect justifies a change.
- Smoke: seeds 10,11, duration .4 s. Development: 4000–4004, all three motion families.
  Held-out validation: 40000–40099, excited family. No held-out seed may tune parameters.
- Every nominal trial pairs diagnostics-only fusion with identical-prior/IMU dead
  reckoning. Validation's first 20 seeds additionally run mixed faults both gated and
  ungated; its first 10 additionally run covariance multipliers `Q=.25,4` and `R=.25,4`
  one at a time without gates. Pairing is deliberate; variants are not independent seeds.
- Faults, both observation types: single additive outlier at 5 s, bursts `[12,14)` and
  `[22,24)` s, uniformly random direction/sign with magnitude uniform `[2,3]` m.
  Position dropout `[8,10)` s; both sensors delayed .1 s on `[15,17)` s and at the
  final acquisition. Delay beyond the horizon is pending, not dropped or fused.
- Fault gate: existing 99% preset (position 11.345, altitude 6.635), fixed before
  evaluation. Confusion matrices include all scored fresh observations; dropped,
  stale, pending and disabled observations have separate counts, never true negatives.
  Retain rejected NIS. Report per-sensor precision/recall/FPR and first-burst detection
  delay. Empty denominators are unavailable, not perfect scores.
- Nominal targets: zero numerical failure/divergence across 100 randomized trajectories;
  mean position RMSE at least 50% below dead reckoning; mean final accelerometer and
  gyro bias error norms at least 50% below their initial norms under excited motion.
  This operationalizes “material” reduction before seeing results, not universal observability.
- Covariances must remain finite, symmetric and PSD under existing validated boundaries;
  full-state NEES requires SPD. Record maximum quaternion norm defect and minimum
  normalized covariance eigenvalue. Divergence retains ADR 0008's >10 m or >pi/2 rad.
- Individual central95 NIS/NEES coverage .90–.98 is an investigation band, not a tuning
  target or an automatic pass. Report pointwise seed-mean bounds and temporal dependence.
- Fault targets: precision >.90 and recall >.85 for each sensor's declared distribution;
  visible horizontal covariance growth through dropout and reduction after return.
  Recovery compares pre-dropout, final dropout and one-second-post-return epochs.
  Missed targets remain visible, with causes and limitations; no post-holdout retuning.
- Q/R sensitivity is diagnostic; it intentionally includes wrong assumptions and has no
  fabricated consistency-pass threshold. Nominal and fault failures still fail the CLI.

## Reproducibility

The canonical finite JSON protocol and source digests are recorded before each campaign.
Exact seeds, trajectory parameters, priors, failures, event counts and measured summaries
are retained. A failed variant suppresses its complete-ensemble summary; finite divergent
trials remain included. Plots derive only from stored results and preserve failures and
out-of-band findings. Generated reports, figures and logs stay outside Git;
the evidence records command, environment and source identities.

Validation data is not tuning data. A mathematical correction requires its own
verified implementation and fresh validation seeds; the original evaluation
retains its outcome.

## Recorded consistency qualification

Protocol v1 completed without numerical failures or nominal/gated divergence. Position,
bias-reduction, outlier precision/recall and dropout recovery targets were met. Full-state
NEES central95 coverage was 88.8367%, below the predeclared investigation band; this
target is **not** marked passed. NIS coverage was 94.9568% (position) and 94.7682%
(altitude). The protocol, Q/R values and gates were retained unchanged.

Development-only exact/noiseless and step-refinement probes identify a deterministic
integration contribution. They are diagnostic evidence, not replacement held-out results
or a proof that every contribution has been isolated. The first-order campaign
does not support an unqualified statistical-consistency claim. The separate
[endpoint campaign](0010-eskf-endpoint-propagation.md) evaluates a matched
sampled-IMU propagation and covariance model.
