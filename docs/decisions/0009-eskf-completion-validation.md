# 0009: Estimator Completion Scope and Validation Protocol

- Date: 2026-09-23
- Baseline: `a95fbfcc2054a9a18151cb4c7d58d3b5817d4642`
- Status: Protocol specified before implementation and held-out execution

## Completion boundary

Complete the measurement-driven, known-prior, fixed-gravity 15-state ESKF described
by ADRs 0004–0008 and the master plan's Weeks 10–12. Prediction, position/altitude
correction, bias estimation, Joseph covariance, right-local injection/reset and NIS
gating already exist. This task closes their remaining validation and experiment
requirements. It does not assert that an arbitrary Kalman filter can estimate an
unobservable state or that a finite test campaign proves correctness everywhere.

Retain explicit stale rejection: the plan allows a non-rewinding filter and names
history rewind as stretch scope. Live transport, asynchronous IMU, unknown-pose
initialization, controllers, mission logic, ROS/PX4 and flight validation are separate
integration work. G2 is not closed by estimator-only evidence.

## Work packages and evidence

1. Independent continuous analytic rigid-body motion, with exact position derivatives,
   Euler-composed body-to-world quaternion and analytically differentiated body rates.
   Verify derivatives numerically and compare noise-free integration under step refinement.
   These are estimator fixtures, not dynamically feasible rotor-command trajectories.
2. Deterministic, immutable position/altitude fault injection: dropout, additive single
   outlier or burst, and additional delivery delay. Preserve acquisitions, reindex surviving
   streams explicitly, and keep original identities and fault labels outside the estimator.
   Verify ordering, pending/stale behavior, ownership, invalid inputs and atomic failure.
3. A frozen Monte Carlo protocol with distinct development and held-out seeds, measured
   errors, NIS/NEES, bias convergence, paired dead reckoning, fault detection, uncertainty
   growth/recovery and Q/R sensitivity. Count every failed or divergent seed.
4. Headless, reproducible plots and compact scalar/position-velocity Kalman examples with
   analytical tests. Matplotlib is a development dependency only; core remains NumPy-only.
5. Compatibility checks, full tests/static checks, documentation, GitHub CI and merge review.

## Protocol v1 (frozen before evaluation)

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

## Review and reproducibility rules

The canonical finite JSON protocol and source digests are recorded before each campaign.
Exact seeds, trajectory parameters, priors, failures, event counts and measured summaries
are retained. A failed variant suppresses its complete-ensemble summary; finite divergent
trials remain included. Plots derive only from stored results and preserve failures and
out-of-band findings. Generated reports/figures/logs stay out of Git. Record commands,
versions, commits and results in the completion progress record after verification.

Production behavior must not be changed just to improve reported metrics. If analytic
checks expose a mathematical error, fix and test its cause, document the protocol revision
and use fresh held-out seeds; do not conceal the original evaluation.
