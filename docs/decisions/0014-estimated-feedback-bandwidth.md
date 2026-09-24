# 0014: Damped Estimated-Feedback Cascade

- Date: 2026-09-24
- Audited commit: `94c074ef20d6e862f7143c39b00418799b5c4c6e`
- Status: First frozen design unqualified: 29/30 validation passes; revision required

## Scope

Close ADR 0013's noisy-hover performance qualification with a model-based
joint horizontal/roll/pitch bandwidth profile, moment-matched prior covariance,
and independent validation. Preserve the existing
baseline as a reproducible comparison. No changes to filter equations, noise,
prior mean, sensor timing, motor/plant equations, vertical/yaw gains, limits, references,
acceptance windows or historical seeds. No new planner, geometric controller,
automatic alignment, fault response or middleware. This is bounded numerical
feedback validation, not hardware readiness or global stability.

## Audit and hypothesis

The authenticated failed hover reaches 0.10756338079020142 m at 5.395 s.
The dominant component is north error -0.100275 m; the position-estimation
error there is about 0.0219 m. Early tilt estimation error produces lateral
acceleration error (first-second north/east RMS about 0.240/0.218 m/s²).
The vehicle drifts before the attitude estimate settles and then overshoots
during recovery. This suggests coupled-loop damping, not a reason to retune
sensor covariance. No demonstrated estimator/frame/sign defect was identified.
Fresh baseline tests and full fixed-case reproduction are required before
changing execution source; exact results belong in the progress record.

## Local mathematical design

Linearize about matched, level hover with zero yaw and inactive limits. For
one horizontal axis define tilt eta so p_ddot = g*eta: eta=-pitch for north,
eta=roll for east. Let r=eta_dot, a=eta_ddot and tau be motor lag. Nominal
inertia compensation gives:

    p_dot=v, v_dot=g*eta, eta_dot=r, r_dot=a
    tau*a_dot+a = kr*(ka*(eta_d-eta)-r)
    eta_d = (-kp*p-kv*v)/g.

The homogeneous characteristic polynomial is

    tau*s^5+s^4+kr*s^3+kr*ka*s^2+kr*ka*kv*s+kr*ka*kp.

kp is s^-2; kv, ka and kr are s^-1. For the retained kp=1, kv=1.8,
tau=.025, baseline ka=3, kr=12 produces a slow oscillatory pair
-1.26270 +/- 1.80018j (damping ratio .57425), despite ideal outer-loop
damping .9. Thus the ideal double-integrator calculation alone omits
important inner-loop phase lag.

Use coefficient matching with the complete motor/inner/outer model:

    D(s) = .025*(s+3)^2*(s+6)*(s²+28s+392)
    kp=3528/1003, kv=3192/1003, ka=6018/773, kr=773/40.

The s^4 coefficient is .025*(3+3+6+28)=1, so all four gains can be
matched without pretending to change the physical motor lag. The continuous
poles are -3 (double), -6, -14 +/- 14j. Equivalent sampled-data poles,
log(z)/.02, are approximately -2.4599, -4.3654 +/- 1.1289j,
-12.2840 +/- 16.3269j. All have negative real parts; the minimum sampled
damping ratio is .6012. Leave yaw at ka=2, kr=8 and vertical kp=2.25,
kv=3, and leave all limits unchanged. This is a local linear result only.

With errors defined as estimate minus truth, the horizontal forcing law is

    D(d/dt)*p = -kr*ka*(kp*e_p+kv*e_v+g*e_eta) - g*kr*e_r.

For constant tilt error the resulting local position offset is -g*e_eta/kp;
for constant velocity-estimation error it is -kv*e_v/kp. Thus faster inner
feedback alone cannot improve these static sensitivities. The joint design
reduces their magnitudes from 9.81 to 2.789 and from 1.8 to .9048,
respectively, without any integral state. Position-estimation error still
has unit static gain. Higher bandwidth can increase noise-driven control
effort, so report the measured effort and limits, not just tracking error.

The sampled-data verification must retain 50 Hz outer/100 Hz inner clocks
and held commands, rather than presenting continuous poles as a discrete
stability proof. Build the exact local zero-order-hold transition, compare
its Jacobian with the nonlinear simulator over an outer period, and test
spectral radius below one. Also reconstruct the local forced response from
recorded estimation errors, keeping limitations of linearization explicit.

## Initial uncertainty: a distribution-derived design input

The inherited explicit prior mean remains zero p/v/bias and identity q_WB.
Its original standard deviations (.03 m, .03 m/s, 3 degrees, .03 m/s²,
.005 rad/s per axis) were conservative design choices, not the actual
second moments of the experiment's generated initial errors. In weakly
excited hover, yaw/bias corrections based on noisy position data are
sensitive to those assumptions. Do not infer initial state from a sampled
truth realization. Instead specify the prior covariance once from the
declared independent zero-mean uniform distributions:

    Var(U[-a,a]) = integral(-a..a, x²/(2a) dx) = a²/3
    P0 = diag([.02 (x6), 2*pi/180 (x3), .02 (x3), .003 (x3)]² / 3).

At the identity prior, the supplied bounded rotation vector is exactly
the right-local attitude error, so its three-component covariance has
the stated meaning. Sample/process noise and bias-walk assumptions remain
unchanged and already match their generators. The noiseless limit still
has zero prior covariance. This moment match is not a Gaussian-distribution
claim or new evidence of full closed-loop NIS/NEES calibration.

## Development evidence and design refinement

The initial inner-only candidate ka=6,kr=24 kept the old outer gains; it
missed the .08 m target on development hover 8200 (.087947 m). Joint
pole placement with slow poles -2 reduced that peak to .064014 m, but
development hover 8202 exceeded the 15-degree attitude-estimation target
(16.710 degrees). Raising the slow poles to -3 without correcting the
prior produced 15.770 degrees on seed 8201. All such outcomes remain
development results, not concealed or relabeled held-out successes.

Moment matching with slow poles -2 reduced the three development attitude
peaks to 2.343, 9.245 and 13.006 degrees. However, the unchanged seed-30
startup regression still reached .088084 m. The selected joint -3 design
therefore combines the declared moment match with greater local disturbance
rejection and faster recovery. No acceptance threshold, truth distribution,
seed, reference or scoring window was changed during these iterations.

## Frozen acceptance and sequence

1. Fresh preceding full gate and 5-case reproduction, including unchanged
   failed hover. Preserve the original protocol and archives.
2. Analytic/sign/dimension tests, exact motor transition, nonlinear
   finite-difference correspondence, noise-input response, invalid inputs
   and immutable result contracts for the analysis code.
3. Development: full hover seeds 8200..8205 and square seeds 8206..8208.
   No seed reuse from the prior held-out campaign as new validation evidence.
4. Fixed: the exact five ADR 0013 identities, now with the declared damped
   profile, paired with true-state runs using that same profile. All must
   pass the unchanged performance targets. Hover remains 5..65 s, .08 m.
5. Before held-out evaluation freeze execution hashes, all protocol hashes,
   development/fixed results and the quality gate. New held-out cases:
   hover seeds 91000..91019; square seeds 92000..92009. All must complete
   and pass the inherited per-case/estimation/actuator criteria. These
   are finite-sample results, not a universal probability guarantee.
6. Retain every result, including errors. Audit online/offline equality,
   sensor/force/control reconstruction, covariance validity, complete
   clocks and artifacts. Include unchanged original-path regression.
7. Render and inspect diagnostics; run the full pinned toolchain with
   warnings as errors. Publish, verify CI and exact tree, merge the exact
   tested head, then verify merged state. If a frozen held-out condition
   fails, record the failure and investigate; do not change that batch
   and relabel it held-out.

## Evidence preservation

The new campaign has its own protocol identity and version. Existing
campaign defaults and evidence remain readable. Core control functions
still accept explicit caller-supplied parameters; this profile is an
illustrative, documented configuration rather than a hidden change to
every caller. Full histories remain outside Git; source, tests, derivation,
commands and verified result summaries are versioned.
