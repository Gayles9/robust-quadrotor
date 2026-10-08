# 0014: Damped Estimated-Feedback Cascade

The outer position loop, inner attitude loop and motor response all affect
how quickly the vehicle recovers from estimation error. Tuning any one loop
in isolation can miss the combined system's lag and overshoot. This decision
explains the model used to choose joint gains and a prior covariance matched
to the simulated initial uncertainty.

The implemented profiles are explicitly selected by `--design-version`.
Version 2 is the retained comparison reference; it passed 28 of 30 required
validation cases, while version 1 passed 29 of 30. Neither passed the full
qualification requirement. Their local stability calculations and improved
performance do not erase those misses. The [feedback guide](../design/feedback-design.md)
connects these profiles to the current comparisons.

## Scope and motivation

The experiment changes horizontal position and roll/pitch bandwidth together
with the prior covariance. Filter equations, noise, prior mean, sensor timing,
motor/plant equations, vertical/yaw gains, limits, references and scoring
windows remain fixed. This isolates a feedback-design question without
retuning sensor uncertainty to fit the observed flight.

The original failed hover reaches 0.10756338079020142 m at 5.395 s.
The dominant component is north error -0.100275 m; the position-estimation
error there is about 0.0219 m. Early tilt estimation error produces lateral
acceleration error (first-second north/east RMS about 0.240/0.218 m/s²).
The vehicle drifts before the attitude estimate settles and overshoots during
recovery. This supports investigating coupled-loop damping; it does not
demonstrate an estimator, frame or sign defect.

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

The sampled-data model includes the 50 Hz outer and 100 Hz inner clocks with
held commands. Its exact local zero-order-hold transition is compared with the
nonlinear simulator's Jacobian over one outer period, and its spectral radius
is checked to be below one. Recorded estimation errors drive a separate
reconstruction of the local forced response. These checks retain the limits of
linearization; continuous poles alone are not a discrete stability proof.

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

## Evidence for the joint profile

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

## Evaluation protocol for version 1

1. The five original fixed cases, including the failed hover, provide the
   reproducible comparison baseline.
2. Analytic/sign/dimension tests, exact motor transition, nonlinear
   finite-difference correspondence, noise-input response, invalid inputs
   and immutable result contracts for the analysis code.
3. Development: full hover seeds 8200..8205 and square seeds 8206..8208.
   No seed reuse from the prior held-out campaign as new validation evidence.
4. Fixed: the exact five ADR 0013 identities, now with the declared damped
   profile, paired with true-state runs using that same profile. All must
   pass the unchanged performance targets. Hover remains 5..65 s, .08 m.
5. Execution hashes, protocol hashes and prerequisite development/fixed
   results identify the inputs to validation. Independent validation cases:
   hover seeds 91000..91019; square seeds 92000..92009. All must complete
   and pass the inherited per-case/estimation/actuator criteria. These
   are finite-sample results, not a universal probability guarantee.
6. Retain every result, including errors. Audit online/offline equality,
   sensor/force/control reconstruction, covariance validity, complete
   clocks and artifacts. Include unchanged original-path regression.
7. Rendered diagnostics and the full pinned toolchain, with warnings as errors,
   check the implementation and evidence. A failed validation case keeps its
   result; changing its conditions cannot turn it into independent validation.

## Evidence preservation

The campaign has its own protocol identity and version. Existing
campaign defaults and evidence remain readable. Core control functions
still accept explicit caller-supplied parameters; this profile is an
illustrative, documented configuration rather than a hidden change to
every caller. Full histories remain outside Git; source, tests, derivation,
commands and verified result summaries are versioned.

## Version 2: maximum stiffness within the all-real pole family

Version 1 retains its definitions and recorded 29/30 validation result. Seed
91001 misses at the unchanged 5.0-s hold entry, with .0813936606 m error; after
10 s its error stays below .039612 m. All attitude conditions pass. This points
to insufficient startup settling margin, not a reason to move the scoring window,
relax the target or change estimator noise. The independent physical audit passes
all 30 histories, including the failed performance case.

The first all-real development candidate, roots -5,-5,-8,-11,-11, reduces
seed 91001's startup peak to .0706366 m. This meets the original .08 m criterion
but narrowly misses the stricter .07 m development margin. Its failed test log
is retained; the .07 m check is not relaxed. No version-2 held-out seed was opened.

Use the real-pole stiffness bound to select the final version-2 target. Write
positive real decay rates as a1..a5 with sum 1/tau=40. Coefficient matching gives
kp = 1 / sum(i<j, 1/(ai*aj)). Cauchy-Schwarz gives

    sum(i<j, ai*aj) * sum(i<j, 1/(ai*aj)) >= 100,
    sum(i<j, ai*aj) <= ((sum ai)^2 - (sum ai)^2/5)/2 = 640.

Consequently kp<=6.4 s^-2, with equality for five equal rates of 8 s^-1.
This maximizes static position stiffness within the stated all-real continuous
pole family, not over arbitrary nonlinear controllers or complex-pole designs:

    D2(s) = .025*(s+8)^5
          = .025*s^5+s^4+16*s^3+128*s^2+512*s+819.2.
    kp=6.4, kv=4, ka=8, kr=16.

The decay rates still sum to 40 s^-1, preserving the physical motor time constant.
The same held-clock local model has slowest equivalent decay rate about
3.92665 s^-1 and minimum damping ratio .76816, compared with 2.45992 and .60122
for version 1. Static tilt sensitivity falls from about 2.789 to 1.533 and
velocity-error sensitivity from .9048 to .625. The rate gain decreases and the
attitude-times-rate gain product falls from 150.45 to 128; the attitude gain
increases slightly from 7.785 to 8. Horizontal position/velocity feedback
increases. These are local mathematical predictions;
the nonlinear campaigns decide whether the profile is acceptable.

Version 2 uses the version-1 moment-matched prior, all plant/sensor/noise
definitions, vertical/yaw settings, limits, references, timers and acceptance
thresholds.
The `--design-version 2` selection is explicit; version 1 remains the default for
backward compatibility and its three existing protocol hashes remain unchanged.

The prerequisites for version-2 validation are passes on the original five fixed
identities and thirteen development identities: hover 8200..8205, square
8206..8208, and observed diagnostic hover seeds 91001, 91011, 91016, 91019.
The latter four are reused observed data, not fresh validation. The startup CI regressions for
30 and 91001 also require a .07 m development margin with takeoff/hold start
unchanged. Full acceptance still scores the entire 60-second hold against .08 m.

The independent version-2 validation
batch uses hover **93000..93019** and square **94000..94009**. Retain all outcomes
and require all inherited conditions for each of the 30 planned trials. Do not
present the first failed batch as a successful version-2 validation.

Full-history audits may execute independently in separate worker processes.
Every existing replay, command, phase, clock and metric check still runs; the
entire iterator must complete before any report directory is created. Tests
require exact serial/parallel histories and saved bytes and prove that corruption
in a later worker prevents publication. This changes scheduling of verification,
not numerical execution, mathematical criteria or evidence completeness.

## Rejected alternative: stiffness with a damping constraint

Version 2's failures at 5.52 and 7.4125 s are startup recovery peaks, not just
hold-entry samples; changing the scoring window is not a remedy. All attitude
and numerical conditions pass. The all-real family has exhausted its stiffness
bound. The equal-decay complex-pole alternative uses the same physical model:

    D_b(s) = .025*(s+8)*((s+8)^2+b^2)^2.

All real parts remain -8 s^-1, so their sum preserves the fixed motor lag.
Require continuous complex-mode damping at least 1/sqrt(2), giving 0<=b<=8.
Writing x=b^2, coefficient matching gives

    kp(x)=(64+x)^2/(640+6*x),
    d(kp)/dx=(64+x)*(896+6*x)/(640+6*x)^2 > 0.

Consequently b=8 maximizes position stiffness in this explicitly constrained
family. It is not an optimum over arbitrary controllers. The selected candidate is

    D3(s)=.025*(s+8)*(s^2+16*s+128)^2
         =.025*s^5+s^4+19.2*s^3+204.8*s^2+1228.8*s+3276.8,
    kp=16 s^-2, kv=6 s^-1, ka=32/3 s^-1, kr=96/5 s^-1.

The unchanged held-clock map predicts minimum damping .59084 and slowest
equivalent decay 5.38638 s^-1. The tilt and velocity-error static sensitivities
are .613125 and .375, respectively. This exchanges some sampled damping for
faster recovery and greater disturbance rejection; control effort and limiting
must be measured. The sensor/filter prior and every remaining configuration
field stay as in version 2.

The short startup probe keeps the same takeoff and 5-s hold entry. Its
development condition is <.065 m over 5..10 s for the four known failed
identities 30, 91001, 93003 and 93012, with no limiting or abort. Additional
observed diagnostic seeds retain the original .08 m criterion. These short
probes are not full-hover qualification.

The proposed full evaluation would preserve versions 1 and 2 and their exact
protocol hashes. Its prerequisite comprises the original five fixed missions
and 33 development missions: the thirteen version-2 development identities plus
all twenty observed version-2 validation hovers (93000..93019). Reusing those
observations broadens regression evidence but does not create new validation.

The reserved validation design comprises hover seeds 95000..95019 and square
seeds 96000..96009, with all 30 required to meet the unchanged full-mission
conditions, including the 5..65 s hover window and .08 m limit. This proposal
did not reach that stage because the startup probe failed. These seed ranges
are historical protocol identities, not a list of currently unused seeds;
the [final comparison](../results/final-geometric.md) documents their
separate use in the completed controller study.

### Rejected prototype and diagnostic boundary

The preceding b=8 proposal did not pass its pre-implementation probe. Seed 93012
reached .0844626 m and activated a limit; seed 93003 reached .0759334 m, above the
declared .065 m development margin. No version-3 production profile was added.
The failed probe and its exact script are retained as development evidence.

A separate causal bounded-integral prototype also failed. Its continuous target
was .025*(s+20/3)^6, with kp=40/3, kv=5, ka=80/9, kr=50/3 and integral gain
400/27 s^-3. The horizontal integral acceleration used the previous outer-tick
position error and was bounded at +/-.5 m/s². Seed 93012 reached .111452 m;
93003 reached .069758 m, and the integral bound became active. These observations
do not support adding integral state to the maintained controller. The hook was
confined to the external prototype process; repository control behavior and
serialized schemas were unchanged.

Recorded-error screens treat estimation errors as fixed external inputs; they
cannot predict their change under a new controller or establish infeasibility.
The extended screen correctly uses pitch inertia .025 kg m² for the north axis
and roll inertia .02 kg m² for east. The earlier screen used .02 on both axes,
so its pitch moment magnitudes are not valid; its position predictions are
unaffected. Retain the earlier output with this limitation, not as a quantitative
moment bound. The corrected expanded PD/integral screen found no candidate below
.08 m within both requested rate and moment bounds among its 129 stable entries.
This is finite-grid diagnostic evidence, not an optimal-control impossibility proof.

The inherited full-mission actuator contract permits at most .5 s of consecutive
actuator limiting and requires the final second to be limit-free. This is distinct
from the extra absence-of-any-limiting development condition above. Additional
complex-pole probes report both conditions separately and keep all actual limits
unchanged. They do not retroactively qualify either frozen batch or the failed
strict-margin prototype. Qualification requires a candidate that passes its
prerequisites and a separately fixed validation protocol.

The b=10,12,14 saturation diagnostics all failed even the inherited .08 m startup
target: seed 93012 reached .0822047, .0826201 and .0913843 m, respectively.
The b=14 profile additionally failed seeds 91016 and 8200. Across these probes,
the longest actuator-limiting intervals were .05, .10 and .15 s; brief limiting
was therefore not the disqualifying inherited condition. Higher bandwidth did
not resolve the performance issue and increased measured moment effort. None
of these profiles qualified for the proposed validation batch.

These gain/prior profiles remain limited comparison configurations. A sampled
local pole calculation and a finite recorded-error screen do not substitute
for passing the stated nonlinear flight conditions. Startup uncertainty and
controller comparisons are evaluated separately in the
[current results](../results/README.md).

## Protocol identity and numerical backends

The golden protocol test accounts for a narrow difference between numerical
backends. One backend reproduces the recorded version-1 validation digest;
another produces `de28fa6bedf6986df434a515181c954a28f5dc54f7bcd229ffb8ced90c338f5b`.
Selecting OpenBLAS HASWELL locally reproduces that exact digest. Compared with
the recorded kernel, only the q_WB y component for hover seed 91010 differs:
-.011563447064241896 versus -.011563447064241898 (one float64 unit).
The same audit finds only derived initial-quaternion differences in the other
protocols across seven kernel selections; all remaining fields are identical.

A small golden initial-attitude fixture retains the authenticated recorded
inputs. Before checking the unchanged digest, the regression test substitutes
only those attitudes, and only when each component agrees within four float64
units. Every other protocol field remains in the exact digest check. Negative
tests reject a five-unit attitude change and changes elsewhere. Both design
versions and the legacy profile are checked on the observed kernels.

Production protocol generation, strict report/digest validation, online/offline
array equality, source behavior and all flight acceptance criteria remain
unchanged. Byte-identical numerical replay requires matching the numerical
backend as well as package versions. This regression test establishes preserved
mathematical inputs to roundoff across the checked kernels; it does not establish
portable byte-identical trajectories or rewrite the frozen experiment records.
