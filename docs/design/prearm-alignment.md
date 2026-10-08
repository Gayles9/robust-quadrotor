# Stationary pre-arm alignment design

Pre-arm alignment uses measurements taken before flight to estimate the
vehicle's tilt and gyroscope bias. A physically supported, stationary 0.5 s
interval supplies useful information from the inertial measurement unit (IMU).
It does not reveal heading, establish physical support by itself, or authorize
arming.

This page explains the physical assumptions and the first-order uncertainty
model. That model misses the joint calibration limit: some combinations of
attitude and bias error vary more than its covariance predicts. The
[nonlinear model](prearm-nonlinear-uncertainty.md) meets that limit and is used by
the [standalone component](prearm-component.md). The
[alignment specification](../decisions/stationary-prearm-alignment-design.md)
and [feasibility evidence (ZIP archive)](../../evidence/development-records.zip) give the
evaluation criteria and measured results.

## Required physical condition

An external owner must assert that the vehicle is mechanically supported, motors
are off, attitude is constant, and world acceleration and angular velocity are
zero throughout acquisition **and through release**. A physical fixture and
an explicit operator procedure would need to establish that condition in a
hardware integration. The ordinary free-flight API does not establish it.
Experimental [supported-start flights](../results/supported-start-flight.md)
use an explicit modeled fixture.
An arming command, small gyro reading, low sample variance or flight phase label
does not establish it. Defining this operating mode does not establish that a
particular vehicle is stationary.

The IMU gates below can contradict an assertion; they cannot authenticate it.
Missing, expired or revoked support evidence is a hard rejection. There is no
fallback that applies the gravity alignment in flight. A false assertion
invalidates the result even if every numerical check passes.

## Component boundary

The table describes the operating contract. The
[component guide](prearm-component.md) describes the corresponding APIs.

| Input or output | Contract |
| --- | --- |
| Support assertion | Unique acquisition ID, responsible source, start/end clock times, mechanical support, motors-off and stationary attestations; explicit revocation and release handshake |
| IMU window | 201 paired body-FRD accelerometer/gyro samples at 400 Hz, unique monotonically ordered sample IDs and common acquisition clock; t=0 through 0.5 s |
| Prior | Heading mean and variance; accelerometer and gyro bias prior means/covariances; immutable sensor/noise profile and positive gravity magnitude; this study uses one fixed profile |
| Admissible prior | Independent heading, accelerometer bias and gyro bias; no unrepresented correlation with position/velocity; unsupported prior structure is rejected, never silently zeroed |
| Candidate | Unit quaternion `q_WB`, unchanged accelerometer-bias mean, estimated terminal gyro bias, full 9x9 right-local covariance ordered theta/ba/bg, source sample range, reference time and validity deadline |
| Diagnostics | All gate statistics/thresholds, estimated inclination, world-axis uncertainty radius, reasons, model/profile identity and support provenance |
| State | `collecting`, `rejected`, or `ready-for-release`; incomplete data cannot be ready, rejection is latched for that acquisition |
| Release | Recheck support/motors and clocks, receive one fresh disjoint paired sample at t=0.5025 s, then permit downstream initialization; this stage never issues an arm command |

The reference script accepts only the fixed zero-mean bias/heading case and a
harness support flag. It is not a substitute for the production provenance,
state machine, sample-ID checks or release handshake. Those are implemented
separately by `PrearmAlignmentSession` in `src/quadrotor_math/prearm_alignment.py`.

## Information and estimate

The world frame is North–East–Down (NED), and the body frame is
Forward–Right–Down (FRD). With body-to-world rotation R, supported stationarity gives

\[
 f_k=-R^T g_W+b_{a,k}+n_{a,k},\qquad
 \omega_k=b_{g,k}+n_{g,k},\qquad g_W=(0,0,g)^T.
\]

In particular a level supported accelerometer reads approximately **-g on body
z**. This equation relies on the support reaction; arbitrary accelerated flight
does not supply this observation. Let s=mean(f)-the prior accelerometer-bias
mean, and retain the prior heading psi. Then

\[
 \hat\phi=\operatorname{atan2}(-s_y,-s_z),\quad
 \hat\theta=\operatorname{atan2}(s_x,\sqrt{s_y^2+s_z^2}),\quad
 \hat R=R_z(\psi)R_y(\hat\theta)R_x(\hat\phi),
\]

\[
 \hat b_{a,T}=\hat b_{a,0},\qquad \hat b_{g,T}=\operatorname{mean}(\omega).
\]

Gravity supplies two inclination directions. It supplies no heading. A single
orientation cannot separate transverse accelerometer bias from inclination;
estimating both independently from this same mean would invent information.
Gyro bias can be estimated because angular velocity is externally known to be
zero. The sample mean estimates the window-average bias; its difference from
the **terminal** bias must remain in the uncertainty.

The fixed profile has sample standard deviations (sigmas) 0.04 m/s² and
0.002 rad/s, bias-walk sigmas 0.0002 m/s²/sqrt(s) and 0.00002 rad/s/sqrt(s),
accelerometer-bias prior sigma 0.03 m/s², gyro-bias prior sigma 0.005 rad/s,
and heading sigma 3 degrees.
These are the repository's model assumptions, not independently validated
hardware specifications. Revalidating them, including environmental effects,
is a prerequisite for hardware use.

## Duration allocation and irreducible uncertainty

For N samples separated by dt=0.0025 s, T=(N-1)dt. If bias increments have
covariance W dt, define

\[
 A_N=\frac{dt(N-1)(2N-1)}{6N},\qquad B_N=\frac{T}{2}.
\]

The average walk has covariance W A_N, its covariance with the terminal walk
is W B_N, and the terminal walk has covariance W T. These follow by giving
increment j the mean weight (N-j)/N and terminal weight 1.

The white accelerometer contribution must satisfy sigma_a/(g sqrt(N)) <=0.02
degree. Terminal gyro-bias variance is sigma_g²/N+w_g² A_N, constrained to
three sigma <=0.03 degree/s. Solving these inequalities gives 137 and 132
samples respectively, hence minimum span 0.34 s. Rounding upward to the specified
0.5 s block gives **201 samples**. This is an analytic allocation, not a
flight-score duration search.

At that duration, white accelerometer inclination sigma is 0.0164784 degree
per tangent axis; terminal gyro-bias three sigma is 0.0242885 degree/s. Near
level, the accelerometer-bias floor alone is 0.1752 degree per axis. More samples
cannot average away that constant-bias uncertainty. Heading remains 3 degrees
in its original Euler parameter, even though its body-local covariance entries
change when tilted.

## Joint covariance and the first-order limitation

Covariance describes both the size of each error and how errors vary together.
The correlations matter here: the same accelerometer error can affect both
the inferred tilt and its relationship to the accelerometer bias.

Errors use R_true=R_hat Exp([delta-theta]x), true bias minus estimated bias.
Let J be the right-local derivative of the inclination estimate with respect
to s, u=R_hat^T e_D the retained-heading direction, and Sigma_a/g the
per-sample white-noise covariance. With the stated independent priors,

\[
 V_a=P_{a,0}+W_a A_N+\Sigma_a/N,\qquad
 C_a=P_{a,0}+W_a B_N,
\]

\[
 P_{\theta\theta}=J V_aJ^T+\sigma_\psi^2 uu^T,\quad
 P_{\theta a}=-J C_a,\quad
 P_{aa}=P_{a,0}+W_aT,\quad
 P_{gg}=\Sigma_g/N+W_g A_N.
\]

All other blocks are zero under the stated independence assumptions, and
P_a-theta is the transpose. The attitude/bias cross block is essential. At
level, J has J_roll,fy=-1/g and J_pitch,fx=+1/g, so P_roll,ba_y is positive
and P_pitch,ba_x negative. The implementation checks these signs, matrix finite
differences and an independently assembled latent-increment covariance.

For completeness, with r=sqrt(s_y²+s_z²), the inclination derivative is

\[
 D=\begin{bmatrix}0&s_z/r^2&-s_y/r^2\\
 r/\|s\|^2&-s_xs_y/(r\|s\|^2)&-s_xs_z/(r\|s\|^2)\end{bmatrix},\quad
 J=E_{:,1:2}D,
\]

where E maps ZYX Euler perturbations to right-local rotation perturbations:

\[
 E=\begin{bmatrix}1&0&-\sin\theta\\
 0&\cos\phi&\sin\phi\cos\theta\\
 0&-\sin\phi&\cos\phi\cos\theta\end{bmatrix},\qquad u=E_{:,3}.
\]

The world body-z/thrust-axis direction is d=R e_3. Its first-order error is
-R[e_3]x delta-theta. The two nonzero covariance eigenvalues are those of
P_theta-theta[0:2,0:2]. The largest-radius local 99% ellipse therefore has
radius sqrt(-2 ln(0.01) lambda_max), required <=0.75 degree. This retains the
heading contribution when tilted; using a fixed level-only radius would hide it.
The 99% label is a **local Gaussian approximation**, not a deterministic bound.

The complete nine-dimensional covariance is first order. The matching Gaussian
study obtained maximum whitened empirical variance **1.112164**, exceeding the
predeclared **1.10** limit. Its linearized counterpart was 1.062171. This is
consistent with neglected nonlinear coupling in strongly correlated attitude
and bias errors; the finite study does not uniquely establish the cause. Good
marginal axis coverage (99.4%) does not validate every joint direction. No
covariance inflation or second candidate was fitted after observing this result.

The local rotation conventions are consistent with
[Sola's quaternion/ESKF treatment](https://arxiv.org/abs/1711.02508); the
stationary-window sums and covariance above are derived specifically here.

## Rejection rules

Reject missing/revoked support, motors enabled, incomplete or nonfinite paired
data, duplicate/out-of-order sample IDs, unknown units/frame, incompatible
prior, or any clock deviation over 1e-12 s from this simulator's 400 Hz grid.
That clock tolerance is a numerical design constraint, not hardware timing
qualification. The candidate must also have a finite normalized quaternion and
symmetric positive definite covariance; invalid uncertainty never becomes ready.

Reject a gravity vector with norm below 1 m/s² or s_y²+s_z² below 0.25||s||²,
an inferred inclination above 15 degrees, a failed noise allocation, or an axis
radius above 0.75 degree. The uncertainty limit is usually tighter than the
15-degree numerical domain limit.

Three additional compatibility tests use alpha=0.001 each and
c(d,alpha)=d+2 sqrt(d ln(1/alpha))+2 ln(1/alpha):

| Test | Statistic | Threshold |
| --- | --- | --- |
| Mean gyro | (mean(omega)-prior_bg)^T V_g^-1 (mean(omega)-prior_bg), V_g=P_g0+W_g A_N+Sigma_g/N | c(3,alpha) |
| Gravity magnitude | (norm(s)-g)²/lambda_max(V_a) | c(3,alpha) |
| IMU variation | Sum of squared whitened accelerometer/gyro Helmert contrasts | c(6(N-1),alpha) |

Helmert columns Q are orthonormal and perpendicular to the constant vector.
For each sensor axis their covariance is
Q^T(Sigma_white I+W min(t_i,t_j))Q; whitening retains walk-induced temporal
correlation. Bias and constant gravity vanish under the contrast. The magnitude
gate uses |norm(h+eta)-norm(h)|<=norm(eta), rather than pretending the norm
residual is exactly Gaussian.

For a standard Gaussian vector, the squared norm has moment-generating function
(1-2t)^(-d/2); a Chernoff upper bound gives the stated c. Thus the three-test
union false-rejection probability is <=0.003 **only under the matching Gaussian
white-noise, walk and bias-prior model and genuinely stationary support**.
It does not cover malformed data, physical-support errors, or the separate
inclination/uncertainty budget gates. The nominal bounded-uniform population
is reported empirically, not assigned that exact Gaussian guarantee.

The fixtures reject an impulse, vibration, changing gravity and excessive mean
rate. They do not demonstrate detection of every disturbance. For example,
a level vehicle accelerating at [0.1712081, 0, 0.0014941] m/s² produces the same
mean accelerometer signal as a supported vehicle pitched by one degree. With
identical bias/noise, every sample is indistinguishable; all IMU gates can pass.
Likewise a constant rotation component can be confused with gyro bias within
the prior range. External support remains mandatory.

One acquisition produces one decision. Do not silently slide the window until
random noise passes; a new attempt requires new support evidence and a disjoint
window, with attempt history retained. No automatic arming or in-flight retry.

## Handoff and sample ownership

The window owns samples 0 through 200. While the vehicle remains supported,
hold attitude fixed and add W_a dt and W_g dt only to the bias diagonal blocks
for the one-interval delay. Preserve P_theta-a and all other existing blocks.
Reject stale results, support loss, overlapping IDs or a gap at release.

At sample 201, t=0.5025 s, acquire a **fresh** sample independent of all window
white noise. Assemble the 15-state ESKF covariance using the existing p/v prior
and the nine-state alignment block only if their independence is explicitly
valid. Otherwise the cross terms require another derivation; zeroing them is
not an admissible default. The 21-state endpoint joint covariance can then
append the fresh six-dimensional white sample covariance with zero cross block
and zero conditional noise mean. Bias uncertainty stays in the state block.
This independence claim concerns fresh sensor white noise, not the full reading.

The first flight prediction uses samples 201 and 202. Never replay samples
0 through 200 as independent observations or reuse their mean as a fresh
endpoint. Report the 0.5025 s pre-arm overhead separately; it cannot remove
initial flight time from later scoring.

## Verification and implemented uncertainty model

The feasibility study retains all 5,000 nominal and 5,000 Gaussian outcomes, including
rejected windows. It uses separate deterministic PCG64 streams and no reserved
flight seeds. Independent tests cover frame signs, finite-difference Jacobians,
latent-walk covariance, correlation whitening, uncertainty rejection,
nonstationarity ambiguity and supported handoff.

The implemented [nonlinear uncertainty model](prearm-nonlinear-uncertainty.md)
passes the same <=10% calibration gate. It uses the same 0.5 s window, priors,
sensor model and full correlations without an empirical inflation factor.
The [standalone component](prearm-component.md) provides the acquisition,
rejection and release state machine.

The [supported-start studies](../results/supported-start-flight.md) add a physically
modeled fixture, paired noise, complete flight durations and unchanged limits.
[Independent validation](../results/independent-supported-start.md) retains a
failed hover case. Support failures and acquisition time remain part of the
contract; supported-start results do not relabel earlier free-flight failures.
This alignment study does not establish controller promotion or hardware qualification.

## Reproduce

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.prearm_alignment_feasibility --output results/prearm-alignment
uv run pytest -q tests/unit/test_prearm_alignment_feasibility.py
```

Use a new output directory. The report contains source/protocol fingerprints,
software provenance, both populations, all rejection counts and explicit go/no-go.
The NPZ preserves errors, covariance matrices, whitened errors, gate statistics,
axis errors, uncertainty radii and rejection flags for all trials. Raw samples
are regenerated exactly from the fixed per-trial random streams. These are
stationary alignment trials, not flight logs.
