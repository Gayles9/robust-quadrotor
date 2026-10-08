# Independent-inclination measurement contract and feasibility

A separate measurement of the vehicle's tilt could help distinguish tilt error
from accelerometer bias. This study derives the required measurement geometry
and uncertainty handling, then checks whether the current project provides
enough evidence to support such a sensor.

**The current assumptions do not justify an implemented measurement component.**
No independent orientation source/model, calibration, timing or simultaneous
error allocation is established. The [study protocol](../decisions/independent-inclination-feasibility.md)
defines the scope; the [capability ledger](operating-envelope.md) and
[official report](../report.md) place the result alongside the flight evidence.

This is a simulation-first project. Hardware is not required to define a valid
simulated sensor, but a new sensor still needs an explicit physical model and
justified assumptions. Adding a camera/pose source would expand the existing
IMU/position/altitude problem. Perfect simulator orientation, or arbitrary noise
placed on it, does not establish that an implementable source meets the budget.
The [record (ZIP)](../../evidence/development-records.zip) gives the
verification results and source identities.

## Physical source and frame contract

An eligible source would observe the orientation of a rigid target `C` in an
external reference frame `E`, using information beyond the current IMU/position
measurements. With known registration `R_WE` and mounting calibration `R_CB`,

```math
d_W=R_{WE}R_{EC}R_{CB}e_3,\qquad e_3=(0,0,1)^T.
```

Every rotation maps the right subscript frame into the left subscript frame.
The external reference must be registered to the project's NED world; the body
is FRD. The mounting maps body coordinates to target coordinates. These are
calibration requirements, not rotations inferred from simulator truth in flight.
Position co-produced by the same source must refer to the body origin, or have
a calibrated lever arm and appropriately propagated uncertainty.

The current [mission sensors](../../src/quadrotor_math/estimated_mission.py) contain
IMU, local position and barometric altitude. The
[replay observation kinds](../../src/quadrotor_math/eskf_replay.py) support position
and altitude. No independent orientation model or calibrated external data is
present at this boundary. The [compatibility spike (ZIP archive)](../../evidence/development-records.zip)
establishes that PX4/Gazebo could execute a simulation flight; it does not provide
this measurement. An attitude output from a system using the same IMU is not
automatically independent. In-flight accelerometer normalization cannot distinguish
gravity from arbitrary translational acceleration.

## Two-coordinate geometry

Let the nominal body-down direction be `d_hat=R_hat_WB e3`. Choose a fixed local
basis `E_W` with two orthonormal columns perpendicular to `d_hat`. For a unit
observed direction `z_W`, define a tangent **chord** residual

```math
r=E_W^T(z_W-\hat d_W).
```

The chart is fixed for this linearization. Under the existing right-local
attitude convention `R_true=R_hat Exp([delta_theta_B]x)`,

```math
H_\theta=-E_W^T\hat R_{WB}[e_3]_\times,\qquad
H=[0\;0\;H_\theta\;0\;0\;0_{2\times6}].
```

The full row has 21 columns, including the six endpoint IMU-noise coordinates.
Zero direct measurement columns for those six coordinates do not mean their
cross-covariances may be discarded. This is a Jacobian specification, not a
new observation type or an implemented ESKF correction.

`H_theta` has rank two and annihilates `e3`. The exact unobserved motion is
`R_WB -> R_WB Exp(psi [e3]x)`: a twist about body down. Away from level, a
rotation about **world** vertical generally changes `R_WB e3`. Body-axis twist and Euler/world
yaw coincide only in the relevant level configuration. The local hover rank
calculation applies at that configuration.

A unit antipodal observation `z=-d_hat` has zero tangent projection. A local
contract must therefore reject `d_hat.T z <= 0` before evaluating NIS. Passing
the hemisphere check does not prove that a large residual has a valid linear
Gaussian likelihood. Nonunit/nonfinite directions, invalid rotations, unknown
frame registration and invalid uncertainty are also ineligible. Normalizing a
malformed packet would conceal an invalid measurement rather than establish its
physical meaning.

Changing tangent basis to `E_W Q`, for orthogonal `Q`, transforms residual,
Jacobian and covariance together by `Q.T`; NIS and the linear posterior remain
unchanged. Basis changes must not be treated as an extra source of information.

## Uncertainty and shared information

For small left/world angular noise `epsilon_W` on external orientation,

```math
\delta d_W=-[d_W]_\times\epsilon_W,\qquad
J=-E_W^T[d_W]_\times,\qquad R_t=J C_\epsilon J^T.
```

This is first-order uncertainty propagation. Mounting/reference calibration
errors, persistent bias, nonlinear curvature and temporal correlation require
their own justified treatment. They are not removed by projecting to two
coordinates. If the source also supplies body-origin position, its joint noise
covariance contains `R_pt=Cov(n_p,epsilon_W) J.T`. This block is part of the
measurement model and must be retained.

For state error `delta_x`, observation noise `n`, and
`U=Cov(delta_x,n)`, the linear Gaussian innovation and cross covariance are

```math
S=HPH^T+HU+U^TH^T+R,\quad C=PH^T+U,
```

```math
\delta\hat x=CS^{-1}r,\qquad P^+=P-CS^{-1}C^T.
```

Both `[[P,U],[U.T,R]]` and the innovation covariance must satisfy their covariance
conditions: the joint covariance is positive semidefinite, and the innovation
covariance is positive definite so its solve is well defined. A reused IMU
stream can create `U`, including correlation with the
endpoint sample memory. Assuming zero correlation because the data arrive on a
different interface is invalid. These equations concern original linear error
coordinates; a future implementation would still need correct quaternion
injection/reset and every corresponding covariance transformation.

When noise is independent of state (`U=0`) but position and direction noise are
correlated, a correct sequential factorization uses

```math
L=R_{tp}R_{pp}^{-1},\quad
\tilde r_t=r_t-Lr_p,\quad\tilde H_t=H_t-LH_p,
\quad\tilde R_t=R_{tt}-LR_{pt}.
```

After the position update, use innovation
`tilde_r_t - tilde_H_t delta_x_p` for the second linear conditioning. This agrees
with the joint update. It does not justify fusing the original rows independently,
and it is not directly valid for nonzero `U` or an unaccounted intervening reset.

## Calibration, timing, gating and availability requirements

| Requirement | Conditional contract | Current feasibility |
| --- | --- | --- |
| Provenance | Identify what physically observes orientation; disclose reused IMU/position inputs | No justified independent model/data source |
| Registration | NED/FRD mapping, rigid mounting, body-origin position and calibration uncertainty | Unknown |
| Bias/noise | Angular bias bounds, tangent covariance, correlations across channels/time and with state | Unknown |
| Ownership | Unique sample identity, acquisition epoch and delivery epoch; no repeated fusion | No orientation stream |
| Latency | Same-epoch correction, or separately derived delayed-state handling | Existing replay rejects stale correction; no new delay handling |
| Rate/outages | Acquisition rate, maximum accepted age, losses, recovery and required/optional role | Unspecified |
| Gating | Geometry/provenance eligibility before statistical testing; retain all rejection records | Conditional mathematics only |
| Control budget | Posterior joint error bounds, initial response and physical/vertical residual allowance | No justified allocation |

Known direction-rate bounds would give

```math
\angle(d(t_a),d(t_d))\leq
\min\!\left(\pi,\int_{t_a}^{t_d}\|\omega(t)\|\,dt\right)
\leq\min(\pi,\Omega_{max}(t_d-t_a)).
```

The required bound is on **true** motion. Clipped desired rates are not such a
bound. Using the old single-input angle ceiling, an illustrative true-rate
bound of 1 rad/s would consume the entire allowance after 8.156 ms of age.
That figure reserves nothing for noise, bias, other estimation errors or physical
residuals; it is not a selected sensor latency or a demonstrated controller limit.
A delayed sample represents an earlier state and cannot be treated as a current
measurement. The current API accepts only same-epoch corrections; delayed-state
handling is not implemented.

For a conditional two-dimensional zero-mean Gaussian innovation, the radial
tail is `Pr(NIS>c)=exp(-c/2)`, so a 99% threshold would be
`c=-2 log(0.01)=9.21034037`. This is not added to configuration. Original position
and altitude thresholds stay 11.345 and 6.635. Nonlinear/biased residuals do not
inherit that coverage automatically, and eligibility/hemisphere rejections
remain separate events.

If each of `N` zero-mean tangent noise marginals has covariance bounded by
`sigma_max² I`, a union bound gives a sequence radius

```math
s=\sigma_{max}\sqrt{2\log(N/\alpha)}
```

with failure probability at most `alpha`. Temporal independence is unnecessary
for this upper bound. The dimensionless example `N=1000`, `alpha=0.01` gives
`s=4.798526 sigma_max`. This does not cover calibration, bias, nonlinear model
error or posterior estimator error. None of those missing quantities is zeroed
to manufacture an 8 cm allocation.

## Information benefit and stopping decision

The calibrated local model has rank 13 of 15: the two tilt/accelerometer-bias
ambiguities are removed, while yaw and yaw gyro bias remain. Add two unknown
constant inclination-measurement bias coordinates and the rank becomes 13 of
17. The two absolute tilt ambiguities return, now involving external sensor bias
as well. Explicit null vectors verify this result. Motion or independent
calibration could change the information available; this local model does not
prove impossibility in every maneuver.

The prospective controller screen still requires

```math
b_p\epsilon_p+b_v\epsilon_v+b_\eta\epsilon_\eta
+b_{\dot\eta}\epsilon_{\dot\eta}+b_{initial}+\rho\leq0.08\;\mathrm m.
```

Use the unrounded [saved coefficients](whole-flight-error-budget.md). Raw angular
measurement noise is not a bound on the estimator's inclination error, and a
position/direction update changes coupled navigation and bias errors and possibly
the gate sequence. Neither local rank nor a sensor's marginal accuracy closes
this joint condition. No complete allocation can be established here.

The mathematical contract passes its checks. Source/model eligibility and the
joint budget do not pass, so the component decision is no-go. No scientific
flight, synthetic sensor campaign, production integration or default change
follows. Implementing this extension would require a justified source/model and
an acceptance protocol tied to the complete error budget. Varying arbitrary
sensor noise or rate cannot establish those missing assumptions.

The [capability ledger](operating-envelope.md) distinguishes implemented, tested
and qualified capabilities. Broader flight qualification remains incomplete;
[planned work](../next-steps.md) is separate from the results established here.

## Reproduction

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.inclination_feasibility \
  --budget-report /path/to/whole-flight-error-budget/report.json \
  --output /path/to/new-inclination-feasibility
```

The runner binds the preceding execution source and report, then saves only
deterministic geometry/Gaussian fixtures and the conditional decision. It does
not generate observations from a newly assumed sensor or call a flight runner.
