# 0017: Geometric tracking with filtered force derivatives

## Purpose and scope

Geometric control compares the actual and desired orientations as rotation
matrices, avoiding a local roll/pitch/yaw error representation. The implemented
controller combines a geometric moment law with a twice-differentiated
force-to-attitude map and a 30 rad/s causal derivative filter. It is optional;
the cascade remains the default.

Fixed-yaw HOLD, quintic SMOOTH and minimum-snap references are supported; STEP
is rejected because its discontinuity does not provide the needed derivatives.
SMOOTH uses analytic one-sided jerk and snap at segment boundaries. No true
acceleration, drag, wind, actual motor state or true bias enters the controller.
The nominal-model derivative closure is excluded because it produced wind bias.

The equations below define the original filtered-force configuration. Its
finite-case performance criteria are distinct from a proof of stability.
The [geometric guide](../design/geometric-control.md) connects this configuration
to the implemented alternatives and completed comparisons.

## Equations

Use NED/FRD frames, active $`R_{WB}`$, $`\mathbf e_3=[0,0,1]^T`$, nominal mass $`m`$ and gravity $`g`$:

```math
\begin{aligned}
\mathbf c&=m\bigl(K_p(\mathbf p-\mathbf p_d)+K_v(\mathbf v-\mathbf v_d)\bigr),\\
\mathbf u&=m(g\mathbf e_3-\mathbf a_d)+\mathbf c,\\
\dot{\mathbf u}&=-m\mathbf j_d+D_1(\mathbf c),\\
\ddot{\mathbf u}&=-m\mathbf s_d+D_2(\mathbf c).
\end{aligned}
```

Here $`K_p,K_v`$ are diagonal gains; $`\mathbf a_d,\mathbf j_d,\mathbf s_d`$ are reference acceleration, jerk, and snap. The operators $`D_1,D_2`$ supply filtered derivative estimates.

Use $`L(s)=\omega_f/(s+\omega_f)`$, $`D_1(s)=sL(s)^3`$ and
$`D_2(s)=s^2L(s)^3`$, with $`\omega_f=30\;\mathrm{rad/s}`$.
Three trapezoidal low-pass sections give derivative estimates
$`\omega_f(\mathbf y_2-\mathbf y_3)`$ and
$`\omega_f^2(\mathbf y_1-2\mathbf y_2+\mathbf y_3)`$.
Initialize all sections to $`\mathbf c[0]`$; this yields zero feedback
derivatives at startup. Own state, require consecutive sample indices, and
update filter state only after successful finite arithmetic. Require
$`0<\omega_f h\leq1`$ for sample period $`h`$.

For heading $`\psi`$, construct and twice differentiate the desired axes:

```math
\begin{aligned}
\mathbf y_\psi&=[-\sin\psi,\cos\psi,0]^T,\\
\mathbf b_{3d}&=\frac{\mathbf u}{\lVert\mathbf u\rVert},\qquad
\mathbf b_{1d}=\frac{\mathbf y_\psi\times\mathbf b_{3d}}{\lVert\mathbf y_\psi\times\mathbf b_{3d}\rVert},\\
\mathbf b_{2d}&=\mathbf b_{3d}\times\mathbf b_{1d},\qquad
R_d=[\mathbf b_{1d}\ \mathbf b_{2d}\ \mathbf b_{3d}],\\
\boldsymbol\Omega_d&=(R_d^T\dot R_d)^\vee,\qquad
\boldsymbol\alpha_d=(R_d^T\ddot R_d-[\boldsymbol\Omega_d]_\times^2)^\vee.
\end{aligned}
```

Rates are in the desired body frame. With $`R=R_{WB}`$ and $`A=R^TR_d`$:

```math
\begin{aligned}
\mathbf e_R&=\tfrac12(R_d^TR-R^TR_d)^\vee,\\
\mathbf e_\Omega&=\boldsymbol\Omega-A\boldsymbol\Omega_d,\\
\mathbf M&=-k_R\mathbf e_R-k_\Omega\mathbf e_\Omega+\boldsymbol\Omega\times(J\boldsymbol\Omega)\\
 &\quad-J\bigl(\boldsymbol\Omega\times(A\boldsymbol\Omega_d)-A\boldsymbol\alpha_d\bigr),\\
f&=\mathbf u^T(R\mathbf e_3).
\end{aligned}
```

The vee map $`(\cdot)^\vee`$ converts a skew-symmetric matrix to its three-vector.

Default $`k_R=0.64\;\mathrm{N\,m}`$ and $`k_\Omega=0.32\;\mathrm{N\,m\,s}`$ preserve the recorded
design. Reject nonsmooth outer limiting/domain violations; retain visible moment
clipping and bounded nominal allocation. Projection is recomputed at inner ticks.
Held reference jets and filtered derivatives are approximations; the ideal
continuous-time Lyapunov identity is a unit-test oracle, not a sampled-system proof.
Reference: Lee, Leok, McClamroch, CDC 2010, DOI 10.1109/CDC.2010.5717652.

## Evaluation criteria for the original configuration

1. Independent normalization derivatives, moving-frame inertia/energy identity,
   quaternion sign, NED hover/thrust direction, finite/domain checks and ownership.
2. Independent direct transfer recurrence, constant startup, reset/isolation,
   indices, frequency response <=3% magnitude error / <=20° lag at 0.1, 0.25,
   0.5 Hz. White-noise derivative gain <=10% of raw cubic stencils.
3. Mission clocks, no terminal command on abort, default-cascade compatibility,
   estimated-command reconstruction and exact offline ESKF replay.
4. Fixed true-state pairs: nominal, half plant step, +/- initial offset, mild
   wind, reversed wind, wind plus offset. Existing 15 cm whole-mission RMSE,
   25 cm peak, 8 cm final position, 8 cm/s final speed, no limiting/rotor contact.
   Geometric RMSE must not exceed cascade per case; first five mean ratios <=0.80.
   Refinement <=5 mm position / 0.05° attitude; one nominal and one noisy exact repeat.
5. Fixed estimated pairs, seeds 30 and 31, on the same minimum-snap mission,
   with and without mild wind. Same physical gates, no per-case RMSE regression;
   report actual-moment squared effort and requested-moment variation, allowing
   at most 2x paired actual-moment effort for performance promotion. Two full
   60-second hovers, seeds 30 and 93012, preserve the original 5..65 s / 8 cm gate.
   Use the original geometric position gains and matched cascade gains for spline
   pairs; also report the established v2 cascade for full-hover context.
6. Warning-strict software checks and authenticated saved histories verify the
   implementation independently of the performance outcome. A controller can
   pass its implementation tests while failing flight-performance criteria.

The criteria are specific to this configuration and campaign. Later comparison
results do not retroactively pass a failed original case.
