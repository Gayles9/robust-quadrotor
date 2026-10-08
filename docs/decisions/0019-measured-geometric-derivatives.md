# ADR 0019: Measurement-derived physical feedback derivatives

## Motivation

All five gain/rebasing profiles in [ADR 0018](0018-geometric-estimator-corrections.md)
missed the required hover gates. Rebasing suppresses correction impulses but
leaves a delayed estimate of physical force derivatives.

An IMU measures specific force, so the controller can estimate physical
acceleration without reading the simulated true state or inferring force from
a nominal rotor/drag model. This alternative uses that estimate to construct
the force derivatives at the original position and geometric gains. The
acceptance criteria and filter frequency are unchanged.

## Equations and measurement boundary

Let f_m [m/s²] be current measured body specific force, b_a its posterior bias,
n_a the endpoint ESKF's conditional current accelerometer-noise mean, R_hat the
posterior active R_WB and g nominal gravity. All are available causally:

    a_hat_W = R_hat * (f_m - b_a - n_a) + g*e3.

At each outer tick feed F_hat=m*a_hat_W [N] into the unchanged 30 rad/s
three-section filter. Its first derivative estimates m*j_hat_W [N/s]; the
second output is unused. For e_v=v_hat-v_d, planned a_d, j_d, s_d, and the
unchanged raw PD correction c=m*(Kp*e_p+Kv*e_v):

    u       = m*(g*e3-a_d)+c
    u_dot   = -m*j_d + m*Kp*e_v + Kv*(F_hat-m*a_d)
    u_ddot  = -m*s_d + Kp*(F_hat-m*a_d) + Kv*(D1(F_hat)-m*j_d).

These are physical kinematics with a filtered measured jerk. They ignore
instantaneous posterior-state revisions as physical derivatives; raw feedback
still uses the posterior. Constant wind load/drag is present in measured
acceleration, so the rejected nominal-model force closure is not resurrected.
No actual rotor speeds, wind, true acceleration, true biases or future sample
are supplied to the controller. No ideal continuous-time proof is claimed.

## Interfaces and frozen acceptance

The explicit `use_measured_acceleration=False` geometric selection is mutually
exclusive with estimator-correction rebasing. It requires the estimated-state
adapter; true-state requests must fail clearly rather than receive a hidden
truth-acceleration oracle. The original true-state filtered-force design remains
available and is used for the true-state comparison matrix.

Before flight, test independent force-jet kinematics, constant biased-position
hover, NED hover acceleration sign, sensor-only command reconstruction, ESKF
replay, reset/ownership, invalid/missing mode inputs and default compatibility.
The development comparison uses the same observed spline pair and two full
hovers at original gains. Passing them is a prerequisite for the proposed
40-case characterization/qualification matrix in ADR 0018, with its original
tracking, effort, hover-window, refinement and repeat conditions. Each
independent validation seed is evaluated once, with all failures retained.

Software verification and saved-history checks establish implementation
correctness separately from measured flight performance. The experiment makes
no claim of a global performance optimum.

## Outcome and limits

The measured derivative design also fails development: nominal spline RMSE is
8.5360 cm versus 7.4059 cm for the cascade, although effort falls to 1.8242 times
cascade. Full-hover peaks are 11.9179 and 17.4527 cm. The unchanged 8 cm condition
is missed. Across all six declared profiles, no candidate passes the combined
tracking/effort/hover conditions. None is promoted; original defaults remain.

**The proposed 40-case validation did not run because the required development
cases failed.** New random cases cannot erase those failures. The proposed seed
identities 95000..95003 and 96000..96003 identify this unexecuted protocol;
the [final comparison](../results/final-geometric.md) documents their
use in a separate study. The implemented runner and failed evidence remain
available for reproduction.

The original true-state configuration has a separate 15-flight regression
(seven pairs and an exact repeat). Neither that regression nor these failed
profiles proves that further controller improvement is mathematically impossible.
