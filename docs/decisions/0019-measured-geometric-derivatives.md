# ADR 0019: Measurement-derived physical feedback derivatives

Date: 2026-09-26. Status: bounded alternative; no performance promotion.

## Preceding audit and decision

Checkpoint `0f5439dc902937e6633a40cea47b9cdf54453aff` is published on draft
PR #16. The complete warning-strict gate passed 3,304 tests in 506.24 s, plus
Ruff and strict mypy. All five ADR 0018 gain/rebasing profiles still miss both
hover gates. Their complete failures remain evidence. Gain search stops.

The remaining design weakness is estimating physical force derivatives by
twice differentiating corrected estimator states. Rebasing suppresses impulses
but also leaves a delayed estimate of the physical derivatives. The existing IMU
provides specific force, so a physical acceleration estimate is available without
any true plant input or nominal rotor/drag model closure. Test this one separate
derivative design at the original position and geometric gains before opening
fresh validation seeds. This is not another gain extension or a relaxed filter
frequency/flight specification.

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

Add explicit `use_measured_acceleration=False` geometric selection, mutually
exclusive with estimator-correction rebasing. It requires the estimated-state
adapter; true-state requests must fail clearly rather than receive a hidden
truth-acceleration oracle. The original true-state filtered-force design remains
available and is used for the true-state comparison matrix.

Before flight, test independent force-jet kinematics, constant biased-position
hover, NED hover acceleration sign, sensor-only command reconstruction, ESKF
replay, reset/ownership, invalid/missing mode inputs and default compatibility.
Run the same observed development pair and two full hovers at original gains.
Do not change gains after this result. Then freeze the better declared candidate
for the 40-case characterization/qualification matrix from ADR 0018, retaining
the original tracking, effort, hover-window, refinement and repeat conditions.
Every fresh seed is evaluated once. Preserve all failures and evidence hashes.

Run the full warning-strict software gate, publish tested source and exact
limitations, and close the engineering log. Further estimator/controller design,
additional sensing or startup alignment would require a separately justified
future experiment; this session makes no claim of a global performance optimum.

## Development rejection and closeout decision

The measured derivative design also fails development: nominal spline RMSE is
8.5360 cm versus 7.4059 cm for the cascade, although effort falls to 1.8242 times
cascade. Full-hover peaks are 11.9179 and 17.4527 cm. The unchanged 8 cm condition
is missed. Across all six declared profiles, no candidate passes the combined
tracking/effort/hover conditions. None is promoted; original defaults remain.

**Early disqualification supersedes the proposed 40-case validation run.** New
random cases cannot erase failures on required known cases. Preserve seeds
95000..95003 and 96000..96003 unopened for a future candidate that clears
development. This is a failure decision, not a relaxed acceptance rule. Retain
the implemented qualification runner and all failed development evidence.
Complete the source-wide software gate and a fresh 15-flight regression of the
original true-state controller (seven pairs plus an exact repeat), then stop
this tuning study. There is no claim that further design improvement is
mathematically impossible; future progress needs a new justified joint design,
not continued selection on these observed seeds.
