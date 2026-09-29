# Residual supported-hover diagnosis

The remaining seed-47001 hover failure is mainly explained by the saved
position/velocity estimation errors driving the outer controller. An unfitted
sampled cascade model reproduces the full horizontal trajectory to 0.761 mm RMS.
The exact response accounting assigns -7.729 cm of the -9.693 cm East error at
the failed peak to navigation-error forcing. This is a useful explanation of the
observed path, not a claim that deleting those terms would produce a real flight.

The [frozen protocol](decisions/0032-residual-supported-hover-diagnosis.md) permits
offline diagnosis only. The [verification record](progress/2026-09-28-residual-supported-hover.md)
preserves all six histories, numerical checks and limitations. No new scientific
flight, gain change or filter change was made; the 10.18 cm failure still stands.

## What the vehicle and estimator are doing

At t=6.2825 s, true NED tracking error is
`[2.8264, -9.6928, 1.3071] cm`. The estimate reports
`[1.4257, -8.0461, 0.5598] cm`; navigation error (estimate minus truth) is
`[-1.4007, 1.6467, -0.7473] cm`. The physical error norm is 10.1808 cm and
the estimated tracking norm is 8.1906 cm. Neither is just an estimator-display
artifact: the vehicle is actually displaced.

Instantaneous navigation error does not explain the accumulated displacement
alone. Before the peak, East position and velocity estimation errors persist
with the same sign. Through the controller, they command motion in the opposite
direction. Their complete earlier histories must be propagated through the
feedback dynamics; a snapshot at the peak misses that memory.

| Signed contribution at the failed peak | North, cm | East, cm | Down, cm |
| --- | ---: | ---: | ---: |
| Navigation position | 0.0376 | -2.5713 | 0.5843 |
| Navigation velocity | 1.0164 | -5.1578 | 0.5547 |
| Thrust-axis estimation | 1.4736 | -1.0275 | -0.0010 |
| Thrust-axis tracking | 0.2975 | -0.9431 | -0.0018 |
| Motor response | 0.0021 | 0.0036 | 0.1418 |
| Initial state | 0.0002 | 0.0002 | 0.0009 |
| Reference following | 0.0000 | 0.0000 | 0.0348 |
| Within-step integration | -0.0009 | 0.0030 | -0.0064 |
| Mass, drag and command limits | 0.0000 | 0.0000 | 0.0000 |
| **Total physical tracking error** | **2.8264** | **-9.6928** | **1.3071** |

The combined initial-state and nonreference forcing restricted to the first
0.5 s contributes less than 0.8 mm in norm at this later peak. That does not
prove that startup has no indirect effect: subsequent estimator/controller
histories retain feedback from earlier motion. It does rule out identifying
the direct retained release impulse as the dominant term in this accounting.
There are no active inner or outer command limits in any of the six histories.

During the failed flight's 5..11 s hover interval, navigation-error RMS norms
are 2.449 cm and 2.059 cm/s. Thrust-axis estimation error is 0.0870 degrees RMS,
with a 0.1205 degree peak. Accelerometer-bias error is 0.00812 m/s² RMS;
gyro-bias error is 0.000179 rad/s RMS. These coupled estimation errors do not
separately identify a bad bias update, an outlier or a covariance defect.

## Exact response accounting

Let `n_p = p_hat - p`, `n_v = v_hat - v`, and let `j` be the last outer-control
epoch at or before plant interval `k`. The requested acceleration is

```text
a_req = a_ref[j] + Kp*(p_ref[j] - p[j]) + Kv*(v_ref[j] - v[j])
        - Kp*n_p[j] - Kv*n_v[j].
```

The gains are unchanged: horizontal `Kp=1 s^-2`, `Kv=1.8 s^-1`; vertical
`Kp=2.25 s^-2`, `Kv=3 s^-1`. Outer updates are every 20 ms, inner updates every
10 ms and plant steps every 2.5 ms. The ordered physical identity uses actual
thrust `T`, commanded thrust `Tc`, mass `m`, nominal mass `m0`, and true,
estimated and commanded body-down axes `b`, `b_hat`, `b_cmd`:

```text
a_true = a_feasible
       + (1-m0/m)*(g*e_D-a_feasible)
       - (Tc/m)*(b-b_hat)
       - (Tc/m)*(b_hat-b_cmd)
       - ((T-Tc)/m)*b + drag_W/m.
```

The gravity assumptions match in these six cases. Command limiting contributes
`a_feasible-a_req`; it is roundoff only here. Each channel receives its own
sample-held PD feedback and its own forcing:

```text
a_i[k] = -Kp*p_i[j] - Kv*v_i[j] + forcing_i[k]
p_i[k+1] = p_i[k] + h*v_i[k] + h^2*a_i[k]/2 + delta_p_i[k]
v_i[k+1] = v_i[k] + h*a_i[k] + delta_v_i[k].
```

Only the initial-state channel starts nonzero. Reference forcing is retained
over the entire flight; subtracting the current reference from that channel
turns the position decomposition into a tracking-error decomposition. The
within-step channel contains the exact difference between saved plant
increments and left-epoch constant acceleration. Every projected RK4 plant
and motor step is independently reconstructed before this residual is used.
It is not an unexplained residual allowed to absorb a broken plant replay.

The acceleration identity closes within 3.553e-15 m/s² and the complete
position/velocity responses within 4.552e-15 in their respective SI units.
Every saved command, plant step and motor step reconstructs exactly. Signed
vectors add; component norms and alleged percentages of causation do not.

## Unfitted local model

For each horizontal axis, use states `p, v, u, w, alpha`, where
`u=-g*b_horizontal` is the near-level acceleration due to inclination,
`w=du/dt` and `alpha=dw/dt`. The flow is

```text
p_dot=v; v_dot=u; u_dot=w; w_dot=alpha
tau*alpha_dot = alpha_command-alpha
alpha_command = kr*(ka*(held_acceleration-u-inclination_error)
                    -w-inclination_rate_error).
```

The existing gains are `ka=3 s^-1`, `kr=12 s^-1`, and the motor lag is
`tau=0.025 s`. The actual nested update clocks are retained. Initial horizontal
position, inclination and rates and the saved navigation/inclination/rate error
histories drive the model. There are no fitted coefficients. It assumes
near-level motion, hover thrust, matched inertia and no limits/drag; it omits
the vertical release transient, changing collective and nonlinear rotations.

| Seed | Unaligned horizontal RMS discrepancy, mm | Aligned discrepancy, mm |
| --- | ---: | ---: |
| 47001 | 1.389 | 0.761 |
| 47002 | 0.685 | 0.443 |
| 47003 | 0.366 | 0.169 |

These are discrepancies from the saved physical trajectories, not new tracking
scores. Maximum discrepancy across the six full histories is 3.159 mm. The
20 ms lifted controller/plant map has spectral radius 0.983152; equivalent
poles are `-0.849558`, `-1.242003 +/- 1.809948i`, and
`-16.986931 +/- 7.756805i` per second. This local controller/plant model is
stable while driven by the saved errors. It does not prove stability of the
joint nonlinear estimator/controller, estimate reliability, or hardware flight.

## Decision and reproduction

The diagnosis meets its scope. The evidence supports **one bounded navigation
feedback isolation experiment** next: on the failed aligned seed47001 only,
substitute true position/velocity at the outer controller, leaving the estimator,
guards, attitude/rate inputs, support, sensor draws and all scoring unchanged.
Freeze that protocol before execution and compare it with this exact saved
baseline. Truth is diagnostic and cannot be deployed. A useful improvement
would justify a subsequent implementable navigation-information design; a
failure would redirect the diagnosis. Do not choose new gains or filter tuning
by subtracting the response table.

Keep the initializer experiment-only and the cascade default. The 8 cm hover
gate and separate mass failure remain open; geometric qualification and ordinary
mission integration are unchanged.

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.residual_hover_diagnostic --campaign SAVED_INDEPENDENT/campaign --output NEW_DIAGNOSIS
```

The runner authenticates all campaign payload references and recomputes the 34
original flight scores before diagnosing six hover histories. It writes a new
directory, full derived signals and a fingerprinted report. The prior archive's
independent verifier reconstructs all 17 campaign decisions; the new bundle's
NumPy-only verifier binds six raw histories to that report and reconstructs
their scores and complete response histories. No scientific flight is executed.
