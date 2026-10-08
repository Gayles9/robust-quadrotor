# Why the combined supported prior changes the flight errors

A more accurate starting estimate does not guarantee better tracking throughout
a flight. The [combined startup experiment](combined-supported-prior.md) passes
the three tested 8 cm hover limits, but some errors increase relative to its
controls. This analysis explains that tradeoff using all 27 saved clean histories:
three seeds, three missions and three configurations. It reconstructs the saved
flights without changing their behavior. The
[analysis protocol](../decisions/combined-prior-tradeoff-diagnosis.md)
specifies the input identities, equations and checks.

## Observation corrections

A more informative initial velocity covariance changes later cross-covariances
and measurement gains. An observation innovation contains both sensor noise and
the difference between the physical observation and the filter prediction. The
diagnostic independently reconstructs the full 21-state joint update, including
the stored IMU-noise coordinates and the right-local attitude reset:

```text
nu = measurement - prediction = sensor_noise + prediction_error
S = H C H^T + R
K = C H^T S^-1
correction = K sensor_noise + K prediction_error
```

The code uses a linear solve. The original observation gate, which rejects measurements
whose residual is too large relative to predicted uncertainty, still decides
whether to fuse; a rejected observation has zero effective gain for the attribution.
Position and altitude observations retain their original order. Complete replay
must match the saved nominal states, covariances, events and command decisions.

For each candidate/control pair, the fixed accounting order is:

```text
delta_correction = (K_candidate - K_control) nu_control
                 + K_candidate (nu_candidate - nu_control).
```

The second term includes the changed trajectory and previous corrections. These
coupled terms cannot be removed independently to predict a new nonlinear flight.
Attitude corrections are expressed in each filter's local body coordinates;
their coordinate difference is not itself a common-world attitude intervention.

## Controller and physical response

Let e_p and e_v be estimated-minus-true position and velocity. At each actual
outer-loop epoch, before limits, the requested NED acceleration is exactly:

```text
a_requested = a_reference + Kp p_reference + Kv v_reference
            - Kp p_true - Kv v_true - Kp e_p - Kv e_v.
```

The existing sampled response decomposition retains the real controller clocks,
initial state, reference, navigation errors, limits, attitude estimation/tracking,
mass, motor lag, drag and within-step integration remainder. Its components sum
to the recorded physical position and velocity. This is pathwise accounting
using saved forcing histories, with simulated truth used only offline.

For complete tracking error e and additive response channels e_i, the signed
quantity `mean(e dot e_i)` sums to the full mean-squared tracking error. Negative
shares are expected when effects cancel. Comparisons use each flight's own full
duration, preserving the original RMSE score. Event and command differences use
the common time prefix and state its endpoint explicitly.

The separate near-level hover model uses the original gains, inertia, motor lag
and clocks without fitting. Its omitted drag, vertical motion, saturation and
nonlinear coupling remain limitations. Agreement with nine saved hover paths
does not establish nonlinear stability or general flight qualification.

## Result and decision

All 27 complete histories reconstruct and all independent identities pass.
The first position observation at 0.2 s has much less velocity authority after
conditioning. Later coupled velocity, attitude and bias corrections also change.
At 1.6 s, the combined seed47001 cases reject one position observation that both
controls accept under the same unchanged NIS gate. The gain comparison retains
this discrete change, rather than treating every correction as smoothly varying.

Navigation feedback and attitude estimation both improve the seed47001 error
budget. Seed47002 has smaller navigation contributions but larger attitude
contributions, producing a small net regression in hover and nominal tracking.
Both contributions grow for seed47003. The unfitted hover model agrees with all
nine saved paths to 0.146..0.769 mm horizontal RMS, below the predeclared 2 mm bound.
The [verification record (ZIP)](../../evidence/development-records.zip) gives
the signed budgets, numerical checks and limits of that explanation.

No correctable implementation defect or justified further startup variant was
demonstrated. The combined prior remains experimental because it fails the
[paired comparison](combined-supported-prior.md); normal defaults are unchanged.
The [whole-flight error budget](whole-flight-error-budget.md) extends this
analysis by relating estimation errors to the controller's tracking limits.

## Reproduce

The [runner](../../experiments/combined_prior_diagnostic.py) authenticates and
rescores all 84 source outcomes before selecting the 27 clean histories.
It then saves every update decomposition, physical response and comparison.
Use the matching source snapshot and original evidence directories:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.combined_prior_diagnostic --original ../original-evidence/campaign --boundary ../boundary-evidence/nonlinear-release/flights --combined ../combined-prior-evidence/flights --output ../combined-prior-diagnosis --workers 2
OPENBLAS_NUM_THREADS=1 .venv/bin/python -W error -m experiments.combined_prior_diagnostic --original ../original-evidence/campaign --boundary ../boundary-evidence/nonlinear-release/flights --combined ../combined-prior-evidence/flights --verify ../combined-prior-diagnosis --workers 2
```

Both commands use saved flights only. The second additionally compares the
complete regenerated diagnostic records and arrays with the saved diagnosis.
The [tests](../../tests/unit/test_combined_prior_diagnostic.py) cover independently
calculable gain changes, NED feedback signs, rejection, ordered observation
ownership, correlated cancellation, clocks, shapes and report tampering.
