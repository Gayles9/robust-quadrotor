# Supported-hover navigation-feedback isolation

This diagnostic gives the outer position controller the simulator's exact
position and velocity while leaving the estimator and inner attitude controller
in place. This idealized feedback is called an **oracle**: it isolates a possible
source of error but is unavailable in an ordinary flight.

The intervention reduces seed47001's hover peak from **10.1808 cm
to 1.9896 cm**, an 80.46% reduction. Whole-flight RMSE falls from 6.4891 cm to
3.9256 cm, a 39.50% reduction. Both flights complete at 15.5 s. The oracle passes
every original flight condition and the separately predeclared headroom comparison.

This demonstrates substantial room for improvement in the outer navigation
feedback channel. It uses simulated true position and velocity, so it is a
diagnostic result. The current deployable-input configuration still has the
original 10.18 cm failure; alignment remains experimental and the cascade default
does not change. The [verification record (ZIP)](../../evidence/development-records.zip)
contains the evidence, and the [study protocol](../decisions/navigation-feedback-isolation.md)
specifies the intervention and acceptance criteria.

## What the experiment changes

At each 20 ms outer-controller update, the original law is evaluated using the
simultaneous true position and velocity:

```text
a_requested = a_reference + Kp*(p_reference-p_true) + Kv*(v_reference-v_true).
```

The original command limits, force-to-attitude construction and gains remain.
The estimator still processes the actual noisy IMU and slow observations. Guards
and completion still use estimated position/velocity, and the inner controller
still receives estimated attitude and body rate. The original observer tuple is
returned unchanged; a process-local adapter replaces only the outer arguments.
The adapter restores both patched functions on normal and exceptional exit.

The same mechanically supported 0.5 s acquisition and fresh endpoint precede
release. Initial pose, covariance, biases and zero motor speeds match the saved
aligned baseline. The first accelerometer sample retains the supported
left-limit force; every later sample describes free flight. The changed motion
changes physical sensor signals and hence estimator trajectories. We preserve
the underlying random draws, not falsely require identical raw measurements.

## Complete outcomes

| Metric | Saved baseline | Navigation oracle | Original condition |
| --- | ---: | ---: | --- |
| Hover peak, cm | 10.180776 | 1.989620 | <=8 |
| Whole-flight RMSE, cm | 6.489135 | 3.925627 | <=15 |
| Final position error, cm | 1.217038 | 0.744815 | <=15 |
| Final true speed, cm/s | 2.135696 | 0.136219 | <=15 |
| Flight duration, s | 15.5 | 15.5 | Complete |
| Maximum thrust-axis estimation error, degrees | 0.133470 | 0.133861 | Descriptive |

The scored hover interval remains inclusive 5..11 s. Whole-flight scores retain
the entire support-release/motor startup transient. The oracle peak occurs at
5.76 s with NED error `[1.389820, -1.408474, 0.207822] cm`. No window, threshold,
noise scale or seed was changed and no valid outcome was retried. The slight
increase in maximum attitude-estimation error is retained; improving physical
tracking does not mean every estimator metric improves.

The earlier [offline response decomposition](residual-supported-hover.md) held
all error histories fixed. This experiment actually changes the coupled
nonlinear motion and resulting measurement history. Its passing result supports
the navigation-channel explanation without assuming that response components
can simply be subtracted from the original trajectory. The combined position
and velocity intervention does not identify their individual causal effects or
prove an estimator implementation defect.

## Verification and limits

The saved baseline is authenticated and fully reconstructed without rerunning
its flight. After the one new flight is written, all history chunks are
authenticated and decoded before scoring. Both saved histories reproduce every
ESKF state/covariance/event, command, guard, health and supervision decision;
the oracle verifier substitutes only the documented outer arguments. It checks
all 775 outer input records against the simultaneous saved truth and estimates.
No terminal-epoch command is permitted.

Every nonlinear plant and motor reconstruction residual is zero. Independent
named-stream reconstruction differs by at most 6.971e-15; common-prefix
accelerometer/gyro/slow-sensor draw differences are at most 7.106e-15, below the
predeclared 1e-12 tolerance. Bias walks and initialization agree exactly. Full replay
after saving does not execute another scientific flight.

This is one selected failed case, not a reliability population. Truth feedback
is unavailable to an ordinary controller, and the experiment supplies no new
sensor, hardware guarantee or general qualification. Mass mismatch remains a
separate open requirement; geometric qualification remains unchanged.

## What this implies for a supported zero-velocity prior

The candidate identified by this experiment uses information already promised
by the supported operating boundary: the vehicle is stationary in the world frame through the
fresh release endpoint. The current navigation prior retains a 0.03 m/s velocity
standard deviation from the free-flight initialization even under that support
contract. Its mean is already zero; this is about conditioning uncertainty using
a valid constraint, not replacing an estimated velocity with truth during flight.

For a prior error covariance `P`, velocity selector `H_v`, prior nominal velocity
`v_bar` and independent support-velocity uncertainty `R_s`, the conditioning is

```text
S = H_v*P*H_v^T + R_s
K = P*H_v^T*S^-1
delta_x = K*(0-v_bar)
P_conditioned = (I-K*H_v)*P*(I-K*H_v)^T + K*R_s*K^T.
```

The candidate is restricted to the current independent navigation/alignment
profile; this conditioning does not inject an attitude correction.

Under the existing ideal fixture, `v=0` is an exact boundary condition. With its
independent navigation block and zero mean, `R_s=0` gives zero initial velocity
variance and leaves position, alignment, bias and fresh-sample blocks unchanged.
That covariance is positive semidefinite and rank deficient: it represents some
exactly known directions and cannot be inverted as a full-rank matrix. An
arbitrary positive floor would change the stated constraint. A real support
procedure with nonzero residual motion needs externally justified `R_s`; IMU
quietness or this oracle's score cannot supply that justification.

The [supported velocity prior screen](supported-velocity-prior.md)
evaluates this candidate. Its conditioning,
rank and support rejection checks pass, but all 17 release configurations fail
the necessary uncertainty screen. The supported left-limit sample causes a
deterministic integration error much larger than the conditioned prior's first
propagated uncertainty. Under the predeclared rule, no new mission flights run.

The [release-aware first prediction](../design/release-prediction.md) and
[nonlinear release uncertainty](nonlinear-release.md) address that boundary.
The [combined-prior comparison](combined-supported-prior.md) evaluates them with
the velocity constraint. The oracle's 80.46% improvement is diagnostic headroom,
not a prediction of what a one-time velocity constraint will achieve.
Independent validation remains necessary before ordinary mission integration.

## Reproduce

Use the pinned repository environment and the complete independent campaign:

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.navigation_feedback_oracle --campaign SAVED_INDEPENDENT/campaign --output NEW_ORACLE
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.navigation_feedback_oracle --campaign SAVED_INDEPENDENT/campaign --verify NEW_ORACLE
```

The output directory must be new. Verification requires the recorded source
fingerprint and reconstructs saved evidence without a flight. The command exits
0 for useful diagnostic headroom and 1 for a valid failed comparison; neither
exit status qualifies truth-assisted feedback for deployment. The separate
evidence bundle includes a NumPy-only verifier for byte identities, paired
metrics, outer dataflow, initial parity and common noise draws.
