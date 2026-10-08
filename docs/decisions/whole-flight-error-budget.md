# Whole-flight estimation and control error budget

An error budget asks which estimation errors matter most to tracking and how
they interact through the controller. This review combines exact accounting of
saved flights with a local linear model of sensitivity. It does not simulate
an improved flight by deleting inconvenient error terms.

## Evidence and fixed requirements

The analysis uses all 27 authenticated response/update records from the
[combined-prior diagnosis](combined-prior-tradeoff-diagnosis.md): nine jobs and
three arms, with full durations and failed outcomes retained. Input report
SHA-256 is
`09c53c8e86f94f16b685fa6f297e54ecd4bcc3a7e78cb7c806217ed020910276`;
its execution fingerprint is
`01d663368ed67174c9b300fdd2ef7450781da5f6ec3947e7f9a9b2d8473f2b0e`.

Original requirements remain 8 cm hover peak, 15 cm complete-flight RMSE and
final position, 15 cm/s final speed and strict paired no-regression conditions.
Sensor assumptions, clocks, gains and limits are unchanged.

## Accounting for correlated errors

For every flight and mission window, the uncentred response Gram matrix contains
the average inner product of every pair of response channels. Its diagonal
terms and signed cross terms recover tracking mean-square error. This preserves
cancellation and reinforcement between correlated channels instead of treating
them as independent variance sources.

Triangle bounds are reported separately from measured scores. They are
conservative bounds, not evidence that an individual contribution can be
removed while leaving all other closed-loop signals unchanged.

## Local sensitivity model

The near-level horizontal cascade is lifted over one original outer-controller
period: the inner updates and intersample outputs are retained inside the
larger step. Its gains, inertia, motor lag and clocks come from the implemented
controller.

The model gives frequency sensitivity to position, velocity, inclination and
inclination-rate errors, along with finite-horizon worst-case input bounds over
the 5..11 s hover window. The frequency grid is DC plus 241 logarithmic points
from 0.01 to 25 Hz. No gain or cutoff is optimized.

Analytic DC sensitivities and time-domain evolution check the lifted model.
Bounds use absolute impulse coefficients and retain both horizontal axes.
Single-input ceilings against the 8 cm limit are screening values only: they
are not a simultaneous error allocation, leave no allowance for vertical or
physical residual error, and cannot qualify a flight.

Accelerometer and gyro biases stay within the navigation, inclination and rate
errors they induce; they are not counted again as independent sources. A
level-hover nullspace calculation compares existing position/altitude
observations with hypothetical direct velocity and independent two-axis
inclination information. A nullspace identifies locally indistinguishable
perturbations. It does not prove nonlinear observability. Original innovation
rejections and paired branch changes are retained because a linear sensitivity
cannot predict a changed observation-gate sequence.

## Verification and interpretation

Score and Gram identities must close within 1e-12 m², and response sums within
1e-10 m. Lifted and time-domain trajectories agree within 1e-10 on deterministic
fixtures; DC gains agree within 1e-10. Adversarial signs check impulse-bound
attainment, and direct sampled evolution checks frequency phasors. Independent
null-vector/rank tests also cover signs, clocks, correlated cancellation,
invalid inputs and evidence tampering.

The analysis introduces no new scientific flight, sensor assumption, controller,
covariance estimate or seed search. A possible measurement contract still needs
an available source and justified noise, bias, timing and correlation assumptions.
Unknown physical information remains unknown. The
[independent-inclination feasibility study](independent-inclination-feasibility.md)
examines that boundary; mathematical sensitivity alone is not improved flight
performance.
