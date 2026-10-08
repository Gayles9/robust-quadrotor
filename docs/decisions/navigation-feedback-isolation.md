# Isolating navigation feedback

Saved-error accounting suggests that position and velocity estimation contribute
to the failed hover. This experiment checks that hypothesis by giving only the
outer controller perfect simulated position and velocity. This **oracle** is a
diagnostic comparison, not an implementable estimator or a flight
qualification.

## One controlled intervention

The comparison adds one complete supported-aligned nominal-hover flight at seed
47001 with supervision enabled. Its control is the saved aligned flight from
[independent supported-start validation](independent-supported-start-validation.md).
The campaign report SHA-256 is
`b47297b76114f046915fc167ac114287bec3b15a195748bbde73151a41a70f63`;
the preceding diagnosis report is
`949375db59e656659f8c7378060094b427a2958c1a17ccbc211cff0fa1efe928`.

At each 20 ms outer-controller epoch, the intervention replaces its position
and velocity inputs with simultaneous simulated truth. Everything else retains
its original behavior:

- Estimator equations, initialization, covariance, actual measurements,
  sample ownership and causal observation processing.
- The estimated observer tuple used by guards and completion, and estimated
  attitude/body rate used by the inner controller.
- Controller gains, plant/motors, reference, limits, health and supervision.
- Supported acquisition, the fresh supported endpoint, release from zero motor
  speeds, full startup transient, named random streams and bias walks.

Changed motion changes sensor signals and estimator trajectories. Pairing
therefore reconstructs the random draws after removing physical signal and
bias, rather than requiring equal raw measurements. Both first accelerometer
samples use the supported left-limit force.

There is no true attitude/rate feedback, ideal sensor, altered guard, estimator
reset, command smoothing, mass compensation or geometric controller in this
comparison. The process-local adapter restores on exceptions and does not
share a process with another mission thread. Seed 47821 belongs only to smoke
checks of data flow, not performance evidence.

## Verification and acceptance

Saved payloads include the complete outer-input trace. Reconstruction covers
every estimator state/covariance/event, changed outer command, unchanged inner
law, estimated guard/completion decision, health/supervision transition and
plant/motor interval. Command and estimator data flow must match exactly;
plant/motor and named-random-draw residuals must be at most 1e-12. The baseline
is reconstructed too, and there is no terminal-epoch command.

Every outer epoch is checked against simultaneous saved truth and estimates.
The mission must receive its original estimated observer tuple. Checks cover
normal and exceptional restoration, wrong channel or epoch, tampering and saved
replay.

Scoring starts at flight zero, with the inclusive 5..11 s hover window. Limits
remain 0.08 m hover peak, 0.15 m whole-flight RMSE/final-position error,
0.15 m/s final true speed, and completed mission. Useful navigation-channel
headroom requires all of those conditions, a strictly smaller hover peak, and
whole-flight RMSE no greater than baseline plus 1e-12 m.

A partial improvement or failure remains informative without extra runs or
changed thresholds. Even a passing oracle cannot establish that an estimator
using the actual sensors can achieve the same result. The subsequent
[supported velocity constraint](supported-velocity-prior.md) tests a specific
implementable source of information under explicit support assumptions.
