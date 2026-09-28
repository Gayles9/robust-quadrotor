# Controller problems and tradeoffs

The main unresolved problem is keeping the vehicle sufficiently close to its
reference when control uses noisy state estimates. The geometric controller
performs well in the tested true-state flights. With the ESKF in the loop,
hover excursions exceed the original 8 cm limit and its feedforward can demand
much more moment than the cascade.

I have kept the original requirements and failed cases. The aim is to explain
what causes the tradeoff before choosing another design. The evidence currently
supports several interacting mechanisms; it does not identify a single defect
whose removal would guarantee the required performance.

## What the requirements measure

The hover test checks the largest position error at every saved sample from
5 to 65 seconds. A brief excursion can fail it even if the rest of the hold is
accurate. Spline tracking uses root-mean-square error (RMSE), which summarizes
error over the complete flight. The geometric controller must not regress
against its paired cascade, and its effort must stay within twice the cascade's.

Effort here means the time integral of squared actual body moment, in N² m² s.
It is a useful comparison of torque demand, not a measurement of battery energy
or motor temperature. Mission completion, tracking accuracy, peak hover error,
effort and limiting are evaluated separately.

## Why an estimator correction can disturb the controller

The ESKF predicts motion between measurements. When a position or altitude
observation arrives, it revises that estimate. For example, correcting the
estimated position by a centimetre does not mean the physical vehicle moved a
centimetre at that instant. It means the filter changed its estimate of where
the vehicle already was.

The geometric reference uses position and velocity errors to form a desired
force. It also needs the force's rate of change and second derivative to
compute desired angular velocity and acceleration. Differentiation reacts
strongly to sudden changes, including estimator revisions. The resulting
angular-acceleration feedforward can request sharp moments even when the true
motion is smooth.

The audit reconstructed saved moments from measurement-derived estimates to
within about 1.12e-16 N m. This confirmed the path producing the commands.
Removing identified correction jumps in an offline reconstruction reduced the
acceleration-feedforward moment RMS substantially. That diagnosis motivated a
closed-loop experiment; the reconstruction alone did not establish an improved
flight. See the [audit](progress/2026-09-26-geometric-project-audit.md) for the
exact calculations.

## Why more filtering or higher gains are not automatic fixes

The derivative filter smooths rapid changes, but also delays the response. Its
three sections at 30 rad/s have approximately 0.1 seconds of low-frequency
delay. The raw position-feedback force is not filtered, so its derivatives and
value do not describe exactly the same instantaneous reference.

Motor response and attitude dynamics add delay of their own. The outer position
loop can request a correction before the vehicle has finished responding to the
last one. Raising gains can reduce tracking error, but may also amplify
measurement changes, increase moment demand or reduce damping. A faster update
rate does not by itself make the physical response faster.

Startup is another part of the problem. A small attitude-estimation error tilts
the commanded thrust in the wrong direction. The estimate may improve after
new observations, while the physical vehicle still needs time to recover the
velocity and position error already accumulated. At one failing hover peak,
position-estimation error was only about 7.7 mm while true position error was
13.7 cm. That observation shows why looking only at the estimator error at the
peak is insufficient; the preceding coupled response matters.

Persistent wind is a separate issue. The current position law has proportional
and velocity feedback, with no integral disturbance estimate. It needs a
position offset to generate the force balancing a persistent load. Integral
action could address that offset, but adds state, reset and anti-windup design
and does not automatically solve startup transients. Earlier bounded integral
probes also failed to close the qualification gap.

## What I tested

The study examined two derivative strategies and a small set of justified gains:

- **Correction rebasing:** translate the filter memory by the known force change
  from each accepted estimator correction. Its existing derivative state is
  preserved, while the raw feedback still uses the full revised estimate.
- **Measured physical derivatives:** estimate acceleration from measured
  specific force, posterior attitude and bias, then filter its rate to obtain
  jerk. This avoids twice differentiating corrected position/velocity, but
  retains sensor noise, attitude uncertainty and filtering delay.

Rebasing was tested at three horizontal bandwidths, plus two profiles with
higher attitude stiffness derived from an ideal critical-damping calculation.
The measured approach used the original gains. All six candidates failed the
combined requirements.

Representative results show why choosing a replacement from one metric would
be misleading. All spline rows use seed 30; the paired cascade RMSE is 7.4059 cm.
Hover columns are maxima over the complete required hold interval.

| Design | Spline RMSE [cm] | Effort / cascade | Hover seed 30 [cm] | Hover seed 93012 [cm] |
| --- | ---: | ---: | ---: | ---: |
| Original geometric filter | 6.0517 | 10.860 | 8.7649* | 13.6855 |
| Rebasing, original gains | 8.3764 | 1.136 | 11.7185 | 17.1268 |
| Rebasing, horizontal frequency 2 rad/s | 4.6956 | 5.213 | 8.8138 | 15.5239 |
| Measured acceleration/jerk, original gains | 8.5360 | 1.824 | 11.9179 | 17.4527 |
| Required | ≤7.4059 | ≤2.000 | ≤8.0000 | ≤8.0000 |

*The original seed-30 hover value comes from the preceding implementation
record; the other original values were reproduced in the audit. These are
observed development cases, not independent validation of the new designs.
The [full results](progress/2026-09-26-geometric-project-audit.md#bounded-development-results)
include both remaining gain profiles.

Rebasing with the original gains nearly meets cascade effort, but worsens
tracking and hover. Raising the horizontal frequency to 2 rad/s improves spline
RMSE by 22.4% and reduces effort by 52.0% relative to the original geometric
design, yet effort is still 5.21 times the cascade and hover is worse. The
measured-derivative option meets the effort ratio but regresses tracking and
hover. None is an acceptable overall replacement.

## What remains supported

The original geometric controller passed seven true-state comparisons, a plant
timestep refinement check and an exact repeat. Its RMSE was 0.20–3.75 cm across
those cases. This supports the implementation and its bounded true-state
performance. It does not resolve the noisy-feedback failures.

The cascade remains the default; explicit `design_version=2` is the limited
estimated-feedback comparison profile. Its original 28/30 hover result and
version 1's 29/30 result also retain their misses. No current profile satisfies
every original estimated-hover condition.

The subsequent [startup/hover investigation](progress/2026-09-26-geometric-transient-closeout.md)
tested one further mechanism: filtering the force value together with its
derivatives at a frozen 10 rad/s cutoff. Spline RMSE was 6.77 cm and effort fell
to 0.85 times cascade, but full-hover maxima worsened to 10.87 and 17.33 cm.
The shaped reference reduces sharp commands while adding delay to the physical
recovery. This is a useful diagnosis, but fails the unchanged joint requirement.

That bounded study is now closed, with no further gain or cutoff search and no
reserved validation seeds opened. The cascade default and original geometric
controller remain unchanged. [Observation health monitoring](observation-health.md)
now reports persistent accepted-data loss. The [next plan](next-steps.md) defines
supervisor responses as a separate task; qualification is still open.
The [geometric guide](geometric-control.md) contains the equations and API, and
[cascade design](feedback-design.md) explains the coupled local model.
