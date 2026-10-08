# ADR 0024: one bounded vertical disturbance-compensation candidate

The vertical compensation candidate is implemented and evaluated, but rejected. The
[complete record (ZIP)](../../evidence/development-records.zip) retains all
24 executions, 12 passing response cases, and the mass-RMSE and hover comparison
failures. The frozen policy and acceptance conditions below are unchanged.

## Motivation

A proportional–derivative (PD) controller can settle with a nonzero position
error when its assumed mass is too small. Integral action accumulates that
error to supply the missing steady force. This study tests one bounded vertical
integrator while keeping the existing PD gains and estimator fixed.

The saved baseline mass case has report SHA-256
`ec85a0d6b55acc278acb18b1c7af7b02cb56c6e985dc48eaf66fbc7fd520bb09`.

The 1.1 kg truth / 1.0 kg nominal case has mean vertical error 0.436975 m over
17..25 s; the last two seconds' mean estimated-minus-true vertical position is
only 0.001055 m. There are zero outer or inner limit flags. Commanded thrust
ranges 9.81..11.317405 N. The nominal PD equilibrium predicts
`(1.1/1.0 - 1)*9.81/2.25 = 0.436 m`, matching the persistent offset.

The earlier bounded-integral prototype targeted horizontal startup response
and reached its 0.5 m/s² bound while failing the hover requirement. The vertical
candidate addresses the steady-force deficit with one independently specified
scalar state. Estimator, references, PD gains and geometric control stay fixed.

## Candidate and state contract

The explicit opt-in vertical integral acceleration `I` uses NED m/s²:
`a_command = a_PD + [0, 0, I]`, with `I_dot = 0.5*(z_reference-z_estimate)`.
The existing position-control feasibility projection and all bounds also apply
to the compensated command.
The integral gain is **0.5 s^-3**, update period **0.02 s**, and state bound
**±1.5 m/s²**. Initialization and reset set the integral state to zero. No mass or fault label is an input.

The gain follows `(s+0.5)^2*(s+2) = s^3+3s^2+2.25s+0.5` for the nominal
unsaturated vertical model with the existing PD gains. The 1.1 kg case with
0.025 s linear motor lag has continuous poles approximately -37.1202, -1.8277,
-0.6195 and -0.4326 s^-1. This is a design rationale, not a nonlinear proof.
The state bound exceeds the required -0.981 m/s² steady correction without
changing the existing ±2 m/s² acceleration limit. The study evaluates this one
gain without a parameter sweep.

At each outer epoch, the current integral state contributes to the command.
Forward Euler then computes the next state from the current estimated error.
Learning occurs only when both observation streams are HEALTHY. WAITING,
DEGRADED, LOST and RECOVERING freeze the state; confirmed recovery resumes
learning. Compensation persists across phase changes and observation loss,
avoiding an abrupt reset.
Terminal mission guards execute before controller calls. An attitude-domain
abort may retain a computed-but-unused outer attempt, explicitly diagnostic.

Integration freezes when the previous inner command had rate, moment or
allocation limiting. At an outer acceleration/thrust limit, an increment is
rejected only if it drives further into the vertical limit; unwinding remains
allowed. A 1e-12 m/s² tolerance distinguishes a projected vertical limit from
roundoff. The proposed integral state is then clipped to its declared bound. This is bounded
conditional integration, not a complete actuator-dynamics anti-windup proof.
Validation rejects wrong clocks, nonfinite inputs, reused state, controller
mismatch, disabled/missing health streams and geometric combination without
partially updating state.

Mission and saved-history dataclasses are unchanged. A separate authenticated
trace records applied/next integral state, error, health/limit flags and update
reason. Replay reconstructs that trace and every affected command.

## Evaluation and acceptance

Reuse all twelve ADR 0023 cases, both supervision modes, original seeds, priors,
plant/sensor clocks, fault plans, references, controller gains, response budgets
and flight limits. The **24 candidate executions** use at most two process
workers and single-threaded BLAS. The comparator is the authenticated, immutable
24-execution baseline above, identified by its complete report and evidence
hashes rather than replaced by a newly generated baseline.

Candidate acceptance requires zero numerical/evidence failures, all twelve
inherited response checks, mass-case completion within every inherited flight
limit, and preservation of the existing passing nominal-tracking, recovery and
wind flight gates. For the already failing hover, retain every previously
passing flight condition and require its peak not to exceed the baseline peak
(1e-12 m arithmetic tolerance). Its unchanged 8 cm gate remains a failure until
actually passed. Report every metric delta; retaining an inherited pass is not
a claim that every scalar metric improved. Require bounded compensation,
correct freeze/unwind/reset behavior and no terminal command.

The canonical campaign protocol SHA-256 is
`6bab1e089730485310a9d69a3d49361618c73982663b75f2a9efe8fc9fa9635e`,
anchored in tests before the candidate maneuver campaign.

The rejected candidate remains an explicit experimental option for reproduction.
It is not the default controller, a qualified whole-flight solution or a physical
fallback procedure. Its measured failures retain the original acceptance limits.

Software acceptance includes independent sign/step/anti-windup tests, strict
causal and invalid-call checks, unchanged default payloads, exact saved ESKF,
health, mission, controller and compensation-trace reconstruction, rejection of
changed protocol/claims/trace, and retained numerical failures. Short smoke
fixtures check these paths in CI; they are not flight qualification. Full
experimental histories are kept separately from the source repository.
