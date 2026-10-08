# Experimental vertical disturbance compensation

The original position controller needs a position offset to balance an
unmodeled constant load. In the integrated robustness campaign, a 10% mass
increase produced about 43.6 cm of vertical error, accurate state estimates,
no command limiting, and a landing timeout. This optional experiment adds one
bounded vertical integral state to address that steady-force deficit.
An integral accumulates position error over time, allowing the controller to
learn the extra steady force needed. The
[study protocol](../decisions/0024-bounded-vertical-compensation.md) defines
the candidate and acceptance criteria; the
[verification record (ZIP)](../../evidence/development-records.zip) preserves
its measured outcome. The original cascade remains the default.

The completed study **rejects this candidate**. It completes the heavier-mass
landing and reduces final error to 6.02 cm, but its 17.57 cm whole-flight RMSE
misses the 15 cm limit. Hover also slightly regresses and remains above 8 cm.
All twelve response cases pass. The interface remains an experimental option;
it is not the default controller and does not meet the full flight requirements.

## Law and rationale

With NED position, the new state is an acceleration correction:

$$
a_{command}=a_{PD}+[0,0,I]^T, \qquad
\dot I=K_i(z_{reference}-\hat z), \qquad |I|\le I_{max}.
$$

An aircraft below its reference has a negative reference-minus-estimate error,
so the correction becomes negative: more upward thrust. The existing feasible
acceleration, tilt, thrust and motor constraints still apply. The correction
uses only estimated position and nominal controller parameters, not true mass.

This single candidate fixes `Ki=0.5 s^-3`, `Imax=1.5 m/s²` and an outer update
period of 0.02 s. With the existing vertical gains `Kp=2.25 s^-2` and
`Kv=3 s^-1`, the nominal ideal characteristic polynomial factors as
`(s+0.5)^2*(s+2)`. The correction needed to balance the tested extra mass is
-0.981 m/s², within the state bound. A separate exact held-input linear motor
model checks sampled stability and constant-load rejection. Neither local
model replaces the noisy nonlinear flight campaign.

## Causality, limiting and lifecycle

At each outer epoch the command uses the current integral. Forward Euler then
prepares the next integral from the current error. Both monitored observation
streams must be HEALTHY before learning. Startup, loss and unconfirmed recovery
freeze the value without accumulating a hidden backlog. Confirmed recovery
resumes learning; phase changes retain the correction through virtual landing.

The previous inner command's rate, moment or allocation limit also freezes
learning. An outer vertical limit blocks an update that pushes further into
saturation while permitting an update that unwinds it. The final state is
clipped to its bound. This prevents the specified forms of windup, but does not
model every transient actuator or attitude-tracking error.

Use a fresh object for each mission or call `reset()`. Wrong clocks, nonfinite
inputs and invalid flags fail atomically. Mission preflight requires matching
controller parameters, a matching outer period, enabled position/altitude fusion
and their health monitor. Geometric control cannot be combined with this option.
Terminal mission guards precede controller calls. The diagnostic trace can
retain an outer attempt discarded by a later attitude-domain guard; it is not
a commanded sample.

## Interface and evidence

```python
from experiments.robustness_protocol import configuration, policies
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.observation_health import ObservationHealthMonitor
from quadrotor_math.observation_supervision import ObservationSupervisor
from quadrotor_math.vertical_compensation import (
    VerticalCompensationPolicy,
    VerticalIntegralCompensator,
)

args = configuration("mass_tracking", "campaign")
health_policy, response_policy = policies(args, "campaign")
monitor = ObservationHealthMonitor(health_policy)
compensation = VerticalIntegralCompensator(
    args["position_controller"], VerticalCompensationPolicy(0.5, 1.5, 0.02)
)
result = simulate_estimated_mission(
    **args,
    observation_health=monitor,
    observation_supervision=ObservationSupervisor(monitor, response_policy),
    vertical_compensation=compensation,
)
```

Passing no compensation object preserves the original behavior. The existing
mission and history dataclasses are unchanged. The separate trace records
applied/next correction, vertical error, observed health, previous inner
limiting, update reason and state clipping. It is authenticated and reproduced
from the saved measurements, estimates and commands alongside the existing
ESKF, mission and supervisor audit.

The candidate runner reuses all twelve cases and both supervision modes. Its
baseline is the exact audited report and full histories from the
[integrated robustness campaign](integrated-robustness.md), reauthenticated
before comparison. It performs 24 new candidate flights;
it does not silently replace the saved baseline or open fresh seeds.

```bash
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.vertical_compensation_validation --baseline results/robustness-campaign --partition campaign --workers 2 --output results/vertical-candidate
OPENBLAS_NUM_THREADS=1 PYTHONWARNINGS=error uv run python -m experiments.vertical_compensation_validation --baseline results/robustness-campaign --verify results/vertical-candidate
```

The baseline directory is the extracted original campaign evidence. Use a new
candidate output directory. The output retains that baseline report, its exact
identity, the predeclared candidate protocol and all new histories. Verification
requires both directories and the recorded candidate execution source. The
original baseline's hashes remain strict even though it came from an earlier
source commit. Its complete campaign was already reconstructed; smoke baselines
are reconstructed again by the comparison checker.

`candidate_accepted` describes only the predeclared vertical study.
`flight_campaign_passed` remains the separate original campaign decision.
Preserving a passing gate does not mean every scalar metric improved; full
metric differences remain in the report. A failed candidate or a smoke run
returns status 1 with evidence retained. No result automatically changes the
default controller or closes the separate hover, geometric or hardware gates.

Source: [scalar law](../../src/quadrotor_math/vertical_compensation.py),
[mission integration](../../src/quadrotor_math/estimated_mission.py),
[campaign](../../experiments/vertical_compensation_validation.py),
[evidence reconstruction](../../experiments/robustness_evidence.py).
