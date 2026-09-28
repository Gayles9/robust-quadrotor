# Observation loss supervision

The [health monitor](observation-health.md) reports whether accepted position
and altitude observations remain available. The optional
[`ObservationSupervisor`](../src/quadrotor_math/observation_supervision.py)
uses that evidence to stop a numerical mission after a declared unhealthy-time
budget expires. This makes persistent loss actionable without giving the
supervisor simulated truth or claiming that fresh data proves accuracy.

## Requirements and action

| Mission phase | Local position | Barometric altitude |
| --- | --- | --- |
| INITIALIZE hold | Required | Required if its timeout is supplied |
| TAKEOFF | Required | Required if its timeout is supplied |
| TRACK | Required | Required if its timeout is supplied |
| LAND and completion dwell | Required | Required if its timeout is supplied |
| COMPLETE or ABORT | Execution has ended | No further commands |

The current position controller runs in every active phase, including
initialization. A phase change does not clear a loss timer or pause the
reference schedule. Local position stays mandatory; accepted altitude cannot
replace horizontal information. Set `barometric_altitude_timeout_s=None` to
make altitude explicitly optional. Its health is still recorded, including
DISABLED, but it cannot veto the mission. Requiring a disabled fusion stream
is a configuration error, caught before running.

The policy permits continued execution within each budget. A sufficiently short
mission can therefore complete before a startup timer expires; this interface
does not add an alignment or takeoff-readiness gate.

The response is the existing numerical ABORT: retain the terminal state and
observations, issue no command at that epoch, and stop the simulation. It is
not a hardware motor command, touchdown procedure or demonstrated safe landing.
The current virtual landing still uses position/velocity feedback, so it is
not a justified fallback after losing that information.

## Timing and recovery

Each required stream has its own positive timeout in seconds. The API has no
default timeout. These budgets are separate from the monitor's warning/loss
ages, and must be chosen for the declared simulation protocol.

- Initial WAITING spends the budget from the first observed epoch.
- After a healthy period, the first observed DEGRADED, LOST or RECOVERING
  snapshot starts a new timer. Those states share one continuous budget.
- Partial recovery, rejection, stale delivery and a phase change do not
  restart the timer. Only HEALTHY **before** expiry clears it.
- At or after `unhealthy_since + timeout`, abort wins. Even a HEALTHY snapshot
  at that deadline cannot erase the expired interval. The test uses the stored
  floating-point clock and the first sampled epoch meeting that comparison.
- Abort remains latched until explicit reset. Position wins simultaneous
  observation deadlines. Existing truth safety guards retain first priority,
  followed by estimated-state safety guards, then this observation guard.
  Observation abort precedes landing completion and command computation.

The response time is measured from detected unhealthiness, not directly from
the first missing packet. For uninterrupted silence after a healthy acceptance,
age detection occurs after the monitor's warning threshold, then the response
budget elapses. A regular clock adds up to one sampling interval to each
comparison. Initial silence instead spends its response budget immediately.
Late observations remain stale under the current ESKF; delivery alone does not
make them usable.

## Use and evidence

With `arguments` and `monitor` configured as in the health guide, add a bound
supervisor. These illustrative budgets suit that API example's sensor periods;
they are not vehicle safety limits:

```python
from quadrotor_math.observation_supervision import (
    ObservationSupervisionPolicy,
    ObservationSupervisor,
)

monitor.reset()
supervisor = ObservationSupervisor(
    monitor,
    ObservationSupervisionPolicy(
        local_position_timeout_s=0.6,
        barometric_altitude_timeout_s=0.2,
    ),
)
result = simulate_estimated_mission(
    **arguments,
    observation_health=monitor,
    observation_supervision=supervisor,
)
assert len(supervisor.history) == len(result.mission.time_s)
print(supervisor.latest)
```

For direct use, call `monitor.step(time_s, events)` and then `supervisor.step()`
at every estimator epoch. Binding to the monitor avoids accepting an arbitrary
user-constructed health record. Missing, repeated or skipped epochs are refused
without consuming supervisor memory. Policies and decision records are
immutable. For another run, reset the monitor first, then the supervisor.
Both must be fresh and bound together at mission preflight.

The decision history stays outside the existing mission result and saved
archive schema. It contains the clock, both timer origins and the latched
reason. A delivered-event replay of an authenticated mission reproduces it
exactly. If numerical execution raises, only the observed diagnostic prefix
remains; no partial mission result is returned. Histories grow with run length
and are intended for simulation, not embedded real-time memory management.

The [decision record](decisions/0022-observation-loss-supervision.md) freezes the
acceptance protocol; the [verification record](progress/2026-09-28-observation-loss-supervision.md)
contains the results. Core fault tests use the real online ESKF with explicit
dropouts, offsets and delayed deliveries. Closed-loop tests use short stationary
phase fixtures with independent stale/rejected streams and both controllers.
They verify response correctness and unchanged nominal behavior, not general
robust flight. The separate [integrated campaign](integrated-robustness.md)
evaluates actual maneuvers, causal live faults and model mismatch with paired
histories and unchanged controller settings. Its response and flight results
remain separate criteria.
