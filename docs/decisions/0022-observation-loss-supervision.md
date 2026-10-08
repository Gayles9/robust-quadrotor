# ADR 0022: bounded supervision of observation loss

## Purpose and scope

The passive health monitor reports missing or rejected observations. The
supervisor adds a decision: stop the numerical mission if a required stream
remains unhealthy beyond an explicit time budget. It uses only the monitor's
clock and health states, so it cannot anticipate a fault or inspect simulated
truth. It is optional and does not change the estimator or controller equations.

## Supervision policy

- Local position is required in every active phase: INITIALIZE, TAKEOFF,
  TRACK and LAND. The position controller already runs during initialization
  hold. Phase changes never restart the availability budget.
- Barometric altitude is independently required when its timeout is a number.
  An explicit `None` makes it optional. Required fusion cannot be disabled;
  reject that configuration before running the mission. Position remains
  mandatory because altitude does not establish horizontal availability.
- Each required stream has an explicitly supplied, positive timeout in
  seconds. There is no default flight deadline. WAITING starts its timer at
  the initial observed epoch; DEGRADED, LOST and RECOVERING share one timer
  starting at their first observed unhealthy epoch after a healthy period.
  Neither receiving a rejected/stale observation nor incomplete recovery
  restarts that timer. LOST does not introduce a second deadline.
- A HEALTHY snapshot clears the timer only **before** its deadline. At or
  after the deadline, abort wins even if recovery arrives in that epoch.
  Evaluate deadlines before clearing timers so a skipped wall-clock interval
  cannot hide an expired budget. The supervisor must consume every monitor
  epoch exactly once, in order.
- Abort is latched until explicit reset. Simultaneous observation deadlines
  give local position priority. Existing truth and estimate safety guards
  retain their priority; observation abort precedes completion and command
  computation. Record the terminal epoch and issue no terminal command.
- The action is the existing **numerical ABORT**, not a motor shutdown or a
  hardware emergency procedure. A virtual landing also needs a usable
  position estimate, so this step does not invent a loss-triggered landing.
- Supervisor inputs are the bound monitor's clock and health states only.
  Neither truth, future fault labels, measurements nor estimate accuracy
  enter this decision. Diagnostics remain caller-owned, outside the existing
  mission/evidence dataclasses. Legacy calls remain unchanged.

## Acceptance and deterministic protocol

1. Invalid or disabled-required configurations and skipped/repeated monitor
   epochs are rejected without changing supervisor state. Tests cover reset,
   immutability, independent clocks, deadline equality, recovery before/at
   expiry and latched abort after recovery.
2. Deterministic observation dropouts, offsets and delays drive the real online
   ESKF. Cases cover an isolated outlier, persistent rejection, independent
   stream loss, incomplete/full recovery and identical prefixes with different
   future faults. Neither the monitor nor the supervisor receives fault labels.
   This component protocol does not establish feedback flight performance
   under dropout.
3. Public closed-loop integration tests use seed 31, a short initialization/
   takeoff/track/virtual-land plan, both controllers, fixed sensor schedules
   and explicit independent delayed/rejected streams. Comparisons cover every
   legacy numerical/event payload for nominal equivalence, unchanged command
   prefixes, phase boundaries, guard priority, abort before completion and
   absence of commands at terminal epochs.
4. The existing digest boundary authenticates saved complete mission histories.
   Delivered events and clock alone reconstruct health/supervisor decisions;
   the evidence format is unchanged.
5. Warning-strict software checks complement the component and integration
   checks. Existing flight-performance limitations remain separate from
   whether the supervisor implements its timeout policy correctly.

The test budgets are bounded simulation examples. Their suitability for a real
vehicle, all model mismatches, heading observability, or safe landing is not
established by this acceptance protocol.
