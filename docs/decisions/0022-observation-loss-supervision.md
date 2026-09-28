# ADR 0022: bounded supervision of observation loss

Status: accepted for the numerical mission interface, 2026-09-28.

## Audit and scope

I audited `d01e440594c4a588f78291455242618e6db5e41b`, including the
passive observation monitor, mission phase transitions, guard priority and
terminal command handling. The preceding merge's GitHub CI passed. Fresh
monitor/integration/mission checks passed 103 tests in 21.77 s. I found no
in-scope defect in the preceding milestone.

This step adds an opt-in, causal observation supervisor. It does not tune the
estimator or either controller, add automatic alignment, or qualify flight.

## Policy frozen before implementation

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

1. Reject invalid/disabled-required configurations and skipped/repeated
   monitor epochs; preserve state on failed supervisor calls. Verify reset,
   immutability, independent clocks, deadline equality, recovery before/at
   expiry, and latched abort after recovery.
2. Drive the real online ESKF with explicit, deterministic observation
   dropouts, offsets and delays. Cover an isolated outlier, persistent
   rejection, independent stream loss, incomplete/full recovery and identical
   prefixes with different future faults. Keep fault labels out of the
   monitor and supervisor. This component protocol does not claim feedback
   flight performance under dropout.
3. Exercise public closed-loop mission integration with seed 31, a short
   initialization/takeoff/track/virtual-land plan, both controllers, fixed
   sensor schedules, and explicit independent delayed/rejected streams.
   Compare every legacy numerical/event payload for nominal equivalence;
   verify unchanged command prefixes, phase boundaries, guard priority,
   abort before completion, and absence of commands at terminal epochs.
4. Authenticate saved complete mission histories using the existing digest
   boundary and reconstruct health/supervisor decisions solely from delivered
   events and clock. Do not change the existing evidence format.
5. Run the complete warning-strict software gate and GitHub CI. Record all
   outcomes, including unrelated baseline flight limitations. The reserved
   geometric qualification seeds remain unopened.

The test budgets are bounded simulation examples. Their suitability for a real
vehicle, all model mismatches, heading observability, or safe landing is not
established by this acceptance protocol.
