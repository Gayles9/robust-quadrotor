# Observation health

The error-state Kalman filter (ESKF) records whether each position or altitude
measurement was used, rejected, too late to use, disabled or still pending.
A single rejected outlier is different from losing usable measurements for a
sustained interval. The observation health monitor makes that distinction
explicit for each sensor stream.

I keep this monitor passive: it reports evidence that the optional
[observation supervisor](observation-supervision.md) can use. The monitor itself
does not change an estimate, command, mission phase or abort decision.
Accepted observations can still be biased or provide little information about
some quantities being estimated. HEALTHY therefore means accepted measurements
are available. It does not establish accuracy, observability (whether the
measurements can reveal the relevant state), or safe flight.

## Inputs and timing

[`ObservationHealthMonitor`](../../src/quadrotor_math/observation_health.py) consumes
the current clock and the current epoch's `EskfReplayEvent` tuple. An epoch is
one estimator update time. The monitor receives every epoch, including times
when the slower sensors provide no measurement. Event
indices resolve against that observed clock history, so acquisition and actual
delivery times stay distinct. It never reads the measurements, estimated state,
covariance, simulated truth or future delivery queue.

| Event | Effect |
| --- | --- |
| `FUSED` | Refresh accepted acquisition/delivery times and extend the acceptance streak |
| `REJECTED` | Extend the rejection streak; preserve last accepted time |
| `STALE` | Record the late delivery without refreshing accepted time |
| `DISABLED` | Record delivery for a stream explicitly disabled in the estimator |
| `PENDING` | Refuse at this live interface because the observation has not arrived |

The current ESKF does not rewind its state for delayed observations. A delivery
from an older acquisition epoch is therefore stale, even if its innovation
would have been small. Here, an innovation is the difference between a measured
value and the filter's prediction. Estimator arithmetic failures raise
exceptions; they are not converted into rejection events.

Every nonaccepted delivery clears the acceptance streak. Every disposition
other than rejection clears the rejection streak. Silence changes information
age without inventing a rejected observation. Recent evidence is the last
configured number of delivered dispositions; it is not a time window, so use
its timestamps and accepted age when interpreting an old record.

## Explicit thresholds

Each stream supplies its sample period T, delivery delay D, monitor clock
interval h, warning period count w and loss count l. The thresholds are
$`a_w=wT+D+h`$ and $`a_l=lT+D+h`$, with `l > w`.
The symbols $`a_w,a_l`$ denote the warning and loss ages. The final clock interval allows delivery quantization. A mission verifies these
schedule and clock values against its own configuration before running.

The age is the current clock minus the last accepted acquisition time. Before
any acceptance, elapsed time from the initial epoch defines a startup grace;
the reported accepted age is `None`. Equality is still within a threshold.
The first observed clock strictly beyond it changes the state. Current
deliveries are applied before reporting the state, but a preceding gap beyond
the warning age clears any old acceptance streak before recovery starts.

For example, T=0.2 s, D=0, h=0.0025 s, w=2 and l=5 gives warning and loss ages
of 0.4025 and 1.0025 seconds. At the 0.04 s altitude period the same counts give
0.0825 and 0.2025 seconds. These are illustrative availability settings;
supervisor action deadlines require a separate control design.

| State | Meaning |
| --- | --- |
| `WAITING` | No accepted observation yet, within startup grace |
| `HEALTHY` | Accepted information is fresh and no persistent rejection condition applies |
| `DEGRADED` | Warning age exceeded, rejection limit reached, or recovery awaits acceptance |
| `LOST` | Loss age exceeded |
| `RECOVERING` | Acceptance resumed after degradation, but the recovery count is not complete |
| `DISABLED` | Fusion intentionally disabled for this stream |

Disabled status has priority, then loss age, warning age, repeated rejection,
recovery, startup and healthy status. The first accepted observation inside
startup grace establishes health. Recovery after degradation requires an
explicit count of at least two consecutive acceptances. An intervening
nonaccepted delivery resets that count. Reasons distinguish aging, rejection
and recovery even when they share the DEGRADED state. Transitions report state
changes only, in position/altitude order, at the clock where they were observed.

## Using the monitor

This short example uses the existing noisy smoke fixture. It is an API example,
not a performance qualification:

```python
from experiments.estimated_feedback_validation import make_configuration
from quadrotor_math.estimated_mission import simulate_estimated_mission
from quadrotor_math.observation_health import (
    ObservationHealthConfiguration,
    ObservationHealthMonitor,
    ObservationHealthPolicy,
)

arguments = make_configuration({"case": "smoke", "seed": 31, "noiseless": False})
sensors = arguments["sensors"]
estimator = arguments["estimator_configuration"]
policies = [
    ObservationHealthPolicy(
        sample_period_s=schedule.sample_period_s,
        delivery_delay_s=schedule.delivery_delay_s,
        check_interval_s=arguments["numerics"].time_step_s,
        warning_periods=2,
        lost_periods=5,
        rejection_limit=3,
        recovery_acceptances=2,
        evidence_window_size=10,
        enabled=enabled,
    )
    for schedule, enabled in zip(
        (sensors.local_position_schedule, sensors.barometric_altitude_schedule),
        (estimator.fuse_local_position, estimator.fuse_barometric_altitude),
        strict=True,
    )
]
monitor = ObservationHealthMonitor(ObservationHealthConfiguration(*policies))
result = simulate_estimated_mission(**arguments, observation_health=monitor)
assert len(monitor.history) == len(result.mission.time_s)
print(monitor.latest)
```

For direct use, call `monitor.step(time_s, events)` after each successful ESKF
epoch. The first time must equal the declared initial time; subsequent times
must increase. Event delivery indices must match that clock index. Duplicate
identities or acquisition epochs, future/pending events and noncanonical order
raise without consuming any clock or evidence. Valid out-of-order stale
deliveries are supported. Policy enable flags must match fusion dispositions.

Snapshots and policy records are immutable and contain only scalar values,
enums and tuples. `monitor.history` returns an immutable observed prefix;
`monitor.latest` is `None` before the first step. The simulation requires a
fresh monitor, and `monitor.reset()` starts a new run with cleared identities
and evidence. If a mission raises, the observer retains its successfully
observed prefix; the simulation returns no partial mission result.

The mission result and its existing authenticated archive schema stay unchanged.
Health history is a derived diagnostic owned by the monitor. It can be recreated
from a saved event ledger and its exact policy by stepping over the recorded
clock and supplying only events delivered at each epoch. Pending records are
excluded. This avoids storing diagnostic objects as NumPy object arrays.

The monitor retains its clock, identities and snapshot history until reset;
storage grows with run length. It is a simulation interface with no bounded
real-time memory or flight-hardware claim.

## Verification and scope

The [monitor specification](../decisions/0021-observation-health-monitoring.md)
defines the rules. [Verification evidence (ZIP archive)](../../evidence/development-records.zip)
covers timing boundaries, missing/stale observations, repeated rejection,
recovery, reset, atomic retry and passive integration checks. Integration cases
compare complete numerical payloads and saved byte digests, reconstruct the
health history from recorded events, and detect deliberately damaged files.

The separate [supervisor](observation-supervision.md) implements explicit
timed numerical aborts for sustained information loss.
The [supported alignment component](prearm-component.md) is implemented separately.
Broader controller qualification, additional sensors and custom ROS/PX4 integration
remain outside this monitor's scope.
