# 2026-09-11: Fixed-Rate Sensor Scheduling

## Milestone purpose and scope

This milestone adds a ROS/PX4-independent timing boundary that schedules fixed-rate sensor
acquisition on the existing fixed simulation truth grid and separates acquisition time from
fixed-delay delivery time. That separation is required before estimator, controller, or ROS
integration because a measurement must represent truth at the instant it was sampled even
when the consumer observes it later.

The scheduler transports values produced by caller-owned sensor functions. It does not
interpret their frames or physical units, calculate a sensor measurement, interpolate truth,
or own any random generator. The increment establishes timing, ordering, validation,
ownership, and deterministic replay semantics only. Sprint 3, Week 6 remains in progress,
and Gate G1 remains open and partially satisfied.

## Repository baseline

Run 3 began on branch `main`. Both `HEAD` and `origin/main` resolved to the synchronized
committed baseline:

```text
fd7726ec41268a8c3aec6ca982d1d98d0b2066a0
```

The committed tree at that revision was unchanged. The working tree contained exactly the
two untracked source-and-test files completed during Runs 1 and 2:

```text
?? src/quadrotor_math/sensor_scheduling.py
?? tests/unit/test_sensor_scheduling.py
```

Nothing was staged, and this progress-record target did not yet exist. The initial checksums
were:

```text
ecfc9481a5e0c413f09abf042d2cc19c1ade3c65be8590c586381f6590400d7e  README.md
98abbcfda8ce4c8fd39e8a9ec6eea7c75065f2e6d63bb713447e1bd276d57ae5  src/quadrotor_math/sensor_scheduling.py
180beec8af0e38c0288a38e15e03e65bc1f2901dd1819cbb9fbbf52dce728e03  tests/unit/test_sensor_scheduling.py
```

Before documentation was edited, the focused scheduler command completed successfully:

```text
UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache uv run pytest tests/unit/test_sensor_scheduling.py -q
..........................................................               [100%]
58 passed in 0.27s
```

## Architecture decisions

### ROS-independent ownership

Scheduling belongs in `src/quadrotor_math` because acquisition and delivery semantics must be
reproducible without middleware clocks, message queues, executors, or transport behavior.
Future ROS 2, PX4, or Gazebo adapters may consume this boundary, but their types and timing
conventions must not leak into it.

### Public period and canonical truth-grid stride

The public configuration is `sample_period_s` because a sensor's acquisition period is the
physical scheduling quantity its caller selects. The scheduler also receives the fixed truth
time step. The period must align to a positive integer number of truth intervals; otherwise a
requested acquisition would fall between stored truth rows and require an interpolation
policy that this milestone deliberately does not define.

The aligned ratio is rounded once into an integer stride. That stride and integer truth
indices are authoritative. The effective period is canonicalized to
`sample_stride * truth_time_step_s`, and acquisition timestamps are derived from truth indices
instead of accumulated by repeated floating-point addition. This prevents cumulative clock
drift from changing which truth row a sample represents.

### First acquisition and catch-up

Sequence index zero is acquired after one complete sample period. Time zero is the initial
truth row, not an implicit sensor sample. This makes every acquisition correspond to a
completed positive stride and gives the first callback truth index `sample_stride`.

An update can cross more than one acquisition boundary. Every crossed acquisition is caught
up in chronological order rather than skipping older rows and producing only the newest
measurement. This preserves acquisition count, producer call order, and caller-owned RNG
consumption regardless of whether the caller advances through individual boundaries or
reaches the same final time in one update.

### Producer callback boundary

The caller-provided acquisition callable receives the authoritative truth-history index and
the derived acquisition time. The index lets a future composition select the exact stored
truth row without recovering it from a float timestamp. The timestamp lets the producer
record or use elapsed simulation time when its model requires it. The callback owns the
meaning, frame, units, validation, and stochastic generation of its returned value.

All due timestamp calculations and scheduler-owned validation complete before the first
producer call. Once validation succeeds, all due acquisitions are performed before any
delivery is released. A producer can therefore inspect the scheduler without seeing a
delivery from the same update interleaved into the acquisition catch-up phase.

### Fixed-delay FIFO delivery and held value

Every record receives the same configured nonnegative delivery delay. Since acquisition times
increase with sequence index, adding one fixed delay preserves their ordering. A FIFO pending
queue therefore releases records in both acquisition and delivery order. Delivery returns
stored measurements and never calls the producer again.

An update returns every pending record whose delivery time is due. The most recently released
record becomes the internally held latest delivery and remains held through acquisition-only
and idle updates until a newer record is delivered. This models the common consumer boundary
that reads the newest available sample while still exposing complete delivery batches.

### Record shape, identity, and ownership

`SensorMeasurement` is frozen and slotted so its timestamp and sequence metadata cannot be
reassigned and instances do not carry per-instance dictionaries. It uses `eq=False`, leaving
record comparison as identity comparison; generated dataclass field equality would be
ambiguous for NumPy-array measurements.

The zero-based sequence index supplies stable ordering without inferring it from floating
timestamps. Array values are copied when enqueued, copied again for returned deliveries, and
copied for every latest-delivered snapshot. These boundaries prevent producer buffers or
publicly returned arrays from mutating pending or held internal values. Immutable Python
floats need no copy.

### Caller-owned random generators

The scheduler owns no RNG because each stochastic sensor or bias process needs an explicit,
independent random stream controlled by its caller. A producer consumes random values only
when the scheduler invokes it for an acquisition. Delivery never resamples and never advances
an RNG. This keeps scheduling generic and makes acquisition replay depend on explicit caller
state rather than hidden scheduler state.

## Mathematical contract

Let `k` be the zero-based sequence index. Construction calculates:

```text
sample_stride = round(sample_period_s / truth_time_step_s)
```

The scheduled truth index, acquisition time, and delivery time are:

```text
truth_index_k = (k + 1) * sample_stride
acquisition_time_k = truth_index_k * truth_time_step_s
delivery_time_k = acquisition_time_k + delivery_delay_s
```

Period alignment compares the configured period with the canonical effective period using:

```text
rtol = 1e-12
atol = 0.0
```

Timestamp boundary and monotonicity comparisons use the scale-aware float64 tolerance:

```text
time_tolerance(a, b) =
    16 * eps_float64 * max(1, abs(a), abs(b))
```

A scheduled acquisition or delivery is due when its timestamp is no later than current time
plus the corresponding tolerance. Equivalently:

```text
scheduled_time <= current_time_s + time_tolerance(scheduled_time, current_time_s)
```

A proposed update time is materially backward and rejected when:

```text
current_time_s
    < last_authoritative_update_time_s
      - time_tolerance(current_time_s, last_authoritative_update_time_s)
```

If a smaller time falls within tolerance, the update is accepted but authoritative time is:

```text
max(current_time_s, last_authoritative_update_time_s)
```

Thus tolerance never moves scheduler time backward.

## Exact public API

The immutable delivery record is:

```python
@dataclass(frozen=True, slots=True, eq=False)
class SensorMeasurement[MeasurementT: (float, NDArray[np.float64])]:
    sequence_index: int
    acquisition_time_s: float
    delivery_time_s: float
    measurement: MeasurementT
```

The scheduler constructor is keyword-only:

```text
FixedRateSensorScheduler[MeasurementT](
    *,
    sample_period_s: float,
    truth_time_step_s: float,
    delivery_delay_s: float = 0.0,
) -> None
```

Its update method is:

```text
update(
    current_time_s: float,
    acquire_measurement: Callable[[int, float], MeasurementT],
) -> tuple[SensorMeasurement[MeasurementT], ...]
```

`current_time_s` and all record timestamps are elapsed simulation seconds. The callback's
first argument is the authoritative truth-history index, and its second argument is the
derived acquisition time in elapsed simulation seconds. Its return value becomes the stored
measurement. The scheduler supports the declared value types of Python `float` and float64
NumPy arrays.

The held-value property is:

```text
latest_delivered_measurement
    -> SensorMeasurement[MeasurementT] | None
```

It returns `None` until the first delivery. Thereafter each access returns a new record; an
array measurement within that record is also an independently owned copy.

Measurement frames and physical units remain defined by the producer or sensor function. The
scheduler transports values without inspecting or transforming their frames, units, array
shapes, dtype, or physical validity.

## Validation contract

Constructor validation occurs in this exact order:

1. `sample_period_s` is finite;
2. `sample_period_s` is positive;
3. `truth_time_step_s` is finite;
4. `truth_time_step_s` is positive;
5. the sample period aligns to a positive integer truth-grid stride;
6. `delivery_delay_s` is finite; and
7. `delivery_delay_s` is nonnegative.

The exact constructor messages are:

```text
sample_period_s must be finite
sample_period_s must be positive
truth_time_step_s must be finite
truth_time_step_s must be positive
sample_period_s must be an integer multiple of truth_time_step_s
delivery_delay_s must be finite
delivery_delay_s must be nonnegative
```

Each update validates current time before building its due-acquisition list. Its exact
current-time messages are:

```text
current_time_s must be finite
current_time_s must be nonnegative
current_time_s must be monotonically nondecreasing
```

The scheduler then validates every scheduled timestamp that it calculates before invoking the
producer. The exact messages are:

```text
scheduled acquisition_time_s must be finite
scheduled delivery_time_s must be finite
```

All scheduler-owned validation completes before producer invocation. Invalid scheduler
updates therefore invoke no producer and cannot consume the producer's RNG. The producer
remains responsible for validating its own inputs and output semantics.

## State and ownership

The scheduler's internal state conceptually contains:

- the next zero-based sequence index;
- the last authoritative update time;
- a FIFO queue of acquired, pending records; and
- the latest internally held delivered record, or no record before first delivery.

For values within the public typed contract:

- producer arrays are copied when they enter the pending queue;
- each returned array delivery owns storage independent of the producer and held state;
- every latest-delivered array snapshot owns storage independent of internal state and other
  snapshots;
- mutating a returned delivery cannot change a later latest snapshot;
- Python floats remain Python floats; and
- float64 arrays preserve their dtype, shape, and values through copying.

Record metadata is frozen, but a publicly returned NumPy array remains mutable by its owner.
Defensive copies make that mutability local to the returned snapshot rather than exposing
internal scheduler state.

## RNG and deterministic replay

RNG consumption occurs inside the caller's producer at acquisition. The scheduler does not
own a generator and delivery consumes no random value. A delivery-only update does not call
the producer. An invalid scheduler update invokes no producer and therefore leaves any
producer RNG untouched.

Each stochastic sensor and each bias process should use a separate caller-owned generator so
one channel's acquisition count does not shift another channel's random stream. Equal
configuration, truth, initial scheduler state, RNG state, update sequence, and producer
behavior reproduce equal acquired values and timestamp records.

Incremental updates and one catch-up update preserve acquisition indices, chronological
producer-call order, and values when they reach the same final time with equivalent caller
state. Delivery batch boundaries can differ because delivery is observable only during
explicit update calls. Equal final time alone does not require identical returned batch
partitioning when the update sequences differ.

The scheduler and RNG are not serialized by this milestone. Replay therefore requires the
caller to reproduce or separately persist all listed inputs and states.

## RED/GREEN development evidence

The failures below are historical TDD evidence, not active final failures.

### Run 1

- The initial RED failed because `quadrotor_math.sensor_scheduling` did not exist.
- The final focused scheduler result was 39 passed.
- Bounded Ruff lint and non-modifying format verification passed.
- Bounded mypy passed.
- One test expectation was corrected from literal `0.6` to the authoritative
  integer-derived `6 * 0.1` representation. Production code was unchanged by that
  correction.

### Run 2

- The focused baseline was 39 passed.
- The ownership characterization produced the historical result `10 failed, 48 passed`.
- All ten failures were caused by missing defensive copying or snapshot independence.
- The minimum production change added array copying at pending storage and public exposure.
- The final focused scheduler result was 58 passed.
- Bounded Ruff lint and non-modifying format verification passed.
- Bounded mypy passed.

Run 3 independently re-established the 58-test focused baseline before changing
documentation.

## Verification before the repository gate

The initial Run 3 preflight confirmed synchronized refs, the exact three expected baseline
checksums, only the two expected untracked Python files, an empty index, an absent target
progress record, and clean staged and unstaged diff checks. The focused scheduler module then
passed all 58 tests in 0.27 seconds.

After documentation integrity review, the bounded non-modifying checks completed with these
exact results:

```text
$ UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache uv run ruff check src/quadrotor_math/sensor_scheduling.py tests/unit/test_sensor_scheduling.py
All checks passed!

$ UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache uv run ruff format --check src/quadrotor_math/sensor_scheduling.py tests/unit/test_sensor_scheduling.py
2 files already formatted

$ UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache uv run mypy src/quadrotor_math/sensor_scheduling.py
Success: no issues found in 1 source file

$ UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache uv run pytest tests/unit/test_sensor_scheduling.py -q
..........................................................               [100%]
58 passed in 0.22s
```

These focused results preceded and are distinct from the later repository-wide gate.

## Deferred work and risks

This milestone does not establish or implement:

- ROS 2, PX4, Gazebo, message, executor, or transport integration;
- asynchronous acquisition or delivery threads;
- truth interpolation or off-grid acquisition;
- variable or adaptive truth stepping;
- stochastic or time-varying latency;
- packet loss or sensor dropout;
- sensor or system clock offsets and drift;
- a bounded pending queue or overflow policy;
- transactional recovery from producer exceptions;
- scheduler-state or RNG-state serialization;
- automatic end-of-run queue draining;
- estimator or controller integration;
- a complete sensor layer or run-level simulation composition;
- a Monte Carlo program or robustness evidence; or
- completion of Gate G1.

Producer exceptions are not transactional. The scheduler cannot restore external RNG, sensor,
or other producer-owned state after a callback raises, and a catch-up batch may already have
invoked earlier producer calls. Callers must treat producer failure and retry policy as a
separate integration concern.

High-rate IMU scheduling may eventually require profiling or batching. Extremely long
simulations may eventually justify integer-nanosecond timestamps instead of float64 elapsed
seconds. Neither optimization is introduced without measured need.

No pending record is automatically flushed at simulation termination. A caller that requires
all delayed records must explicitly update through their delivery times. The current
unbounded queue can grow if delivery delay and acquisition rate create many pending records.

## Gate G1 status and exact next architecture action

Sprint 3, Week 6 remains in progress. Gate G1 remains open and partially satisfied. This
milestone resolves fixed-grid sample scheduling, elapsed acquisition and delivery timestamps,
fixed delay, FIFO delivery, and held-value semantics, but it does not establish the remaining
truth/nominal, saved-replay, environmental-model, estimation, robustness, or integration
evidence.

The authoritative README roadmap places truth/nominal separation and run-level replay next.
The exact next action is a bounded read-only architecture review for that Gate G1 step,
including how independent caller-owned RNG streams are preserved across sensor and bias
processes. Wind, drag, and deliberate model mismatch follow within Gate G1. This record does
not authorize or implement estimator, controller, Monte Carlo, ROS 2, or PX4 work.

## Repository gate

After the focused and documentation-integrity checks passed, the final repository gate was
run exactly once with:

```text
UV_CACHE_DIR=/tmp/robust-quadrotor-uv-cache make check
```

Its exact output was:

```text
uv run ruff check .
All checks passed!
uv run ruff format --check .
56 files already formatted
uv run mypy src experiments
Success: no issues found in 14 source files
uv run pytest
============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.1.1, pluggy-1.6.0
rootdir: /path/to/robust-quadrotor
configfile: pyproject.toml
testpaths: tests
collected 460 items

tests/unit/test_actuation.py ........................................... [  9%]
.................                                                        [ 13%]
tests/unit/test_dynamics.py ............................................ [ 22%]
...                                                                      [ 23%]
tests/unit/test_imu.py ................................................. [ 33%]
.....................................................................    [ 48%]
tests/unit/test_integration.py ..........                                [ 51%]
tests/unit/test_metrics.py ....................................          [ 58%]
tests/unit/test_position_sensors.py .................................... [ 66%]
..................                                                       [ 70%]
tests/unit/test_randomness.py .                                          [ 70%]
tests/unit/test_rotations.py ..................................          [ 78%]
tests/unit/test_sensor_scheduling.py ................................... [ 85%]
.......................                                                  [ 90%]
tests/unit/test_simulation.py ...................                        [ 95%]
tests/unit/test_validation.py ......................                     [ 99%]
tests/unit/test_vectors.py .                                             [100%]

============================= 460 passed in 1.55s ==============================
```

The active gate status is therefore:

- Ruff lint: passed with `All checks passed!`;
- Ruff format: 56 files already formatted;
- mypy: no issues in 14 source files;
- pytest: 460 collected and 460 passed in 1.55 seconds;
- sensor-scheduling tests: 58 passed within the repository run; and
- failures, errors, skips, and warnings: none.

The previous 402-test repository baseline plus the 58 new scheduler tests accounts for the
verified 460-test total. This subsequent correction records actual gate results in README and
this progress record only. It does not change a Python file and does not rerun the repository
gate.

## Files in the milestone

The complete milestone consists of:

```text
src/quadrotor_math/sensor_scheduling.py
tests/unit/test_sensor_scheduling.py
README.md
docs/progress/2026-09-11-fixed-rate-sensor-scheduling.md
```

Run 3 changes only `README.md` and this new progress record. The source and test files predate
Run 3 and retain their reviewed checksums. Nothing is staged, committed, or pushed by this
documentation increment.
