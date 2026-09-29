# Standalone supported pre-arm alignment

The standalone component implements the accepted
[nonlinear uncertainty model](prearm-nonlinear-uncertainty.md). It acquires a
supported stationary IMU window, checks uncertainty and motion compatibility,
and releases one initialized endpoint ESKF state. It does not issue an arm
command or run a flight. The [frozen interface](decisions/0029-standalone-prearm-alignment.md)
and [verification record](progress/2026-09-28-prearm-component.md) define its scope.

## Ownership and physical support

`PrearmAlignmentSession` binds acquisition, IMU stream, common clock, support,
support source and first sample IDs. The caller must allocate unique acquisition
IDs and disjoint sample windows across instances. This object has no global ID
registry. Its inputs and outputs own their arrays; reading a snapshot cannot
modify the session's estimate.

Every call supplies `StationarySupportEvidence` covering t=0 through the supplied
current time, with explicit mechanical support, motors-off, zero world
acceleration and zero angular rate assertions. The source and IDs must match;
revocation or missing, false, stale or future-dated evidence rejects the session.
Evidence attests an observed interval, not future stationarity. Its physical
authenticity belongs to the support owner; this library checks its structure and
continuity. A quiet IMU cannot establish physical support. Constant acceleration
can be indistinguishable from tilt and still pass every numeric gate.

The relative acquisition clock starts at zero. Paired FRD/SI samples 0..200
arrive at k*0.0025 s with consecutive IDs. The acquisition spans 0.5 s; support
continues to the fresh release sample at 0.5025 s. The 1e-12 s tolerance is a
software contract, not hardware clock qualification. A future adapter must
establish any conversion from absolute sensor clocks.

## State and call contract

| State | Allowed next action | Outcome |
| --- | --- | --- |
| COLLECTING | `consume` next sample or `check` support/time | Remain collecting until all 201 samples pass |
| COLLECTING | `release` | Latch incomplete-window rejection |
| READY | `check` or `release` with the independent next sample | Remain ready at the checked epoch, or release once |
| READY | `consume` additional alignment data | Latch unexpected-sample rejection |
| REJECTED or RELEASED | Any further action | Raise; no second endpoint and no retry |

Call `check` while waiting without samples. It detects a missed next-sample
deadline or an expired release. There is no background clock. A READY snapshot
is valid only at `observed_through_s`; release always revalidates evidence,
time, identity and fresh sample ownership. To attempt another acquisition the
external owner must authorize and allocate a new independent session.

Structural errors raise when constructing value objects, before any session
call. Invalid inputs delivered to an active session instead latch rejection.
Numeric failures preserve every failed gate and the available estimate and
statistics. A numerical construction/conditioning failure yields no estimate.
No rejected call can return an ESKF endpoint.

## Fixed model and information boundary

The approved profile is `adr0028-400hz-ned-frd-v1`: 400 Hz, g=9.81 m/s²,
sample sigmas 0.04 m/s² and 0.002 rad/s, bias-walk densities 0.0002 and
0.00002 in their corresponding units per square-root second. Initial attitude
sigma is 3 degrees, accel-bias sigma 0.03 m/s² and gyro-bias sigma 0.005 rad/s.
Only the independent alignment prior, identity attitude mean and zero bias
means are supported. Unsupported noise, timing, gravity or correlated alignment
priors raise rather than silently selecting another model.

Position/velocity means and a valid PSD 6x6 covariance are caller supplied.
The caller must explicitly declare navigation/alignment independence; both
cross blocks must be zero. The alignment does not estimate position or velocity.
Even zero navigation covariance is permitted as a mathematical input and does
not certify that such a prior is physically justified.

The 625-node positive quadrature preserves the complete right-local attitude,
terminal accelerometer-bias and terminal gyro-bias covariance at the computed
rotation mean. It retains heading uncertainty and does not independently
calibrate accelerometer bias. The rotation solver and all three alpha=0.001
compatibility gates, 0.75-degree local 99% axis radius and 15-degree inclination
domain are unchanged. The approximation and calibration limits in ADR 0028
remain applicable.

Release preserves the alignment mean, adds one interval of bias-walk variance
and retains all cross terms. It assembles the declared independent navigation
block and initializes the 21-state endpoint covariance with fresh white-sample
noise, zero sample-noise mean and zero physical/sample-noise cross covariance.
The fresh measurement is not reused in the alignment estimate or gated as
another alignment observation. This preserves the stated independent-sample
handoff; one noisy sample does not prove or disprove physical support.

## Verification and next scope

All 15,000 archived covariance matrices agree exactly; maximum rotation
difference is 2.579e-17 rad against 2e-14. Calibration and rejection decisions
are unchanged. The 200 complete nominal/Gaussian sessions reproduce every
compatibility statistic and endpoint covariance exactly. Independent tests
exercise timing, support, ownership, invalid inputs, motion rejection,
simultaneous failure reasons and one-time release.

This establishes the bounded software component. It does not establish improved
flight performance, hardware support, real-time scheduling or motor safety.
The cascade controller and existing endpoint ESKF remain unchanged. The later
[supported-start comparison](supported-start-flight.md) now demonstrates improved
known-seed hover and tracking under an explicit fixture model. Its mass failure
remains open. The [independent validation](independent-supported-start.md) now
finds improvement across three fresh hover seeds, with one still above the
8 cm limit. Normal mission integration remains blocked; next is a bounded
diagnosis using the saved histories. Reserved geometric qualification seeds
remain unopened.

```bash
OPENBLAS_NUM_THREADS=1 uv run python -W error -m experiments.prearm_component_validation --prior-evidence results/prearm-nonlinear --output results/prearm-component
uv run pytest -q tests/unit/test_prearm_alignment.py
```

The reference directory must contain the exact authenticated ADR 0028 report
and trial payload. Use a new output directory; earlier results are preserved.
