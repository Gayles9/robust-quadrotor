# Pre-arm alignment session and handoff

`PrearmAlignmentSession` turns a fixed supported IMU window into an estimator
endpoint. It validates sample timing and support assertions, reports rejection
reasons, and releases its output once. It issues no arm command and runs no
mission.

The component uses the accepted
[nonlinear uncertainty model](nonlinear-prearm-uncertainty.md). Its independent
reference comparison covers 15,000 prior outcomes. The reported maximum
normalized variances are 1.061394 on the original failure population and
1.082222 on the fresh Gaussian population, both below 1.10.

## Session states and input ownership

The states are COLLECTING, READY, REJECTED and RELEASED. Construction binds an
acquisition ID, IMU stream ID, common clock ID, support ID, support source and
first sample ID. IDs are nonempty strings; sample IDs are nonnegative integers,
excluding booleans. There is no reset or retry method. Inputs and output records
are immutable owned snapshots, with no aliasing of caller arrays or internal
state.

The normalized clock starts at t=0. Paired samples 0..200 occur at
`k*0.0025 s`; the fresh release sample is at 0.5025 s. IDs are consecutive from
the bound first ID. FRD/SI profile identification and the 1e-12 s clock tolerance
are required. This normalized-clock contract does not qualify physical hardware
clocks; absolute-clock conversion belongs in an adapter.

Every consume, check and release call supplies contemporaneous external support
evidence. It includes bound IDs/source, coverage from t=0 through the call time,
and explicit mechanical support, motors-off, zero-world-acceleration,
zero-angular-rate and revocation fields. Missing, expired, future-dated,
mismatched, revoked or false assertions reject the session.

The object checks evidence structure and continuity, not physical authenticity.
Quiet IMU data cannot create a support assertion. The caller owns unique
support/acquisition IDs and disjoint windows across instances; this object is
neither a global identity registry nor a hardware interlock.

A partial window cannot become READY. Extra data after READY rejects the
session. Explicit time checks detect a missed next-sample deadline or expired
release; there is no background clock. A READY snapshot is valid at its recorded
epoch, and release revalidates time and support.

Malformed value objects raise before entering the session. Invalid evidence or
data received by an active session latches rejection and cannot produce an
endpoint. REJECTED and RELEASED objects accept no further actions; repeated
release raises. Numeric window gates retain every failed reason and diagnostic.

## Priors and uncertainty

The fixed profile accepts identity attitude prior (including the equivalent
quaternion sign), zero bias means, 3-degree attitude standard deviation,
0.03 m/s² accelerometer-bias standard deviation and 0.005 rad/s gyro-bias standard
deviation, with the original independent alignment covariance. Position/velocity
mean and a valid 6×6 positive-semidefinite covariance are supplied separately
within the initial 15-state prior. Navigation/alignment independence and zero
cross blocks are explicit requirements; other profiles are rejected.

The component retains order-five positive quadrature, at most eight rotation
mean corrections at 1e-13 rad, full terminal-bias cross-covariance, three
alpha=0.001 compatibility gates, axis radius at most 0.75 degree, inclination
at most 15 degrees, and shape/finiteness/conditioning checks.

Release keeps the alignment mean, adds exactly one interval of bias-walk
covariance and preserves all cross blocks. The declared independent position
and velocity block completes the initial 15-state prior. Output contains an
`EskfEndpointState`, the owned fresh IMU sample and acquisition provenance.
Sample-noise mean and cross block start at zero because the sample is disjoint,
not because state uncertainty has vanished. Release does not propagate an
airborne state.

## Verification contract

Independent checks cover ownership, accepted priors, every state transition,
support continuity/revocation/motor status, sample IDs and clock order, window
and deadline boundaries, numeric rejection, full covariance and one-time release.

All 15,000 authenticated nonlinear reference results must agree within
2e-14 rad in rotation/error and 1e-9 in maximum covariance discrepancy whitened
by the reference covariance. Full sessions also use the first 100 fresh nominal
and first 100 fresh Gaussian trials, with independent release-walk/noise streams
`SeedSequence([0x50524541,3,partition,trial])`. Those checks verify the full
release matrix, fresh sample ownership and unchanged initial position/velocity.

The undetectable-acceleration example remains a limitation, alongside motion
and invalid-data rejection fixtures. A verified component establishes its
software and mathematical contract. Supported-flight evaluation separately
checks common flight noise, complete startup and unchanged performance limits.
