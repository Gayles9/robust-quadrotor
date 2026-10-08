# Understanding attitude error during startup

The startup analysis asks why the estimated orientation initially moves away
from the true orientation, and whether that change comes from a filter defect
or from limited measurement information. It is an offline diagnosis of saved
flights, not a controller change or a flight qualification.

An **attitude oracle** is a diagnostic comparison that supplies simulated true
orientation to a selected feedback channel. It measures possible improvement;
it is not an implementable sensor.

## Evidence and reconstruction

The analysis uses the original hover and attitude-oracle hover at seed 30, each
covering the full 15.5 s flight. It resolves IMU prediction and each position and
altitude correction separately during the first second, then follows the
1..5 s and inclusive 5..11 s windows to the original hover peak. The initial
attitude error grows from 2.2941 to 3.1614 degrees. A correction that worsens one
realized error is not, by itself, evidence of a filter defect.

The reconstructed quantities are right-local true-minus-estimate attitude error,
thrust-axis error, physical covariance, coupled velocity and bias corrections,
innovation timing, and rejected observations. “Right-local” means orientation
errors are expressed as small rotations in the estimated body frame.

The saved inputs are authenticated against their source, configuration, seed
and content hashes. Exact input identities and recovery provenance are retained
in the [evidence archive](../../evidence/development-records.zip).

## Mathematical checks

Independent algebra and finite differences check the endpoint sample-noise
model, full joint conditioning and covariance reset, measurement Jacobians,
and right-local injection signs. The declared prior moments are compared with
the truth generator. Innovation, gain, Joseph covariance and reset calculations
are reconstructed independently of the saved estimator state.

The measurement-information analysis states its local hover model and checks
its rank and nullspace. It explains inclination/bias ambiguity without claiming
that a local linear model proves nonlinear or time-varying unobservability.
Covariance and innovation statistics from correlated epochs of one seed are
descriptive diagnostics, not a population calibration test.

Truth is used after each update for scoring, never as an input to estimator
reconstruction. A freely flying accelerometer measures specific force, not
necessarily gravity alone. The audit retains the original priors, sensor model,
Q/R, controller, limits, clocks, trajectories and thresholds.

## Interpretation

Acceptance requires authenticated inputs, reconstruction of the saved state and
covariance, and an explanation of the separate propagation and update effects.
A successful diagnosis can identify missing information without finding an
implementation defect. It does not qualify a new controller or close the
separate mass-transient requirement. The cascade remains the default.

The resulting supported-stationary design is explained in
[pre-arm alignment](stationary-prearm-alignment-design.md).
