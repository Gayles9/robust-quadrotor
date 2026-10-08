# Comparing supported starts in flight

This experiment separates the effect of physical support from the effect of
pre-arm estimation. Both new comparison arms have the same supported startup;
only one uses the IMU window to improve its initial orientation and bias
estimate. Earlier freely flying runs remain a distinct baseline because their
physical starting conditions differ.

## Comparison arms

Four existing cases are used: `nominal_hover` at seed 30, `nominal_tracking`,
`wind_tracking` and `mass_tracking` at seed 31. The archived supervised flights
retain their original scores. Each case adds two supervised flights, eight in
total:

- Supported release with the unaligned prior.
- The same supported release with the pre-arm alignment estimate.

Each pair shares the true starting state, acquired support IMU window, terminal
true biases and complete flight random draws. The unaligned arm retains its
independent attitude/bias means and adds elapsed bias-walk covariance without
using the stationary IMU measurements. Initial position/velocity means and
covariance and the heading prior are unchanged in both arms.

The comparison uses the original controller, sensor noise, duration, priors and
scoring. It covers these maneuvers, not the entire fault campaign.

## Support and release physics

An ideal external fixture holds the original random true position and attitude
constant, with zero true velocity, angular rate and rotor speed. These conditions
differ from the freely flying baseline's random velocities/rates and initially
spinning motors. No state is teleported at release.

During the entire 0..0.5025 s supported interval, the fixture balances gravity
and existing wind drag:

\[
 F_{support,W}=-mg_W-R_{WB}F_{drag,B},\qquad
 a_W=0,\quad\omega_B=0,\quad f_B=-R_{WB}^Tg_W.
\]

Rotors are off; no rotor torque or drag moment is modeled. Saved equilibrium
residuals, pose, motion and motor histories substantiate the simulated support
assertion. This is an explicit fixture model, not a stationarity inference or
a hardware demonstration.

The fresh sample is acquired while still supported at pre-arm t=0.5025 s and
becomes flight t=0. The fixture is then removed. The original motor and plant
dynamics respond from zero rotor speed, with continuous state and motor speeds.
The external force has a finite step immediately after the boundary, without
an impulse or ground-contact model.

The first IMU endpoint is therefore the supported **left-limit** sample: the
measurement just before support disappears. The original trapezoidal estimator
integrates across that force discontinuity. This comparison retains that startup
integration error and its full flight effect. It is not a ground-takeoff or
physical landing/disarming qualification.

## Samples, randomness and feedback

Samples 0..200 belong only to alignment; sample 201 is the fresh release
endpoint. Flight index `k` maps to global sample `201+k`, with no double use.
The complete alignment endpoint preserves right-local attitude/bias cross
blocks. Reconstruction from the physical prior and independent sample noise
must match all 21×21 entries, means and the actual fresh sample.

Support noise and 201 bias-walk increments use independent PCG64 streams
`SeedSequence([0x50524541,4,seed,stream])`: streams 1/2 supply accelerometer/gyro
white noise and 3/4 supply bias walks. Starting biases come from the original
case. Fresh-sample noise comes from original flight index 0; support acquisition
does not advance a flight stream. Later noise, bias increments, slow-sensor
samples and schedules retain the original named-stream construction.

Pairing is checked after removing each run's physical signal and bias. Raw
measurements need not match when trajectories differ. Truth is used only in
the fixture/sensor generator and evaluation. Alignment receives measurements,
a declared prior and support evidence; feedback receives the normal estimated
state and rate. Process-local adapters restore on exit, with one mission per
process and at most two workers.

## Acceptance and interpretation

Independent verification covers equilibrium including wind and mass, sample
signs and ownership, fresh noise continuity, prior isolation, full handoff
covariance, continuous motor release, adapter restoration and rejection before
flight. Saved histories reconstruct plant/motors, every estimator state,
covariance and event, controller commands, guards and supervision.

The original complete flight is scored. Whole-flight RMSE and final position
limits remain 0.15 m, final speed 0.15 m/s, and the inclusive 5..11 s hover peak
0.08 m. Extra pre-arm time is reported separately.

Further supported-start validation requires valid evidence for all eight new
flights, an aligned hover passing every original limit, and hover peak no worse
than either original or supported-unaligned hover, with 1e-12 m comparison
tolerance. Across all four cases, every condition passed by either comparator
must remain passed. Remaining failures, including mass mismatch, are reported
separately. Startup improvement does not close an unrelated flight requirement.

All outcomes are retained without candidate changes after seeing results.
[Independent supported-start validation](independent-supported-start-validation.md)
uses separate seeds and fault comparisons.
