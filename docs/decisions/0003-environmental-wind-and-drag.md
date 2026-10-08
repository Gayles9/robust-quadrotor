# 0003: Environmental Wind and Quadratic Drag

## Context

Wind changes the air velocity seen by the vehicle, and drag resists that relative
motion. The simulation uses a simple, reproducible model with explicit NED world
and FRD body coordinates. It keeps simulated truth separate from the assumptions
available to the estimator and controller, and records the parameters in the run
manifest without changing the 35-array artifact format.

A higher-fidelity aerodynamic model would require additional assumptions and parameters that
the project cannot yet identify or validate. The immediate need is a transparent disturbance
model that exposes frame and sign errors without implying flight-validated aerodynamics.

## Decision

- Represent wind as one constant `wind_velocity_W` vector in the NED world frame, in m/s.
- Compute vehicle-relative air velocity as
  `velocity_air_W = velocity_W - wind_velocity_W`.
- Transform it into FRD with `velocity_air_B = R_WB.T @ velocity_air_W`.
- Represent drag through one nonnegative lumped coefficient per FRD body axis,
  `quadratic_drag_coefficient_B`, in kg/m.
- Compute the body force elementwise as
  `force_drag_B = -quadratic_drag_coefficient_B * abs(velocity_air_B) * velocity_air_B`.
- Apply this force at the modelled centre of mass without an aerodynamic moment.
- Recompute drag at every derivative evaluation. Projected RK4 therefore evaluates drag at
  the separate velocity and projected attitude of k1, k2, k3, and k4.
- Store independent truth and nominal wind and drag values. Only truth values enter physical
  propagation and sensor truth; differences require explicit mismatch declarations.
- Persist any nonzero environmental configuration with manifest version 5 when unbound and
  version 6 when SHA-256-bound. Both versions support historical ideal and complete motorized
  actuation.
- Keep `RunArtifactData` at exactly 35 arrays. Environmental state is reproducible from the
  manifest and does not require another artifact history.

## Reason

This model is deterministic, frame-explicit, dissipative relative to the air, inexpensive to
evaluate, and easy to verify by hand. Anisotropic body-axis coefficients represent different
forward, lateral, and vertical resistance without introducing an unverified vehicle geometry
or flow model. Per-derivative evaluation is required because drag depends nonlinearly on the
stage velocity and, for anisotropic coefficients, on the stage attitude.

Truth/nominal separation supports deliberate model mismatch while keeping the generated
physical and stochastic data determined only by truth. Manifest versions 5 and 6 preserve
that configuration without changing the stable data archive.

The project does not yet decompose coefficients into air density, drag coefficient, and
reference area because those quantities are not independently identified in the current
model. Gusts and turbulence require a time-varying stochastic environment and its own replay
contract. Aerodynamic moments require centre-of-pressure and rotational-aerodynamics
assumptions. CFD and blade-element models would add substantial geometry, calibration, and
validation scope beyond this model.

## Consequences/Risks

- Wind is spatially and temporally constant during a run.
- Drag is axis-lumped and quadratic; it omits cross-axis coupling, Reynolds-number effects,
  rotor wake, ground effect, and interaction with vehicle geometry.
- No aerodynamic moment is produced, so rotational effects of wind are absent.
- The coefficients are effective model parameters, not separately interpretable physical
  density, area, or dimensionless drag coefficients.
- Truth environmental changes affect trajectories and accelerometer truth. Nominal-only
  changes intentionally do not affect any of the 35 generated artifact arrays.
- Version-5/6 manifests are required for any nonzero truth or nominal environmental element;
  versions 1–4 retain their historical zero-environment schemas and canonical bytes.
- Exact replay evidence applies to the tested Python/NumPy environment and does not promise
  bitwise identity across arbitrary platforms or dependency versions.
- This decision does not constitute real-world flight validation or a safety claim.

## Revisit Trigger

Revisit when flight or higher-fidelity simulation data requires time-varying wind, turbulence,
aerodynamic moments, cross-axis coupling, identified density/Cd/area parameters, rotor-wake or
ground-effect modeling, or a CFD/blade-element boundary with validated parameters.
