# ADR 0017: Reimplement the missing geometric controller

Date: 2026-09-26. Status: accepted implementation scope; performance unqualified.

## Starting point and scope

Audit base `23936ebd8403a3620f755a53143fa7c9f641d491` matches current main.
The published cascade, estimator and trajectory modules exist; the geometric
implementation described in master-log chapters 26–28 does not. This is new
source and fresh verification, not recovery of the missing implementation.

Rebuild the corrected geometric moment law, twice-differentiated force-to-attitude
map, 30 rad/s three-pole causal derivative filter, and opt-in true/estimated-state
mission composition. Retain the cascade default and its legacy call signature.
Support fixed-yaw HOLD, quintic SMOOTH and minimum-snap references; reject STEP.
SMOOTH uses the analytic one-sided jerk/snap at segment boundaries. No truth
acceleration, drag, wind, actual motors or true biases enter the controller.

Do not resurrect the superseded nominal-model derivative closure that caused
wind bias. Preserve its negative result in the historical log. No dependency,
tool-version, estimator, sensor, plant, cascade-gain or threshold changes.

## Equations

NED/FRD, active R_WB, e3=[0,0,1], nominal mass m and gravity g:

    c = m (Kp (p-pd) + Kv (v-vd))
    u = m (g e3-ad) + c
    u_dot = -m jd + D1(c); u_ddot = -m sd + D2(c)

Use L(s)=w/(s+w), D1=s L^3, D2=s² L^3, w=30 rad/s.
Three trapezoidal low-pass sections give D1=w(y2-y3),
D2=w²(y1-2y2+y3). Initialize all sections to c[0]; this yields zero
feedback derivatives at startup. Own state, require consecutive sample indices,
commit filter state only after successful finite arithmetic. Require 0<w*h<=1.

b3d=normalize(u); b1d=normalize(heading_y cross b3d);
b2d=b3d cross b1d. Differentiate normalization/cross products twice.
Omega_d=vee(Rd.T Rd_dot), alpha_d=vee(Rd.T Rd_ddot-Omega_d_hat²).
Rates are in the desired body frame. With A=R.T Rd:

    eR = vee(Rd.T R - R.T Rd)/2
    eOmega = Omega - A Omega_d
    M = -kR eR - kOmega eOmega + Omega cross J Omega
        - J (Omega cross (A Omega_d) - A alpha_d)
    f = u dot (R e3)

Default kR=0.64 N m and kOmega=0.32 N m s preserve the recorded
design. Reject nonsmooth outer limiting/domain violations; retain visible moment
clipping and bounded nominal allocation. Projection is recomputed at inner ticks.
Held reference jets and filtered derivatives are approximations; the ideal
continuous-time Lyapunov identity is a unit-test oracle, not a sampled-system proof.
Reference: Lee, Leok, McClamroch, CDC 2010, DOI 10.1109/CDC.2010.5717652.

## Frozen fresh acceptance, before implementation

1. Independent normalization derivatives, moving-frame inertia/energy identity,
   quaternion sign, NED hover/thrust direction, finite/domain checks and ownership.
2. Independent direct transfer recurrence, constant startup, reset/isolation,
   indices, frequency response <=3% magnitude error / <=20° lag at 0.1, 0.25,
   0.5 Hz. White-noise derivative gain <=10% of raw cubic stencils.
3. Mission clocks, no terminal command on abort, default-cascade compatibility,
   estimated-command reconstruction and exact offline ESKF replay.
4. Fixed true-state pairs: nominal, half plant step, +/- initial offset, mild
   wind, reversed wind, wind plus offset. Existing 15 cm whole-mission RMSE,
   25 cm peak, 8 cm final position, 8 cm/s final speed, no limiting/rotor contact.
   Geometric RMSE must not exceed cascade per case; first five mean ratios <=0.80.
   Refinement <=5 mm position / 0.05° attitude; one nominal and one noisy exact repeat.
5. Fixed estimated pairs, seeds 30 and 31, on the same minimum-snap mission,
   with and without mild wind. Same physical gates, no per-case RMSE regression;
   report actual-moment squared effort and requested-moment variation, allowing
   at most 2x paired actual-moment effort for performance promotion. Two full
   60-second hovers, seeds 30 and 93012, preserve the original 5..65 s / 8 cm gate.
   Use the original geometric position gains and matched cascade gains for spline
   pairs; also report the established v2 cascade for full-hover context.
6. Fresh full make check with warnings as errors. Commit tested experimental
   code even if performance promotion fails, recording all failures. Qualification
   requires fresh evidence for this implementation; historical test counts do not
   transfer to it. No automatic gain search is part of this reconstruction.

Generated histories/logs stay outside Git. Publish source and tests on the
reimplementation branch so the implementation remains recoverable independently
of the local working directory.
