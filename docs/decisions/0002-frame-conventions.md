# 0002: Frame Conventions

## Context

Quadrotor dynamics are sensitive to frame, axis, and sign conventions. The
mathematical core needs one explicit convention that aligns gravity, thrust,
state variables, and future flight-stack integration without implicit
coordinate conversions.

## Decision

Use a North-East-Down (NED) world frame and a Forward-Right-Down (FRD) body
frame. Positive rotations follow the right-hand rule.

Represent orientation as the active body-to-world rotation `R_WB`, so
`v_W = R_WB v_B` and `v_B = R_WB^T v_W`. Use `q_WB` for the equivalent Hamilton,
scalar-first, unit quaternion.

Keep future ROS ENU/FLU conversions at adapter boundaries and out of the
mathematical core.

## Reason

NED/FRD matches common aerospace and autopilot conventions. It makes world
gravity positive along `+z`, while upward collective thrust is negative along
body `z`. Choosing an active body-to-world rotation makes state propagation and
the transformation of body forces into world coordinates explicit. Directional
names expose frame intent at each use site.

## Consequences/Risks

- Increasing world `z` represents descent, so altitude has the opposite sign.
- Every vector and orientation operation must preserve its stated expression
  frame and direction.
- Transposing `R_WB`, reversing quaternion direction, or mixing active and
  passive interpretations can create plausible but incorrect motion.
- Hamilton scalar-first quaternions may require conversion when an external
  API uses another ordering or convention.
- ROS ENU/FLU integration requires explicit, tested adapter conversions.

## Revisit Trigger

Revisit if a required flight stack or simulator cannot be integrated through
isolated, well-tested frame adapters, or if project-wide interface requirements
mandate a different canonical frame convention.
