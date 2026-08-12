# Repository Instructions

This repository supports a six-month Robust Autonomous Quadrotor project. Build only the subsystem required by the current task; do not implement future dynamics, estimation, planning, control, fault-handling, validation, ROS 2, PX4, or Gazebo work ahead of schedule.

## Architecture and Conventions

- Use Python 3.12 with a `src`-layout package named `quadrotor_math`.
- Put core algorithms under `src/quadrotor_math`. Keep them independent of ROS 2 and PX4.
- Use NED world coordinates and FRD body coordinates.
- Define `R_WB` as mapping body-coordinate vectors into world coordinates.
- Public quaternion names must state their frame direction. Never expose an ambiguous variable named `q`.

## Engineering Practice

- Work in small, testable increments.
- Add or update tests with every behavioral code change.
- Use deterministic random-number generators with explicit seeds.
- Prefer typed, documented interfaces and explicit checks for numerical shapes and units.
- Fix frame, sign, numerical, and test failures at their source; never hide them by tuning around them.
- Do not add dependencies, change tool versions, or modify system configuration without explaining why.
- Do not commit generated results or large logs.

## Completion

- Before finishing a task, run the relevant tests and lint checks. Report the exact commands and their results.
- Never commit changes unless Luke explicitly requests a commit.
