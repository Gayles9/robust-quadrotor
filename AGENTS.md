# Repository Instructions

This repository supports a six-month Robust Autonomous Quadrotor project. Build only the subsystem required by the current task; do not implement future dynamics, estimation, planning, control, fault-handling, validation, ROS 2, PX4, or Gazebo work ahead of schedule.

## Architecture and Conventions

- Use Python 3.12 with a `src`-layout package named `quadrotor_math`.
- Put core algorithms under `src/quadrotor_math`. Keep them independent of ROS 2 and PX4.
- Use NED world coordinates and FRD body coordinates.
- Define `R_WB` as mapping body-coordinate vectors into world coordinates.
- Public quaternion names must state their frame direction. Never expose an ambiguous variable named `q`.

## Engineering Practice

- Begin every new scoped implementation step by auditing the preceding milestone
  against current source, tests, documentation and fresh relevant checks. Record
  the audited commit and findings; fix demonstrated in-scope defects, otherwise
  leave the existing design intact. Define the next step and its acceptance
  criteria before implementing it.
- Work in small, testable increments.
- Add or update tests with every behavioral code change.
- Use deterministic random-number generators with explicit seeds.
- Prefer typed, documented interfaces and explicit checks for numerical shapes and units.
- Fix frame, sign, numerical, and test failures at their source; never hide them by tuning around them.
- Do not add dependencies, change tool versions, or modify system configuration without explaining why.
- Do not commit generated results or large logs.

## Completion

- Before finishing a task, run the relevant tests and lint checks. Report the exact commands and their results.
- At the end of each scoped step, assess the quality and limits of the results, state explicitly whether the step met its acceptance criteria, and give the next concrete step.
- Never commit changes unless the repository owner explicitly requests publication.
- Keep documentation in the project author's voice. Do not identify the repository owner by name or include personal home-directory paths. Explain current design choices in guides and preserve dated numerical evidence in progress records.
