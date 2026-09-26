# 2026-08-16: Week 2 Compatibility Spike

## Objective

Verify that the pinned ROS 2, Gazebo, PX4, and QGroundControl environment can support the
project's planned simulation workflow under WSL2.

## Environment Decisions

- Use PX4 v1.17.0 at commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
- Keep default PX4 terminals free of ROS environment setup and use `/usr/bin/gz` version
  8.15.0 with an empty `GZ_CONFIG_PATH`.
- Source `/opt/ros/jazzy/setup.bash` manually only in dedicated ROS terminals, and do not
  build or launch PX4 from a ROS-activated terminal.
- Use QGroundControl v5.1 stable AppImage for the simulation ground station.

## Concepts Learned

- ROS 2 Jazzy can communicate with its vendor Gazebo through `ros-jazzy-ros-gz`, while PX4
  SITL requires a clean terminal that resolves its system Gazebo independently.
- Gazebo is usable on the laptop but graphically laggy, so headless simulation will be
  important for routine work.
- Gazebo's full reset is not safe for this PX4 SITL workflow. A reset removed the dynamically
  spawned x500 model while leaving the PX4 bridge running. The world then contained only
  `ground_plane` and `sunUTC`, and the lockstep clock returned to 4000 us. A clean shutdown
  and relaunch restored the model. Do not use the full reset button during PX4 SITL; relaunch
  the complete session instead.
- Brief `NodeShared::Publish` interrupted-system-call and vehicle timestamp warnings appeared
  around clock discontinuities. They were not persistent and should continue to be monitored.

## Changes

- Recorded the pinned compatibility matrix and the required separation between ROS and PX4
  terminal environments.
- Added this compatibility-spike verification record.
- No code, dependencies, generated files, or system configuration were changed. No future
  dynamics, estimation, planning, control, or fault-handling work has begun.

## Verification Evidence

- The ROS 2 C++ talker/Python listener test passed.
- The ROS--Gazebo IMU bridge test passed.
- PX4 v1.17.0 built successfully with `make px4_sitl gz_x500`.
- Gazebo detected version 8.15.0.
- QGroundControl connected to PX4 over simulation UDP.
- `commander check` returned `Preflight check: OK`.
- The x500 completed a normal takeoff to the default 2.5 m altitude, stable hover, landing,
  and automatic disarm.
- The final commander state was Disarmed, Hold mode, with no failsafe.
- The main repository remained clean after the compatibility test.
- `make check` passed Ruff linting, Ruff format verification, strict mypy checking, and 2 tests.
- `git diff --check` passed with no whitespace errors.

## Blockers

No compatibility blocker was found. Gazebo's graphical performance is limited on the laptop,
and the transient clock-discontinuity warnings require continued monitoring.

## Next Exact Action

Review and publish the compatibility documentation, then select the next
bounded task from the six-month project plan.
