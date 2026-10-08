# Technical documentation

Start with the [project overview](../README.md), then follow this path:

1. [Run the planning example and a virtual flight](guides/getting-started.md).
2. [Understand the complete flight loop](guides/system-design.md).
3. Use the subsystem map below to connect equations with implementation.
4. [Inspect results, validation and limitations](results/README.md).
5. Read the [official report](report.md) for the complete technical write-up.

## Models and estimation

| Topic | Explanation | Main implementation |
| --- | --- | --- |
| Coordinate frames | [Frame contract](architecture/frame-contract.md) | [Rotations](../src/quadrotor_math/rotations.py) |
| Plant, motors and sensors | [Foundations](design/foundations.md) | [Dynamics](../src/quadrotor_math/dynamics.py), [actuation](../src/quadrotor_math/actuation.py), [IMU](../src/quadrotor_math/imu.py) |
| Recorded runs | [Configuration and random streams](design/foundations.md#reproducible-run-configuration-and-named-random-streams) | [Generation](../src/quadrotor_math/run_generation.py), [artifact persistence](../src/quadrotor_math/run_artifact.py) |
| State estimation | [Error-state Kalman filter](design/estimation.md) | [Prediction and correction](../src/quadrotor_math/eskf.py), [endpoint model](../src/quadrotor_math/eskf_endpoint.py) |
| Supported initialization | [Alignment assumptions](design/prearm-alignment.md), [nonlinear uncertainty](design/prearm-nonlinear-uncertainty.md), [component contract](design/prearm-component.md) | [Pre-arm alignment](../src/quadrotor_math/prearm_alignment.py) |
| Support removal | [First prediction interval](design/release-prediction.md), [nonlinear release study](results/nonlinear-release.md) | [Release prediction](../experiments/release_prediction.py), [joint moment model](../experiments/nonlinear_release.py) |
| Observation availability | [Health monitor](design/observation-health.md), [timed supervision](design/observation-supervision.md) | [Health](../src/quadrotor_math/observation_health.py), [supervisor](../src/quadrotor_math/observation_supervision.py) |

## Control and planning

| Topic | Explanation | Main implementation |
| --- | --- | --- |
| Attitude control | [Cascade inner loop](design/control.md) | [Attitude control](../src/quadrotor_math/attitude_control.py) |
| Position control | [Position loop and missions](design/position-control.md) | [Position control](../src/quadrotor_math/position_control.py), [mission state machine](../src/quadrotor_math/missions.py) |
| Estimated feedback | [Sensor-to-controller integration](design/estimated-feedback.md), [bandwidth design](design/feedback-design.md) | [Online ESKF](../src/quadrotor_math/eskf_online.py), [estimated missions](../src/quadrotor_math/estimated_mission.py) |
| Geometric control | [Rotation errors and force derivatives](design/geometric-control.md), [final comparison](results/final-geometric.md) | [Moment law](../src/quadrotor_math/geometric_control.py), [reference](../src/quadrotor_math/geometric_reference.py), [filter](../src/quadrotor_math/geometric_filter.py) |
| Trajectory planning | [Minimum-snap solver](design/trajectories.md) | [Minimum snap](../src/quadrotor_math/minimum_snap.py) |
| Trajectory execution | [Bounds, retiming and missions](design/trajectory-missions.md) | [Feasibility](../src/quadrotor_math/trajectory_feasibility.py), [mission execution](../src/quadrotor_math/mission_simulation.py) |

Each study in [results](results/README.md) links its experiment, assumptions,
saved evidence and interpretation. Run modules from the repository root with
`uv run python -m experiments.NAME --help` before choosing a campaign. Some
stages require authenticated outputs from earlier stages; the study guides give
the required order. Previously inspected seeds are reproduction cases, not fresh
validation.

For deeper reasoning, use the [design decision index](decisions/README.md).
[Development and validation practices](development.md) describes the checks,
reproducibility contract and tooling. [Next steps](next-steps.md) contains planned
work. [Historical material](archive/README.md) is retained separately and does
not define the current project scope.
