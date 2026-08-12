# Robust Autonomous Quadrotor

> “How can a quadrotor estimate its state and track dynamically feasible trajectories when its sensors, actuators, and mathematical model are imperfect?”

This project will build and validate a simulated quadrotor stack involving nonlinear dynamics, state estimation, trajectory generation, feedback control, fault handling, Monte Carlo validation, ROS 2, and PX4/Gazebo integration.

**Current status:** Week 1, Sprint 1 — environment and reproducible workflow.

The canonical development environment is a Lenovo Yoga 9 running a Windows host with WSL2, Ubuntu 24.04 LTS, and Python 3.12.

## Tested Setup

With uv 0.12.3 installed:

```sh
uv sync
make check
```

Core algorithms live in `src/quadrotor_math`, and tests live under `tests/`.
