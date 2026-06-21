# Vision26 (sp_vision) — Developer Documentation

This folder is a **practical map of the codebase**: what every directory and file is
for, how the pieces fit together, and how to build, run, and extend the project.

> If you are looking for the *theory* behind the system (the "trajectory view of
> auto-aim", hit-rate / kill-time analysis, the trajectory planner derivation, the
> competition results), read the top-level [`../readme.md`](../readme.md). That is the
> team's open-source writeup. **This `docs/` folder is the engineering guide.**

## What is this project?

`sp_vision` is the **vision/decision software** for a RoboMaster robot. It runs on the
on-board mini-PC (NUC) and is responsible for the full auto-aim pipeline:

```
camera image  ─►  armor detection  ─►  pose solving  ─►  target estimation  ─►  aim & fire decision  ─►  command to the gimbal board
```

It also supports **auto-buff** (打符 / power-rune shooting), **omni-perception**
(multiple USB cameras for 360° awareness), camera/hand-eye **calibration**, and a large
set of **standalone test programs** so each module can be debugged in isolation.

The project has **no ROS dependency** for its core (a small ROS interface is reserved
only for sentry-to-navigation communication).

## How the docs are organised

| Doc | Read it when you want to… |
|-----|---------------------------|
| [01-architecture.md](01-architecture.md) | Understand the 4-layer design and how a `main()` wires the pipeline together. |
| [02-directory-guide.md](02-directory-guide.md) | Find out what a specific folder or file is for. |
| [03-build-and-run.md](03-build-and-run.md) | Build the code, and know which executable / config to run. |
| [04-development-guide.md](04-development-guide.md) | Add a new robot, swap a detector, or change a config — and not break anything. |

## 30-second mental model

The code is split into **four layers**, each a CMake library or set of executables:

| Layer | Folder | Role |
|-------|--------|------|
| **Application** | `src/` | The `main()` programs — one per robot/use-case (`standard`, `sentry`, `uav`, …). Each picks which features to run. |
| **Function (tasks)** | `tasks/` | The actual algorithms: `auto_aim`, `auto_buff`, `omniperception`. |
| **Hardware (io)** | `io/` | Hardware abstraction: cameras, gimbal/C-board serial & CAN, IMU. |
| **Tools** | `tools/` | Shared utilities: Kalman filter, logger, math, plotting, recording, CRC, YAML. |

Everything is configured by a single **YAML file** under `configs/` that is passed to
the program on the command line. Neural-network weights and demo material live in
`assets/`.

Start with [01-architecture.md](01-architecture.md).
