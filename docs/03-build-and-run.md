# 03 — Build & Run

## Prerequisites

From the top-level [`../readme.md`](../readme.md):

- **OS:** Ubuntu 22.04
- A camera SDK — [MindVision](https://mindvision.com.cn/category/software/sdk-installation-package/)
  **or** [HikRobot MVS](https://www.hikrobotics.com/).
- [OpenVINO](https://docs.openvino.ai/) (the `CMakeLists.txt` expects
  `/opt/intel/openvino_2024.6.0/` — adjust `OpenVINO_DIR` there if yours differs).
- [Ceres](http://ceres-solver.org/) (used by the buff builds).
- System packages:

  ```bash
  sudo apt install -y git g++ cmake can-utils libopencv-dev libfmt-dev \
      libeigen3-dev libspdlog-dev libyaml-cpp-dev libusb-1.0-0-dev \
      nlohmann-json3-dev openssh-server screen
  ```

## Build

```bash
cmake -B build
make -C build -j$(nproc)
```

All executables land in `build/` (next to `build/standard`, `build/auto_aim_test`, …).

> **Optional GPU inference, serial / CAN udev rules, and autostart registration** are
> covered step-by-step in [`../readme.md`](../readme.md) §3.2 (items 4–7). Summary:
> - Autostart: create `~/.config/autostart/sp_vision.desktop` pointing (absolute path)
>   at `autostart.sh`, and `chmod +x autostart.sh`.
> - Serial: add yourself to `dialout`, create a udev rule giving the board a stable
>   `SYMLINK` name (e.g. `/dev/gimbal`).

## Quick smoke test (no hardware needed)

Runs auto-aim over a recorded video in `assets/`:

```bash
./build/auto_aim_test
```

## Running on the robot

Every `main()` takes a **config path** as its first positional argument; each has a
default baked in (e.g. `standard` defaults to `configs/standard3.yaml`). So:

```bash
./build/standard                       # uses the default config
./build/standard configs/standard4.yaml   # explicit config
./build/standard --help                # print the argument help
```

Pick the executable for your robot and the config that matches it:

| Robot / case | Run | Typical config |
|--------------|-----|----------------|
| Infantry (deploy) | `./build/standard` | `configs/standard3.yaml` / `standard4.yaml` |
| Infantry (high FPS) | `./build/mt_standard` | `configs/standard*.yaml` |
| Infantry (planner) | `./build/standard_mpc` | `configs/standard*.yaml` |
| Sentry | `./build/sentry` *(needs ROS 2 at build time)* | `configs/sentry.yaml` |
| Drone | `./build/uav` | `configs/uav.yaml` |
| Buff debugging | `./build/auto_buff_debug` | `configs/standard*.yaml` |

## The full target list

`CMakeLists.txt` defines ~39 executables in four groups. To list them after building:

```bash
grep add_executable CMakeLists.txt
```

- **`#### src ####`** — the deploy/debug `main()` programs (see
  [02-directory-guide.md](02-directory-guide.md#src--application-layer-the-main-programs)).
- **`## calibration ##`** — `capture`, `calibrate_camera`, `calibrate_handeye`,
  `calibrate_robotworld_handeye`, `split_video`.
- **`## tests ##`** — per-module test programs.

Library targets `tools`, `io`, `auto_aim`, `auto_buff`, `omniperception` are built from
the `add_subdirectory(...)` lines and linked into the executables — you rarely invoke
them directly.

## Calibration workflow

1. `./build/capture` — collect chessboard images.
2. `./build/calibrate_camera` — compute intrinsics.
3. `./build/calibrate_handeye` (or `calibrate_robotworld_handeye`) — camera↔gimbal.
4. Copy the results into the robot's YAML / `configs/calibration.yaml`.
5. Verify with `./build/handeye_test`.

## Debugging aids

- **Logs:** written to `logs/<timestamp>.log` by `tools::logger`.
- **Live curves:** `*_debug` builds use `tools::Plotter` → open PlotJuggler to watch
  yaw tracking, fire decisions, target angular velocity, etc.
- **Recording / replay:** uncomment `recorder.record(...)` in a `main()` to save
  image+quaternion+timestamp into `records/`; the `*_test` / offline programs replay
  these for "reproduce a bug after the match" debugging.

Next: [04-development-guide.md](04-development-guide.md).
