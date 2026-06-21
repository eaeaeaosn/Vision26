# 02 — Directory & File Guide

A file-by-file map of the repository. Each table lists the purpose of a file and, where
useful, how it is used.

```
Vision26/
├── src/           application layer — main() programs (one per robot)
├── tasks/         function layer  — the algorithms
├── io/            hardware layer  — cameras, board, IMU
├── tools/         utility layer   — shared helpers
├── calibration/   camera & hand-eye calibration programs
├── tests/         standalone test programs (one feature each)
├── configs/       per-robot YAML config files
├── assets/        NN weights + demo material
├── records/       rosbag-like recordings (image + quaternion + timestamp)
├── logs/          runtime log files
├── patterns/ imgs/  reference images
├── CMakeLists.txt  build definition (libraries + every executable)
├── autostart.sh    boot launcher (registered via ~/.config/autostart)
└── readme.md       team open-source writeup (theory, results)
```

---

## `src/` — Application layer (the `main()` programs)

One executable per robot / use-case. Pick the one that matches the hardware. Pass a
config path as the first argument (default shown in each file's `keys`).

| File | Program | What it runs |
|------|---------|--------------|
| `standard.cpp` | `standard` | Single-thread infantry: auto-aim (+ buff) full pipeline. **Best starting point to read.** |
| `mt_standard.cpp` | `mt_standard` | Multithreaded infantry — higher frame rate. |
| `standard_mpc.cpp` | `standard_mpc` | Infantry using the MPC **trajectory planner** instead of the classic aimer. |
| `auto_aim_debug_mpc.cpp` | `auto_aim_debug_mpc` | Auto-aim + planner debugging build. |
| `mt_auto_aim_debug.cpp` | `mt_auto_aim_debug` | Multithreaded auto-aim debugging build. |
| `auto_buff_debug.cpp` | `auto_buff_debug` | Buff (power-rune) debugging. |
| `auto_buff_debug_mpc.cpp` | `auto_buff_debug_mpc` | Buff debugging with planner. |
| `sentry.cpp`, `sentry_debug.cpp`, `sentry_bp.cpp`, `sentry_multithread.cpp` | `sentry*` | Sentry robot variants. **Only built when a ROS 2 environment is found** (they use `omniperception` + the `io/ros2` nav bridge). |
| `uav.cpp`, `uav_debug.cpp` | `uav` / `uav_debug` | Aerial-robot (drone) auto-aim. Uses `Command::horizon_distance`. |

> **Naming conventions:** `mt_` = multithreaded, `_mpc` = uses trajectory planner,
> `_debug` = extra visualization/plotting, no `_` prefix = deploy build.

---

## `tasks/` — Function layer (the algorithms)

### `tasks/auto_aim/` — armor auto-aim

| File | Class / role |
|------|--------------|
| `armor.{hpp,cpp}` | `Armor` data type — 4 light-bar corners, class/`ArmorName`, `ArmorType`, color, and (after solving) 3-D pose. The currency passed between modules. |
| `detector.{hpp,cpp}` | `Detector` — **traditional** image-processing detector (light-bar pairing). Tunable by the `传统方法参数` block in the YAML. |
| `classifier.{hpp,cpp}` | `Classifier` — small CNN (`tiny_resnet.onnx`) that labels the armor digit/pattern. |
| `yolo.{hpp,cpp}` + `yolos/` | `YOLO` — neural-net detector wrapper. `yolos/` holds the concrete backends: `yolov5`, `yolov5_trt` (TensorRT), `yolov8`, `yolo11`. The backend is chosen by `yolo_name` in the YAML. |
| `solver.{hpp,cpp}` | `Solver` — PnP pose estimation, camera↔gimbal↔world transforms, and **yaw optimisation** (reprojection-error minimisation). Needs the gimbal quaternion via `set_R_gimbal2world()`. |
| `target.{hpp,cpp}` | `Target` — the estimated whole-robot motion state (center, velocity, yaw spin, radii, heights). |
| `tracker.{hpp,cpp}` | `Tracker` — associates armors across frames and drives the EKF to produce `Target`s. Handles temp-loss counting and outpost mode. |
| `aimer.{hpp,cpp}` | `Aimer` — classic decision logic: choose aim point + fire timing → `io::Command`. Tuned by the `aimer参数` block. |
| `shooter.{hpp,cpp}` | `Shooter` — fire-control / tolerance logic. |
| `voter.{hpp,cpp}` | `Voter` — voting / arbitration helper for target selection. |
| `multithread/` | `mt_detector` and `commandgener` — threaded detection and command generation used by the `mt_*` programs. |
| `planner/` | The **trajectory planner** (see below). |

#### `tasks/auto_aim/planner/`

| File | Role |
|------|------|
| `planner.{hpp,cpp}` | `Planner` — the "auto-aim trajectory planning" decision maker; replaces `Aimer` in `*_mpc` builds. Produces a `Plan` (gimbal yaw/pitch + vel + acc + fire flag over a horizon). |
| `tinympc/` | Bundled **TinyMPC** QP solver (ADMM) used by the planner: `tiny_api`, `admm`, `codegen`, `rho_benchmark`, `types.hpp`. Has its own `CMakeLists.txt`. |

### `tasks/auto_buff/` — power-rune (buff) shooting

| File | Role |
|------|------|
| `buff_type.{hpp,cpp}` | Core types: `FanBlade`, `PowerRune`, `Track_status`. |
| `yolo11_buff.{hpp,cpp}` | `YOLO11_BUFF` — NN detector for buff (uses `yolo11_buff_int8.xml`). |
| `buff_detector.{hpp,cpp}` | `Buff_Detector` — finds the rune R-center and active fan blade. |
| `buff_solver.{hpp,cpp}` | Pose solving for the rune. |
| `buff_predict.hpp` | Rotation prediction (the rune spins by a known/estimated law). |
| `buff_target.{hpp,cpp}` | Buff target state. |
| `buff_aimer.{hpp,cpp}` | Aim + fire decision for buff. |

### `tasks/omniperception/` — 360° awareness (multi-camera)

| File | Role |
|------|------|
| `perceptron.{hpp,cpp}` | `Perceptron` — runs up to 4 USB cameras in parallel threads, each with its own `YOLO`, and aggregates a detection queue. |
| `detection.hpp` | `DetectionResult` type. |
| `decider.{hpp,cpp}` | `Decider` — picks which detection/target to act on. Feeds the auto-aim `Tracker`. |

---

## `io/` — Hardware abstraction layer

| File / dir | Role |
|------------|------|
| `camera.{hpp,cpp}` | `Camera` — a thin façade over the concrete camera SDK. Constructed from the config; `read(img, t)` returns frame + timestamp. Implements `CameraBase`. |
| `hikrobot/` | HikRobot industrial-camera driver (`include/`, prebuilt `lib/`). |
| `mindvision/` | MindVision industrial-camera driver (`include/`, prebuilt `lib/`). |
| `usbcamera/` | `USBCamera` — plain USB/UVC cameras (used by omni-perception). |
| `cboard.{hpp,cpp}` | `CBoard` — link to the RoboMaster C-board: provides `imu_at(t)` (attitude lookup by timestamp), `mode`, `bullet_speed`, and `send(command)`. Defines `enum Mode` and `ShootMode`. |
| `command.hpp` | `io::Command` struct — the vision→board message (control, shoot, yaw, pitch, horizon_distance). |
| `gimbal/` | `Gimbal` — newer serial-based gimbal link; defines the packed wire structs `GimbalToVision` / `VisionToGimbal`. |
| `dm_imu/` | Damiao (达妙) external IMU driver. |
| `serial/` | Vendored serial-port library (`include/`, `src/`). |
| `socketcan.hpp` | SocketCAN helper for USB2CAN transport. |
| `ros2/` | Optional ROS 2 bridge for sentry↔navigation: `ros2`, `publish2nav`, `subscribe2nav`. Only built when ROS is available. |

---

## `tools/` — Utility layer

All header-light helpers shared everywhere. Most are a `.hpp` + `.cpp` pair.

| File | Role |
|------|------|
| `extended_kalman_filter.{hpp,cpp}` | Generic EKF used by the trackers. |
| `math_tools.{hpp,cpp}` | Euler/quaternion helpers (e.g. `tools::eulers`), angle math. |
| `trajectory.{hpp,cpp}` | Ballistic trajectory / bullet-drop solving. |
| `pid.{hpp,cpp}` | PID controller. |
| `ransac_sine_fitter.{hpp,cpp}` | RANSAC sine fit — used for buff rotation-speed estimation. |
| `img_tools.{hpp,cpp}` | Drawing/annotation helpers for debug images. |
| `logger.{hpp,cpp}` | `tools::logger()` — spdlog wrapper; writes to `logs/`. |
| `plotter.{hpp,cpp}` | `Plotter` — streams data to PlotJuggler for live curves. |
| `recorder.{hpp,cpp}` | `Recorder` — rosbag-like recording (image + quaternion + timestamp) into `records/`. |
| `crc.{hpp,cpp}` | CRC16 checksums for serial frames. |
| `exiter.{hpp,cpp}` | `Exiter` — Ctrl-C / signal-based clean shutdown (`exiter.exit()`). |
| `thread_safe_queue.hpp` | Lock-based queue for inter-thread handoff. |
| `thread_pool.hpp` | Simple thread pool (omni-perception, multithread builds). |
| `yaml.hpp` | Thin yaml-cpp wrapper for reading config values. |

---

## `calibration/` — Calibration programs

| File | Program | Purpose |
|------|---------|---------|
| `capture.cpp` | `capture` | Grab calibration images from the camera. |
| `calibrate_camera.cpp` | `calibrate_camera` | Camera **intrinsics** calibration. |
| `calibrate_handeye.cpp` | `calibrate_handeye` | **Hand-eye** calibration (camera↔gimbal). |
| `calibrate_robotworld_handeye.cpp` | `calibrate_robotworld_handeye` | Hand-eye + board-pose calibration. |
| `split_video.cpp` | `split_video` | Split a recorded video into frames. |

Calibration outputs feed `configs/calibration.yaml` and the camera matrix used by `Solver`.

---

## `tests/` — Standalone test programs

Each builds to an executable of the same stem. Use them to debug one module without the
whole pipeline.

| File | Tests |
|------|-------|
| `auto_aim_test.cpp` | Auto-aim on a **recorded video** (the quick demo: `./build/auto_aim_test`). |
| `auto_buff_test.cpp` | Buff on a recorded video. |
| `detector_video_test.cpp` | Detector on a video file. |
| `camera_test.cpp` | Industrial camera grab. |
| `camera_thread_test.cpp` | Threaded camera grab. |
| `camera_detect_test.cpp` | Detector live on the industrial camera. |
| `usbcamera_test.cpp` / `usbcamera_detect_test.cpp` | USB camera grab / detection. |
| `multi_usbcamera_test.cpp` | Multiple USB cameras (omni-perception). |
| `cboard_test.cpp` | C-board communication. |
| `gimbal_test.cpp` / `gimbal_response_test.cpp` | Gimbal serial link / response. |
| `dm_test.cpp` | Damiao IMU. |
| `fire_test.cpp` | Fire/shoot path. |
| `handeye_test.cpp` | Hand-eye calibration result. |
| `planner_test.cpp` / `planner_test_offline.cpp` | Trajectory planner on robot / offline. |
| `minimum_vision_system.cpp` | Smallest end-to-end pipeline. |
| `publish_test.cpp` / `subscribe_test.cpp` / `topic_loop_test.cpp` | ROS 2 messaging — **only built when ROS 2 is found.** |

> Not every test file is wired as a CMake target, and the ROS-related ones
> (`publish/subscribe/topic_loop_test` and the `sentry*` programs) are gated behind a
> "ROS 2 found" check in `CMakeLists.txt`. Check the `##### tests #####` block for the
> exact set currently built.

---

## `configs/` — Per-robot YAML

One file per robot / scenario, passed to the program on the command line. They set the
enemy color, which detector/weights to use, ROI, detector thresholds, tracker/aimer/
shooter parameters, camera intrinsics, and offsets.

| File | For |
|------|-----|
| `standard3.yaml`, `standard4.yaml` | Infantry robots #3 / #4 (the default for `standard`). |
| `sentry.yaml` | Sentry. |
| `uav.yaml` | Aerial robot. |
| `ascento.yaml` | Ascento (balance) robot. |
| `camera.yaml`, `mvs.yaml` | Camera-only configs. |
| `calibration.yaml` | Calibration parameters / results. |
| `demo.yaml`, `example.yaml` | Demo / template to copy from. |

See [04-development-guide.md](04-development-guide.md) for the key fields.

---

## `assets/` — Weights & demo material

Neural-network weights in several formats (OpenVINO `.xml`/`.bin`, TensorRT `.engine`,
ONNX) plus demo images:

| File | Used by |
|------|---------|
| `yolov5.xml/.bin`, `yolov5.engine`, `yolov5_trt.onnx` | YOLOv5 armor detector (OpenVINO / TensorRT). |
| `yolov8.xml/.bin`, `yolo11.xml/.bin` | YOLOv8 / YOLO11 armor detectors. |
| `tiny_resnet.onnx` | Armor digit `Classifier`. |
| `yolo11_buff_int8.xml/.bin` | Buff detector. |
| `best2-sim.onnx`, `standard_fanblade.jpg` | Buff assets. |
| `demo/`, `img_with_q/` | Demo videos / images-with-quaternion for offline tests. |

The path to each is set in the YAML (`yolov5_model_path`, `classify_model`, etc.).

---

## Generated / runtime folders

| Folder | Contents | Note |
|--------|----------|------|
| `build/` | CMake/Ninja build output and all compiled executables. | Generated; do not edit. |
| `logs/` | Timestamped runtime logs from `tools::logger`. | Generated. |
| `records/` | rosbag-like recordings from `tools::Recorder`. | Generated. |

Next: [03-build-and-run.md](03-build-and-run.md).
