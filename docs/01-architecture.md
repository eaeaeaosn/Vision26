# 01 — Architecture

## Why a framework instead of separate programs?

A robot needs several vision features (auto-aim, buff, …), but they **cannot be separate
processes**: they all need the camera, and a camera can only be opened by one process.

So `sp_vision` is a **single framework**. There is exactly one running program (one
`main()`), which:

1. Opens the camera and the gimbal board once.
2. Reads the **mode** sent by the electrical-control board (idle / auto-aim / small-buff
   / big-buff / outpost).
3. Dispatches each frame to the matching **feature group** (auto-aim, auto-buff, …).
4. Sends the resulting command back to the board.

Different robots have different dispatch logic, so each robot gets its **own `main()`**
in `src/` — but they all reuse the same shared libraries.

## The four layers

```
┌──────────────────────────────────────────────────────────────┐
│  src/        APPLICATION   one main() per robot                │
│              standard / sentry / uav / mt_standard / *_debug   │
└───────────────┬──────────────────────────────────────────────┘
                │ calls
┌───────────────▼──────────────────────────────────────────────┐
│  tasks/      FUNCTION       the algorithms                     │
│   ├ auto_aim        detector → solver → tracker → aimer/planner│
│   ├ auto_buff       buff detector → solver → predictor → aimer │
│   └ omniperception  multi-camera detection & target picking    │
└───────────────┬──────────────────────────────────────────────┘
                │ uses
┌───────────────▼──────────────────────────────────────────────┐
│  io/         HARDWARE       hardware abstraction layer         │
│   camera (hik/mindvision/usb) · cboard · gimbal · dm_imu · ros2│
└───────────────┬──────────────────────────────────────────────┘
                │ uses
┌───────────────▼──────────────────────────────────────────────┐
│  tools/      UTILITIES      EKF · logger · math · plotter · …  │
└──────────────────────────────────────────────────────────────┘
```

Each layer is built as a CMake target (see `CMakeLists.txt`):
`add_subdirectory(tools)`, `add_subdirectory(io)`, `add_subdirectory(tasks/auto_aim)`,
etc. Higher layers link against lower ones; never the reverse.

## The auto-aim data flow

This is the canonical pipeline. The cleanest place to read it is
[`src/standard.cpp`](../src/standard.cpp); here is the heart of its loop, annotated:

```cpp
io::CBoard cboard(config_path);           // serial/CAN link to the gimbal board + IMU
io::Camera camera(config_path);           // industrial camera (Hik or MindVision)

auto_aim::YOLO    detector(config_path);  // armor detector (NN or traditional)
auto_aim::Solver  solver(config_path);    // PnP pose + yaw optimisation
auto_aim::Tracker tracker(config_path, solver);  // EKF whole-robot state estimator
auto_aim::Aimer   aimer(config_path);     // aim position + fire decision
auto_aim::Shooter shooter(config_path);

while (!exiter.exit()) {
  camera.read(img, t);                    // 1. grab frame + timestamp
  q = cboard.imu_at(t - 1ms);             // 2. gimbal attitude (quaternion) at that time
  mode = cboard.mode;                     //    current mode from the board

  solver.set_R_gimbal2world(q);           // 3. align image to world via the quaternion
  auto armors  = detector.detect(img);    // 4. detect armor plates (4 corners + class)
  auto targets = tracker.track(armors, t);// 5. estimate enemy motion state
  auto command = aimer.aim(targets, t, cboard.bullet_speed);  // 6. decide aim & fire
  cboard.send(command);                   // 7. send command back to the board
}
```

So the module responsibilities are:

| Step | Module | In → Out |
|------|--------|----------|
| 1 | `io::Camera` | — → `cv::Mat` image + timestamp |
| 2 | `io::CBoard` | timestamp → gimbal quaternion, mode, bullet speed |
| 3–4 | `auto_aim::YOLO` / `Detector` | image → list of `Armor` (4 pixel corners + class) |
| 4 | `auto_aim::Solver` | `Armor` → 3-D pose in world (PnP + yaw optimisation) |
| 5 | `auto_aim::Tracker` → `Target` | armors over time → enemy whole-robot motion state (EKF) |
| 6 | `auto_aim::Aimer` / `Planner` | target state + bullet speed → `io::Command` (aim yaw/pitch + fire flag) |
| 7 | `io::CBoard` | `io::Command` → bytes on the wire |

`Planner` (under `tasks/auto_aim/planner/`) is the **trajectory-planning** alternative
to `Aimer`. The `*_mpc` executables use it; it solves a small MPC problem (via the
bundled TinyMPC) to produce a gimbal trajectory that respects the gimbal's max
acceleration. See the theory section of [`../readme.md`](../readme.md).

## The communication contract (vision ↔ board)

Defined as packed structs in [`io/gimbal/gimbal.hpp`](../io/gimbal/gimbal.hpp) and the
mode/command types in [`io/cboard.hpp`](../io/cboard.hpp) and
[`io/command.hpp`](../io/command.hpp):

- **Board → Vision** (`GimbalToVision`): mode, IMU quaternion `q[4]`, yaw/pitch +
  velocities, bullet speed, bullet count.
- **Vision → Board** (`VisionToGimbal` / `io::Command`): control flag, fire flag, target
  yaw/pitch (+ velocity/acceleration feed-forward).

Modes (`io::Mode`): `idle`, `auto_aim`, `small_buff`, `big_buff`, `outpost`.
The `main()` switches feature group on the mode it receives.

## Multithreading

The single-thread loop above is the simplest form. Higher-frame-rate variants
(`mt_standard`, `mt_auto_aim_debug`) split capture / inference / sending across threads
using `tools/thread_safe_queue.hpp` and `tools/thread_pool.hpp`, with the
multithread detector in `tasks/auto_aim/multithread/`.

Next: [02-directory-guide.md](02-directory-guide.md).
