# 04 — Development Guide

How to make common changes without fighting the architecture.

## Golden rules

1. **Respect the layers.** `src/` → `tasks/` → `io/` → `tools/`. A lower layer must
   never include a higher one. If `tools/` needs to know about an `Armor`, the design is
   wrong.
2. **Everything tunable goes in the YAML**, not as a hard-coded constant. Read it through
   `tools/yaml.hpp` in the module's constructor (every module takes a `config_path`).
3. **One `main()` per robot.** Don't add `if (robot == ...)` branches to an existing
   `main()`; copy it and register a new target.
4. **Pass data as the existing types** (`Armor`, `Target`, `io::Command`) so modules stay
   swappable.

## Where do I put…?

| I want to… | Put it in | Then |
|------------|-----------|------|
| A new shared helper (math, IO format, filter) | `tools/` | Add the `.cpp` to `tools/CMakeLists.txt`. |
| A new camera / board / IMU driver | `io/<name>/` | Implement the relevant base (`io::CameraBase` for cameras); register in `io/CMakeLists.txt`. |
| A new detector backend | `tasks/auto_aim/yolos/` | Implement `YOLOBase`; select it via `yolo_name` in the YAML and wire it in `yolo.cpp`. |
| A new aim/fire strategy | `tasks/auto_aim/` | Produce an `io::Command`; swap it into the `main()`. |
| A whole new feature group | `tasks/<feature>/` | Give it its own `CMakeLists.txt` + `add_subdirectory` in the root. |
| A new robot program | `src/<robot>.cpp` | Add `add_executable` + `target_link_libraries` in the root `CMakeLists.txt`. |
| A way to test one module | `tests/<thing>_test.cpp` | Add it to the `tests` block of `CMakeLists.txt`. |

## Adding a new robot program

1. Copy the closest existing `main()` (usually `src/standard.cpp`).
2. Change the default config in its `keys` string.
3. Adjust which feature groups it dispatches (auto-aim vs buff) based on `cboard.mode`.
4. In the root `CMakeLists.txt`, under `#### src ####`:
   ```cmake
   add_executable(myrobot src/myrobot.cpp)
   target_link_libraries(myrobot ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
   ```
5. Create `configs/myrobot.yaml` (copy `configs/example.yaml` or a `standard*.yaml`).
6. `cmake -B build && make -C build -j$(nproc)`.

## Adding / swapping a detector backend

The detector is abstracted behind `auto_aim::YOLO`, which holds a `YOLOBase`
(`tasks/auto_aim/yolo.hpp`). The concrete backends live in `tasks/auto_aim/yolos/`
(`yolov5`, `yolov5_trt`, `yolov8`, `yolo11`).

1. Add `yolos/mynet.{hpp,cpp}` implementing `YOLOBase::detect/postprocess`.
2. Construct it in `yolo.cpp` when `yolo_name == "mynet"`.
3. Drop the weights in `assets/` and point the YAML at them.
4. Add the source to `tasks/auto_aim/CMakeLists.txt`.

Output must be a `std::list<auto_aim::Armor>` so the rest of the pipeline is untouched.

## Understanding a config file

Configs are grouped by the module that reads them. From `configs/standard3.yaml`:

```yaml
enemy_color: "red"                    # which color to shoot

#####-----neural network-----#####
yolo_name: yolov5_trt                 # which backend in yolos/ to use
classify_model: assets/tiny_resnet.onnx
yolov5_model_path: assets/yolov5.xml
device: CPU                           # CPU | GPU (OpenVINO)
min_confidence: 0.8
use_traditional: true                 # also run the classic Detector

#####-----ROI-----#####
roi: { x: 420, y: 50, width: 600, height: 600 }
use_roi: false

#####-----traditional detector-----#####
threshold: 150
max_angle_error: 45                   # light-bar geometry tolerances
min_lightbar_ratio: 1.5
...

#####-----tracker-----#####
min_detect_count: 5                   # frames before a target is "confirmed"
max_temp_lost_count: 15               # frames kept after losing sight

#####-----aimer-----#####
yaw_offset: 2                         # mechanical aim offsets (degrees)
pitch_offset: 6.5
decision_speed: 7                     # rad/s threshold for high/low-speed behavior
low_speed_delay_time: 0.0             # prediction-time offset (planner)
```

The field names map directly to the private members you saw in each module's header
(e.g. `min_detect_count` → `Tracker::min_detect_count_`,
`yaw_offset` → `Aimer::yaw_offset_`). To find what a field does, grep the module:

```bash
grep -rn "yaml\[\"yaw_offset\"\]" tasks/
```

## Code style

`.clang-format` is provided. Format before committing:

```bash
clang-format -i path/to/file.cpp
```

## Debugging recipe

1. Reproduce offline: record a session (`tools::Recorder`) or use clips in `assets/`,
   then run the matching `*_test` / `*_debug` build.
2. Watch live signals in **PlotJuggler** via `tools::Plotter`.
3. Check `logs/` for what the run actually did.
4. Isolate the layer with a `tests/` program (camera vs detector vs board) before
   blaming the full pipeline.

See [`../readme.md`](../readme.md) for the algorithmic theory behind the planner and the
trajectory view of auto-aim.
