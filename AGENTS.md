# Repository Guidelines

## Project Structure & Module Organization

This repository contains one ROS 2 package, `armor_detector`, for RoboMaster armor plate detection.

- `src/armor_detector/include/armor_detector/`: public headers, organized by component (`detector/`, `debug/`, `types/`, `tools/`, `yaw/`).
- `src/armor_detector/src/`: C++ implementations for the node, camera provider, detector pipeline, pose solver, yaw search, and debug UI.
- `src/armor_detector/config/`: runtime YAML configuration, including camera calibration and playback settings.
- `src/armor_detector/launch/`: ROS 2 launch files — `video1.launch.py` (interactive) and `auto_test.launch.py` (headless).
- `src/armor_detector/Test/video/`: rosbag-based test data (`video1` through `video5`).
- `docs/`: design notes and data conventions. Follow `docs/Conventions.md` for units, coordinate frames, corner ordering, and `cv::Mat` lifetime rules.

Build output directories (`build/`, `install/`, `log/`) are generated and should not be edited manually.

## Build, Test, and Development Commands

Run commands from the repository root:

```bash
colcon build --packages-select armor_detector
source install/setup.bash

# 交互模式
ros2 launch armor_detector video1.launch.py

# 自动测试
ros2 launch armor_detector auto_test.launch.py frame_count:=150

# 格式检查
clang-format --dry-run --Werror src/armor_detector/src/*.cpp src/armor_detector/include/armor_detector/*.hpp

colcon test --packages-select armor_detector
```

## Coding Style & Naming Conventions

Use C++ with the existing `armor_detector` namespace. Formatting is defined by `.clang-format`: LLVM style, 4-space indentation, 120-column limit, and indented namespaces. Static analysis is configured in `.clang-tidy` with bugprone, modernize, performance, and readability checks.

Prefer descriptive names that include units where relevant, for example `angle_deg`, `duration_ms`, or `xyz_camera`. Internal 3D angles use radians; image geometry thresholds use degrees.

## Testing Guidelines

Current validation is primarily rosbag playback plus ament lint. Use `auto_test.launch.py` for headless fixed-frame testing. When adding tests, keep them inside the package, name them after the behavior under test, and make them runnable through `colcon test --packages-select armor_detector`. Use the existing `Test/video` datasets for integration checks and document any new dataset assumptions.

## Commit & Pull Request Guidelines

Recent commits use short Chinese subject lines such as `文档约定` and `框架搭建1`, without prefixes. Keep commit subjects concise, imperative or noun-phrase based, and focused on one change.

Pull requests should include a brief description, affected modules, build/test results, and screenshots or terminal notes for GUI/debug behavior changes. Link related issues when available and update `docs/` when data conventions or pipeline behavior change.
