# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

**SVO Pro** is a ROS-based C++14 visual odometry / SLAM system developed at RPG (Robotics and Perception Group, UZH). It is a multi-package catkin workspace supporting:
- Visual odometry (monocular, stereo, fisheye/catadioptric)
- Visual-inertial odometry (VIO) with a Ceres/OKVIS-style sliding window backend
- Full VI-SLAM with an iSAM2-based global map and loop closure via DBoW2

## Build System

This is a **catkin workspace** project. The repo lives in `src/rpg_svo_pro_open/` inside a catkin workspace (typically `~/svo_ws/`).

```sh
# Standard build (no global map)
catkin build

# Force re-run of CMake (required after changing .cmake files)
catkin build --force-cmake

# Clean rebuild
catkin clean --all && catkin build

# Build a single package
catkin build svo_ros
```

The build is configured in `svo_cmake/cmake/Modules/SvoSetup.cmake`:
- Toggle global map support: `SET(USE_GLOBAL_MAP TRUE/FALSE)`
- Toggle loop closing: `SET(USE_LOOP_CLOSING TRUE/FALSE)`
- After changing these flags, always do `catkin build --force-cmake` or a full clean rebuild

To enable the global map (`svo_global_map` package is CATKIN_IGNOREd by default):
```sh
rm svo_global_map/CATKIN_IGNORE
# set USE_GLOBAL_MAP TRUE in svo_cmake/cmake/Modules/SvoSetup.cmake
catkin build
```

## Running

Source the workspace first:
```sh
source ~/svo_ws/devel/setup.bash
```

Key launch files (in `svo_ros/launch/`):

| Use case | Command |
|---|---|
| Pinhole camera from bag | `roslaunch svo_ros run_from_bag.launch cam_name:=svo_test_pinhole` |
| Fisheye camera from bag | `roslaunch svo_ros run_from_bag.launch cam_name:=bluefox_25000826_fisheye` |
| Monocular frontend + IMU (EuRoC) | `roslaunch svo_ros euroc_mono_frontend_imu.launch` |
| Stereo frontend + IMU (EuRoC) | `roslaunch svo_ros euroc_stereo_frontend_imu.launch` |
| Monocular VIO (EuRoC) | `roslaunch svo_ros euroc_vio_mono.launch` |
| Stereo VIO (EuRoC) | `roslaunch svo_ros euroc_vio_stereo.launch` |
| VI-SLAM with global map | `roslaunch svo_ros euroc_global_map_mono.launch` |

Vocabulary files for loop closure must be downloaded before use:
```sh
cd svo_online_loopclosing/vocabularies && ./download_voc.sh
```

## Package Architecture

The codebase is split into focused catkin packages:

| Package | Role |
|---|---|
| `svo` | Core frame handlers (`frame_handler_mono/stereo/array`), map, reprojector, pose optimizer, initialization |
| `svo_common` | Shared data structures: `Frame`, `Point`, `Feature`, camera models |
| `svo_direct` | Direct image alignment and depth filters |
| `svo_img_align` | Image-to-image alignment (sparse/dense) |
| `svo_tracker` | Feature tracking (KLT, etc.) |
| `svo_vio_common` | Shared VIO utilities (IMU pre-integration, etc.) |
| `svo_ceres_backend` | Sliding window optimization backend (OKVIS-based, Ceres solver) |
| `svo_pgo` | Pose graph optimization (lightweight loop closure alternative) |
| `svo_online_loopclosing` | DBoW2 loop closure detection + ORB feature extraction |
| `svo_global_map` | iSAM2-based globally consistent bundle-adjusted map (optional, requires GTSAM 4.0.3) |
| `svo_ros` | ROS node/nodelet wrapper, launch files, parameter files, calibration files |
| `svo_msgs` | Custom ROS message definitions |
| `svo_cmake` | Shared CMake configuration (`SvoSetup.cmake`) — build flags, feature toggles |
| `svo_benchmarking` | Python tools for dataset evaluation |
| `rpg_common` | General-purpose C++ utilities shared across RPG projects |
| `vikit` | Vision utility kit: camera models, math helpers, image utilities |
| `rqt_svo` | rqt plugin for SVO visualization |
| `svo_test_utils` | Test helpers |

**Data flow**: Images (+ optional IMU) → `svo_ros` node → `frame_handler_*` (in `svo`) → direct tracking (`svo_direct`, `svo_img_align`, `svo_tracker`) → reprojector + map → optional backend (`svo_ceres_backend`) → optional global map (`svo_global_map`) + loop closure (`svo_online_loopclosing`).

## Configuration

- Parameter files: `svo_ros/param/` — `pinhole.yaml`, `fisheye.yaml`, `vio_mono.yaml`, `vio_stereo.yaml`, `global_map.yaml`
- Camera calibration files: `svo_ros/param/calib/` (YAML format, see `doc/calibration.md`)
- The `pipeline_is_stereo` flag in param files switches mono/stereo mode

## OpenCV Version Pitfall

If you see OpenCV linking errors, explicitly pin the version in `CMakeLists.txt` for each affected package (`rpg_common`, `svo_ros`, `svo_direct`, `vikit/vikit_common`, `svo_online_loopclosing`):
```cmake
find_package(OpenCV 3 REQUIRED)  # Ubuntu 18.04 / Melodic
find_package(OpenCV 4 REQUIRED)  # Ubuntu 20.04 / Noetic
```

## Eigen Consistency

All packages must use the same Eigen (system Eigen at `/usr/include/eigen3`). The workspace is configured with:
```sh
catkin config --cmake-args -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
```
If GTSAM is used, ensure `GTSAM_USE_SYSTEM_EIGEN=ON` and `-mno-avx` is added to its compile flags.
