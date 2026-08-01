# Curt Mini + Piper

This folder contains two ROS 2 packages:

- `curtmini_piper_description`: the existing Curt Mini model combined with a
  Piper arm whose links and joints use the `piper_` prefix.
- `curtmini_piper_bringup`: Curt Mini hardware bringup, AGX Piper control,
  MoveIt, and RViz.

The Piper meshes remain in `agx_arm_description`; the Curt Mini model and base
controllers remain in `curt_mini`. The only copied robot source is the Piper
kinematic description, adapted to support a prefix and a configurable mount.

## Workspace setup

The following instructions assume ROS 2 Jazzy and a workspace at
`~/piper_tests2`.

Install the workspace tools and system dependencies:

```bash
source /opt/ros/jazzy/setup.bash

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep \
  python3-can \
  python3-scipy \
  can-utils \
  ethtool
```

The Python dependency installation below requires
[`uv`](https://docs.astral.sh/uv/) to be available on `PATH`.

Initialize rosdep once per machine, if it has not already been initialized:

```bash
sudo rosdep init
```

An `already initialized` message can be ignored. Update the rosdep database:

```bash
rosdep update
```

Import the source dependencies pinned by the Curt Mini packages. The
`--skip-existing` option makes these commands safe to repeat without replacing
existing checkouts:

```bash
cd ~/piper_tests2

vcs import --recursive --skip-existing src \
  < src/curt_mini/ipa_ros2_control/ipa_ros2_control.repos
vcs import --recursive --skip-existing src \
  < src/curt_mini/curt_mini/curt_mini.repos
```

These manifests provide:

- `candle_ros2` v2.1.2, required by the Curt Mini hardware interface.
- `openzen_driver`, required by the Curt Mini IMU launch.

Install the Python SDK used by the Piper hardware node:

Create a virtual environment
```sh
uv venv
```

Install pyAxArm python requirements
```bash
uv pip install --user --break-system-packages \
  "git+https://github.com/agilexrobotics/pyAgxArm.git"
```

Install the remaining declared ROS and system dependencies:

```bash
cd ~/piper_tests2
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
  --skip-keys "warehouse_ros_mongo"
```

`warehouse_ros_mongo` is declared by the upstream `agx_arm_moveit` package but
has no rosdep definition for Ubuntu Noble. This bringup does not start the
optional MongoDB warehouse backend.

## Build

From the workspace root:

```bash
cd ~/piper_tests2
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to \
  curtmini_piper_description curtmini_piper_bringup
source install/setup.bash
```

## Real robot

```bash
ros2 launch curtmini_piper_bringup bringup.launch.py
```

The defaults start the Curt Mini hardware, joystick, IMU, Piper on `can0`,
MoveIt, and RViz. MoveIt uses prefixed names such as `piper_joint1`; a bridge
translates these to the unprefixed names expected by `agx_arm_ctrl`.

The AGX command interface starts gated off and is opened only while the
trajectory action is active.

MoveIt uses the original Curt Mini visual mesh and a simplified box collision
for the chassis. The upstream 30 MB chassis STL remains in the full
robot-state-publisher description, but is not used as an FCL collision mesh.

## Hardware-free MoveIt/RViz

```bash
ros2 launch curtmini_piper_bringup bringup.launch.py \
  start_base:=false start_arm_hardware:=false
```

## Mount calibration

The default arm mount is `0 0 0.18` relative to `chassis`. Adjust it without
editing the xacro:

```bash
ros2 launch curtmini_piper_bringup bringup.launch.py \
  arm_mount_xyz:="0 0 0.20" arm_mount_rpy:="0 0 0"
```

## Dependency troubleshooting

The setuptools deprecation messages printed before the build starts are
warnings. The fatal error in the example above is CMake being unable to find
`candle_ros2`.

If CMake cannot find `candle_ros2`, confirm that the source import succeeded:

```bash
cd ~/piper_tests2
colcon list | grep -E '^(candle_ros2|openzen_driver)[[:space:]]'
```

Both packages should be listed. If either is missing, repeat the corresponding
`vcs import` command from the workspace setup section.

Before starting real Piper hardware, verify that its Python SDK is importable:

```bash
python3 -c "import pyAgxArm; print(pyAgxArm.__file__)"
```

The current MoveIt installation also reports a missing
`libgeometric_shapes.so.2.3.4` when loading its optional point-cloud octomap
updater. Arm planning and trajectory control still start successfully, but
depth-camera octomap updates require that system library mismatch to be fixed.
