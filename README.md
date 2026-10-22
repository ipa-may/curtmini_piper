# Curt Mini with Piper Arm

<p align="center">
  <img src="doc/curtmini_piper_picture.png" alt="Curt Mini with Piper arm" width="300">
</p>

This repository contains five ROS 2 packages:

- `curtmini_piper_description`: the existing Curt Mini model combined with a
  Piper arm whose links and joints use the `piper_` prefix.
- `curtmini_piper_moveit_config`: shared SRDF, kinematics, joint limits,
  planning, perception, and RViz configuration.
- `curtmini_piper_bringup`: Piper arm control, the combined robot model,
  MoveIt runtime, and RViz. Base hardware remains in `curt_mini_bringup`.
- `curtmini_piper_navigation`: simulation mapping, localization and goal navigation. See the [navigation guide](curtmini_piper_navigation/README.md).
- `curtmini_piper_sensors`: physical RealSense camera launch and mount transforms. See the [sensor guide](curtmini_piper_sensors/README.md).

Student-facing MoveItPy, AprilTag, and Nav2 clients live in the separate
[`curtmini_piper_tools` repository](../curtmini_piper_tools/README.md).

Each package owns its Dockerfile and Compose services under `docker/`.
The root `compose.yaml` includes them; the project infrastructure remains
responsible for middleware and workspace configuration. See the
[container guide](doc/containers.md) for image dependencies, local builds,
and tests.

The prefix-aware Piper model and meshes come from `agx_arm_urdf`; the Curt Mini
model lives in `curt_mini_description`, and base controllers remain in
`curt_mini_bringup`. This package owns only the mount and TCP joints that
connect the two models.

### 0. Dependencies

- ROS 2 Jazzy 

## 1. Workspace setup

### 1.1 Create a ROS workspace
```sh
mkdir -p ros2_ws/src
cd ros2_ws/src
```

### 1.2. Clone the repo in the source folder of the ROS workspace
```sh
git clone https://github.com/ipa-may/curtmini_piper.git
```

### 1.3 Import the dependencies

Import the dependencies repos using vcs
```sh
cd ~/ros2_ws
vcs import src/ < src/curtmini_piper/dependencies_without_hardware.repos
```

Your workspace should be:
```text
src/
├── curt_mini/
├── curtmini_piper/
└── piper_driver/
    ├── agx_arm_ros/
    ├── agx_arm_urdf/
```

From the ROS workspace:

```sh
rosdep init
rosdep install --from-paths src -y --ignore-src
```


## 2. Building the packages

From the ROS workspace 


```sh
colcon build --packages-up-to curtmini_piper_bringup
```
<!-- moveit_config packages:
- curtmini_piper_l_gripper_moveit_config
- curtmini_piper_h_gripper_moveit_config
- curtmini_piper_x_gripper_moveit_config -->


Source the ROS workspace:
```sh
source install/setup.bash
```

## 3. Launch scripts

The launch scripts take the `arm_model` as argument.


### 3.1 Visualizing the robot:
```sh
ros2 launch curtmini_piper_description view_robot.launch.py arm_model:=piper_l
```
arm_model:
- piper_l
- piper_h
- piper_x
- piper (without gripper)


**About the robot structure:**

The arm mount, TCP offset, and lidar mount are defined in
`curtmini_piper_description/config/geometry.yaml`. 

The arm and lidar mounts are relative to `chassis`; the TCP offset is relative to `piper_link6`.

Update the YAML and restart bringup and planning clients together so their
robot descriptions use the same geometry.


### 3.2 Hardware-free Moveit

```sh
ros2 launch curtmini_piper_bringup bringup_l.launch.py
```
For each arm type, use a different bringup launch:
- for piper_l : bringup_l.launch.py
- for piper_h : bringup_h.launch.py
- for piper_x : bringup_x.launch.py

Those bringups defaults to start_arm_hardware:=false and starts:
- mock ros2_control
- arm and gripper controllers
- joint-state and robot-state publishers
- move_group
- RViz


## 4. Using the real robot

Instruction TBD

## 5. Simulation

Simulation lives in the separate [curtmini_piper_gz_sim repository](https://github.com/ipa-may/curtmini_piper_gz_sim).
Clone that repository and follow its setup instructions to build and run simulation.

