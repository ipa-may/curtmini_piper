# Curt Mini Piper navigation

This package configures **Nav2 for Gazebo simulation**. Nav2 plans a path and
drives the mobile base to a goal position and orientation. SLAM Toolbox builds
the map; AMCL estimates the robot's position on a saved map.

The package belongs in `curtmini_piper` because its frames, controller topics,
lidar and footprint are specific to this robot. Nav2 and SLAM Toolbox are
installed from ROS packages. They do not need separate source repositories.
Future docking, task sequencing or custom planner packages can be added when
needed; the initial setup does not need them.

## Files

| File | Purpose |
| --- | --- |
| `config/slam.yaml` | Lidar, frames and map-building settings |
| `config/nav2.yaml` | Localization, path planning, obstacle avoidance and speed limits |
| `launch/mapping.launch.py` | Starts SLAM Toolbox |
| `launch/navigation.launch.py` | Starts the map server, AMCL and Nav2 servers |
| `launch/rviz.launch.py` | Opens the navigation view |
| `rviz/navigation.rviz` | Map, scan, footprint, path and goal tools |
| `maps/` | Maps saved on the host; generated files are ignored by Git |

## 1. Build

Run these commands from `~/ROB4_Fraunhofer/curtmini_piper_containers`.
Use this same environment in each terminal:

```bash
export RMW=cyclonedds
export ROS_DOMAIN_ID=42
export SIM_WORLD=curtmini_piper_map
export SIM_USE_HOKUYO=true
export COMPOSE_FILE=compose.yaml:compose.cyclonedds.yaml:compose.workspace.yaml:compose.gui.yaml
```

These are the existing Compose files. There is no navigation overlay.
The workspace builder includes navigation, simulation and MoveIt; it does not
build hardware bringup.

```bash
docker compose build workspace-builder
docker compose run --rm workspace-builder
xhost +si:localuser:root
```

After changing navigation launch files or configuration, rerun the workspace
builder and restart the affected services. Rebuild the image only when image
dependencies change. Build and install folders remain in Docker volumes.

## 2. Make a map

Start Gazebo, SLAM Toolbox and the navigation view:

```bash
docker compose up -d gz-sim mapping-sim nav2-rviz-sim
docker compose logs -f mapping-sim
```

Stop following the logs with Ctrl+C. This leaves the services running.
In RViz, the laser points should follow walls and furniture. If most points
land on the robot itself, correct the lidar mounting before mapping. Its
position is in `curtmini_piper_description/config/geometry.yaml`.

Drive slowly around the area with keyboard teleoperation:

```bash
docker compose run --rm --build keyboard-teleop-sim
```

Keep the arm in the simulator's initial pose. Visit the space where you want
to navigate, then stop teleoperation with Ctrl+C. Save the map from another
terminal:

```bash
docker compose exec mapping-sim /opt/container/ros_entrypoint.sh \
  ros2 run nav2_map_server map_saver_cli -f /maps/map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

This creates `map.yaml` and `map.pgm` in this package's `maps/` directory on
your host. `/maps` inside the container is a bind mount of that directory.
The files may be owned by root because the containers run as root.
The YAML records the image filename, map resolution and map origin. Keep both
files together.

## 3. Navigate on the saved map

Leave Gazebo running. Stop mapping before starting AMCL:

```bash
docker compose stop mapping-sim
docker compose up -d nav2-sim nav2-rviz-sim
docker compose logs -f nav2-sim
```

In RViz:

1. Select **2D Pose Estimate**. Click where the robot currently is on the map
   and drag the arrow to its heading. This initializes AMCL.
2. Check that the scan lines up with the map. Allow localization and the Nav2
   servers to become active.
3. Select **Nav2 Goal**. Click a reachable free location and drag to choose the
   final heading. The robot should plan a path and follow it.
4. Use **Cancel** in the Navigation 2 panel to cancel a goal.

Do not run keyboard teleoperation while Nav2 is driving: both publish to the
same controller. Mapping and AMCL also must not run together: each publishes
`map → odom`.

You can also send a goal through the ROS action interface:

```bash
docker compose exec nav2-sim /opt/container/ros_entrypoint.sh \
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

Those coordinates are an example, not a known free point in your map. Choose
coordinates from RViz. The orientation is a quaternion; `w: 1.0` means yaw zero.
Closing the CLI is not a reliable way to cancel an accepted goal. RViz's
Cancel button is for goals sent from RViz. To cancel the active navigation
goal from a terminal:

```bash
docker compose exec nav2-sim /opt/container/ros_entrypoint.sh \
  ros2 service call /navigate_to_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal '{}'
```

The empty request asks Nav2 to cancel all current NavigateToPose goals.

To load another map saved in `maps/`:

```bash
NAV2_MAP=office.yaml docker compose up -d nav2-sim
```

To finish:

```bash
docker compose stop nav2-sim mapping-sim nav2-rviz-sim gz-sim
```

## How the pieces connect

| Interface | Used for |
| --- | --- |
| `/scan` (`sensor_msgs/msg/LaserScan`) | Mapping, localization and obstacle detection |
| `/base_controller/odom` | Wheel odometry for navigation |
| `/base_controller/cmd_vel` (`geometry_msgs/msg/TwistStamped`) | Base velocity commands |
| `/clock` | Simulation time for every navigation node |
| `/navigate_to_pose` | Goal action, including feedback and cancellation |
| `map → odom` | Published by SLAM while mapping, or AMCL while navigating |
| `odom → base_link` | Published by the simulated base controller |
| `base_link → … → hokuyo_link` | Published from the robot description |

NavFn plans the global path. DWB chooses local velocities while checking the
robot footprint against obstacles. The controller and recovery behaviors send
stamped commands directly to the base controller. The initial limits are
0.25 m/s forward and 0.5 rad/s turning.

The footprint is a fixed 1.0 × 0.7 m rectangle, plus 3 cm padding. It assumes
the stationary initial arm pose. Moving the arm does not automatically update
the footprint; an extended arm or changed attachments require a new footprint.
This is a 2D lidar setup and does not provide full 3D arm clearance checking.

## Troubleshooting

- **Map does not exist:** save a map first and check `maps/map.yaml`, or set
  `NAV2_MAP` to the saved YAML filename.
- **RViz has no `map` frame during localization:** provide a 2D Pose Estimate.
- **Scan does not line up:** check the initial pose, lidar geometry and TF.
- **Robot does not move:** inspect `docker compose logs nav2-sim`, check Nav2
  is active, and select a goal in mapped free space with room for the footprint.
- **Transform or time errors:** keep the same ROS domain and middleware in
  all terminals; all navigation nodes must use the simulator clock.

For regular images without the workspace overlay, build `mapping-sim` once:
the three navigation services share the same `navigation` image. The workspace
workflow above uses your local robot and simulation sources together.

References: [Nav2 setup guide](https://docs.nav2.org/setup_guides/index.html)
and [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox).
