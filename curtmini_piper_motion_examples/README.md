# Curt Mini Piper Motion Examples

MoveItPy examples for the combined Curt Mini and prefixed Piper model.

Start either the Gazebo simulation or real robot bringup first. The command
plans only by default:

```bash
ros2 run curtmini_piper_motion_examples moveit_goal
```

Plan and execute in simulation:

```bash
ros2 run curtmini_piper_motion_examples moveit_goal --ros-args \
  -p controller_mode:=simulation \
  -p plan_only:=false
```

Plan and execute against the physical robot:

```bash
ros2 run curtmini_piper_motion_examples moveit_goal --ros-args \
  -p controller_mode:=hardware \
  -p plan_only:=false
```

Set a pose goal for `piper_tcp_link` in `piper_base_link`:

```bash
ros2 run curtmini_piper_motion_examples moveit_goal --ros-args \
  -p goal_type:=pose \
  -p pose_position:="[0.25, 0.0, 0.30]" \
  -p pose_orientation:="[0.0, 0.7071068, 0.0, 0.7071068]"
```

The available `controller_mode` values are `simulation` and `hardware`.
Mount and TCP offsets can be supplied through `arm_mount_xyz`,
`arm_mount_rpy`, `tcp_offset_xyz`, and `tcp_offset_rpy`.
