# Curt Mini Piper MoveIt configuration

This package owns the robot's MoveIt model, planning settings, and the
`move_group` launch. It has three launch files:

- `move_group.launch.py` starts the planning server. It expects robot state
  publishing and an arm trajectory controller from the caller. Use
  `controller_mode:=sim use_sim_time:=true use_sensors_3d:=false` for Gazebo;
  the defaults select the real robot's controller and depth sensor.
- `moveit_rviz.launch.py` starts only the MoveIt RViz client.
- `demo.launch.py` starts a mock arm controller, robot state publisher,
  `move_group`, and RViz without the physical robot or Gazebo.

Run the standalone demo with:

```bash
ros2 launch curtmini_piper_moveit_config demo.launch.py
```

The `moveit_controllers_real.yaml` and `moveit_controllers_sim.yaml` files
select the arm controller that MoveIt sends trajectories to. The demo uses
the real controller name with a mock `ros2_control` system. Do not start
`move_group.launch.py` separately while simulation, bringup, or the demo is
already running; each of those starts it once.

`config/ompl_planning.yaml` selects `RRTConnect` by default for the `arm`
planning group. Set the OMPL planner ID to `RRTstar` or `PRMstar` to try the
other configured planners. MoveIt's Jazzy defaults supply the definitions for
these IDs; this file selects which ones the arm offers. Other planning
pipelines retain their MoveIt defaults.
