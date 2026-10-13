# Curt Mini Piper sensors

This package launches either physical D435i camera by serial number. Both
instances publish color, depth, and combined IMU streams. The RealSense driver
publishes each camera's internal optical transforms; this launch publishes only
the robot-to-camera mount transform.

Set measured poses in `config/camera_mounts.yaml` before hardware use. The
included values are nominal simulation starting values, not calibrations.
`base_camera` is mounted on `chassis`; `tool_camera` is mounted on
`piper_tcp_link`. Static mount transforms remain valid while the arm moves.

Examples from a sourced Jazzy workspace:

```bash
ros2 launch curtmini_piper_sensors realsense_camera.launch.py \
  camera:=base_camera serial_no:=_123456789
ros2 launch curtmini_piper_sensors realsense_camera.launch.py \
  camera:=tool_camera serial_no:=_987654321
```

Use the cameras' actual serial numbers. The launch refuses an empty serial to
prevent both instances from claiming the same USB device. Expected topics for
each camera include `/base_camera/color/image_raw`,
`/base_camera/color/camera_info`, `/base_camera/depth/image_rect_raw`, and
`/base_camera/imu`, with equivalent `/tool_camera` topics.
