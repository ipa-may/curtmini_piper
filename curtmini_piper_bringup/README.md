# Piper arm bringup

## 1. Activate CAN interface on the host

You need sudo rights on your device.

After connecting the CAN dongle, run in a terminal:

```sh
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

## 2. Run the docker containers

For the arm bringup:
```sh
docker compose up --build piper-bringup
```

For the Rviz visualization
```sh
docker compose up --build moveit-rviz-hardware
```

## Piper H or Piper L with gripper

After building and sourcing the ROS workspace, use one of these launches:

```sh
ros2 launch curtmini_piper_bringup bringup_h.launch.py
ros2 launch curtmini_piper_bringup bringup_l.launch.py
```

Both use mock arm and gripper controllers by default. To connect the selected
arm over CAN, add `start_arm_hardware:=true can_port:=can0` after confirming the
physical arm model and a clear workspace. Live hardware operation has not yet
been validated.

In the project Docker setup, `ARM_MODEL=piper_h` or `ARM_MODEL=piper_l` selects
the matching description and MoveIt config for both `piper-bringup` and
`moveit-rviz-hardware`. The `piper-bringup` Compose service starts CAN hardware;
the plain host launches above are the safer first check.

