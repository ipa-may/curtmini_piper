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

