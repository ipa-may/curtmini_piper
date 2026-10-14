# syntax=docker/dockerfile:1.7
FROM ros_base AS sensors-deps
ARG ROS_DISTRO=jazzy
COPY curtmini_piper_sensors/docker/packages.txt /tmp/packages.txt
COPY curtmini_piper_sensors/docker/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && xargs -r -a /tmp/packages.txt apt-get install -y --no-install-recommends \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/*packages.txt

FROM sensors-deps AS sensors-builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential cmake python3-colcon-common-extensions "ros-${ROS_DISTRO}-ament-cmake" \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/ws
COPY curtmini_piper_sensors /opt/ws/src/curtmini_piper_sensors
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --merge-install --packages-select curtmini_piper_sensors

FROM sensors-deps AS sensors
LABEL org.opencontainers.image.title="Curt Mini Piper RealSense Sensors"
COPY --from=sensors-builder /opt/ws/install/ /opt/ws/install/
CMD ["ros2", "launch", "curtmini_piper_sensors", "realsense_camera.launch.py"]
