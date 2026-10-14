# syntax=docker/dockerfile:1.7
FROM ros_base AS dependencies
ARG ROS_DISTRO=jazzy
COPY curtmini_piper_moveit_config/docker/packages.txt /tmp/runtime-packages.txt
COPY curtmini_piper_moveit_config/docker/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && xargs -r -a /tmp/runtime-packages.txt apt-get install -y --no-install-recommends \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/*-packages.txt

FROM dependencies AS builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential cmake python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/ws
COPY --from=robot_description /opt/ws/install/ /opt/ws/install/
COPY curtmini_piper_moveit_config /opt/ws/src/curtmini_piper_moveit_config
RUN source /opt/ws/install/setup.bash \
    && apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /opt/ws/src/curtmini_piper_moveit_config \
      --ignore-src --rosdistro "${ROS_DISTRO}" \
      --skip-keys "gz_ros2_control warehouse_ros_mongo" -r -y \
    && rm -rf /var/lib/apt/lists/*
RUN source /opt/ws/install/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-select curtmini_piper_moveit_config

FROM builder AS moveit-rviz-test
RUN source /opt/ws/install/setup.bash \
    && colcon test --merge-install --packages-select curtmini_piper_moveit_config \
    && colcon test-result --verbose \
    && ros2 launch curtmini_piper_moveit_config moveit_rviz.launch.py --show-args

FROM dependencies AS config
LABEL org.opencontainers.image.title="Curt Mini Piper MoveIt Configuration"
COPY --from=builder /opt/ws/install/ /opt/ws/install/
CMD ["true"]

FROM config AS moveit-rviz
LABEL org.opencontainers.image.title="Curt Mini Piper MoveIt RViz"
COPY curtmini_piper_moveit_config/docker/rviz-ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/ros-packages.txt
CMD ["ros2", "launch", "curtmini_piper_moveit_config", "moveit_rviz.launch.py"]
