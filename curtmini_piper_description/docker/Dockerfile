# syntax=docker/dockerfile:1.7
FROM ros_base AS dependencies
ARG ROS_DISTRO=jazzy
COPY curtmini_piper_description/docker/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/ros-packages.txt

FROM dependencies AS builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-colcon-common-extensions \
      python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/ws
COPY docker/dependencies/arm.repos /tmp/arm.repos
RUN mkdir -p /opt/ws/src \
    && vcs import --recursive /opt/ws/src < /tmp/arm.repos
COPY curtmini_piper_description /opt/ws/src/curtmini_piper/curtmini_piper_description
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install \
      --from-paths /opt/ws/src/piper_driver/agx_arm_urdf \
        /opt/ws/src/curt_mini/curt_mini_description \
        /opt/ws/src/curtmini_piper/curtmini_piper_description \
      --ignore-src --rosdistro "${ROS_DISTRO}" \
      --skip-keys "ament_python candle_ros2 gz_ros2_control warehouse_ros_mongo" -r -y \
    && rm -rf /var/lib/apt/lists/*
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-select agx_arm_urdf curt_mini_description curtmini_piper_description

FROM builder AS test
RUN source /opt/ws/install/setup.bash \
    && colcon test --merge-install --packages-select curtmini_piper_description \
    && colcon test-result --verbose \
    && xacro /opt/ws/install/share/curtmini_piper_description/urdf/curtmini_piper.urdf.xacro -o /tmp/robot.urdf

FROM dependencies AS runtime
LABEL org.opencontainers.image.title="Curt Mini Piper Description"
COPY --from=builder /opt/ws/install/ /opt/ws/install/
CMD ["true"]
