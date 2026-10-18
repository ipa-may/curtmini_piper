# syntax=docker/dockerfile:1.7
FROM robot_description AS ros-cli
ARG ROS_DISTRO=jazzy
LABEL org.opencontainers.image.title="Curt Mini Piper Diagnostics"
COPY curtmini_piper_bringup/docker/cli/packages.txt /tmp/cli-packages.txt
COPY curtmini_piper_bringup/docker/cli/ros-packages.txt /tmp/cli-ros-packages.txt
RUN apt-get update \
    && xargs -r -a /tmp/cli-packages.txt apt-get install -y --no-install-recommends \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/cli-ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/cli*-packages.txt
COPY curtmini_piper_bringup/docker/cli/entrypoint.sh /opt/container/cli_entrypoint.sh
COPY curtmini_piper_bringup/docker/cli/bashrc /opt/container/cli.bashrc
RUN chmod 0755 /opt/container/cli_entrypoint.sh \
    && echo 'source /opt/container/cli.bashrc' >> /etc/bash.bashrc
ENV HOME=/diagnostics/home
WORKDIR /diagnostics
ENTRYPOINT ["/opt/container/cli_entrypoint.sh"]
CMD ["bash"]

FROM ros_base AS teleop
ARG ROS_DISTRO=jazzy
LABEL org.opencontainers.image.title="Curt Mini Piper Teleop"
COPY curtmini_piper_bringup/docker/teleop/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/ros-packages.txt
CMD ["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard", "--ros-args", "-p", "stamped:=true", "-p", "frame_id:=base_link"]

FROM teleop AS teleop-test
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && ros2 pkg prefix teleop_twist_keyboard \
    && ros2 pkg executables teleop_twist_keyboard \
      | grep -F "teleop_twist_keyboard teleop_twist_keyboard"
FROM moveit_config AS hardware-builder
ARG ROS_DISTRO=jazzy
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      python3-pip build-essential cmake git python3-colcon-common-extensions \
      python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/ws
COPY docker/dependencies/arm.repos /tmp/arm.repos
RUN mkdir -p /opt/ws/src \
    && vcs import --recursive /opt/ws/src < /tmp/arm.repos
COPY curtmini_piper_bringup /opt/ws/src/curtmini_piper/curtmini_piper_bringup
COPY curtmini_piper_bringup/docker/dependencies/${ROS_DISTRO}-hardware-python.env /tmp/python.lock.env
RUN source /tmp/python.lock.env \
    && python3 -m pip install --break-system-packages --no-deps \
      --target /opt/ws/python "git+${PYAGXARM_URL}@${PYAGXARM_REF}"
RUN source /opt/ws/install/setup.bash \
    && apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install \
      --from-paths \
        /opt/ws/src/piper_driver/agx_arm_ros/src \
        /opt/ws/src/curtmini_piper/curtmini_piper_bringup \
      --ignore-src --rosdistro "${ROS_DISTRO}" \
      --skip-keys "gz_ros2_control warehouse_ros_mongo" -r -y \
    && rm -rf /var/lib/apt/lists/*
RUN source /opt/ws/install/setup.bash \
    && colcon build --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-up-to curtmini_piper_bringup \
      --packages-skip agx_arm_urdf curt_mini_description \
        curtmini_piper_description curtmini_piper_moveit_config

FROM ros_base AS base-hardware-builder
ARG ROS_DISTRO=jazzy
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential cmake git libbluetooth-dev libusb-1.0-0-dev \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/ws
COPY docker/dependencies/base.repos /tmp/base.repos
RUN mkdir -p /opt/ws/src \
    && vcs import --recursive /opt/ws/src < /tmp/base.repos
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install \
      --from-paths \
        /opt/ws/src/candle_ros2 \
        /opt/ws/src/openzenros2 \
        /opt/ws/src/curt_mini/curt_mini_description \
        /opt/ws/src/curt_mini/ipa_ros2_control \
        /opt/ws/src/curt_mini/curt_mini_bringup \
        /opt/ws/src/curt_mini/curt_mini_teleop \
      --ignore-src --rosdistro "${ROS_DISTRO}" \
      --skip-keys "gz_ros2_control warehouse_ros_mongo" -r -y \
    && rm -rf /var/lib/apt/lists/*
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-up-to curt_mini_bringup openzen_driver

FROM hardware-builder AS hardware-test
ENV PYTHONPATH=/opt/ws/python
RUN source /opt/ws/install/setup.bash \
    && colcon test --merge-install --packages-select \
      curtmini_piper_bringup \
    && colcon test-result --verbose
FROM moveit_config AS hardware
ARG ROS_DISTRO=jazzy
LABEL org.opencontainers.image.title="Curt Mini Piper Arm Hardware"
ENV PYTHONPATH=/opt/ws/python
COPY curtmini_piper_bringup/docker/hardware/packages.txt /tmp/runtime-packages.txt
COPY curtmini_piper_bringup/docker/hardware/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && xargs -r -a /tmp/runtime-packages.txt apt-get install -y --no-install-recommends \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/*-packages.txt
COPY --from=hardware-builder /opt/ws/install/ /opt/ws/install/
COPY --from=hardware-builder /opt/ws/python/ /opt/ws/python/
CMD ["ros2", "launch", "curtmini_piper_bringup", "bringup.launch.py", "use_rviz:=false"]

FROM ros_base AS base-hardware
ARG ROS_DISTRO=jazzy
LABEL org.opencontainers.image.title="Curt Mini Base Hardware"
COPY curtmini_piper_bringup/docker/base-hardware/packages.txt /tmp/runtime-packages.txt
COPY curtmini_piper_bringup/docker/base-hardware/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && xargs -r -a /tmp/runtime-packages.txt apt-get install -y --no-install-recommends \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/*-packages.txt
COPY --from=base-hardware-builder /opt/ws/install/ /opt/ws/install/
CMD ["ros2", "launch", "curt_mini_bringup", "robot_base.launch.py"]
