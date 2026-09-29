# syntax=docker/dockerfile:1.7

ARG ROS_DISTRO=jazzy
ARG ROS_IMAGE=ros:${ROS_DISTRO}-ros-base-noble

# Shared runtime foundation for all four images.
FROM ${ROS_IMAGE} AS ros-base
ARG ROS_DISTRO=jazzy
ARG RMW=cyclonedds
ARG OCI_SOURCE=""
ARG OCI_REVISION=unknown
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV ROS_DISTRO=${ROS_DISTRO} \
    RMW_IMPLEMENTATION=rmw_${RMW}_cpp \
    DEBIAN_FRONTEND=noninteractive
LABEL org.opencontainers.image.source="${OCI_SOURCE}" \
      org.opencontainers.image.revision="${OCI_REVISION}" \
      org.opencontainers.image.licenses="MIT"

COPY docker/base/packages.txt /tmp/base-packages.txt
RUN apt-get update \
    && xargs -r -a /tmp/base-packages.txt apt-get install -y --no-install-recommends \
    && apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-rmw-${RMW}-cpp" \
    && rm -rf /var/lib/apt/lists/* /tmp/base-packages.txt
COPY --from=container_common ros_entrypoint.sh wait_for_ros.sh /opt/container/
RUN chmod 0755 /opt/container/*.sh
ENTRYPOINT ["/opt/container/ros_entrypoint.sh"]

FROM ros-base AS teleop
LABEL org.opencontainers.image.title="Curt Mini Piper Teleop"
COPY docker/teleop/ros-packages.txt /tmp/ros-packages.txt
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

# Runtime dependencies shared by hardware, RViz and MoveItPy.
FROM ros-base AS moveit-base
COPY docker/moveit-base/packages.txt /tmp/runtime-packages.txt
COPY docker/moveit-base/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && xargs -r -a /tmp/runtime-packages.txt apt-get install -y --no-install-recommends \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/*-packages.txt

# Import upstream sources and build the shared robot packages once.
FROM moveit-base AS robot-builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-colcon-common-extensions \
      python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/ws
COPY dependencies.repos /tmp/dependencies.repos
COPY docker/dependencies/ /tmp/dependencies/
RUN manifest=/tmp/dependencies.repos \
    && if [[ -f "/tmp/dependencies/${ROS_DISTRO}.repos" ]]; then manifest="/tmp/dependencies/${ROS_DISTRO}.repos"; fi \
    && mkdir -p /opt/ws/src \
    && vcs import --recursive /opt/ws/src < "${manifest}"
COPY . /opt/ws/src/curtmini_piper
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install \
      --from-paths \
        /opt/ws/src/piper_driver/agx_arm_urdf \
        /opt/ws/src/curt_mini/curt_mini_description \
        /opt/ws/src/curtmini_piper/curtmini_piper_description \
        /opt/ws/src/curtmini_piper/curtmini_piper_moveit_config \
      --ignore-src --rosdistro "${ROS_DISTRO}" \
      --skip-keys "ament_python candle_ros2 gz_ros2_control warehouse_ros_mongo" \
      -r -y \
    && rm -rf /var/lib/apt/lists/*
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-select agx_arm_urdf curt_mini_description \
        curtmini_piper_description curtmini_piper_moveit_config

FROM robot-builder AS moveit-rviz-test
RUN source /opt/ws/install/setup.bash \
    && colcon test --merge-install --packages-select curtmini_piper_moveit_config \
    && colcon test-result --verbose \
    && ros2 launch curtmini_piper_moveit_config moveit_rviz.launch.py --show-args

FROM robot-builder AS moveitpy-builder
RUN source /opt/ws/install/setup.bash \
    && apt-get update \
    && rosdep install \
      --from-paths /opt/ws/src/curtmini_piper/curtmini_piper_motion_examples \
      --ignore-src --rosdistro "${ROS_DISTRO}" --skip-keys "ament_python" -r -y \
    && rm -rf /var/lib/apt/lists/*
RUN source /opt/ws/install/setup.bash \
    && colcon build --merge-install --packages-select curtmini_piper_motion_examples

FROM moveitpy-builder AS moveitpy-test
RUN source /opt/ws/install/setup.bash \
    && colcon test --merge-install --packages-select curtmini_piper_motion_examples \
    && colcon test-result --verbose

FROM robot-builder AS hardware-builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      libbluetooth-dev libusb-1.0-0-dev python3-pip \
    && rm -rf /var/lib/apt/lists/*
COPY docker/dependencies/${ROS_DISTRO}-hardware-python.env /tmp/python.lock.env
RUN source /tmp/python.lock.env \
    && python3 -m pip install --break-system-packages --no-deps \
      --target /opt/ws/python "git+${PYAGXARM_URL}@${PYAGXARM_REF}"
RUN source /opt/ws/install/setup.bash \
    && apt-get update \
    && rosdep install \
      --from-paths \
        /opt/ws/src/piper_driver/agx_arm_ros/src \
        /opt/ws/src/candle_ros2 \
        /opt/ws/src/openzenros2 \
        /opt/ws/src/curt_mini/ipa_ros2_control \
        /opt/ws/src/curt_mini/curt_mini \
        /opt/ws/src/curt_mini/curt_mini_teleop \
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

FROM hardware-builder AS hardware-test
ENV PYTHONPATH=/opt/ws/python
RUN source /opt/ws/install/setup.bash \
    && colcon test --merge-install --packages-select \
      curtmini_piper_description curtmini_piper_moveit_config curtmini_piper_bringup \
    && colcon test-result --verbose

# Runtime images receive only installed artifacts from their build stages.
FROM moveit-base AS moveit-rviz
LABEL org.opencontainers.image.title="Curt Mini Piper MoveIt RViz"
COPY docker/moveit-rviz/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/ros-packages.txt
COPY --from=robot-builder /opt/ws/install/ /opt/ws/install/
CMD ["ros2", "launch", "curtmini_piper_moveit_config", "moveit_rviz.launch.py"]

FROM moveit-base AS moveitpy
LABEL org.opencontainers.image.title="Curt Mini Piper MoveItPy"
COPY docker/moveitpy/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/ros-packages.txt
COPY --from=moveitpy-builder /opt/ws/install/ /opt/ws/install/
CMD ["ros2", "run", "curtmini_piper_motion_examples", "moveit_goal"]

FROM moveit-base AS hardware
LABEL org.opencontainers.image.title="Curt Mini Piper Hardware"
ENV PYTHONPATH=/opt/ws/python
COPY docker/hardware/packages.txt /tmp/runtime-packages.txt
COPY docker/hardware/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && xargs -r -a /tmp/runtime-packages.txt apt-get install -y --no-install-recommends \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/*-packages.txt
COPY --from=hardware-builder /opt/ws/install/ /opt/ws/install/
COPY --from=hardware-builder /opt/ws/python/ /opt/ws/python/
CMD ["ros2", "launch", "curtmini_piper_bringup", "bringup.launch.py", "use_rviz:=false"]
