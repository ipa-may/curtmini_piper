# syntax=docker/dockerfile:1.7
FROM ros_base AS navigation-base
ARG ROS_DISTRO=jazzy
COPY curtmini_piper_navigation/docker/ros-packages.txt /tmp/ros-packages.txt
RUN apt-get update \
    && sed "s|^|ros-${ROS_DISTRO}-|" /tmp/ros-packages.txt \
      | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/ros-packages.txt

FROM navigation-base AS navigation-builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends build-essential cmake python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/ws
COPY curtmini_piper_navigation /opt/ws/src/curtmini_piper_navigation
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --merge-install --packages-select curtmini_piper_navigation


FROM navigation-base AS navigation
LABEL org.opencontainers.image.title="Curt Mini Piper Navigation"
# RViz resolves mesh package URLs from the simulator's robot_description topic.
COPY --from=robot_description /opt/ws/install/ /opt/ws/install/
COPY --from=navigation-builder /opt/ws/install/ /opt/ws/install/
CMD ["ros2", "launch", "curtmini_piper_navigation", "navigation.launch.py"]
