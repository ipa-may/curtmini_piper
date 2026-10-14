# syntax=docker/dockerfile:1.7

ARG ROS_DISTRO=jazzy
ARG ROS_IMAGE=ros:${ROS_DISTRO}-ros-base-noble

# Shared runtime foundation for the robot images.
FROM ${ROS_IMAGE} AS runtime
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

COPY docker/common/packages.txt /tmp/base-packages.txt
RUN apt-get update \
    && xargs -r -a /tmp/base-packages.txt apt-get install -y --no-install-recommends \
    && apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-rmw-${RMW}-cpp" \
    && rm -rf /var/lib/apt/lists/* /tmp/base-packages.txt
COPY --from=container_common ros_entrypoint.sh wait_for_ros.sh /opt/container/
RUN chmod 0755 /opt/container/*.sh
ENTRYPOINT ["/opt/container/ros_entrypoint.sh"]
