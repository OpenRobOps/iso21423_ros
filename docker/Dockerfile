# Build: docker build --build-context iso21423=../iso21423 --target dev -f docker/Dockerfile .
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-nav2-msgs ros-${ROS_DISTRO}-tf2-ros \
      python3-paho-mqtt python3-jsonschema python3-pip \
    && rm -rf /var/lib/apt/lists/*
COPY --from=iso21423 py-sdk /opt/iso21423/py-sdk
# Jazzy/noble pip supports --break-system-packages; Humble/jammy pip does not.
RUN if pip install --help | grep -q break-system-packages; then FLAG=--break-system-packages; fi; \
    pip install $FLAG /opt/iso21423/py-sdk

FROM base AS dev
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pytest ros-${ROS_DISTRO}-launch-testing ros-${ROS_DISTRO}-launch-testing-ament-cmake \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws

FROM base AS runtime
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
RUN apt-get update && apt-get install -y --no-install-recommends python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
COPY . /ws/src/iso21423_ros
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && cd /ws && colcon build
ENTRYPOINT ["bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2","launch","iso21423_ros","connector.launch.py"]
