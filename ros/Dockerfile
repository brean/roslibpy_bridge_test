ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

RUN apt-get update && apt-get install -y \
  python-catkin-tools \
  ros-${ROS_DISTRO}-rosbridge-suite \
  ros-${ROS_DISTRO}-rosbridge-server \
  && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /opt/workspace/src/

COPY ./catkin_ws/src/my_robot /opt/workspace/src/my_robot
COPY ./catkin_ws/src/custom_service /opt/workspace/src/custom_service

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && cd /opt/workspace/ \
  && catkin build

COPY ./entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash