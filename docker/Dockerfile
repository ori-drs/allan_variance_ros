FROM ros:noetic-robot

ARG CATKIN_WORKSPACE_DIR="/catkin_ws"

LABEL org.opencontainers.image.authors="tobit@robots.ox.ac.uk"
LABEL description="Allan Variance ROS"
LABEL version="1.0"

WORKDIR ${CATKIN_WORKSPACE_DIR}
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get -y install \
    git \
    python3-catkin-tools \
    python3-osrf-pycommon \
 && rm -rf /var/lib/apt/lists/*

 RUN apt-get update \
 && apt-get -y install \
    libomp-dev \
    libyaml-cpp-dev \
    python3-matplotlib \
    python3-numpy \
    python3-scipy \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-rosbag \
    ros-${ROS_DISTRO}-rospy \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
 && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

RUN echo "source ${CATKIN_WORKSPACE_DIR}/devel/setup.bash || source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

