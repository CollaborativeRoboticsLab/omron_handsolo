#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start base image
#----
#---------------------------------------------------------------------------------------------------------------------------

# Use a base image suitable for Raspberry Pi with ROS 2 Humble
FROM ros:jazzy AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE_ROOT=/omron_ws

#############################################################################################################################
#####
#####   Install Dependencies
#####
#############################################################################################################################

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-dev \
    build-essential \
    curl \
    qtdeclarative5-dev \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-joy \
 && rm -rf /var/lib/apt/lists/*

# Install python packages
RUN pip3 install --no-cache-dir --break-system-packages pymodbus

#############################################################################################################################
#####
#####   Clone and build the workspace
#####
#############################################################################################################################

# Create and clone workspace
WORKDIR ${WORKSPACE_ROOT}/src

RUN git clone https://github.com/CollaborativeRoboticsLab/tmr_ros2.git 
RUN git clone --recurse-submodules https://github.com/CollaborativeRoboticsLab/omron_amr.git
RUN git clone https://github.com/CollaborativeRoboticsLab/omron_gripper.git
RUN git clone https://github.com/CollaborativeRoboticsLab/omron_moma.git
RUN git clone https://github.com/CollaborativeRoboticsLab/omron_handsolo.git

WORKDIR ${WORKSPACE_ROOT}

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

WORKDIR /

#############################################################################################################################
#####
#####   Remove workspace source and build files that are not relevant to running the system
#####
#############################################################################################################################

RUN rm -rf ${WORKSPACE_ROOT}/src
RUN rm -rf ${WORKSPACE_ROOT}/log
RUN rm -rf ${WORKSPACE_ROOT}/build

RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/*
RUN apt-get clean


#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start final release image
#----
#---------------------------------------------------------------------------------------------------------------------------

FROM ros:jazzy AS final

## Parameters
ENV WORKSPACE_ROOT=/omron_ws
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

WORKDIR /

COPY --from=base / /

# uncomment if buiding through git workflow
COPY docker/ros_entrypoint.sh ros_entrypoint.sh

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
