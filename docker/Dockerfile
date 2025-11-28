#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start base image
#----
#---------------------------------------------------------------------------------------------------------------------------

# Use a base image suitable for Raspberry Pi with ROS 2 Humble
FROM ros:humble-ros-base-jammy AS base

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
    libx11-dev \
    libxext-dev \
    libxtst-dev \
    libxrender-dev \
    libxrandr-dev \
    libgl1-mesa-glx \
    libglu1-mesa \
    libxi-dev \
    libsm6 \
    libxkbcommon-x11-0 \
    x11-xserver-utils \
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
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-joy 

# Install python packages
RUN pip3 install --no-cache-dir pymodbus

#############################################################################################################################
#####
#####   Clone and build the workspace
#####
#############################################################################################################################

# Create and clone workspace
WORKDIR ${WORKSPACE_ROOT}/src

RUN git clone https://github.com/CollaborativeRoboticsLab/omron_moma.git
RUN git clone https://github.com/CollaborativeRoboticsLab/omron_arm.git 
RUN git clone https://github.com/CollaborativeRoboticsLab/omron_base.git
RUN git clone https://github.com/CollaborativeRoboticsLab/omron_gripper.git
RUN git clone https://github.com/CollaborativeRoboticsLab/omron_handsolo.git

WORKDIR ${WORKSPACE_ROOT}

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

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

FROM ros:humble-ros-base-jammy AS final

## Parameters
ENV WORKSPACE_ROOT=/omron_ws
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

ENV USE_RVIZ=true
ENV USE_MOVEIT=true
ENV USE_NAV2=true
ENV ARM_USE_SIMULATION=false

WORKDIR /

COPY --from=base / /

# uncomment if buiding through git workflow
COPY docker/ros_entrypoint.sh ros_entrypoint.sh

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
