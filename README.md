# Omron Hand Solo



This repository allows connecting the Hand Solo digital twin using packages,

- [tmr_ros2](https://github.com/CollaborativeRoboticsLab/tmr_ros2) package 
- [omron_amr](https://github.com/CollaborativeRoboticsLab/omron_amr) package
- [omron_gripper](https://github.com/CollaborativeRoboticsLab/omron_gripper.git) package
- [omron_moma](https://github.com/CollaborativeRoboticsLab/omron_moma.git) package

For supported features and limitations, see the individual repositories on the features supported by the MoMa.

## Branch Status

| Branch | ROS2 Version | Compile |
|--------|--------------|---------|
| main | Jazzy | [![main](https://github.com/CollaborativeRoboticsLab/omron_handsolo/actions/workflows/compile.yml/badge.svg?branch=main)](https://github.com/CollaborativeRoboticsLab/omron_handsolo/actions/workflows/compile.yml?query=branch%3Amain) |
| develop | Jazzy | [![develop](https://github.com/CollaborativeRoboticsLab/omron_handsolo/actions/workflows/compile.yml/badge.svg?branch=develop)](https://github.com/CollaborativeRoboticsLab/omron_handsolo/actions/workflows/compile.yml?query=branch%3Adevelop) |
| humble | Humble | [![humble](https://github.com/CollaborativeRoboticsLab/omron_handsolo/actions/workflows/compile.yml/badge.svg?branch=humble)](https://github.com/CollaborativeRoboticsLab/omron_handsolo/actions/workflows/compile.yml?query=branch%3Ahumble) |


## Setup

Create a workspace

```sh
mkdir -p omron_ws/src
cd omron_ws/src
```

Install dependencies
```sh
sudo apt install ros-jazzy-moveit ros-jazzy-controller-manager ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-broadcaster ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-vision-opencv ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```
```sh
pip install pymodbus
```

Clone the repositories into the `src` folder by

```sh
git clone https://github.com/CollaborativeRoboticsLab/tmr_ros2.git
git clone https://github.com/CollaborativeRoboticsLab/omron_amr.git
git clone https://github.com/CollaborativeRoboticsLab/omron_gripper.git
git clone https://github.com/CollaborativeRoboticsLab/omron_moma.git
git clone https://github.com/CollaborativeRoboticsLab/omron_handsolo.git
```

finally build by

```sh
cd ..
colcon build
```
**or save time and use devcontainer** 

## Launch Parameters

The top-level launch entry point is `ros2 launch handsolo_ros handsolo.launch.py`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tm_use_simulation` | `false` | Runs the TM arm bringup in simulation mode instead of connecting to the physical arm. |
| `tm_robot_ip` | `192.168.1.2` | IP address of the TM arm controller. |
| `use_arm` | `true` | Starts the TM arm hardware stack. If `false`, the arm hardware and MoveIt are not started. |
| `use_base` | `true` | Starts the AMR base hardware stack. If `false`, the base hardware, Nav2, and the HandSolo velocity filter are not started. |
| `use_moveit` | `true` | Starts MoveIt for the arm. This is only effective when `use_arm:=true`. |
| `use_nav2` | `true` | Starts Nav2 for the mobile base. This is only effective when `use_base:=true`. |
| `use_rviz` | `false` | Starts RViz. When enabled, the launch uses the MoveIt RViz layout if the arm and MoveIt are active, otherwise it uses the Nav2 RViz layout. |

## Usage

### Start the full system headless

```bash
source install/setup.bash
ros2 launch handsolo_ros handsolo.launch.py
```

### Start the full system with RVIZ

```bash
source install/setup.bash
ros2 launch handsolo_ros handsolo.launch.py use_rviz:=true
```

### Start the full system without Nav2 or MoveIt to evaluate the hardware connection

```bash
source install/setup.bash
ros2 launch handsolo_ros handsolo.launch.py use_nav2:=false use_moveit:=false
```

### Start the arm only to control the arm and gripper using RVIZ

```bash
source install/setup.bash
ros2 launch handsolo_ros handsolo.launch.py use_base:=false use_rviz:=true
```

### Start the base only to control just the base using RVIZ

```bash
source install/setup.bash
ros2 launch handsolo_ros handsolo.launch.py use_arm:=false use_rviz:=true
```

### Start the system with arm in simulation mode

```bash
source install/setup.bash
ros2 launch handsolo_ros handsolo.launch.py tm_use_simulation:=true
```

## Docker

Clone this reposiotory

```bash
git clone https://github.com/CollaborativeRoboticsLab/omron_handsolo.git 
cd omron_handsolo/docker
```

Pull the Docker image and start compose (No need to run `docker compose build`)
```bash
docker compose pull
docker compose up
```

To clean the system,
```bash
docker compose down
```