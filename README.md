# omron_handsolo

This repository allows connecting a Digital twin using packages,

- [omron_arm](https://github.com/CollaborativeRoboticsLab/omron_arm) package 
- [omron_base](https://github.com/CollaborativeRoboticsLab/omron_base) package
- [omron_gripper](https://github.com/CollaborativeRoboticsLab/omron_gripper.git) package
- [omron_moma](https://github.com/CollaborativeRoboticsLab/omron_moma.git) package

For supported features and limitations, see the individual repositories on the features supported by the MoMa.

## Setup

Create a workspace

```sh
mkdir -p omron_ws/src
cd omron_ws/src
```

Install dependencies
```sh
sudo apt install ros-humble-moveit ros-humble-controller-manager ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster ros-humble-rmw-cyclonedds-cpp ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-vision-opencv ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
```
```sh
pip install pymodbus
```

Clone the repositories into the `src` folder by

```sh
git clone https://github.com/CollaborativeRoboticsLab/omron_arm.git
git clone https://github.com/CollaborativeRoboticsLab/omron_base.git
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

## Usage

### Start the system headless

```bash
source install/setup.bash
ros2 launch handsolo_ros handsolo.launch.py
```

### Start the system with RVIZ

```bash
source install/setup.bash
ros2 launch handsolo_ros handsolo.launch.py use_rviz:=true
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