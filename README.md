# Multi-UAV Swarm Simulation - Version 1

## Description
This tutorial guides you through setting up a swarm simulation environment for 4 quadcopters using Ardupilot + SITL on Gazebo. Each quadcopter has a downward-facing camera to detect colored cubes. The goal is to simulate a swarm scenario where drones autonomously identify colored cubes (markers) in a 20x20 meter grid, confined to divided sections via geofencing to prevent collisions.

## Requirements
- Ubuntu 20.04.6 LTS
- Gazebo 11
- ROS Noetic
- Ardupilot
- Ardupilot Gazebo plugin
- MAVROS
- MAVproxy

## Basic Installations

### Ubuntu
Set up Ubuntu 20.04.6 LTS (Focal Fossa) on a Virtual Machine:
- Memory: 16GB
- Processors: 16 (4 cores per processor)
- Hard Disk (SCSI): 60GB

Download: [Ubuntu 20.04.6 Desktop](https://releases.ubuntu.com/20.04/ubuntu-20.04.6-desktop-amd64.iso)

### ROS Noetic
Install ROS Noetic:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### Gazebo 11
Install Gazebo 11 (optional, included with ROS Noetic):
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo11 libgazebo11-dev
```

### Ardupilot
Install Ardupilot (v4.3.6):
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
alias waf="$PWD/modules/waf/waf-light"
waf configure --board=sitl
waf all
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Fix potential errors:
- Python error:
  ```bash
  sudo apt update
  sudo apt install python-is-python3
  ```
- Module error:
  ```bash
  sudo apt install python3-pip
  pip3 install future
  ```

### Ardupilot Gazebo Plugin
Install in home directory:
```bash
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j2
sudo make install
```

Update `.bashrc`:
```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
```

Apply changes:
```bash
source ~/.bashrc
```

### MAVproxy
Install MAVproxy:
```bash
pip3 install mavproxy
```

Fix path issues:
```bash
export PATH=$PATH:~/.local/bin
source ~/.bashrc
```

### Launch Single UAV SITL
In terminal 1:
```bash
cd ~/ardupilot/Tools/autotest
./sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0
```

In terminal 2:
```bash
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_ardupilot.world
```

Control drone (after seeing `AP: EKF3 IMU1 is using GPS`):
```bash
mode guided
arm throttle
takeoff 5
```

Disable PreArm checks if needed:
```bash
param set ARMING_CHECK 0
```

## Simulation Prep

### Gazebo Models
Clone premade models:
```bash
git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git
export GAZEBO_MODEL_PATH=~/gazebo_models_worlds_collection/models:$GAZEBO_MODEL_PATH
echo "export GAZEBO_MODEL_PATH=~/gazebo_models_worlds_collection/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
source ~/.bashrc
```

### ROSCAM Plugin
Install in home directory:
```bash
source /opt/ros/noetic/setup.bash
git clone https://github.com/r0ch1n/ardupilot_gazebo_roscam.git ~/ardupilot_gazebo_roscam
cd ~/ardupilot_gazebo_roscam
catkin init
```

If `catkin` is missing:
```bash
sudo apt install python3-catkin-tools
```

Initialize and build:
```bash
cd src
catkin_create_pkg ardupilot_gazebo
cd ..
catkin build
```

Update Gazebo paths:
```bash
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo_roscam/src/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:$GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=/opt/ros/melodic/lib:$GAZEBO_PLUGIN_PATH
echo "export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/ardupilot_gazebo_roscam/src/ardupilot_gazebo/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:\$GAZEBO_PLUGIN_PATH" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=/opt/ros/melodic/lib:\$GAZEBO_PLUGIN_PATH" >> ~/.bashrc
source ~/.bashrc
```

### Test ROSCAM Plugin
```bash
source ~/ardupilot_gazebo_roscam/devel/setup.bash
roslaunch ardupilot_gazebo iris_with_roscam.launch
```

Check camera feed:
- Open new terminal, type `rqt`, select camera topic and check whether the live feed from the drone is visible.
