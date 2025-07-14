# Multi-UAV Swarm Simulation - Version 1

## Description
This tutorial guides you through setting up a swarm simulation environment for 4 quadcopters using Ardupilot + SITL on Gazebo. Each quadcopter, modeled as `iris_with_standoffs_demo_1` to `_4`, is equipped with a downward-facing camera for detecting colored cubes. Simulation 1 uses `iris_ardupilot.world` with models: `ground_plane`, `boundary_20by20`, `box_target_red`, `line`, `line_clone`, `redblock_1by1`, and clones. 

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
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:$GAZEBO_PLUGIN_PATH
echo "export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/ardupilot_gazebo_roscam/src/ardupilot_gazebo/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:\$GAZEBO_PLUGIN_PATH" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:\$GAZEBO_PLUGIN_PATH" >> ~/.bashrc
source ~/.bashrc
```

### Test ROSCAM Plugin
```bash
source ~/ardupilot_gazebo_roscam/devel/setup.bash
roslaunch ardupilot_gazebo iris_with_roscam.launch
```

Check camera feed:
- Open new terminal, type `rqt`, select camera topic and check whether the live feed from the drone's camera is visible.

### Simulation 1 Setup
- **World File**: Uses `iris_ardupilot.world` with models: `ground_plane`, `boundary_20by20`, `box_target_red`, `line`, `line_clone`, `redblock_1by1`, `redblock_1by1_clone`, `redblock_1by1_clone_0`, `redblock_1by1_clone_1`, `iris_demo` (includes `iris_with_standoffs_demo_1`), `iris_demo_1` (includes `iris_with_standoffs_demo_2`), `iris_demo_2` (includes `iris_with_standoffs_demo_3`), `iris_demo_3` (includes `iris_with_standoffs_demo_4`).
- **Model Copy**: Copy models (`Custom_models`, `iris_with_lidar`, `iris_with_stanoffs`, `iris_with_stanoffs_demo`, `iris_with_stanoffs_demo_1` to `_3`, `ros_iris_with_ardupilot`) from `ardupilot_gazebo/models` to the `ardupilot_gazebo` package, merging and replacing as needed.
- **Launch Files**: Use the following files from the `sim1_github` directory:
  - `begin_sim1.sh`: Launches Gazebo with `sproj_sim1.launch` and 4 UAV instances (`-I0` to `-I3`).
  - `final_sim1.py`: Main Python script to run the simulation.
  - `restart_sim1.sh`: Restarts the simulation.
  - `utils_sim1.py`: Utility functions for the simulation.
- **Multi-UAV Launch**:
  1. Ensure all models are copied and paths are set in `.bashrc`.
  2. Run the simulation:
     ```bash
     chmod +x begin_sim1.sh
     ./begin_sim1.sh
     ```
  3. In a new terminal, execute the main script:
     ```bash
     python3 final_sim1.py
     ```
