# POINT-LIO-SLAM

This repository provides ready-to-use ROS 2 Humble configuration and launch files for composing [FAST-LIO-SAM-QN](https://github.com/engcang/FAST-LIO-SAM-QN) and [FAST-LIO-Localization-QN](https://github.com/engcang/FAST-LIO-Localization-QN) (original repositories) along with their respective [ROS 2 wrappers](https://github.com/illusionaryshelter/FAST-LIO-SAM-QN) and [FAST-LIO-Localization-QN ROS 2 wrapper](https://github.com/se7oluti0n/FAST-LIO-Localization-QN). These configurations are tailored for UniLidar L1 indoor mapping and localization. No new algorithmic code is introduced; instead, this repository packages configuration files, launch scripts, and RViz presets to enable seamless integration and one-command bring-up of the upstream projects.

## Dependencies
- C++ >= 17
- OpenMP >= 4.5
- CMake >= 3.10.0
- Eigen >= 3.2
- Boost >= 1.54
- ROS 2 Humble (Ubuntu 22.04 recommended)
- GTSAM
- TEASER++
- TBB (used by Quatro for speed)

### Install GTSAM (4.1.1)
```bash
wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
unzip gtsam.zip
cd gtsam-4.1.1/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j16
```

### Install TEASER++
```bash
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
sudo make install -j16
sudo ldconfig
```

### Install TBB
```bash
sudo apt install -y libtbb-dev
```

## Ubuntu 20.04 instructions
Dependencies:
- CMake 3.0.0+
- gcc 4.8.1+

Install CMake using apt:
```bash
sudo apt install -y cmake
```

Compile and install Livox-SDK2:
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

## Workspace layout
Create a ROS 2 workspace and clone the required repositories under `src/`:
```bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src
# This repo
git clone https://github.com/mvu20002/POINT-LIO-SLAM.git
# Upstream components
git clone https://github.com/mvu20002/Localization-QN.git
git clone https://github.com/mvu20002/SAM-QN.git
git clone https://github.com/illusionaryshelter/Quatro.git
git clone https://github.com/illusionaryshelter/nano_gicp.git
git clone https://github.com/mvu20002/livox_ros_driver2_humble.git
git clone https://github.com/dfloreaa/point_lio_ros2.git
```

## Build
```bash
cd ~/my_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run
- Mapping (indoor):
```bash
ros2 launch point-lio-slam mapping.launch.py
```
- Localization (indoor):
```bash
ros2 launch point-lio-slam localization.launch.py
```

RViz presets are in `rviz/`. Core parameters for UniLidar L1 are in `config/` (e.g., `lio_unilidar_l1.yaml`, `mapping_indoor.yaml`, `localization_indoor.yaml`).
