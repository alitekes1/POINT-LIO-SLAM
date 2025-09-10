# POINT-LIO-SLAM

This repository provides ready-to-use ROS 2 Humble configuration and launch files for composing [FAST-LIO-SAM-QN](https://github.com/engcang/FAST-LIO-SAM-QN) and [lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2) (original repositories) along with ROS 2 wrapper [FAST-LIO-SAM-QN](https://github.com/illusionaryshelter/FAST-LIO-SAM-QN). These configurations are tailored for UniLidar L1 indoor mapping and localization. No new algorithmic code is introduced; instead, this repository packages configuration files, launch scripts, and RViz presets to enable seamless integration and one-command bring-up of the upstream projects.

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
- Livox-SDK2 (for UniLidar L1)


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
### Install Livox-SDK2
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### Install TBB
```bash
sudo apt install -y libtbb-dev
```


### Workspace layout
Create a ROS 2 workspace and clone the required repositories under `src/`:
```bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src
git clone https://github.com/alitekes1/POINT-LIO-SLAM.git
git clone https://github.com/alitekes1/lidar_localization_ros2.git 
mkdir third_party && cd third_party
git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
git clone https://github.com/mvu20002/SAM-QN.git
git clone https://github.com/illusionaryshelter/Quatro.git
git clone https://github.com/illusionaryshelter/nano_gicp.git
git clone https://github.com/mvu20002/livox_ros_driver2_humble.git
git clone https://github.com/dfloreaa/point_lio_ros2.git
```

## Build
```bash
cd ~/my_ws
source /opt/ros/{$ROS_DISTRO}/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

NOTE: The configuration parameters are set for UniLidar L1 on a mobile robot platform. Data used for mapping and localization couldn't be shared due to privacy concerns. However, the configuration files are provided for you to adapt to your own data.

### Mapping
To run the mapping process, run the following command:
```bash
ros2 launch point-lio-slam mapping.launch.py
# another terminal
ros2 bag play <path_to_ros2_bag>
```

### Localization
Once you have the map, you can use it for localization. Change the path to map in config file `localization.yaml` to the path of your saved .pcd map. Then, run the following command:

```bash
ros2 launch point-lio-slam lidar_localization.launch.py 
```
 
