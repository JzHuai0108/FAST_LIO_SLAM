# FAST_LIO_SLAM

## News
-  ``Aug 2021``: The Livox-lidar tests and corresponding launch files will be uploaded soon. Currenty only Ouster lidar tutorial videos had been made. 

## What is FAST_LIO_SLAM?
Integration of 
1. [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) (Odometry): A computationally efficient and robust LiDAR-inertial odometry (LIO) package
2. [SC-PGO](https://github.com/gisbi-kim/SC-A-LOAM) (Loop detection and Pose-graph Optimization): [Scan Context](https://github.com/irapkaist/scancontext)-based Loop detection and GTSAM-based Pose-graph optimization

## Features
- An easy-to-use plug-and-play LiDAR SLAM 
    - FAST-LIO2 and SC-PGO run separately (see below How to use? tab).
    - SC-PGO takes odometry and lidar point cloud topics from the FAST-LIO2 node. 
    - Finally, an optimized map is made within the SC-PGO node. 

## Install ceres solver


## Install gtsam

```
sudo apt-get install libtbb-dev
cd $HOME/Documents/slam_src
git clone https://github.com/borglab/gtsam.git --recursive
cd gtsam

git checkout b1f441dea9c2c59354c363a60c6e0f01305985ee
# 6c85850147751d45cf9c595f1a7e623d239305fc
# 342f30d148fae84c92ff71705c9e50e0a3683bda(previously tested commit)
mkdir build
cd build

# GTSAM can be installed locally, e.g., at $HOME/slam_devel, but 
# /usr/local is recommended as it has no issue when debugging in QtCreator.

cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
# -DEIGEN3_INCLUDE_DIR=$HOME/slam_devel/include/eigen3 -DEIGEN_INCLUDE_DIR=$HOME/slam_devel/include/eigen3 # for Ubuntu 16
# In Ubuntu 16, to circumvent the incompatible system-wide Eigen, passing the local Eigen by EIGEN_INCLUDE_DIR is needed.
# -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF # for linux, https://github.com/gisbi-kim/FAST_LIO_SLAM/issues/8

make -j $(nproc) check # (optional, runs unit tests)
make -j $(nproc) install
```

## Build
```
# build livox_ros_driver2
cd cd src/blss_lidar_slam/livox_ros_driver2/
./build.sh ROS1 $HOME/Documents/slam_devel  -DPYTHON_EXECUTABLE=/usr/bin/python3

# build fast-lio2
cd catkin_ws
catkin build -DCMAKE_INSTALL_PREFIX=$HOME/Documents/slam_devel

```

## How to use?
- The below commands and the launch files are made for playing the [MulRan dataset](https://sites.google.com/view/mulran-pr/home), but applicable for livox lidars in the same way (you could easily make your own launch files).
```
    # terminal 1: run FAST-LIO2 
    mkdir -p ~/catkin_fastlio_slam/src
    cd ~/catkin_fastlio_slam/src
    git clone https://github.com/JzHuai0108/FAST_LIO_SLAM.git
    git clone https://github.com/Livox-SDK/livox_ros_driver
    cd .. 
    catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DGTSAM_DIR=/home/pi/Documents/slam_devel/lib/cmake/GTSAM -DCMAKE_BUILD_TYPE=Release

    source devel/setup.bash
    roslaunch fast_lio mapping_ouster64_mulran.launch # setting for MulRan dataset 

    # open the other terminal tab: run SC-PGO
    cd ~/catkin_fastlio_slam
    source devel/setup.bash
    roslaunch aloam_velodyne fastlio_ouster64.launch # setting for MulRan dataset 

    # open the other terminal tab
    # run file_player_mulran (for the details, refer here https://github.com/irapkaist/file_player_mulran)
```

### [NCLT velodyne HDL32 dataset provided by fastlio2](https://drive.google.com/drive/folders/1VBK5idI1oyW0GC_I_Hxh63aqam3nocNK)
```
# one terminal
source devel/setup.bash
roslaunch fast_lio mapping_velodyne.launch
# another terminal
source devel/setup.bash
roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch
# third terminal
rosbag play /media/jhuai/SeagateData/jhuai/data/fastlio2/nclt/20121201.bag /points_raw:=/velodyne_points /imu_raw:=/imu/data
```

### HiltiSLAM2022 dataset
```
# one terminal
source devel/setup.bash
roslaunch fast_lio mapping_hesai32.launch save_directory:=/home/pi/Desktop/temp/
# another terminal
source devel/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Documents/slam_devel/lib
roslaunch aloam_velodyne aloam_hesai32.launch save_directory:=/home/pi/Desktop/temp/
# third terminal
rosbag play /media/pi/BackupPlus/jhuai/data/hiltislam2022/exp21_outside_building.bag
```

### Coloradar dataset
```
mkdir -p /home/pi/Desktop/temp/arpg_lab_run1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Documents/slam_devel/lib
roslaunch aloam_velodyne fastlio_ouster64_coloradar.launch save_directory:=/home/pi/Desktop/temp/arpg_lab_run1/ bagname:=/media/pi/BackupPlus/jhuai/data/coloradar/rosbags/arpg_lab_run1.bag
```

### Custom dataset
Create launch files following the above examples, noting that the aloam nodes should be disabled and 
that aloam outputs should be substituted for by the fastlio outputs as in aloam_velodyne_HDL_32.launch.

```
source devel/setup.bash
roslaunch fast_lio mapping_vlp16_whu_ugv.launch save_directory:=/home/jhuai/Desktop/temp/


source devel/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Documents/slam_devel/lib
roslaunch aloam_velodyne aloam_vlp16_whu_ugv.launch save_directory:=/home/jhuai/Desktop/temp/

rosbag play /media/jhuai/SeagateData/jhuai/data/homebrew/mycar_nav/20221021/lidar1.bag \
    /media/jhuai/SeagateData/jhuai/data/homebrew/mycar_nav/20221021/radar1.bag -s 100

rosbag play /media/jhuai/SeagateData/jhuai/data/homebrew/mycar_nav/20230814/lidar7.bag \
    /media/jhuai/SeagateData/jhuai/data/homebrew/mycar_nav/20230814/radar7.bag -s 40

```

## Utility
- We support keyframe scan saver (as in .pcd) and provide a script reconstructs a point cloud map by merging the saved scans using the optimized poses. See [here](https://github.com/gisbi-kim/FAST_LIO_SLAM/blob/bf975560741c425f71811c864af5d35aa880c797/SC-PGO/utils/python/makeMergedMap.py#L7).

## Example results 
- [Tutorial video 1](https://youtu.be/nu8j4yaBMnw) (using KAIST 03 sequence of [MulRan dataset](https://sites.google.com/view/mulran-pr/dataset))
    - Example result captures 
        <p align="center"><img src="docs/kaist03.png" width=700></p>
    - [download the KAIST03 pcd map](https://www.dropbox.com/s/w599ozdg7h6215q/KAIST03.pcd?dl=0) made by FAST-LIO-SLAM, 500MB
    
- [Example Video 2](https://youtu.be/94mC05PesvQ) (Riverside 02 sequence of [MulRan dataset](https://sites.google.com/view/mulran-pr/dataset))
    - Example result captures
        <p align="center"><img src="docs/riverside02.png" width=700></p>
    -  [download the Riverisde02 pcd map](https://www.dropbox.com/s/1aolth7ry4odxo4/Riverside02.pcd?dl=0) made by FAST-LIO-SLAM, 400MB

## Acknowledgements 
- Thanks for [FAST_LIO](https://github.com/hku-mars/FAST_LIO) authors.
- You may have an interest in [this version of FAST-LIO + Loop closure](https://github.com/yanliang-wang/FAST_LIO_LC), implemented by [yanliang-wang](https://github.com/yanliang-wang)


