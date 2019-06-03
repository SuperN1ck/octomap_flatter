# Octomap Flatter
This package flattens an Octomap given by the Octomap-Flatter.

Briefly following is happening. At all times a visual SLAM algorithm is running - in our case [Dense Visual Odometry and SLAM (dvo_slam)](https://github.com/tum-vision/dvo_slam). The SLAM algorithm publishes a pointcloud based on his current state. This pointcloud is used by an [Octomapping Server](https://github.com/OctoMap/octomap_mapping) to reconstruct an octomap based on the realworld. As this octomap is noisy this package tries to remove the noise based on the reconstructed octomap and re-publishes it.

## Installation
We use following non standard ROS-Kinetic packages
```
sudo apt-get install 
    ros-kinetic-octomap 
    ros-kinetic-octomap-mapping
    ros-kinetic-catkin-virtualenv 
```
We recommend to place all packages in the same workspace such as 
```
$ mkdir -p ~/octomap_flatter_ws/src
$ cd ~/octomap_flatter_ws/src/
$ catkin_init_workspace
```
We assume that [`intel-realsense`](https://github.com/IntelRealSense/realsense-ros)-ROS package is already installed and avaivable in the workspace.
### Installing DVO SLAM
We kindly refer to our other fork of this projcet located at: https://github.com/SuperN1ck/dvo_slam/tree/kinetic-devel

Briefly it is installed by placing it in the previous created `src/`-directory in the workspace.
```
$ git@github.com:SuperN1ck/dvo_slam.git
$ git checkout kinetic-devel # If already on kinetic branch not necessary
$ catkin_make -j8 --directory ../ -DCMAKE_BUILD_TYPE=Release --pkg dvo_slam
```
If `g2o` is not installed please look ath the readme in https://github.com/SuperN1ck/dvo_slam/tree/kinetic-devel
### Installing Octomap Flatter
Similarly `octomap_flatter` can be installed
```
$ git clone git@github.com:SuperN1ck/octomap_flatter.git
catkin_make -j8 --directory ../ --pkg octomap_flatter
```
### Compiling together
Of course it is also possible to drop the `--pkg`-parameter in order to compile both packages at the same time. It is important though that you make sure to build `dvo_slam` in Release-Mode as otherwise it is really slow.

Source the workspace
```
$ source ../devel/setup.bash
```

## Running
Once your camera is running you can use our launch-file
```
$ roslaunch octomap_flatter octomap_flatter.launch 
```
If using with bagfile
```
$ roslaunch octomap_flatter octomap_flatter.launch bagfile:=PATH_TO_BAGFILE 
$ rviz -d rviz_config.rviz
```
