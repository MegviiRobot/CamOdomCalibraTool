# CAM_ODO_CALIB
## calibrate the internel parameters and extrinsec parameters between camera and odometer

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **Others**
OpenCV 3.3 , Eigen3



## 2. Build CAM_ODO_CALIB
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone xxx
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)


## 3 Attention

### 3.1 record your testBag
record the rosbag with image and odom datas.

### 3.2 Configuration file
Write a config file for your rosbag.

### 3.3 Remind
You need to change the "wheel_callback" function for your odo input;
You need to remind the coordinate directions relations between camera and odometer.the struct "scan_match_results" in "data_selection.cpp" ,"axis" in main_node.cpp , "M" matrix in solveQyx.cpp may need to be changed.


## 4 calibrate internal and extrinsic parameters between camera and odometer

```
    roslaunch cam_odo_cal S800.launch
    rosbag play xxx.bag
```

