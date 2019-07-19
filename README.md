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



## 2. Build
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone xxx
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)


## 3. Attention

### 3.1 record your testBag
record the rosbag with image and odom datas.

### 3.2 Configuration file
Write a config file for your rosbag.

### 3.3 Remind
You need to change the "wheel_callback" function for your odo input;
You need to remind the coordinate directions relations between camera and odometer.the struct "scan_match_results" in "data_selection.cpp" ,"axis" in main_node.cpp , "M" matrix in solveQyx.cpp may need to be changed.


## 4. Calibration

```
    roslaunch cam_odo_cal S800.launch
    rosbag play xxx.bag
```

## 5. Authors

1. Dongfu ZHU, Research Intern at Megvii, will graduate from Huazhong University of Science and Technology with a bachelor degree in July 2020, contact with  (dongfuzhu at hust dot edu dot cn).

2. Zheng CHAI, Researcher at Megvii-R-SLAM, if you have any question, please contact (icecc_sunny at 163 dot com)

## 6. References

Guo, C. X., Mirzaei, F. M., & Roumeliotis, S. I. (2012, May). An analytical least-squares solution to the odometer-camera extrinsic calibration problem. In 2012 IEEE International Conference on Robotics and Automation (pp. 3962-3968). IEEE.

Heng, L., Li, B., & Pollefeys, M. (2013, November). Camodocal: Automatic intrinsic and extrinsic calibration of a rig with multiple generic cameras and odometry. In 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems (pp. 1793-1800). IEEE.

Censi, Andrea, Franchi, Antonio, Marchionni, Luca & Oriolo, Giuseppe (2013). Simultaneous calibration of odometry and sensor parameters for mobile robots. IEEE Transactions on Robotics, 29, 475-492.

