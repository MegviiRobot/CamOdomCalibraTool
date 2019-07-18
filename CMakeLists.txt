cmake_minimum_required(VERSION 2.8.3)
project(cam_odo_cal)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport)

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} 
    ${CERES_INCLUDE_DIRS}
    camOdoCalib/include
    )

find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)



catkin_package()

SET(CSM_SRC 
  camOdoCalib/src/csm/laser_data.cpp
  camOdoCalib/src/csm/math_utils.cpp
  camOdoCalib/src/csm/math_utils_gsl.cpp
  camOdoCalib/src/csm/utils.cpp
  camOdoCalib/src/csm/logging.cpp
  #csm/sm_options.cpp
  camOdoCalib/src/csm/orientation.cpp
  camOdoCalib/src/csm/clustering.cpp)

SET(EGSL_SRC
    camOdoCalib/src/egsl/egsl.cpp
    camOdoCalib/src/egsl/egsl_conversions.cpp
    camOdoCalib/src/egsl/egsl_misc.cpp
    camOdoCalib/src/egsl/egsl_ops.cpp)

ADD_LIBRARY(csm_eigen
    ${CSM_SRC}
    ${EGSL_SRC} 
    )

add_library(calc_lib
    camOdoCalib/src/camera_models/src/Utils.cc
    camOdoCalib/src/camera_models/src/Camera.cc
    camOdoCalib/src/camera_models/src/CameraFactory.cc
    camOdoCalib/src/camera_models/src/PinholeCamera.cc
    camOdoCalib/src/camera_models/src/PinholeFullCamera.cc
    camOdoCalib/src/camera_models/src/CataCamera.cc
    camOdoCalib/src/camera_models/src/EquidistantCamera.cc
    camOdoCalib/src/camera_models/src/ScaramuzzaCamera.cc
    camOdoCalib/src/calc_cam_pose/calcCamPose.cpp
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/Edge.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/FloatImage.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/Gaussian.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/GLine2D.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/GLineSegment2D.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/GrayModel.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/Homography33.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/MathUtil.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/Quad.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/Segment.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/TagDetection.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/TagDetector.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/TagFamily.cc
    camOdoCalib/src/calc_cam_pose/ethz_apriltag/UnionFindSimple.cc
    camOdoCalib/src/data_selection/data_selection.cpp)

add_library(calib_lib
    camOdoCalib/src/solver.cpp
    camOdoCalib/src/utils.cpp
    camOdoCalib/src/solveQyx.cpp
 )

target_link_libraries(calc_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(cam_odo_cal camOdoCalib/src/main_node.cpp )
target_link_libraries(cam_odo_cal calib_lib calc_lib csm_eigen ${CERES_LIBRARIES}) 


#### simulation  ###
#add_executable(simulate simulateData/main.cpp simulateData/simulateOdoCam.cpp)
#target_link_libraries(simulate ${OpenCV_LIBS})