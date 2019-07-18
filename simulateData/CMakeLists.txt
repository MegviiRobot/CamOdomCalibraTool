cmake_minimum_required(VERSION 2.8)
project(SIMULATEODOCAM)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR})

add_executable(simulate main.cpp simulateOdoCam.cpp)
target_link_libraries(simulate ${OpenCV_LIBS})