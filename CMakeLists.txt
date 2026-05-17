cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)
add_definitions(-DORB_SLAM3_VERSION="1.1.0-hardened")
add_definitions(-DORB_SLAM3_LIB_PATH="${ORB_SLAM3_ROOT_DIR}")
add_link_options("-Wl,--copy-dt-needed-entries")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d highgui video)
find_package(ORB_SLAM3 REQUIRED)

set(ORB_SLAM3_ROOT_DIR "/opt/slam_ws/src/ORB_SLAM3")

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(ALL_LIBS
  ${ORB_SLAM3_LIBRARIES}
  ${OpenCV_LIBS}
  opencv_calib3d
  opencv_core
  opencv_imgproc
  ${Pangolin_LIBRARIES}
)

macro(build_slam_node target)
  add_executable(${target} ${ARGN})
  ament_target_dependencies(${target} rclcpp sensor_msgs cv_bridge message_filters visualization_msgs geometry_msgs)
  target_link_libraries(${target} ${ALL_LIBS})
endmacro()

build_slam_node(mono src/monocular/mono.cpp src/monocular/monocular-slam-node.cpp)
build_slam_node(rgbd src/rgbd/rgbd.cpp src/rgbd/rgbd-slam-node.cpp)
build_slam_node(stereo src/stereo/stereo.cpp src/stereo/stereo-slam-node.cpp)
build_slam_node(stereo-inertial src/stereo-inertial/stereo-inertial.cpp src/stereo-inertial/stereo-inertial-node.cpp)

install(TARGETS mono rgbd stereo stereo-inertial DESTINATION lib/${PROJECT_NAME})
ament_package()
