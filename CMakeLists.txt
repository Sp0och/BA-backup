cmake_minimum_required(VERSION 3.0.2)
project(image_and_descriptor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  pcl_conversions
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  jsk_rviz_plugins
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  DEPENDS PCL
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  )

set(LIBRARY_FILES
    "src/ImageHandler.cpp"
    "src/helper.cpp"
    "src/ORB.cpp"
    "src/ORB_Framehandler"
    "src/BRISK.cpp"
    "src/BRISK_Framehandler"
    "src/KLT.cpp"
)

add_library(${PROJECT_NAME}_lib ${LIBRARY_FILES})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node "src/image_and_descriptor_node.cpp")
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
