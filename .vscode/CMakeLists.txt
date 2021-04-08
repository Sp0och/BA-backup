cmake_minimum_required(VERSION 3.0.2)
project(descriptor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  # dbow3
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
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  DEPENDS PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRSRY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

file(GLOB all_source_files 
"src/*.cpp"
"ThirdParty/DUtils/*.cpp"
"ThirdParty/DVision/*.cpp"
)

 add_executable(${PROJECT_NAME}_main ${all_source_files})
 target_compile_options(${PROJECT_NAME}_main PRIVATE ${OpenMP_CXX_FLAGS})
 target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${DBoW3_LIBRARIES} ${OpenMP_CXX_FLAGS})
