cmake_minimum_required(VERSION 3.0.2)
project(descriptor_and_image)

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
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  DEPENDS PCL
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${include}
  )

set(LIBRARY_FILES
    src/ImageHandler.cpp
    src/helper.cpp
    src/ORB.cpp
    src/ORB_Framehandler
    src/BRISK.cpp
    src/BRISK_Framehandler
    src/KLT.cpp
    )
    

add_library(${PROJECT_NAME}_lib 
            ${LIBRARY_FILES})

add_executable(${PROJECT_NAME}_main "src/main_node.cpp")

 target_compile_options(${PROJECT_NAME}_main PRIVATE ${OpenMP_CXX_FLAGS})
 target_link_directories(${PROJECT_NAME}_main  
                          PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include") 
 target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES}
                                            ${PCL_LIBRARIES} 
                                            ${OpenCV_LIBRARIES} 
                                            ${OpenMP_CXX_FLAGS}
                                            ${PROJECT_NAME}_lib
                                            )
