#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Header.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/Int64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
// #include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
// #include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
// #include <pcl/registration/icp.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
    // #include <tf/transform_datatypes.h>
    #include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <cassert>
#include <memory>



using namespace std;

typedef pcl::PointXYZI PointType;

extern cv::FileStorage fsSettings;

extern pcl::PointCloud<PointType>::Ptr cloud_traj;
extern string CLOUD_TOPIC;
extern int IMAGE_WIDTH;
extern int IMAGE_HEIGHT;
extern int IMAGE_CROP;
extern cv::Mat MASK;
extern ofstream OUT;

extern int MIN_LOOP_FEATURE_NUM;
extern int DUPLICATE_FILTERING_SIZE;
extern double MAX_DEPTH_DISTANCE;
extern bool APPLY_DUPLICATE_FILTERING;
extern bool APPLY_DISTANCE_FILTERING;
extern bool APPLY_RANSAC_FILTERING;

extern int START_POSE;

extern bool SHOULD_STORE;










