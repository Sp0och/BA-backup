#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
// #include <opencv2/xfeatures2d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
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

extern string PROJECT_NAME;
extern string CLOUD_TOPIC;
extern string PATH_TOPIC;
extern int IMAGE_WIDTH;
extern int IMAGE_HEIGHT;
extern int IMAGE_CROP;
extern int NUM_ORB_FEATURES;
extern int NUM_SIFT_FEATURES;
extern int NUM_BRISK_FEATURES;
extern int MIN_LOOP_FEATURE_NUM;
extern double SKIP_TIME;
extern int NUM_THREADS;
extern int DEBUG_IMAGE;
extern double MATCH_IMAGE_SCALE;
extern cv::Mat MASK;
extern pcl::PointCloud<PointType>::Ptr cloud_traj;





struct PointOuster {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint8_t noise;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointOuster,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint8_t, noise, noise)
    (std::uint16_t, ring, ring)
)




