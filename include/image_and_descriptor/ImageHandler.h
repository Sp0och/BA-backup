#pragma once

#include "setup.h"


//Create images from the laserscan
class ImageHandler
{
public:

    ros::NodeHandle nh;

    ros::Publisher pub_image;
    ros::Publisher pub_intensity;

    //pictures
    cv::Mat image_range;
    cv::Mat image_noise;
    cv::Mat image_intensity;
    int M_IMAGE_WIDTH;
    int M_IMAGE_HEIGHT;
    //blurr strength
    int M_BLURR_SIZE;

    //pointcloud
    pcl::PointCloud<PointType>::Ptr cloud_track;

    //class constructor: reset the pointcloud and declare a publisher
    ImageHandler();

    void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    void pubImage(ros::Publisher *this_pub, const cv::Mat& this_image, std_msgs::Header this_header, string image_format);
};