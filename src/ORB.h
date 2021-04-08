#pragma once

#include "parameters.h"



struct ORB
{
    cv::Mat image;
    cv::Mat image_intensity;
    cv::Mat thumbnail;
    pcl::PointCloud<PointType>::Ptr cloud;

    vector<cv::Point3f> orb_point_3d;
    vector<cv::Point2f> orb_point_2d_uv;
    vector<cv::Point2f> orb_point_2d_norm;
    vector<cv::KeyPoint> orb_keypoints;
    cv::Mat orb_descriptors;

    void create_keypoints();

    ORB::ORB(const cv::Mat &_image_intensity, 
             const pcl::PointCloud<PointType>::Ptr _cloud);

};