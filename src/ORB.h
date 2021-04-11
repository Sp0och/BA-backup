#pragma once

#include "parameters.h"



class ORB
{
    public:
    cv::Mat image;
    cv::Mat image_intensity;
    pcl::PointCloud<PointType>::Ptr cloud;

    vector<cv::Point3f> orb_point_3d;
    vector<cv::Point2f> orb_keypoints_2d;
    vector<cv::Point2f> orb_point_2d_norm;
    vector<cv::KeyPoint> orb_keypoints;
    cv::Mat orb_descriptors;

    void create_keypoints();
    void create_descriptors();
    /* void real_keypoints(const vector<cv::Point2f>& in_vector, 
                        vector<cv::Point3f>& out_3d,
                        vector<cv::Point2f>& out_2d_norm
                        vector<uchar>& out_status); */

    ORB(const cv::Mat &_image_intensity, 
             const pcl::PointCloud<PointType>::Ptr _cloud);

    private:
    ros::NodeHandle n;
    ros::Publisher KP_pub;


};