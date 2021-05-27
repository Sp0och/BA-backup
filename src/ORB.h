
#pragma once

#include "parameters.h"


class ORB
{
    public:
    cv::Mat image;
    cv::Mat input_image;
    pcl::PointCloud<PointType>::Ptr cloud;

    vector<cv::Point2d> orb_keypoints_2d;
    vector<cv::Point3d> orb_points_3d;
    vector<cv::Point2d> orb_point_projected;
    vector<cv::KeyPoint> orb_keypoints;
    cv::Mat orb_descriptors;


    ORB(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _mode);

    /**
     * Stores the ORB keypoints in the vector in the std::vector format, creates the descriptors around the keypoints and calls the ransac point creator
     * @param orb_keypoints_2d is then the vector containing the keypoints
     * */
    void create_descriptors();
    
    void duplicate_filtering();
    
    /**
     * Changed up version for 3D points for ICP
     * */
    void points_for_ransac();

    
    private:
    ros::NodeHandle n;
    ros::Publisher KP_pub_intensity;
    ros::Publisher KP_pub_range;
    ros::Publisher KP_pub_ambient;
    int mode;


};
