
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

    //Set up Constructor
    ORB(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _mode);

    /**
     * Stores the ORB keypoints in the vector in the std::vector format, creates the descriptors around the keypoints and calls the ransac point creator
     * @param orb_keypoints_2d is then the vector containing the keypoints
     * */
    void create_descriptors();
    /**
     * Filter out keypoints in a certain radius arond themselves using a cv::Mat marking system
     * */
    void duplicate_filtering();
    
    /**
     * 3D points for SVD
     * */
    void get_3D_data();

    
    private:
    ros::NodeHandle n;
    ros::Publisher KP_pub_intensity;
    ros::Publisher KP_pub_range;
    ros::Publisher KP_pub_ambient;
    int mode;


};
