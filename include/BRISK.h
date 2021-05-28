#pragma once
#include "parameters.h"






class BRISK
{
    public:
    cv::Mat image;
    cv::Mat input_image;
    pcl::PointCloud<PointType>::Ptr cloud;

    vector<cv::Point2f> brisk_keypoints_2d;
    vector<cv::Point3f> brisk_points_3d;
    vector<cv::Point2f> brisk_point_2d_norm;
    vector<cv::KeyPoint> brisk_keypoints;
    cv::Mat brisk_descriptors;

    BRISK(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _mode);


/**
 * Stores the BRISK keypoints in the vector in the std::vector format, creates the descriptors around the keypoints and calls the ransac point creator
 * @param brisk_keypoints_2d is then the vector containing the keypoints
 * */
    void create_descriptors();
    
    
    
    
    void points_for_ransac();


    void duplicate_filtering();
    
    private:
    ros::NodeHandle n;
    ros::Publisher KP_pub_intensity;
    ros::Publisher KP_pub_range;
    ros::Publisher KP_pub_ambient;
    int mode;



};



