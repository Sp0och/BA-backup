#pragma once
#include "parameters.h"






class BRISK
{
    public:
    cv::Mat image;
    cv::Mat input_image;
    pcl::PointCloud<PointType>::Ptr cloud;

    vector<cv::Point2d> brisk_keypoints_2d;
    vector<cv::Point3d> brisk_points_3d;
    vector<cv::KeyPoint> brisk_keypoints;
    cv::Mat brisk_descriptors;

    
    /**
     * constructor with the publisher initialization, image_source setting, image handling and so on
     * */
    BRISK(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _image_source);

    /**
     * Stores the BRISK keypoints in the vector in the std::vector format, creates the descriptors around the keypoints and calls the ransac point creator
     * @param brisk_keypoints_2d is then the vector containing the keypoints
     * */
    void create_descriptors();
    
    /**
     * Get the 3D coordinates of the extracted 2d keypoints
     * */
    void get_3D_data();

    /**
     * Filter out duplicate keypoints
     * */
    void duplicate_filtering();
    
    private:
    ros::NodeHandle n;
    ros::Publisher KP_pub_intensity,KP_pub_range,KP_pub_ambient,dupl_publisher,pub_3D;
    int image_source;
    

    int BRISK_THRESHOLD;
    int OCTAVES;
    float PATTERN_SCALE;

};



