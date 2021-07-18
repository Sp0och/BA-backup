
#pragma once

#include "setup.h"
#include "helper.h"

class ORB
{
    public:

    cv::Mat image;
    cv::Mat input_image;
    pcl::PointCloud<PointType>::Ptr cloud;

    vector<cv::Point2d> orb_keypoints_2d;
    vector<cv::Point3d> orb_points_3d;
    vector<cv::KeyPoint> orb_keypoints;
    cv::Mat orb_descriptors;

    //Set up Constructor
    ORB(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _image_source, cv::Mat& MASK);

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
    ros::NodeHandle M_n;
    ros::Publisher M_KP_pub_intensity,M_KP_pub_range,M_KP_pub_ambient,M_dupl_publisher,M_pub_3D;
    
    ros::Time M_RAW_TIME;
    int M_image_source;
    int M_IMAGE_WIDTH;
    int M_IMAGE_HEIGHT;
    cv::Mat M_MASK;

    int M_NUM_ORB_FEATURES;
    int M_ORB_ACCURACY;
    float M_SCALE_FACTOR;
    int M_LEVELS;

    bool M_APPLY_DUPLICATE_FILTERING;
    int M_DUPLICATE_FILTERING_SIZE;
    

    helper* M_Helper;
};
