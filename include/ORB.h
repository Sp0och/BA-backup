
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
    vector<cv::KeyPoint> orb_keypoints;
    cv::Mat orb_descriptors;

    //Set up Constructor
    ORB(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _image_source,int& ec,int& dfc, int& mdfc, int& count);

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
    ros::Publisher KP_pub_intensity,KP_pub_range,KP_pub_ambient,dupl_publisher,pub_3D,before_3D,after_3D;
    
    int image_source;

    int NUM_ORB_FEATURES;
    int ORB_ACCURACY;
    float SCALE_FACTOR;
    int LEVELS;
    ros::Time RAW_TIME;

    int extracted_count;
    int duplicate_filtered_count;
    int min_distance_filtered_count;
    int COUNT;


};
