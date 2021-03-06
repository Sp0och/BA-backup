#pragma once

#include "setup.h"
#include "helper.h"
using namespace Eigen;

class KLT {
    public:
    
    // core methods

    KLT(int image_source, cv::Mat& MASK);

    void KLT_Iteration(const cv::Mat& input_image,const pcl::PointCloud<PointType>::Ptr _cloud,ros::Time _raw_time);

    // visualization methods

    /**
     * Publish the extracted points
     * */
    void publish_extraction(ros::Publisher* publisher, const cv::Mat& cur_image,
        const vector<cv::Point2f>& cur_keypoints,cv::Scalar point_color,int circle_size);
    /**
     * publish the KLT trackings version with 1 image
     * */
    void publish_tracking(ros::Publisher* publisher, const cv::Mat& cur_image,
        const vector<cv::Point2f>& cur_keypoints, const vector<cv::Point2f>& prev_keypoints,cv::Scalar point_color,cv::Scalar line_color, int circle_size, bool draw_lines);
    /**
     * publish tracked KLT points in the matching format with two concatenated images
     * */
    void publish_tracking_2F(ros::Publisher* publisher, const cv::Mat& cur_image, const cv::Mat& prev_image,
        const vector<cv::Point2f>& cur_keypoints, const vector<cv::Point2f>& prev_keypoints,cv::Scalar point_color,cv::Scalar line_color, int circle_size, bool draw_lines);

    void visualizer_3D(const MatrixXd& cur_SVD, const MatrixXd& prev_SVD);

    // data storage methods

    void set_plotting_columns_and_start_pose();

    void store_coordinates(const Vector3d& t_input, const Matrix3d& R);

    void store_feature_number(const MatrixXd& cur_SVD);

    // Closed form application

    void SVD(const MatrixXd& cur_SVD,const MatrixXd& prev_SVD);

    // publishing estimated transform

    void publish_tf();

    private:
    ros::NodeHandle M_n_KLT;
    ros::Publisher M_pub_KLT_int,M_pub_KLT_ran,M_pub_KLT_amb,M_pub_KLT_tf,M_kp_pc_publisher_cur,M_kp_pc_publisher_prev,M_line_publisher,M_odom_publisher,M_extraction_publisher,M_ransac_publisher,M_duplicate_publisher,M_match_publisher;
    
    cv::Mat M_cur_image;
    cv::Mat M_prev_image;
    std::vector<cv::Point2f> M_cur_corners;
    std::vector<cv::Point2f> M_prev_corners;
    MatrixXd M_prev_3D_points;
    MatrixXd M_cur_3D_points;
    pcl::PointCloud<PointType>::Ptr M_prev_cloud;
    pcl::PointCloud<PointType>::Ptr M_cur_cloud;
    Matrix4d M_my_pose;

    bool M_APPLY_RANSAC_FILTERING;
    bool M_APPLY_DISTANCE_FILTERING;

    int M_START_POSE;
    int M_image_source;
    int M_IMAGE_WIDTH;
    int M_IMAGE_CROP;
    cv::Mat M_MASK;
    ros::Time M_raw_time;  //timestamps of the bagfile
    int M_MIN_KLT_FEATURES;

    //goodFeaturesToTrack
    double M_QUALITY_LEVEL;
    double M_MIN_KLT_DETECTION_DISTANCE;
    int M_BLOCKSIZE;
    int M_MAX_KLT_FEATURES;
    //OpticalFlowPyr
    double M_EPSILON;
    int M_CRITERIA_REPS;
    int M_OPT_SIZE;
    int M_NUM_PYRAMIDS;
    bool M_USE_HARRIS;

    cv::Scalar M_POINT_COLOR;
    cv::Scalar M_LINE_COLOR;

    bool M_SHOULD_STORE;
    std::string M_FILE_PATH;
    std::string M_DIRECTORY;
    ofstream M_OUT;

    helper* M_Helper;
};