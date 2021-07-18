#pragma once
#include "setup.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  

namespace helper{

    //Conversion and Trimming

    static void keypointTransition(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2d>& points_in);

    template <typename Derived>

    static void trimVector(vector<Derived>& v,const vector<bool>& status);

    static void trim_matrix(Eigen::MatrixXd& m, vector<bool>& status);

    template <typename Derived1>

    static void get_3D_points(vector<Derived1>& points_2D, Eigen::MatrixXd& points_3D, const pcl::PointCloud<PointType>::Ptr PC);

    template <typename Derived2>  

    static void get_3D_points_adapt_status(vector<Derived2>& points_2D, Eigen::MatrixXd& points_3D, const pcl::PointCloud<PointType>::Ptr PC,std::vector<bool>& status);

    //Filtering

    static void RANSAC_filtering(std::vector<cv::Point2d>& sorted_2d_cur, std::vector<cv::Point2d>& sorted_2d_prev, Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD);

    static void filtering_3D(Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD, vector<cv::Point2d>& cur,vector<cv::Point2d>& prev);

    static void RANSAC_filtering_f(std::vector<cv::Point2f>& sorted_fd_cur, std::vector<cv::Point2f>& sorted_fd_prev, Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD);

    static void filtering_3D_f(Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD, vector<cv::Point2f>& cur,vector<cv::Point2f>& prev);

    //Visualization Functions

    template <typename Keypoints>

    static void publish_keypoints (ros::Publisher* publisher, const cv::Mat& input_image, const vector<Keypoints>& keypoints, const int circle_size,const cv::Scalar point_color, int image_source);

    static void publish_3D_keypoints(const Eigen::MatrixXd& points,const  ros::Publisher* kp_pc_publisher, ros::Time raw_time);

    static void publish_lines_3D(const Eigen::MatrixXd& cur_SVD,const Eigen::MatrixXd& prev_SVD,const  ros::Publisher* line_publisher, ros::Time raw_time);

}