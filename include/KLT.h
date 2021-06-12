#pragma once

#include "parameters.h"
using namespace Eigen;

// void publish_tracked_points (ros::Publisher* publisher, cv::Mat& image, const vector<cv::Point2d>& keypoints,cv::Scalar line_color, int circle_size){
//     cv::cvtColor(image, image, CV_GRAY2RGB);
//     for(int i = 0; i < (int)keypoints.size(); i++){
//         cv::Point2d cur_pt = keypoints[i];
//         cv::circle(image,cur_pt,circle_size,line_color);
//         cv::circle(image,cur_pt,2,line_color);
//     }
//     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//     publisher->publish(msg);
// }




class KLT {
    public:
    
    KLT(int _mode,int START_POSE);

    void KLT_Iteration(const cv::Mat& input_image,const pcl::PointCloud<PointType>::Ptr _cloud,ros::Time _raw_time);

    void store_coordinates(const Vector3d& t_input, const Matrix3d& R);

    void SVD(MatrixXd& cur_SVD,MatrixXd& prev_SVD);

    void publish_KLT(ros::Publisher* publisher, cv::Mat& cur_image,
        const vector<cv::Point2f>& cur_keypoints, const vector<cv::Point2f>& prev_keypoints,
        int circle_size);

    void visualizer_3D(const MatrixXd& cur_SVD, const MatrixXd& prev_SVD);

    void publish_tf();

    private:
    cv::Mat cur_image;
    cv::Mat prev_image;


    std::vector<cv::Point2f> cur_corners;
    std::vector<cv::Point2f> prev_corners;
    MatrixXd prev_3D_points;
    MatrixXd cur_3D_points;

    pcl::PointCloud<PointType>::Ptr prev_cloud;
    pcl::PointCloud<PointType>::Ptr cur_cloud;
    std::vector<cv::Scalar> color_vector;

    int mode;
    bool comparison;
    ros::Time raw_time;

    Matrix4d my_pose;

    ros::NodeHandle n_KLT;

    ros::Publisher pub_KLT_int,pub_KLT_ran,pub_KLT_amb,pub_KLT_tf,kp_pc_publisher_cur,kp_pc_publisher_prev,mid_point_line_publisher,odom_publisher;
};