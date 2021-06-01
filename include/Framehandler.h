
#pragma once
#include "parameters.h"
#include "ORB.h"

using namespace Eigen;

/**
 * Framehandler: creates matches between two consecutive frames, publishes these matches 
 * and uses them to indicate the points to use for ICP, also implements ICP
 */
class Framehandler{

    public:
    //Constructor
    Framehandler(int _mode);

    void newIteration(std::shared_ptr<ORB> new_frame, ros::Time _raw_time);

    void matches_filtering_motion();


    void set_plotting_columns_and_start_pose();

    void store_coordinates(const Vector3d& t, const Matrix3d& R);

    void publish_tf();

    void publish_odom();


    /**
     * Calls all necessary functions to visualize keypoints and matches in 3D
     * */
    void visualizer_3D(const MatrixXd& cur_ICP, const MatrixXd& prev_ICP);

    void publish_matches_2F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, cv::Scalar line_color, bool draw_lines);

    void publish_matches_1F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, bool draw_lines);

    void SVD(MatrixXd& cur_ICP,MatrixXd& prev_ICP);
    
    private:

    std::shared_ptr<ORB> cur_orb, prev_orb;
    vector<cv::DMatch> matches; 
    int mode;
    bool first_iteration;

    Matrix4d my_pose;

    ros::Time raw_time;

    ros::NodeHandle n_frame;
    ros::Subscriber raw_sub;
    ros::Publisher match_publisher, range_publisher, ambient_publisher, intensity_publisher, 
    kp_pc_publisher_cur, kp_pc_publisher_prev, midpoint_publisher, odom_publisher, line_publisher;
    
};