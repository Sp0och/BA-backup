
#pragma once
#include "parameters.h"
#include "ORB.h"

using namespace Eigen;

/**
 * Framehandler: creates matches between two consecutive frames, publishes these matches 
 * and uses them to indicate the points to use for SVD, also implements SVD
 */
class Framehandler{

    public:
    //Constructor
    Framehandler(int _mode,int START_POSE);
    //Boundary condition for first iteration and then just calling matches_filtering_motion
    void newIteration(std::shared_ptr<ORB> new_frame, ros::Time _raw_time);
    /**
     * Core of this class: creates matches, performs filtering, calls SVD function, publishes matches + transform
     * */
    void matches_filtering_motion();

    //set the columns names for the plot data as well as the start pose for the overall plot
    void set_plotting_columns_and_start_pose();
    //store the data for plotting
    void store_coordinates(const Vector3d& t, const Matrix3d& R);
    /**
     * Store each iterations feature numbers and the timestamp
     * */
    void store_feature_number(const MatrixXd& cur_SVD);


    //publish prediction transformation
    void publish_tf();
    //publish odometry message from transformation
    void publish_odom();


    /**
     * Calls all necessary functions to visualize keypoints and matches in 3D
     * */
    void visualizer_3D(const MatrixXd& cur_SVD, const MatrixXd& prev_SVD);
    //publish the 2D matches
    void publish_matches_2F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, cv::Scalar line_color, bool draw_lines);
    //publish matches by indicating direction 
    void publish_matches_1F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, bool draw_lines);
    //apply closed form solution to predict transform
    void SVD(MatrixXd& cur_SVD,MatrixXd& prev_SVD);
    
    private:

    std::shared_ptr<ORB> cur_orb, prev_orb;
    vector<cv::DMatch> matches; 
    int mode;
    Matrix4d my_pose;

    ros::Time raw_time;

    ros::NodeHandle n_frame;
    ros::Publisher match_publisher, range_publisher, ambient_publisher, intensity_publisher, 
    kp_pc_publisher_cur, kp_pc_publisher_prev, midpoint_publisher, odom_publisher, line_publisher;
    
};