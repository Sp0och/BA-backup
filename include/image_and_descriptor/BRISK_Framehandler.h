
#pragma once
#include "setup.h"
#include "helper.h"
#include "BRISK.h"

using namespace Eigen;

/**
 * Framehandler: creates matches between two consecutive frames, publishes these matches 
 * and uses them to indicate the points to use for SVD, also implements SVD
 */
class BRISK_Framehandler{

    public:
    //Constructor
    BRISK_Framehandler(int _image_source,int START_POSE);
    //Boundary condition for first iteration and then just calling matches_filtering_motion
    void newIteration(std::shared_ptr<BRISK> new_frame, ros::Time _raw_time);
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
    const std::vector<cv::Point2d>& sorted_KP_prev, cv::Scalar point_color, cv::Scalar line_color, int circle_size, bool draw_lines);
    //publish matches by indicating direction 
    void publish_matches_1F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, cv::Scalar point_color, cv::Scalar line_color, int circle_size, bool draw_lines);
    //apply closed form solution to predict transform
    void SVD(MatrixXd& cur_SVD,MatrixXd& prev_SVD);
    
    private:

    std::shared_ptr<BRISK> M_cur_brisk, M_prev_brisk;
    vector<cv::DMatch> M_matches; 
    int M_image_source;
    Matrix4d M_my_pose;
    std::string M_FILE_PATH;
    std::string M_DIRECTORY;

    ros::Time M_raw_time;

    ros::NodeHandle M_n_frame;
    ros::Publisher M_match_publisher, M_range_publisher, M_ambient_publisher, M_intensity_publisher, 
    M_kp_pc_publisher_cur, M_kp_pc_publisher_prev, M_midpoint_publisher, M_odom_publisher, M_line_publisher,M_ransac_publisher,M_duplicate_publisher;
    
    int M_BRISK_THRESHOLD;
    int M_OCTAVES;
    float M_PATTERN_SCALE;

    helper* Helper;

};