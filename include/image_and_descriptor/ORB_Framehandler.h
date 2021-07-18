
#pragma once
#include "setup.h"
#include "helper.h"
#include "ORB.h"

using namespace Eigen;

/**
 * Framehandler: creates matches between two consecutive frames, publishes these matches 
 * and uses them to indicate the points to use for SVD, also implements SVD
 */
class ORB_Framehandler{

    public:
    //Constructor
    ORB_Framehandler(int _image_source,int START_POSE);
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
    void visualizer_3D(const MatrixXd& cur_SVD, const MatrixXd& prev_SVD,ros::Publisher* cur,ros::Publisher* prev,ros::Publisher* line);
    //publish the 2D matches
    void publish_matches_2F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, cv::Scalar point_color, cv::Scalar line_color, bool draw_lines);
    //publish matches by indicating direction 
    void publish_matches_1F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, cv::Scalar point_color, cv::Scalar line_color, bool draw_lines);
    //apply closed form solution to predict transform
    void SVD(MatrixXd& cur_SVD,MatrixXd& prev_SVD);
    
    private:

    std::shared_ptr<ORB> M_cur_orb, M_prev_orb;
    vector<cv::DMatch> M_matches; 
    int M_image_source;
    Matrix4d M_my_pose;
    int M_COUNT;
    int M_unfiltered_count;
    int M_ransac_filtered_count;
    int M_filtered_count;

    ros::Time M_raw_time;

    ros::NodeHandle M_n_frame;
    ros::Publisher M_match_publisher,M_range_publisher, M_ambient_publisher, M_intensity_publisher, 
    M_kp_pc_publisher_cur, M_kp_pc_publisher_prev,M_ransac_publisher,
    M_odom_publisher, M_line_publisher,M_pc_distance_publisher_p,M_pc_distance_publisher_c,M_line_distance_publisher;
    
    int M_NUM_ORB_FEATURES;
    int M_ORB_ACCURACY;
    float M_SCALE_FACTOR;
    int M_LEVELS;

    cv::Scalar M_POINT_COLOR;
    cv::Scalar M_LINE_COLOR;

    std::string M_FILE_PATH;
    std::string M_DIRECTORY;
    
    helper* Helper;

};