
#pragma once
#include "parameters.h"
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

    std::shared_ptr<ORB> cur_orb, prev_orb;
    vector<cv::DMatch> matches; 
    int image_source;
    Matrix4d my_pose;
    int COUNT;
    int unfiltered_count;
    int ransac_filtered_count;
    int filtered_count;

    ros::Time raw_time;

    ros::NodeHandle n_frame;
    ros::Publisher match_publisher,range_publisher, ambient_publisher, intensity_publisher, 
    kp_pc_publisher_cur, kp_pc_publisher_prev,ransac_publisher,duplicate_publisher,
    odom_publisher, line_publisher,pc_distance_publisher_p,pc_distance_publisher_c,line_distance_publisher;
    
    cv::Scalar POINT_COLOR;
    cv::Scalar LINE_COLOR;

    int NUM_ORB_FEATURES;
    int ORB_ACCURACY;
    float SCALE_FACTOR;
    int LEVELS;
    std::string path;

};