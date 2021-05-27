#pragma once

#include "parameters.h"

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
    
    KLT(int _mode);

    void KLT_Iteration(const cv::Mat& input_image);


    void publish_KLT(ros::Publisher* publisher, cv::Mat& cur_image,
        const vector<cv::Point2f>& cur_keypoints, const vector<cv::Point2f>& prev_keypoints,
        std::vector<uchar>& status, int circle_size);

    private:
    cv::Mat cur_image;
    cv::Mat prev_image;

    std::vector<cv::Point2f> cur_corners;
    std::vector<cv::Point2f> prev_corners;

    std::vector<cv::Scalar> color_vector;

    int mode;
    bool comparison;

    ros::NodeHandle n_KLT;

    ros::Publisher pub_KLT_int;
    ros::Publisher pub_KLT_ran;
    ros::Publisher pub_KLT_amb;
};