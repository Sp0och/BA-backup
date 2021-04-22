#pragma once

#include "parameters.h"

void publish_tracked_points (ros::Publisher* publisher, cv::Mat& image, const vector<cv::Point2d>& keypoints, int circle_size){
    cv::cvtColor(image, image, CV_GRAY2RGB);

    for(int i = 0; i < (int)keypoints.size(); i++){
        // int r = rand()%256;
        // int g = rand()%256;
        // int b = rand()%256;
        // cv::Scalar line_color = cv::Scalar(r,g,b);
        cv::Point2d cur_pt = keypoints[i] * MATCH_IMAGE_SCALE;
        cv::circle(image,cur_pt,circle_size,cv::Scalar(0,255,0));
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher->publish(msg);
}

class KLT {
    public:
    
    KLT(cv::Mat& input_image){
        if(cur_image.rows == 0)
        cur_image = input_image.clone();
        else{
            // prev_image = cur_image;
            cur_image = input_image.clone();
        }
        pub_KLT = n_KLT.advertise<sensor_msgs::Image>("tracked_points", 1);
        create_tracker_points();
    }

    void create_tracker_points(){
    // cvtColor(cur_image,cur_image,cv::COLOR_BGR2GRAY);
        if(cur_corners.size()!=IMAGE_WIDTH*IMAGE_HEIGHT){
            cur_corners.resize(IMAGE_HEIGHT*IMAGE_WIDTH);
            prev_corners.resize(IMAGE_HEIGHT*IMAGE_WIDTH);
        }
        
        // prev_corners = cur_corners;
        cv::goodFeaturesToTrack(cur_image,cur_corners,5,0.3,7,MASK,7,true,0.04);
        publish_tracked_points(&pub_KLT,cur_image,cur_corners,5);   
    }

    private:
    cv::Mat cur_image;
    cv::Mat prev_image;

    cv::Mat mask/* (IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1) */;

    std::vector<cv::Point2d> cur_corners;
    std::vector<cv::Point2d> prev_corners;

    ros::NodeHandle n_KLT;

    ros::Publisher pub_KLT;
};