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
    
    KLT(int _mode){
        mode = _mode;
        comparison = true;
        cur_corners.resize(300);
        prev_corners.resize(300);
        
        for(int i = 0; i < 100; i++){
            int r = rand()%256;
            int g = rand()%256;
            int b = rand()%256;
            cv::Scalar color = cv::Scalar(r,g,b);
            color_vector.push_back(color);
        }

        if(mode == 1)
        pub_KLT_int = n_KLT.advertise<sensor_msgs::Image>("tracked_points_int", 1);
        else if(mode == 2)
        pub_KLT_ran = n_KLT.advertise<sensor_msgs::Image>("tracked_points_ran", 1);
        else
        pub_KLT_amb = n_KLT.advertise<sensor_msgs::Image>("tracked_points_amb", 1);
    }

    void KLT_Iteration(const cv::Mat& input_image){
        if(prev_image.rows == 0 || prev_corners.size()==0){
            prev_image = input_image.clone();
            //create the previous features
            cv::goodFeaturesToTrack(prev_image,prev_corners,300,0.2,7,MASK,7,true,0.04);
        }
        
        else {
            //Update Keypoints and Optical Flow Iteration
            cur_image = input_image.clone();
            std::vector<uchar> status;
            std::vector<float> err;
            //Create Termination criteria for the KLT method
            cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
            //create the new features from the old ones
            assert(prev_corners.size()>0);
            int before = prev_corners.size();
            cv::calcOpticalFlowPyrLK(prev_image,cur_image,prev_corners,cur_corners,status,err,
            cv::Size(15,15),2,criteria);
            //Publish and adapt prev_corners
            if(mode == 1)
            publish_KLT(&pub_KLT_int,cur_image,cur_corners,prev_corners,status,5);
            else if(mode == 2)
            publish_KLT(&pub_KLT_ran,cur_image,cur_corners,prev_corners,status,5);
            else
            publish_KLT(&pub_KLT_amb,cur_image,cur_corners,prev_corners,status,5);
            //after number from the prev_corners after the change into the filtered cur_corners
            int after = prev_corners.size();
            if(comparison){
                std::cout << "true positive rate this iteration: " << (100*after)/before << " " << std::endl;
                comparison = 0;
            }
            
            prev_image = cur_image.clone();
            if(prev_corners.size()<20){
                // std::cout << "GET NEW CORNERS" << std::endl;
                cv::goodFeaturesToTrack(prev_image,prev_corners,300,0.2,7,MASK,7,true,0.04);
                comparison = 1;
            }
            else{
                // std::cout << "All Goodie" << std::endl;
            }
        }
    }


    void publish_KLT(ros::Publisher* publisher, cv::Mat& cur_image,
        const vector<cv::Point2f>& cur_keypoints, const vector<cv::Point2f>& prev_keypoints,
        std::vector<uchar>& status, int circle_size){
        cv::Mat image = cur_image;
        cv::cvtColor(image,image,CV_GRAY2RGB);
        std::vector<cv::Point2f> good_new;
        for(int i = 0; i < cur_corners.size();i++){
            if(status.at(i) == 1 && cur_corners[i].x > IMAGE_CROP && cur_corners[i].x < IMAGE_WIDTH-IMAGE_CROP/2){
                good_new.push_back(cur_corners[i]);
                cv::Scalar line_color = color_vector.at(i);
                cv::circle(image,cur_corners[i],circle_size,line_color,1,8,0);
                cv::line(image,cur_corners[i],prev_corners[i],line_color,1,8,0);
            }
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
        "bgr8", image).toImageMsg();
        publisher->publish(msg);
        prev_corners.clear();
        prev_corners = good_new;
    }

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