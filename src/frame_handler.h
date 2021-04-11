#pragma once
#include "parameters.h"
#include "ORB.h"


class Framehandler{

    public:

    Framehandler(){
        cur_orb = nullptr;
        prev_orb = nullptr;
        match_publisher = n_frame.advertise<sensor_msgs::Image>("orb_matches", 1);
    }

    void newIteration(ORB* new_frame){
        if(cur_orb == nullptr){
            cur_orb = new_frame;
        }
        else{
            prev_orb = cur_orb;
            cur_orb = new_frame;
            create_matches();
            // publish_matches(&match_publisher,true,5,cv::Scalar(0,255,0),cur_orb->image,prev_orb->image,cur_orb->orb_descriptors,prev_orb->orb_descriptors);
            publish_matches(5,cv::Scalar(0,255,0),true);
        }
    }

    void create_matches(){
        //create matches
        matcher.match(cur_orb->orb_descriptors,prev_orb->orb_descriptors,matches);
        //sort matches in ascending order (concerning distance)
        std::sort(matches.begin(),matches.end());
        //only keep the good matches
        for (size_t i = 0; i < matches.size(); i++)
        {
            good_matches.push_back(matches[i]);
            if(matches[i].distance > matches[0].distance * 2)
            break;
        }
        
        //here comes the RANSAC part
    }
    void publish_matches(int circle_size, cv::Scalar line_color, bool draw_lines){
    int gap = 1;
    cv::Mat color_img,gray_img;
    const cv::Mat old_img = prev_orb->input_image;
    const cv::Mat new_img = cur_orb->input_image;
    cv::Mat gap_img(gap, new_img.size().width, CV_8UC1, cv::Scalar(255, 255, 255));
    //create colored concatenated image with gap inbetween
    cv::vconcat(new_img,gap_img,gap_img);
    cv::vconcat(gap_img,old_img,gray_img);

    cv::cvtColor(gray_img,color_img,CV_GRAY2RGB);
    //indicate features in new image
    for(int i = 0; i< (int)cur_orb->orb_keypoints_2d.size(); i++)
    {
        cv::Point2f cur_pt = cur_orb->orb_keypoints_2d[i] * MATCH_IMAGE_SCALE;
        cv::circle(color_img, cur_pt, circle_size*MATCH_IMAGE_SCALE, line_color, MATCH_IMAGE_SCALE*2);
    }
    //indicate features in old image
    for(int i = 0; i< (int)prev_orb->orb_keypoints_2d.size(); i++)
    {
        cv::Point2f old_pt = prev_orb->orb_keypoints_2d[i] * MATCH_IMAGE_SCALE;
        old_pt.y += new_img.size().height + gap;
        cv::circle(color_img, old_pt, circle_size*MATCH_IMAGE_SCALE, line_color, MATCH_IMAGE_SCALE*2);
    }

    if(draw_lines){
        for (int i = 0; i< (int)cur_orb->orb_keypoints_2d.size(); i++)
        {
            cv::Point2f old_pt = prev_orb->orb_keypoints_2d[i] * MATCH_IMAGE_SCALE;
            old_pt.y += new_img.size().height + gap;
            cv::line(color_img, cur_orb->orb_keypoints_2d[i] * MATCH_IMAGE_SCALE, old_pt, line_color, MATCH_IMAGE_SCALE*2, 8, 0);
        }
    }


    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_img).toImageMsg();
    match_publisher.publish(msg);

}


    private:

    ORB* cur_orb;
    ORB* prev_orb;
    ros::Publisher match_publisher;
    ros::NodeHandle n_frame;
    vector<cv::DMatch> matches, good_matches; 
    cv::BFMatcher matcher = cv::BFMatcher(cv::NORM_HAMMING);


};