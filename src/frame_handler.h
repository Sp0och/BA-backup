#pragma once
#include "parameters.h"
#include "ORB.h"

template <typename Derived>

void trim_vector(vector<Derived> &v, vector<uchar>& status){
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

class Framehandler{

    public:

    Framehandler(int _mode){
        mode = _mode;
        cur_orb = nullptr;
        prev_orb = nullptr;
        match_publisher = n_frame.advertise<sensor_msgs::Image>("orb_matches", 1);
        filtered_publisher = n_frame.advertise<sensor_msgs::Image>("filtered_matches", 1);
    }

    void newIteration(std::shared_ptr<ORB> new_frame){
        if(cur_orb == nullptr){
            cur_orb = new_frame;
        }
        else{
            prev_orb = cur_orb;
            cur_orb = new_frame;
            create_matches();
        }
    }

    void create_matches(){
        static cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        // cv::BFMatcher matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        //create matches
        matcher->match(cur_orb->orb_descriptors,prev_orb->orb_descriptors,matches);
        //sort matches in ascending order (concerning distance)
        std::sort(matches.begin(),matches.end());
        //prefiltering for distance
        for (size_t i = 0; i < matches.size(); i++)
        {
            good_matches.push_back(matches[i]);
            // if(matches[i].distance > matches[0].distance * 3)
            // break;
        }
        matches.clear();
        std::vector<cv::Point2f> sorted_2d_cur, sorted_2d_prev, sorted_2d_norm_prev;
        std::vector<cv::Point3f> sorted_3d_cur;
        //pair the keypoints up and thus also the matches:
        for (size_t i = 0; i < good_matches.size(); i++)
        {
            int cur_index = good_matches[i].queryIdx;
            int prev_index = good_matches[i].trainIdx;

            sorted_3d_cur.push_back(cur_orb->orb_point_3d[cur_index]);
            sorted_2d_cur.push_back(cur_orb->orb_keypoints_2d[cur_index]);
            sorted_2d_prev.push_back(prev_orb->orb_keypoints_2d[prev_index]);
            sorted_2d_norm_prev.push_back(prev_orb->orb_point_2d_norm[prev_index]);
        }
        good_matches.clear();

        publish_matches(&match_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        
        //here comes the RANSAC part
        cv::Mat r, rvec, tvec, D, inliers;
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
        //get the filtered keypoint vectors via inliers
        ROS_INFO("length 3d: %i", sorted_3d_cur.size());
        ROS_INFO("length 2d: %i", sorted_2d_norm_prev.size());
        solvePnPRansac(sorted_3d_cur, sorted_2d_norm_prev, K, D, rvec, tvec, false, 100, 0.025, 0.99, inliers);
        //create the status vector for the flags
        std::vector<uchar> status;
        status.resize(sorted_2d_norm_prev.size(), 1);
        for( int i = 0; i < inliers.rows; i++)
        {
            int n = inliers.at<int>(i);
            status[n] = 1;
        }


        trim_vector(sorted_2d_cur,status);  
        trim_vector(sorted_2d_prev,status);

        publish_matches(&filtered_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
    }



    void publish_matches(const ros::Publisher* this_pub, std::vector<cv::Point2f>& sorted_KP_cur, std::vector<cv::Point2f>& sorted_KP_prev, int circle_size, cv::Scalar line_color, bool draw_lines){
    cv::Mat color_img,gray_img;
    const cv::Mat old_img = prev_orb->input_image;
    const cv::Mat new_img = cur_orb->input_image;
    int gap = 1;
    cv::Mat gap_img(gap, new_img.size().width, CV_8UC1, cv::Scalar(255, 255, 255));
    //create colored concatenated image with gap inbetween
    cv::vconcat(new_img,gap_img,gap_img);
    cv::vconcat(gap_img,old_img,gray_img);

    cv::cvtColor(gray_img,color_img,CV_GRAY2RGB);
    //indicate features in new image
    for(int i = 0; i< (int)sorted_KP_cur.size(); i++)
    {
        cv::Point2f cur_pt = sorted_KP_cur[i] * MATCH_IMAGE_SCALE;
        cv::circle(color_img, cur_pt, circle_size*MATCH_IMAGE_SCALE, line_color, MATCH_IMAGE_SCALE*2);
    }
    //indicate features in old image
    for(int i = 0; i< (int)sorted_KP_prev.size(); i++)
    {
        cv::Point2f old_pt = sorted_KP_prev[i] * MATCH_IMAGE_SCALE;
        old_pt.y += new_img.size().height + gap;
        cv::circle(color_img, old_pt, circle_size*MATCH_IMAGE_SCALE, line_color, MATCH_IMAGE_SCALE*2);
    }

    if(draw_lines){
        for (int i = 0; i< (int)sorted_KP_cur.size(); i++)
        {
            cv::Point2f old_pt = sorted_KP_prev[i] * MATCH_IMAGE_SCALE;
            old_pt.y += new_img.size().height + gap;
            cv::line(color_img, sorted_KP_cur[i] * MATCH_IMAGE_SCALE, old_pt, line_color, MATCH_IMAGE_SCALE*2, 8, 0);
        }
    }
    if(mode == 1)
    cv::putText(color_img, "Intensity",   cv::Point2f(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else if (mode == 2)
    cv::putText(color_img, "Range",   cv::Point2f(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else
    cv::putText(color_img, "Ambient",   cv::Point2f(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_img).toImageMsg();
    this_pub->publish(msg);

}



    private:

    std::shared_ptr<ORB> cur_orb;
    std::shared_ptr<ORB> prev_orb;
    vector<cv::DMatch> matches, good_matches; 
    int mode;



    ros::Publisher match_publisher;
    ros::Publisher range_publisher;
    ros::Publisher ambient_publisher;
    ros::Publisher filtered_publisher;
    ros::NodeHandle n_frame;
    


};