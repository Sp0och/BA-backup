#include "ORB.h"



//copy keypoints_in to the initial point2f vector points_in
void keypointTransition(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2f>& points_in)
{
    points_in.resize(keypoints_in.size());
    for(size_t i = 0; i < keypoints_in.size(); i++)
    {
        points_in[i] = keypoints_in[i].pt;
    }
}
//Convert points to keypoints vector
void keypointTransition(vector<cv::Point2f>& points_in, vector<cv::KeyPoint>& keypoints_in)
{
    keypoints_in.resize(points_in.size());
    for(size_t i = 0; i < points_in.size(); i++)
    {
        cv::KeyPoint key;
        key.pt = points_in[i];
        keypoints_in[i] = key;
    }
}

/* void publish_keypoints (ros::Publisher* publisher, cv::Mat& image, const vector<cv::Point2f>& keypoints, const int circle_size,const cv::Scalar line_color){
    cv::cvtColor(image, image, CV_GRAY2RGB);
    for(int i = 0; i < (int)keypoints.size(); i++){
        cv::Point2f cur_pt = keypoints[i] * MATCH_IMAGE_SCALE;
        cv::circle(image,cur_pt,circle_size,line_color);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher->publish(msg);
} */

void publish_keypoints (ros::Publisher* publisher, cv::Mat& image, const vector<cv::Point2f>& keypoints, const int circle_size,const cv::Scalar line_color){
    cv::cvtColor(image, image, CV_GRAY2RGB);
    for(int i = 0; i < (int)keypoints.size(); i++){
        cv::Point2f cur_pt = keypoints[i] * MATCH_IMAGE_SCALE;
        cv::circle(image,cur_pt,circle_size,line_color);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher->publish(msg);
}



ORB::ORB(const cv::Mat &_image_intensity, 
         const pcl::PointCloud<PointType>::Ptr _cloud){

             KP_pub = n.advertise<sensor_msgs::Image>("loop_detector/detected_keypoints", 1);
             image_intensity = _image_intensity.clone();
             cloud = _cloud;
             cv::resize(image_intensity, image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
            //  ROS_INFO("we get so far");
             visualize_keypoints();
}




void ORB::visualize_keypoints(){
    cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES, 1.2f, 8, 1);
    //store keypoints in orb_keypoints
    detector->detect(image,orb_keypoints,MASK);
    keypointTransition(orb_keypoints,orb_point_2d_uv);
    publish_keypoints(&KP_pub,image,orb_point_2d_uv,5,cv::Scalar(0, 255, 0));
}














