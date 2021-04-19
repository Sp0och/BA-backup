#pragma once

#include "parameters.h"

void keypointTransition(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2d>& points_in)
{
    points_in.resize(keypoints_in.size());
    for(size_t i = 0; i < keypoints_in.size(); i++)
    {
        points_in[i] = keypoints_in[i].pt;
    }
}


void publish_keypoints (ros::Publisher* publisher, cv::Mat& image, const vector<cv::Point2d>& keypoints, const int circle_size,const cv::Scalar line_color){
    cv::cvtColor(image, image, CV_GRAY2RGB);
    for(int i = 0; i < (int)keypoints.size(); i++){
        cv::Point2d cur_pt = keypoints[i] * MATCH_IMAGE_SCALE;
        cv::circle(image,cur_pt,circle_size,line_color);
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher->publish(msg);
}
template <typename Derived>
static void trimVector(vector<Derived> &v, vector<uchar>& status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


class ORB
{
    public:
    cv::Mat image;
    cv::Mat input_image;
    pcl::PointCloud<PointType>::Ptr cloud;

    vector<cv::Point2d> orb_keypoints_2d;
    vector<cv::Point3d> orb_point_3d;
    vector<cv::Point2d> orb_point_projected;
    vector<cv::KeyPoint> orb_keypoints;
    cv::Mat orb_descriptors;

    ORB(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _mode)
        {
            mode = _mode;

            input_image = _input_image.clone();
            cv::resize(input_image, image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
            cv::resize(input_image, input_image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
            cloud = _cloud;

            if(mode == 1)
            KP_pub_intensity = n.advertise<sensor_msgs::Image>("orb_keypoints_intensity", 1);
            else if(mode == 2)
            KP_pub_range = n.advertise<sensor_msgs::Image>("orb_keypoints_range", 1);
            else
            KP_pub_ambient = n.advertise<sensor_msgs::Image>("orb_keypoints_ambient", 1);

            create_descriptors();
        }


/**
 * Stores the ORB keypoints in the vector in the std::vector format, creates the descriptors around the keypoints and calls the ransac point creator
 * @param orb_keypoints_2d is then the vector containing the keypoints
 * */
    void create_descriptors(){
        cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES, 1.2f, 8, 1);
        //store keypoints in orb_keypoints
        detector->detect(image,orb_keypoints,MASK);
        keypointTransition(orb_keypoints,orb_keypoints_2d);
        detector->compute(image,orb_keypoints,orb_descriptors);


        if(mode == 1)
        publish_keypoints(&KP_pub_intensity, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
        else if(mode == 2)
        publish_keypoints(&KP_pub_range, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
        else
        publish_keypoints(&KP_pub_ambient, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
    };
    
    
    


    
    private:
    ros::NodeHandle n;
    ros::Publisher KP_pub_intensity;
    ros::Publisher KP_pub_range;
    ros::Publisher KP_pub_ambient;
    int mode;



};



