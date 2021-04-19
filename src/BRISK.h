#pragma once
#include "parameters.h"



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


void publish_keypoints (ros::Publisher* publisher, cv::Mat& image, const vector<cv::Point2f>& keypoints, const int circle_size,const cv::Scalar line_color){
    cv::cvtColor(image, image, CV_GRAY2RGB);
    for(int i = 0; i < (int)keypoints.size(); i++){
        cv::Point2f cur_pt = keypoints[i] * MATCH_IMAGE_SCALE;
        cv::circle(image,cur_pt,circle_size,line_color);
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher->publish(msg);
}
template <typename Derived>
static void trimVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


class BRISK
{
    public:
    cv::Mat image;
    cv::Mat input_image;
    pcl::PointCloud<PointType>::Ptr cloud;

    vector<cv::Point2f> brisk_keypoints_2d;
    vector<cv::Point3f> brisk_point_3d;
    vector<cv::Point2f> brisk_point_2d_norm;
    vector<cv::KeyPoint> brisk_keypoints;
    cv::Mat brisk_descriptors;

    BRISK(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _mode){
        mode = _mode;
        assert(mode == 1 || mode == 2 || mode == 3);

        input_image = _input_image.clone();
        cv::resize(input_image, image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
        cv::resize(input_image, input_image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
        cloud = _cloud;

        if(mode == 1)
        KP_pub_intensity = n.advertise<sensor_msgs::Image>("brisk_keypoints_intensity", 1);
        else if(mode == 2)
        KP_pub_range = n.advertise<sensor_msgs::Image>("brisk_keypoints_range", 1);
        else
        KP_pub_ambient = n.advertise<sensor_msgs::Image>("brisk_keypoints_ambient", 1);

        create_descriptors();
             }


/**
 * Stores the BRISK keypoints in the vector in the std::vector format, creates the descriptors around the keypoints and calls the ransac point creator
 * @param brisk_keypoints_2d is then the vector containing the keypoints
 * */
    void create_descriptors(){
        
        //put keypoints into vector and create descriptors.
        points_for_ransac();

        if(mode == 1)
        publish_keypoints(&KP_pub_intensity, image,brisk_keypoints_2d,5,cv::Scalar(0,255,0));
        else if(mode == 2)
        publish_keypoints(&KP_pub_range, image,brisk_keypoints_2d,5,cv::Scalar(0,255,0));
        else
        publish_keypoints(&KP_pub_ambient, image,brisk_keypoints_2d,5,cv::Scalar(0,255,0));
    };
    
    
    
    
    void points_for_ransac(){
        brisk_point_3d.resize(brisk_keypoints_2d.size());
        brisk_point_2d_norm.resize(brisk_keypoints_2d.size());
        vector<uchar> status;
        status.resize(brisk_keypoints_2d.size());
        #pragma omp parallel for num_threads(NUM_THREADS)

        for(int i = 0; i < brisk_keypoints_2d.size(); i++){
            int row_index = (int)brisk_keypoints_2d[i].y;
            int col_index = (int)brisk_keypoints_2d[i].x;
            int index = row_index*IMAGE_WIDTH + col_index;
            PointType *pi = &cloud->points[index];

            cv::Point3f p_3d(0.f, 0.f, 0.f);
            cv::Point2f p_2d_n(0.f, 0.f);
            //why? Cut off left edge?
            if (abs(pi->x) < 0.01)
            {
                status[i] = 0;
            } 
            //create the points to fill in camera mode
            else 
            {
                status[i] = 1;
                // lidar -> camera
                p_3d.x = -pi->y;
                p_3d.y = -pi->z;
                p_3d.z = pi->x;
                // normalize to projection plane
                p_2d_n.x = p_3d.x / p_3d.z;
                p_2d_n.y = p_3d.y / p_3d.z;
            }
        }
        
        trimVector(brisk_point_3d,status);
        trimVector(brisk_point_2d_norm,status);
    }

    
    private:
    ros::NodeHandle n;
    ros::Publisher KP_pub_intensity;
    ros::Publisher KP_pub_range;
    ros::Publisher KP_pub_ambient;
    int mode;



};



