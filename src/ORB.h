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
// void keypointTransition(vector<cv::Point2f>& points_in, vector<cv::KeyPoint>& keypoints_in)
// {
//     keypoints_in.resize(points_in.size());
//     for(size_t i = 0; i < points_in.size(); i++)
//     {
//         cv::KeyPoint key;
//         key.pt = points_in[i];
//         keypoints_in[i] = key;
//     }
// }


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

    vector<cv::Point2f> orb_keypoints_2d;
    vector<cv::Point3f> orb_point_3d;
    vector<cv::Point2f> orb_point_2d_norm;
    vector<cv::KeyPoint> orb_keypoints;
    cv::Mat orb_descriptors;

    // vector<cv::KeyPoint> reference_keypoints;
    // vector<cv::Point2f> reference_keypoints_2d;
    // vector<cv::Point2f> reference_keypoints_2d_norm;
    // cv::Mat reference_descriptors;

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

        //use reference points
        // cv::Ptr<cv::ORB> detector2 = cv::ORB::create(NUM_ORB_FEATURES*5, 1.2f, 8, 1);
        // detector2->detect(image,reference_keypoints,MASK);
        // keypointTransition(reference_keypoints,reference_keypoints_2d);
        // detector2->compute(image,reference_keypoints,reference_descriptors);

        points_for_ransac();
        // ref_points_for_ransac();

        if(mode == 1)
        publish_keypoints(&KP_pub_intensity, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
        else if(mode == 2)
        publish_keypoints(&KP_pub_range, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
        else
        publish_keypoints(&KP_pub_ambient, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
    };
    
    
    
    /**
     * This function creates keypoint vectors in camera coordinates both in 3D and depth normed 2D
     * */
    void points_for_ransac(){
        orb_point_3d.resize(orb_keypoints_2d.size());
        orb_point_2d_norm.resize(orb_keypoints_2d.size());
        vector<uchar> status;
        status.resize(orb_keypoints_2d.size());
        
        #pragma omp parallel for num_threads(NUM_THREADS)
        for(size_t i = 0; i < orb_keypoints_2d.size(); i++){
            int row_index = cvRound(orb_keypoints_2d[i].y);
            int col_index = cvRound(orb_keypoints_2d[i].x);
            int index = row_index*IMAGE_WIDTH + col_index;
            PointType *pi = &cloud->points[index];

            cv::Point3f p_3d(0.f, 0.f, 0.f);
            cv::Point2f p_2d_n(0.f, 0.f);
            
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
            //fill our 3d and normed 2d pointcloud vectors.
            orb_point_3d[i] = p_3d;
            orb_point_2d_norm[i] = p_2d_n;
        }
        
        trimVector(orb_point_3d,status);
        trimVector(orb_point_2d_norm,status);
    }
    
    /**
     * In the paper code they used reference points which are exactly the same but 5 times more 
     * */
    // void ref_points_for_ransac(){
    //     reference_keypoints_2d_norm.resize(reference_keypoints_2d.size());
    //     vector<uchar> status;
    //     status.resize(reference_keypoints_2d.size());
    //     #pragma omp parallel for num_threads(NUM_THREADS)

    //     for(int i = 0; i < reference_keypoints_2d.size(); i++){
    //         int row_index = cvRound(reference_keypoints_2d[i].y);
    //         int col_index = cvRound(reference_keypoints_2d[i].x);
    //         int index = row_index*IMAGE_WIDTH + col_index;
    //         PointType *pi = &cloud->points[index];

    //         cv::Point3f p_3d(0.f, 0.f, 0.f);
    //         cv::Point2f p_2d_n(0.f, 0.f);
    //         //why? Cut off left edge?
    //         if (abs(pi->x) < 0.01)
    //         {
    //             status[i] = 0;
    //         } 
    //         //create the points to fill in camera mode
    //         else 
    //         {
    //             status[i] = 1;
    //             // lidar -> camera
    //             p_3d.x = -pi->y;
    //             p_3d.y = -pi->z;
    //             p_3d.z = pi->x;
    //             // normalize to projection plane
    //             p_2d_n.x = p_3d.x / p_3d.z;
    //             p_2d_n.y = p_3d.y / p_3d.z;
    //         }
    //     }
        
    //     trimVector(reference_keypoints_2d_norm,status);
    // }


    
    private:
    ros::NodeHandle n;
    ros::Publisher KP_pub_intensity;
    ros::Publisher KP_pub_range;
    ros::Publisher KP_pub_ambient;
    int mode;



};



