// #pragma once

// #include "parameters.h"

// void keypointTransition(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2d>& points_in)
// {
//     points_in.resize(keypoints_in.size());
//     for(size_t i = 0; i < keypoints_in.size(); i++)
//     {
//         points_in[i] = keypoints_in[i].pt;
//     }
// }

// void publish_keypoints (ros::Publisher* publisher, cv::Mat& image, const vector<cv::Point2d>& keypoints, const int circle_size,const cv::Scalar line_color){
//     cv::cvtColor(image, image, CV_GRAY2RGB);
//     for(int i = 0; i < (int)keypoints.size(); i++){
//         cv::Point2d cur_pt = keypoints[i] * MATCH_IMAGE_SCALE;
//         cv::circle(image,cur_pt,circle_size,line_color);
//     }
//     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//     publisher->publish(msg);
// }
// template <typename Derived>
// static void trimVector(vector<Derived> &v, vector<bool>& status)
// {
//     int j = 0;
//     for (int i = 0; i < int(v.size()); i++)
//         if (status[i])
//             v[j++] = v[i];
//     v.resize(j);
// }


// class ORB
// {
//     public:
//     cv::Mat image;
//     cv::Mat input_image;
//     pcl::PointCloud<PointType>::Ptr cloud;

//     vector<cv::Point2d> orb_keypoints_2d;
//     vector<cv::Point3d> orb_point_3d;
//     vector<cv::Point2d> orb_point_projected;
//     vector<cv::KeyPoint> orb_keypoints;
//     cv::Mat orb_descriptors;


//     ORB(const cv::Mat &_input_image, 
//         const pcl::PointCloud<PointType>::Ptr _cloud,
//         int _mode)
//         {
//             mode = _mode;

//             input_image = _input_image.clone();
//             cv::resize(input_image, image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
//             cv::resize(input_image, input_image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
//             cloud = _cloud;

//             if(mode == 1)
//             KP_pub_intensity = n.advertise<sensor_msgs::Image>("orb_keypoints_intensity", 1);
//             else if(mode == 2)
//             KP_pub_range = n.advertise<sensor_msgs::Image>("orb_keypoints_range", 1);
//             else
//             KP_pub_ambient = n.advertise<sensor_msgs::Image>("orb_keypoints_ambient", 1);

//             create_descriptors();
//         }


// /**
//  * Stores the ORB keypoints in the vector in the std::vector format, creates the descriptors around the keypoints and calls the ransac point creator
//  * @param orb_keypoints_2d is then the vector containing the keypoints
//  * */
//     void create_descriptors(){
//         cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES, 1.2f, 8, 1);
//         //store keypoints in orb_keypoints
//         detector->detect(image,orb_keypoints,MASK);
//         keypointTransition(orb_keypoints,orb_keypoints_2d);
//         detector->compute(image,orb_keypoints,orb_descriptors);


//         // points_for_ransac();

//         // if(mode == 1)
//         // publish_keypoints(&KP_pub_intensity, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
//         publish_keypoints(&KP_pub_intensity, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
//         // else if(mode == 2)
//         // publish_keypoints(&KP_pub_range, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
//         // else
//         // publish_keypoints(&KP_pub_ambient, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
//     };
    
    
    
//     /**
//      * This function creates keypoint vectors in camera coordinates both in 3D and depth normed 2D
//      * */
//     void points_for_ransac(){
//         orb_point_3d.resize(orb_keypoints_2d.size());
//         orb_point_projected.resize(orb_keypoints_2d.size());
//         vector<bool> status;
//         status.resize(orb_keypoints_2d.size());
        
//         #pragma omp parallel for num_threads(NUM_THREADS)
//         //get the 3D coordinates of the keypoint in the intensity image
//         for(size_t i = 0; i < orb_keypoints_2d.size(); i++){
//             //The x and y coordinates of the keypoints correspond to u and v!
//             int row_index = cvRound(orb_keypoints_2d[i].y);
//             int col_index = cvRound(orb_keypoints_2d[i].x);
//             int index = row_index*IMAGE_WIDTH + col_index;
//             PointType *pi = &cloud->points[index];

//             cv::Point3d p_3d(0.0, 0.0, 0.0);
//             cv::Point2d p_2d_n(0.0, 0.0);
            
//             // if (abs(pi->x) < 0.01)
//             // {
//             //     status[i] = 0;
//             // } 
//             //consider half
//             if (pi->x < 0.01 || pi->y < 0.01 || IMAGE_WIDTH - pi->x < 0.1 || IMAGE_HEIGHT - pi->y < 0.1)
//             {
//                 status[i] = 0;
//             } 
//             //the following points are the actual coordinates of the points in space
//             else 
//             {
//                 status[i] = 1;
//                 // lidar -> camera
//                 p_3d.x = -pi->y;
//                 p_3d.y = -pi->z;
//                 p_3d.z = pi->x;
//                 // normalize to projection plane with focal length 1
//                 p_2d_n.x = p_3d.x / p_3d.z;
//                 p_2d_n.y = p_3d.y / p_3d.z;
//             }
//             //fill our 3d and normed 2d pointcloud vectors.
//             orb_point_3d[i] = p_3d;
//             orb_point_projected[i] = p_2d_n;
//         }
        
//         trimVector(orb_point_3d,status);
//         trimVector(orb_point_projected,status);
//     }
    


    
//     private:
//     ros::NodeHandle n;
//     ros::Publisher KP_pub_intensity;
//     ros::Publisher KP_pub_range;
//     ros::Publisher KP_pub_ambient;
//     int mode;



// };


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
static void trimVector(vector<Derived> &v, vector<bool>& status)
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
    vector<cv::Point3d> orb_points_3d;
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


        points_for_ransac();

        if(mode == 1)
        publish_keypoints(&KP_pub_intensity, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
        else if(mode == 2)
        publish_keypoints(&KP_pub_range, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
        else
        publish_keypoints(&KP_pub_ambient, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
    };
    
    
    
    /**
     * Changed up version for 3D points for ICP
     * */
    void points_for_ransac(){
        orb_points_3d.resize(orb_keypoints_2d.size());
        // orb_point_projected.resize(orb_keypoints_2d.size());
        vector<bool> status;
        status.resize(orb_keypoints_2d.size());
        
        #pragma omp parallel for num_threads(NUM_THREADS)
        //get the 3D coordinates of the keypoint in the intensity image
        for(size_t i = 0; i < orb_keypoints_2d.size(); i++){
            //The x and y coordinates of the keypoints correspond to u and v!
            int row_index = cvRound(orb_keypoints_2d[i].y);
            int col_index = cvRound(orb_keypoints_2d[i].x);
            int index = row_index*IMAGE_WIDTH + col_index;
            PointType *pi = &cloud->points[index];

            cv::Point3d p_3d(0.0, 0.0, 0.0);
            // cv::Point2d p_2d_n(0.0, 0.0);
            
            // if (abs(pi->x) < 0.01)
            // {
            //     status[i] = 0;
            // } 
            //consider half
            // if (pi->x < 0.01 || pi->y < 0.01 || IMAGE_WIDTH - pi->x < 0.1 || IMAGE_HEIGHT - pi->y < 0.1)
            // {
            //     status[i] = 0;
            // } 
            //the following points are the actual coordinates of the points in space
            // else 
            // {
            //     status[i] = 1;
                // lidar -> camera
                p_3d.x = pi->x;
                p_3d.y = pi->y;
                p_3d.z = pi->z;
                // normalize to projection plane with focal length 1
                // p_2d_n.x = p_3d.x / p_3d.z;
                // p_2d_n.y = p_3d.y / p_3d.z;
            // }
            //fill our 3d and normed 2d pointcloud vectors.
            orb_points_3d[i] = p_3d;
            // orb_point_projected[i] = p_2d_n;
        }
        
        // trimVector(orb_point_3d,status);
        // trimVector(orb_point_projected,status);
    }
    


    
    private:
    ros::NodeHandle n;
    ros::Publisher KP_pub_intensity;
    ros::Publisher KP_pub_range;
    ros::Publisher KP_pub_ambient;
    int mode;



};
