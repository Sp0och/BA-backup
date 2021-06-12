#include "../include/ORB.h"
#include "helper.cpp"

//Member functions:

ORB::ORB(const cv::Mat &_input_image, const pcl::PointCloud<PointType>::Ptr _cloud,
        int _mode)
        {
            mode = _mode;

            input_image = _input_image.clone();
            cv::resize(input_image, image, cv::Size(), 1, 1);
            //input_image used for framehandler to have a non-altered image to draw the matches on
            cv::resize(input_image, input_image, cv::Size(), 1, 1);
            cloud = _cloud;

            if(mode == 1)
            KP_pub_intensity = n.advertise<sensor_msgs::Image>("orb_keypoints_intensity", 1);
            else if(mode == 2)
            KP_pub_range = n.advertise<sensor_msgs::Image>("orb_keypoints_range", 1);
            else
            KP_pub_ambient = n.advertise<sensor_msgs::Image>("orb_keypoints_ambient", 1);
            create_descriptors();
        }
void ORB::create_descriptors(){
        cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES, 1.2f, 8, ORB_ACCURACY/* ,0,2,cv::ORB::HARRIS_SCORE,31,20 */);
        //store keypoints in orb_keypoints
        detector->detect(image,orb_keypoints,MASK);
        keypointTransition(orb_keypoints,orb_keypoints_2d);
        // cout << "orb size before: " << orb_keypoints_2d.size() << " " << endl;
        // duplicate_filtering();
        // cout << "orb size after construction: " << orb_keypoints_2d.size() << " " << endl;
        //Create descriptors
        detector->compute(image,orb_keypoints,orb_descriptors);
        get_3D_data();

        // publishing the keypoints on whatever image type they are
        if(mode == 1)
        publish_keypoints(&KP_pub_intensity, image,orb_keypoints_2d,1,cv::Scalar(0,255,0));
        else if(mode == 2)
        publish_keypoints(&KP_pub_range, image,orb_keypoints_2d,1,cv::Scalar(0,255,0));
        else
        publish_keypoints(&KP_pub_ambient, image,orb_keypoints_2d,1,cv::Scalar(0,255,0));
    }
void ORB::get_3D_data(){
        orb_points_3d.resize(orb_keypoints_2d.size());
        // orb_point_projected.resize(orb_keypoints_2d.size());
        
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

//member filtering functions:

void ORB::duplicate_filtering(){
    std::vector<bool> duplicate_status;
    cv::Mat dubplicate_mask = cv::Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1,cv::Scalar(0));
    for(int i = 0; i < orb_keypoints_2d.size();i++){
        cv::Point2d pt = orb_keypoints_2d.at(i);
        cv::Scalar color = dubplicate_mask.at<uchar>(pt);
        if(color == cv::Scalar(0)){
            cv::circle(dubplicate_mask,pt,0,cv::Scalar(255),DUPLICATE_FILTERING_SIZE);
            duplicate_status.push_back(1);
        }   
        else{
            duplicate_status.push_back(0);
        }
    }
    trimVector(orb_keypoints_2d,duplicate_status);
    trimVector(orb_keypoints,duplicate_status);
}

