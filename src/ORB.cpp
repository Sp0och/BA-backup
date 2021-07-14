#include "../include/ORB.h"
#include "helper.cpp"
//Member functions:

ORB::ORB(const cv::Mat &_input_image, const pcl::PointCloud<PointType>::Ptr _cloud,
        int _image_source)
        {
            M_image_source = _image_source;

            std::string config_file;
            M_n.getParam("parameter_file", config_file);
            cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
            if(!fsSettings.isOpened())
                std::cerr << "ERROR: Wrong path to settings" << std::endl;
            usleep(100);
            fsSettings["num_orb_features"] >> M_NUM_ORB_FEATURES;
            fsSettings["orb_accuracy"] >> M_ORB_ACCURACY;
            fsSettings["scale_factor"] >> M_SCALE_FACTOR;
            fsSettings["levels"] >> M_LEVELS;

            input_image = _input_image.clone();
            cv::resize(input_image, image, cv::Size(), 1, 1);
            //image for framehandler 
            cv::resize(input_image, input_image, cv::Size(), 1, 1);
            cloud = _cloud;

            M_dupl_publisher = M_n.advertise<sensor_msgs::Image>("duplicate_filtered", 1);
            M_pub_3D = M_n.advertise<sensor_msgs::Image>("min_distance_filtered", 1);

            if(M_image_source == 1)
            M_KP_pub_intensity = M_n.advertise<sensor_msgs::Image>("orb_keypoints_intensity", 1);
            else if(M_image_source == 2)
            M_KP_pub_range = M_n.advertise<sensor_msgs::Image>("orb_keypoints_range", 1);
            else
            M_KP_pub_ambient = M_n.advertise<sensor_msgs::Image>("orb_keypoints_ambient", 1);
            create_descriptors();
        }

void ORB::create_descriptors(){
        cv::Ptr<cv::ORB> detector = cv::ORB::create(M_NUM_ORB_FEATURES, M_SCALE_FACTOR, M_LEVELS, M_ORB_ACCURACY/* ,0,2,cv::ORB::HARRIS_SCORE,31,20 */);
        //store keypoints in orb_keypoints
        detector->detect(image,orb_keypoints,MASK);
        keypointTransition(orb_keypoints,orb_keypoints_2d);
        // cout << "orb size after extraction: " << orb_keypoints_2d.size() << " " << endl;
        get_3D_data();
        // cout << "orb size after min distance filtering: " << orb_keypoints_2d.size() << " " << endl;
        publish_keypoints(&M_pub_3D, image,orb_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);
        if(APPLY_DUPLICATE_FILTERING){
            duplicate_filtering();
            // cout << "orb size after duplicate filtering: " << orb_keypoints_2d.size() << " " << endl;
            publish_keypoints(&M_dupl_publisher, image,orb_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);
        }
        //Create descriptors
        detector->compute(image,orb_keypoints,orb_descriptors);
        
        // publishing the keypoints on whatever image type they are
        if(M_image_source == 1)
        publish_keypoints(&M_KP_pub_intensity, image,orb_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);
        else if(M_image_source == 2)
        publish_keypoints(&M_KP_pub_range, image,orb_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);
        else
        publish_keypoints(&M_KP_pub_ambient, image,orb_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);

    }
/**
 * Get 3D coordinates from the scanned points of the 2D matches and filter out too close points
 * */
void ORB::get_3D_data(){

        orb_points_3d.resize(orb_keypoints_2d.size());
        std::vector<bool> status(orb_keypoints_2d.size(),1);
        //get the 3D coordinates 
        for(size_t i = 0; i < orb_keypoints_2d.size(); i++){
            //The x and y coordinates of the keypoints correspond to u and v!
            int row_index = cvRound(orb_keypoints_2d[i].y);
            int col_index = cvRound(orb_keypoints_2d[i].x);
            int index = row_index*IMAGE_WIDTH + col_index;
            PointType *pi = &cloud->points[index];
            if(pi->x == pi->y && pi->y == pi->z && pi->z == 0){
                status.at(i) = 0;
            }
            cv::Point3d p_3d(0.0, 0.0, 0.0);
                p_3d.x = pi->x;
                p_3d.y = pi->y;
                p_3d.z = pi->z;
            orb_points_3d[i] = p_3d;
        }
        
        trimVector(orb_points_3d,status);
        trimVector(orb_keypoints_2d,status);
        trimVector(orb_keypoints,status);
    }

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
    trimVector(orb_points_3d,duplicate_status);
}

