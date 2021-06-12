#include "../include/BRISK.h"
#include "helper.cpp"


//Member functions:

BRISK::BRISK(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _mode){
        mode = _mode;
        assert(mode == 1 || mode == 2 || mode == 3);

        input_image = _input_image.clone();
        cv::resize(input_image, image, cv::Size(), 1, 1);
        cv::resize(input_image, input_image, cv::Size(), 1, 1);
        cloud = _cloud;

        if(mode == 1)
        KP_pub_intensity = n.advertise<sensor_msgs::Image>("brisk_keypoints_intensity", 1);
        else if(mode == 2)
        KP_pub_range = n.advertise<sensor_msgs::Image>("brisk_keypoints_range", 1);
        else
        KP_pub_ambient = n.advertise<sensor_msgs::Image>("brisk_keypoints_ambient", 1);

        create_descriptors();
}

void BRISK::create_descriptors(){
    static cv::Ptr<cv::BRISK> detector = cv::BRISK::create(BRISK_THRESHOLD,3,1.0f);
    detector->detect(image,brisk_keypoints,MASK);
    keypointTransition(brisk_keypoints,brisk_keypoints_2d);
    // duplicate_filtering();
    detector->compute(image,brisk_keypoints,brisk_descriptors);

    get_3D_data();


    if(mode == 1)
    publish_keypoints(&KP_pub_intensity, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0));
    else if(mode == 2)
    publish_keypoints(&KP_pub_range, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0));
    else
    publish_keypoints(&KP_pub_ambient, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0));
}

void BRISK::get_3D_data(){
    brisk_points_3d.resize(brisk_keypoints_2d.size());
    #pragma omp parallel for num_threads(NUM_THREADS)

    for(int i = 0; i < brisk_keypoints_2d.size(); i++){
        int row_index = (int)brisk_keypoints_2d[i].y;
        int col_index = (int)brisk_keypoints_2d[i].x;
        int index = row_index*IMAGE_WIDTH + col_index;
        PointType *pi = &cloud->points[index];

        cv::Point3d p_3d(0.f, 0.f, 0.f);
        cv::Point2d p_2d_n(0.f, 0.f);

            p_3d.x = pi->x;
            p_3d.y = pi->y;
            p_3d.z = pi->z;
            
        brisk_points_3d[i] = p_3d;
    }
}

void BRISK::duplicate_filtering(){
        std::vector<bool> duplicate_status;
        cv::Mat dubplicate_mask = cv::Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1,cv::Scalar(0));
        for(int i = 0; i < brisk_keypoints_2d.size();i++){
            cv::Point2d pt = brisk_keypoints_2d.at(i);
            cv::Scalar color = dubplicate_mask.at<uchar>(pt);
            if(color == cv::Scalar(0)){
                cv::circle(dubplicate_mask,pt,0,cv::Scalar(255),DUPLICATE_FILTERING_SIZE);
                duplicate_status.push_back(1);
            }   
            else{
                duplicate_status.push_back(0);
            }
        }
        trimVector(brisk_keypoints_2d,duplicate_status);
        trimVector(brisk_keypoints,duplicate_status);
    }



