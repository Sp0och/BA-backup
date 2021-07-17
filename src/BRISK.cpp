#include "package_BA_LF/BRISK.h"
#include "helper.cpp"

//Member functions:

BRISK::BRISK(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _image_source){

        M_image_source = _image_source;
        assert(M_image_source == 1 || M_image_source == 2 || M_image_source == 3);

        std::string config_file;
        M_n.getParam("parameter_file", config_file);
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        usleep(100);
        fsSettings["brisk_threshold"] >> M_BRISK_THRESHOLD;
        fsSettings["octaves"] >> M_OCTAVES;
        fsSettings["pattern_scale"] >> M_PATTERN_SCALE;

        input_image = _input_image.clone();
        cv::resize(input_image, image, cv::Size(), 1, 1);
        cv::resize(input_image, input_image, cv::Size(), 1, 1);
        cloud = _cloud;

        M_dupl_publisher = M_n.advertise<sensor_msgs::Image>("duplicate_filtered", 1);
        M_pub_3D = M_n.advertise<sensor_msgs::Image>("min_distance_filtered", 1);

        if(M_image_source == 1)
        M_KP_pub_intensity = M_n.advertise<sensor_msgs::Image>("brisk_keypoints_intensity", 1);
        else if(M_image_source == 2)
        M_KP_pub_range = M_n.advertise<sensor_msgs::Image>("brisk_keypoints_range", 1);
        else
        M_KP_pub_ambient = M_n.advertise<sensor_msgs::Image>("brisk_keypoints_ambient", 1);

        create_descriptors();
}

void BRISK::create_descriptors(){
    static cv::Ptr<cv::BRISK> detector = cv::BRISK::create(M_BRISK_THRESHOLD,M_OCTAVES,M_PATTERN_SCALE);
    
    detector->detect(image,brisk_keypoints,MASK);
    keypointTransition(brisk_keypoints,brisk_keypoints_2d);
    get_3D_data();

    publish_keypoints(&M_pub_3D, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);
    
    if(APPLY_DUPLICATE_FILTERING){
        duplicate_filtering();
        publish_keypoints(&M_dupl_publisher, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);
    }
    detector->compute(image,brisk_keypoints,brisk_descriptors);



    if(M_image_source == 1)
    publish_keypoints(&M_KP_pub_intensity, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);
    else if(M_image_source == 2)
    publish_keypoints(&M_KP_pub_range, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);
    else
    publish_keypoints(&M_KP_pub_ambient, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),M_image_source);

}

void BRISK::get_3D_data(){
    brisk_points_3d.resize(brisk_keypoints_2d.size());
    std::vector<bool> status(brisk_keypoints_2d.size(),1);
    for(int i = 0; i < brisk_keypoints_2d.size(); i++){
        int row_index = (int)brisk_keypoints_2d[i].y;
        int col_index = (int)brisk_keypoints_2d[i].x;
        int index = row_index*IMAGE_WIDTH + col_index;
        PointType *pi = &cloud->points[index];
        if(pi->x == pi->y && pi->y == pi->z && pi->z == 0)
            status.at(i) = 0;
        cv::Point3d p_3d(0.f, 0.f, 0.f);

            p_3d.x = pi->x;
            p_3d.y = pi->y;
            p_3d.z = pi->z;
            
        brisk_points_3d[i] = p_3d;
    }
    trimVector(brisk_points_3d,status);
    trimVector(brisk_keypoints_2d,status);
    trimVector(brisk_keypoints,status);
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
        trimVector(brisk_points_3d,duplicate_status);
    }



