#include "../include/BRISK.h"
#include "helper.cpp"


//Member functions:

BRISK::BRISK(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _image_source,int& ec,int& dfc,int& mdfc,int& count){
        image_source = _image_source;
        assert(image_source == 1 || image_source == 2 || image_source == 3);
        COUNT = count;
        extracted_count = ec;
        duplicate_filtered_count = dfc;
        min_distance_filtered_count = mdfc;;

        std::string config_file;
        n.getParam("parameter_file", config_file);
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        usleep(100);
        fsSettings["brisk_threshold"] >> BRISK_THRESHOLD;
        fsSettings["octaves"] >> OCTAVES;
        fsSettings["pattern_scale"] >> PATTERN_SCALE;

        input_image = _input_image.clone();
        cv::resize(input_image, image, cv::Size(), 1, 1);
        cv::resize(input_image, input_image, cv::Size(), 1, 1);
        cloud = _cloud;

        dupl_publisher = n.advertise<sensor_msgs::Image>("duplicate_filtered", 1);
        pub_3D = n.advertise<sensor_msgs::Image>("min_distance_filtered", 1);


        if(image_source == 1)
        KP_pub_intensity = n.advertise<sensor_msgs::Image>("brisk_keypoints_intensity", 1);
        else if(image_source == 2)
        KP_pub_range = n.advertise<sensor_msgs::Image>("brisk_keypoints_range", 1);
        else
        KP_pub_ambient = n.advertise<sensor_msgs::Image>("brisk_keypoints_ambient", 1);

        create_descriptors();
        count = COUNT;
        ec = extracted_count;
        dfc = duplicate_filtered_count;
        mdfc = min_distance_filtered_count;
}

void BRISK::create_descriptors(){
    static cv::Ptr<cv::BRISK> detector = cv::BRISK::create(BRISK_THRESHOLD,OCTAVES,PATTERN_SCALE);
    detector->detect(image,brisk_keypoints,MASK);
    keypointTransition(brisk_keypoints,brisk_keypoints_2d);
    extracted_count += brisk_keypoints_2d.size();
    get_3D_data();
    publish_keypoints(&pub_3D, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),image_source);
    min_distance_filtered_count += brisk_keypoints_2d.size();
    if(APPLY_DUPLICATE_FILTERING){
        duplicate_filtering();
        publish_keypoints(&dupl_publisher, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),image_source);
    }
    duplicate_filtered_count += brisk_keypoints_2d.size();
    detector->compute(image,brisk_keypoints,brisk_descriptors);



    if(image_source == 1)
    publish_keypoints(&KP_pub_intensity, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),image_source);
    else if(image_source == 2)
    publish_keypoints(&KP_pub_range, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),image_source);
    else
    publish_keypoints(&KP_pub_ambient, image,brisk_keypoints_2d,1,cv::Scalar(0,255,0),image_source);

    COUNT++;
    if(COUNT >= 50){
    cout << "average extracted is : " << 1.0*extracted_count/COUNT<< " " << endl;
    cout << "average duplicate filtered is: " << 1.0*duplicate_filtered_count/COUNT << " " << endl;
    cout << "average min distance filtered is: " << 1.0*min_distance_filtered_count/COUNT<< " " << endl;
    cout << "TRUE POSITIVE EXTRACTOR RATE IS: " << 100*(1.0*duplicate_filtered_count/min_distance_filtered_count)<< " " << endl;
    }
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



