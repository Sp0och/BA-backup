#include "image_and_descriptor/cloud_displayer.h"

/**
 * Class cloud_displayer: create a subscriber that subscribes to the topic of the raw PC and via callback function projects that
 * data and publishes the projections.
 * */
cloud_displayer::cloud_displayer(){
    std::cout << "cloud displayer called" << endl;

    // Load params from parameter server
    std::string config_file;
    n_ch.getParam("parameter_file", config_file);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    usleep(100);

    //set the parameters using parameter server
    fsSettings["cloud_topic"]  >> M_CLOUD_TOPIC;
    fsSettings["image_width"]  >> M_IMAGE_WIDTH;
    fsSettings["image_height"] >> M_IMAGE_HEIGHT;
    fsSettings["image_crop"]   >> M_IMAGE_CROP;
    fsSettings["image_source"] >> M_IMAGE_SOURCE;
    fsSettings["extractor"]  >> M_EXTRACTOR;

    fsSettings["start_timestamp"] >> M_START_TIMESTAMP;

    M_MASK = create_mask();

    M_image_handler = new ImageHandler();
    M_orb_frame_handler = new ORB_Framehandler(M_IMAGE_SOURCE);
    // M_orb_frame_handler2 = new ORB_Framehandler(2);
    // M_orb_frame_handler3 = new ORB_Framehandler(3);
    M_brisk_frame_handler = new BRISK_Framehandler(M_IMAGE_SOURCE);
    // M_frame_handler2 = new Framehandler(2);
    // M_frame_handler3 = new Framehandler(3);
    M_klt = new KLT(M_IMAGE_SOURCE, M_MASK);
    // M_klt1 = new KLT(2, M_MASK);
    // M_klt2 = new KLT(3, M_MASK);

    //subscribe to topic "points_raw"
    M_CSub = M_nps.subscribe(M_CLOUD_TOPIC, 1000, &cloud_displayer::callback_c, this);
}
//callback fct for the cloud.
void cloud_displayer::callback_c(const sensor_msgs::PointCloud2::ConstPtr&  cloud_message){
    assert(M_IMAGE_SOURCE == 1 || M_IMAGE_SOURCE == 2 || M_IMAGE_SOURCE == 3);
    assert(M_EXTRACTOR == "orb" || M_EXTRACTOR == "brisk" || M_EXTRACTOR == "klt");
    //Image handler part: project and visualize the pointcloud
    M_image_handler->cloud_handler(cloud_message);
    auto raw_time = cloud_message->header.stamp;
    ros::Time first_timestamp(M_START_TIMESTAMP);
    
    
    // Case 1: image source choice by parameter server
    cv::Mat& input_image = M_image_handler->image_intensity;
    if(M_IMAGE_SOURCE == 1);
    else if(M_IMAGE_SOURCE == 2)
    input_image = M_image_handler->image_range;
    else
    input_image = M_image_handler->image_noise;
    
    // Case 2: all 3 for comparison
    // cv::Mat& input_image = image_handler.image_intensity;
    // cv::Mat& input_image2 = image_handler.image_range;
    // cv::Mat& input_image3 = image_handler.image_noise;


    if(raw_time >= ros::Time(M_START_TIMESTAMP)){
        // cout << "this TS: " << raw_time << " " << endl;

        //KLT:
        if((M_EXTRACTOR == "klt")){
            M_klt->KLT_Iteration(input_image,M_image_handler->cloud_track,raw_time);
            // M_klt->KLT_Iteration(input_image2,image_handler->cloud_track,raw_time);
            // M_klt->KLT_Iteration(input_image3,image_handler->cloud_track,raw_time);
        }


        //ORB:
        if((M_EXTRACTOR == "orb")){
            //test ORB alone:
            // M_orb = new ORB(input_image,image_handler->cloud_track,M_IMAGE_SOURCE,M_MASK);
            // M_orb1 = new ORB(input_image2,image_handler->cloud_track,2,M_MASK);
            // M_orb2 = new ORB(input_image3,image_handler->cloud_track,3,M_MASK);
            
            //ORB with frame comparison (different instances for possible visual performance comparison)
            std::shared_ptr<ORB> orb = std::make_shared<ORB>(input_image,M_image_handler->cloud_track,M_IMAGE_SOURCE,M_MASK);
            // std::shared_ptr<ORB> orb2 = std::make_shared<ORB>(input_image2,image_handler->cloud_track,2,M_MASK);
            // std::shared_ptr<ORB> orb3 = std::make_shared<ORB>(input_image3,image_handler->cloud_track,3,M_MASK);
            // Matches
            M_orb_frame_handler->newIteration(orb,raw_time);
            // M_orb_frame_handler2.newIteration(orb2,raw_time);
            // M_orb_frame_handler3.newIteration(orb3,raw_time);
        }

        //BRISK:
        if((M_EXTRACTOR == "brisk")){
            //Brisk alone
            // M_brisk = new BRISK(input_image,image_handler->cloud_track,M_IMAGE_SOURCE,M_MASK);
            // M_brisk2 = new BRISK(input_image2,image_handler->cloud_track,2,M_MASK);
            // M_brisk3 = new BRISK(input_image3,image_handler->cloud_track,3,M_MASK);
            //BRISK and frame comparison
            std::shared_ptr<BRISK> brisk = std::make_shared<BRISK>(input_image,M_image_handler->cloud_track,M_IMAGE_SOURCE,M_MASK);
            // std::shared_ptr<BRISK> brisk2 = std::make_shared<BRISK>(input_image2,image_handler->cloud_track,2,M_MASK);
            // std::shared_ptr<BRISK> brisk3 = std::make_shared<BRISK>(input_image3,image_handler->cloud_track,3,M_MASK);
            //Matches
            M_brisk_frame_handler->newIteration(brisk,raw_time);
        }

    }
}

cv::Mat cloud_displayer::create_mask(){
    // block left and right edge of FoW
    cv::Mat MASK = cv::Mat(M_IMAGE_HEIGHT, M_IMAGE_WIDTH, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < M_IMAGE_HEIGHT; ++i)
        for (int j = 0; j < M_IMAGE_WIDTH; ++j)
            if (j < M_IMAGE_CROP || j > M_IMAGE_WIDTH - M_IMAGE_CROP/2)
                MASK.at<uchar>(i,j) = 0;
    return MASK;
}