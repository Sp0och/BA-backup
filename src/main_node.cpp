#include "package_BA_LF/setup.h"
#include "package_BA_LF/ImageHandler.h"
#include "package_BA_LF/ORB.h"
#include "package_BA_LF/ORB_Framehandler.h"
#include "package_BA_LF/BRISK_Framehandler.h"
#include "package_BA_LF/KLT.h"
#include "package_BA_LF/BRISK.h"


std::string CLOUD_TOPIC;
std::string EXTRACTOR; 
int IMAGE_WIDTH;
int IMAGE_HEIGHT;
int IMAGE_CROP;
int IMAGE_SOURCE;
ofstream OUT;
cv::Mat MASK;

//klt
int MAX_KLT_FEATURES;
int MIN_KLT_FEATURES;
//brisk
int BRISK_THRESHOLD;
//filtering
int DUPLICATE_FILTERING_SIZE;
bool APPLY_DISTANCE_FILTERING;
bool APPLY_RANSAC_FILTERING;
bool APPLY_DUPLICATE_FILTERING;
double MAX_DEPTH_DISTANCE;
//start time and pose
int START_POSE;
double START_TIMESTAMP;

bool SHOULD_STORE;

//analysis
int extracted_count;
int duplicate_filtered_count;
int min_distance_filtered_count;
int COUNT;

pcl::PointCloud<PointType>::Ptr cloud_traj(new pcl::PointCloud<PointType>());



ImageHandler *image_handler;
ORB_Framehandler *orb_frame_handler;
// ORB_Framehandler *orb_frame_handler2;
// ORB_Framehandler *orb_frame_handler3;
BRISK_Framehandler *brisk_frame_handler;
// Framehandler *frame_handler2;
// Framehandler *frame_handler3;
KLT *klt;

void updateParams (ros::NodeHandle& n){
    // Load params from parameter server
    std::string config_file;
    n.getParam("parameter_file", config_file);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    usleep(100);

    //set the parameters using parameter server
    fsSettings["cloud_topic"]  >> CLOUD_TOPIC;
    fsSettings["image_width"]  >> IMAGE_WIDTH;
    fsSettings["image_height"] >> IMAGE_HEIGHT;
    fsSettings["image_crop"]   >> IMAGE_CROP;
    fsSettings["image_source"] >> IMAGE_SOURCE;
    fsSettings["extractor"]  >> EXTRACTOR;

    fsSettings["duplicate_filtering_size"] >> DUPLICATE_FILTERING_SIZE;
    fsSettings["apply_duplicate_filtering"] >> APPLY_DUPLICATE_FILTERING;
    fsSettings["apply_ransac_filtering"] >> APPLY_RANSAC_FILTERING;
    fsSettings["apply_distance_filtering"] >> APPLY_DISTANCE_FILTERING;
    fsSettings["max_depth_distance"] >> MAX_DEPTH_DISTANCE;

    fsSettings["start_pose"] >> START_POSE;
    fsSettings["start_timestamp"] >> START_TIMESTAMP;

    fsSettings["should_store"] >> SHOULD_STORE;
}

cv::Mat create_mask(){
    // block left and right edge of FoW
    MASK = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < IMAGE_HEIGHT; ++i)
        for (int j = 0; j < IMAGE_WIDTH; ++j)
            if (j < IMAGE_CROP || j > IMAGE_WIDTH - IMAGE_CROP/2)
                MASK.at<uchar>(i,j) = 0;
    return MASK;
}



/**
 * Class cloud_displayer: create a subscriber that subscribes to the topic of the raw PC and via callback function projects that
 * data and publishes the projections.
 * */
class cloud_displayer{

    public:
    cloud_displayer(){
        std::cout << "cloud displayer called" << endl;
    //subscribe to topic "points_raw"
    CSub = nps.subscribe(CLOUD_TOPIC, 1000, &cloud_displayer::callback_c, this);
    }
    //callback fct for the cloud.
    void callback_c(const sensor_msgs::PointCloud2::ConstPtr&  cloud_message){
        //Image handler part: project and visualize the pointcloud
        image_handler->cloud_handler(cloud_message);
        auto raw_time = cloud_message->header.stamp;
        MASK = create_mask();
        ros::Time first_timestamp(START_TIMESTAMP);
        
        
        // Case 1: image source choice by parameter server
        cv::Mat& input_image = image_handler->image_intensity;
        if(IMAGE_SOURCE == 1);
        else if(IMAGE_SOURCE == 2)
        input_image = image_handler->image_range;
        else
        input_image = image_handler->image_noise;
        
        // Case 2: all 3 for comparison
        // cv::Mat& input_image = image_handler->image_intensity;
        // cv::Mat& input_image2 = image_handler->image_range;
        // cv::Mat& input_image3 = image_handler->image_noise;


        if(raw_time >= ros::Time(START_TIMESTAMP)){
            // cout << "this TS: " << raw_time << " " << endl;

            //KLT:
            if((EXTRACTOR == "klt")){
                klt->KLT_Iteration(input_image,image_handler->cloud_track,raw_time);
                // klt->KLT_Iteration(input_image2,image_handler->cloud_track,raw_time);
                // klt->KLT_Iteration(input_image3,image_handler->cloud_track,raw_time);
            }


            //ORB:
            if((EXTRACTOR == "orb")){
                //test ORB alone:
                // ORB* orb = new ORB(input_image,image_handler->cloud_track,IMAGE_SOURCE);
                // ORB* orb1 = new ORB(input_image2,image_handler->cloud_track,2);
                // ORB* orb2 = new ORB(input_image3,image_handler->cloud_track,3);
                
                //ORB with frame comparison (different instances for possible visual performance comparison)
                std::shared_ptr<ORB> orb = std::make_shared<ORB>(input_image,image_handler->cloud_track,IMAGE_SOURCE);
                // std::shared_ptr<ORB> orb2 = std::make_shared<ORB>(input_image2,image_handler->cloud_track,2);
                // std::shared_ptr<ORB> orb3 = std::make_shared<ORB>(input_image3,image_handler->cloud_track,3);
                // Matches
                orb_frame_handler->newIteration(orb,raw_time);
                // orb_frame_handler2->newIteration(orb2,raw_time);
                // orb_frame_handler3->newIteration(orb3,raw_time);
            }

            //BRISK:
            if((EXTRACTOR == "brisk")){
                //Brisk alone
                // BRISK* brisk = new BRISK(input_image,image_handler->cloud_track,IMAGE_SOURCE);
                // BRISK* brisk2 = new BRISK(input_image2,image_handler->cloud_track,2);
                // BRISK* brisk3 = new BRISK(input_image3,image_handler->cloud_track,3);
                //BRISK and frame comparison
                std::shared_ptr<BRISK> brisk = std::make_shared<BRISK>(input_image,image_handler->cloud_track,IMAGE_SOURCE);
                // std::shared_ptr<BRISK> brisk2 = std::make_shared<BRISK>(input_image2,image_handler->cloud_track,2);
                // std::shared_ptr<BRISK> brisk3 = std::make_shared<BRISK>(input_image3,image_handler->cloud_track,3);
                //Matches
                brisk_frame_handler->newIteration(brisk,raw_time);
            }

        }
    }

    
    private:
    //predeclaration of subscribers and nodehandle
    ros::NodeHandle nps;
    ros::Subscriber CSub;
  };

int main(int argc, char **argv){

    ros::init(argc, argv, "cloud_projection");
    ros::NodeHandle n;
    updateParams(n);
    assert(IMAGE_SOURCE == 1 || IMAGE_SOURCE == 2 || IMAGE_SOURCE == 3);
    assert(EXTRACTOR == "orb" || EXTRACTOR == "brisk" || EXTRACTOR == "klt");
    image_handler = new ImageHandler();
    orb_frame_handler = new ORB_Framehandler(IMAGE_SOURCE,START_POSE);
    // orb_frame_handler2 = new ORB_Framehandler(2,START_POSE);
    // orb_frame_handler3 = new ORB_Framehandler(3,START_POSE);
    brisk_frame_handler = new BRISK_Framehandler(IMAGE_SOURCE,START_POSE);
    // frame_handler2 = new Framehandler(2);
    // frame_handler3 = new Framehandler(3);
    klt = new KLT(IMAGE_SOURCE,START_POSE);
    // klt1 = new KLT(2,START_POSE);
    // klt2 = new KLT(3,START_POSE);
    cloud_displayer cloudDisplayer;
  
    ros::spin();
    return 0;
}