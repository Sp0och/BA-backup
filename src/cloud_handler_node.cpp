#include "../include/ImageHandler.h"
#include "ImageHandler.cpp"
#include "../include/parameters.h"
#include "../include/ORB.h"
#include "ORB.cpp"
// #include "../include/framehandler_old.h"
#include "../include/Framehandler.h"
#include "Framehandler.cpp"
#include "../include/KLT.h"
#include "KLT.cpp"
#include "../include/BRISK.h"
#include "BRISK.cpp"

std::string CLOUD_TOPIC;
int IMAGE_WIDTH;
int IMAGE_HEIGHT;
int IMAGE_CROP;
int NUM_THREADS;
int NUM_ORB_FEATURES;
int ORB_ACCURACY;
int BRISK_THRESHOLD;
int MIN_LOOP_FEATURE_NUM;
int MODE;
int DUPLICATE_FILTERING_SIZE;
int DOUBLE_FILTERING_SIZE;
double MAX_COS;
float MAX_FEATURE_DISTANCE;
float MIN_FEATURE_DISTANCE;
double MATCH_IMAGE_SCALE;
bool START_AT_ZERO;
int COUNT = 0;
ofstream OUT;


cv::Mat MASK;
pcl::PointCloud<PointType>::Ptr cloud_traj(new pcl::PointCloud<PointType>());

ImageHandler *image_handler;
Framehandler *frame_handler;
// Framehandler *frame_handler2;
// Framehandler *frame_handler3;
KLT *klt;

void updateParams (ros::NodeHandle& n){
    // Load params from parameter server
    std::string config_file;
    n.getParam("lio_loop_config_file", config_file);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    usleep(100);

    //set the parameters using parameter server
    fsSettings["cloud_topic"]  >> CLOUD_TOPIC;
    fsSettings["image_width"]  >> IMAGE_WIDTH;
    fsSettings["image_height"] >> IMAGE_HEIGHT;
    fsSettings["image_crop"]   >> IMAGE_CROP;
    fsSettings["num_threads"]  >> NUM_THREADS;
    fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;
    fsSettings["num_orb_features"] >> NUM_ORB_FEATURES;
    fsSettings["orb_accuracy"] >> ORB_ACCURACY;
    fsSettings["brisk_threshold"] >> BRISK_THRESHOLD;
    fsSettings["min_loop_feature_num"] >> MIN_LOOP_FEATURE_NUM;
    fsSettings["max_feature_distance"] >> MAX_FEATURE_DISTANCE;
    fsSettings["min_feature_distance"] >> MIN_FEATURE_DISTANCE;
    fsSettings["max_cos"] >> MAX_COS;
    fsSettings["mode"] >> MODE;
    fsSettings["duplicate_filtering_size"] >> DUPLICATE_FILTERING_SIZE;
    fsSettings["double_filtering_size"] >> DOUBLE_FILTERING_SIZE;
    fsSettings["start_at_zero"] >>START_AT_ZERO;
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
    //create subscriber subscribing to topic "points_raw"
    CSub = nps.subscribe(CLOUD_TOPIC, 1000, &cloud_displayer::callback_c, this);
    }
    //callback fct for the cloud.
    void callback_c(const sensor_msgs::PointCloud2::ConstPtr&  cloud_message){
        //Image handler part: project and visualize the pointcloud
        image_handler->cloud_handler(cloud_message);
        auto raw_time = cloud_message->header.stamp;
        MASK = create_mask();
        assert(MODE == 1 || MODE == 2 || MODE == 3);
        
        // image_intensity 
        cv::Mat& input_image = image_handler->image_intensity;
        if(MODE == 1);
            // image_range 
        else if(MODE == 2)
        input_image = image_handler->image_range;
            // image_ambient (noise) 
        else
        input_image = image_handler->image_noise;

        // static tf::TransformBroadcaster tf_base_to_lidar;
        // static tf::Transform base_to_lidar = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        // tf_base_to_lidar.sendTransform(tf::StampedTransform(base_to_lidar, cloud_message->header.stamp, "base_link", "velodyne"));

        //KLT:

        // klt->KLT_Iteration(input_image);
        if(raw_time >= ros::Time(1598537680.4)){
        //ORB:
        //test ORB alone:
        // ORB* orb = new ORB(input_image,image_handler->cloud_track,MODE);
        std::shared_ptr<ORB> orb = std::make_shared<ORB>(input_image,image_handler->cloud_track,MODE);
        // std::shared_ptr<ORB> orb2 = std::make_shared<ORB>(input_image2,image_handler->cloud_track,2);
        // std::shared_ptr<ORB> orb3 = std::make_shared<ORB>(input_image3,image_handler->cloud_track,3);
            

        //BRISK:

        // std::shared_ptr<BRISK> brisk = std::make_shared<BRISK>(input_image,image_handler->cloud_track,MODE);
        //Brisk alone
        // BRISK* brisk = new BRISK(input_image,image_handler->cloud_track,MODE);



        // Matches
            frame_handler->newIteration(orb,raw_time);
            // frame_handler2->newIteration(orb2);
            // frame_handler3->newIteration(orb3);
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
    image_handler = new ImageHandler();
    frame_handler = new Framehandler(MODE,START_AT_ZERO);
    // frame_handler2 = new Framehandler(2);
    // frame_handler3 = new Framehandler(3);
    cloud_displayer cloudDisplayer;
    klt = new KLT(MODE);
  
    ros::spin();
    return 0;
}