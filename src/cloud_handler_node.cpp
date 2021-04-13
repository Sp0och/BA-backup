#include "image_handler.h"
#include "parameters.h"
#include "ORB.h"
#include "frame_handler.h"

std::string PROJECT_NAME;
std::string CLOUD_TOPIC;
std::string PATH_TOPIC;
std::string ORB_TOPIC;
int IMAGE_WIDTH;
int IMAGE_HEIGHT;
int IMAGE_CROP;
int NUM_THREADS;
int NUM_ORB_FEATURES;
int NUM_SIFT_FEATURES;
int MIN_LOOP_FEATURE_NUM;
int MODE;
double MATCH_IMAGE_SCALE;
cv::Mat MASK;
pcl::PointCloud<PointType>::Ptr cloud_traj(new pcl::PointCloud<PointType>());


ImageHandler *image_handler;
Framehandler *frame_handler;

void updateParams (ros::NodeHandle& n){
    // Load params from parameter server
    std::string config_file;
    n.getParam("lio_loop_config_file", config_file);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    usleep(100);

    //set the parameters using parameter server
    fsSettings["project_name"] >> PROJECT_NAME;
    fsSettings["cloud_topic"]  >> CLOUD_TOPIC;
    fsSettings["path_topic"]   >> PATH_TOPIC;
    fsSettings["orb_topic"]   >> ORB_TOPIC;
    fsSettings["image_width"]  >> IMAGE_WIDTH;
    fsSettings["image_height"] >> IMAGE_HEIGHT;
    fsSettings["image_crop"]   >> IMAGE_CROP;
    fsSettings["num_threads"]  >> NUM_THREADS;
    fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;
    fsSettings["num_orb_features"] >> NUM_ORB_FEATURES;
    fsSettings["num_sift_features"] >> NUM_SIFT_FEATURES;
    fsSettings["min_loop_feature_num"] >> MIN_LOOP_FEATURE_NUM;
    fsSettings["mode"] >> MODE;
    
}

cv::Mat create_mask(){
    // create a mask for blocking feature extraction
    MASK = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < IMAGE_HEIGHT; ++i)
        for (int j = 0; j < IMAGE_WIDTH; ++j)
            if (j < IMAGE_CROP || j > IMAGE_WIDTH - IMAGE_CROP)
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
    PSub = nps.subscribe(PATH_TOPIC, 1000, &cloud_displayer::callback_p, this);
    }
    //callback fct for the cloud.
    void callback_c(const sensor_msgs::PointCloud2::ConstPtr&  cloud_message){
        //Image handler part: project and visualize the pointcloud
        image_handler->cloud_handler(cloud_message);

        MASK = create_mask();

        //choose image source for keypoint detection
        // image_intensity 
        ORB* orb1 = new ORB(image_handler->image_intensity, image_handler->cloud_track,MODE);
        // image_range 
        // ORB* orb2 = new ORB(image_handler->image_range, image_handler->cloud_track,2);
        // image_ambient (noise) 
        // ORB* orb3 = new ORB(image_handler->image_noise, image_handler->cloud_track,3);
        
        //frame pipeline
        frame_handler->newIteration(orb1);
    }

    
    //callback function for the path
    void callback_p(const nav_msgs::PathConstPtr& path_msg)
    {
    cloud_traj->clear();

    for (size_t i = 0; i < path_msg->poses.size(); ++i)
    {
        PointType p;
        p.x = path_msg->poses[i].pose.position.x;
        p.y = path_msg->poses[i].pose.position.y;
        p.z = path_msg->poses[i].pose.position.z;
        cloud_traj->push_back(p);
    }
    }
    
    private:
  //predeclaration of subscribers and nodehandle
    ros::NodeHandle nps;
    ros::Subscriber CSub;
    ros::Subscriber PSub;
  };

int main(int argc, char **argv){

ros::init(argc, argv, "cloud_projection");
ros::NodeHandle n;

    updateParams(n);


    image_handler = new ImageHandler();
    frame_handler = new Framehandler();
    cloud_displayer cloudDisplayer;

  
    ros::spin();

    return 0;
}