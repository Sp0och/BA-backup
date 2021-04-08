#include "image_handler.h"
#include "parameters.h"
#include "Frame.h"
#include "frame_handler.h"

std::string PROJECT_NAME;
std::string CLOUD_TOPIC;
std::string PATH_TOPIC;
int IMAGE_WIDTH;
int IMAGE_HEIGHT;
int IMAGE_CROP;
int USE_BRIEF;
int USE_ORB;
int NUM_BRI_FEATURES;
int NUM_ORB_FEATURES;
int MIN_LOOP_FEATURE_NUM;
int MIN_LOOP_SEARCH_GAP;
double MIN_LOOP_SEARCH_TIME;
// float MIN_LOOP_BOW_TH;
 double SKIP_TIME = 0;
int NUM_THREADS;
int DEBUG_IMAGE;
double MATCH_IMAGE_SCALE;
cv::Mat MASK;
map<int, int> index_match_container;
map<int, int> index_poseindex_container;
pcl::PointCloud<PointType>::Ptr cloud_traj(new pcl::PointCloud<PointType>());

ros::Publisher pub_match_img;
ros::Publisher pub_match_msg;
// ros::Publisher pub_bow_img;
ros::Publisher pub_prepnp_img;
ros::Publisher pub_marker;
ros::Publisher pub_index;
string pkg_path = ros::package::getPath(PROJECT_NAME);
string brief_pattern_file;

BriefExtractor briefExtractor;

ImageHandler *image_handler;
frame_handler FRAME_HANDLER;

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
    fsSettings["image_width"]  >> IMAGE_WIDTH;
    fsSettings["image_height"] >> IMAGE_HEIGHT;
    fsSettings["image_crop"]   >> IMAGE_CROP;
    fsSettings["use_brief"]    >> USE_BRIEF;
    fsSettings["use_orb"]      >> USE_ORB;
    fsSettings["num_bri_features"] >> NUM_BRI_FEATURES;
    fsSettings["num_orb_features"] >> NUM_ORB_FEATURES;
    fsSettings["min_loop_feature_num"] >> MIN_LOOP_FEATURE_NUM;
    fsSettings["min_loop_search_gap"]  >> MIN_LOOP_SEARCH_GAP;
    fsSettings["min_loop_search_time"] >> MIN_LOOP_SEARCH_TIME;
    // fsSettings["min_loop_bow_th"]      >> MIN_LOOP_BOW_TH;
    fsSettings["skip_time"]    >> SKIP_TIME;
    fsSettings["num_threads"]  >> NUM_THREADS;
    fsSettings["debug_image"]  >> DEBUG_IMAGE;
    fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;
    fsSettings["brief_pattern_file"] >> brief_pattern_file;  
}

/**
 * Class cloud_projector: create a subscriber that subscribes to the topic of the raw PC and via callback function projects that
 * data and publishes the projections.
 * */
class cloud_displayer{

    public:
    cloud_displayer(){
    //create subscriber subscribing to topic "points_raw"
    CSub = nps.subscribe/* <const sensor_msgs::PointCloud2> */(CLOUD_TOPIC, 1000, &cloud_displayer::callback_c, this);
    PSub = nps.subscribe/* <const nav_msgs::PathConstPtr&> */(PATH_TOPIC, 1000, &cloud_displayer::callback_p, this);
    }
    //callback fct for the cloud.
    void callback_c(const sensor_msgs::PointCloud2::ConstPtr&  cloud_message){
        image_handler->cloud_handler(cloud_message);

        double cloud_time = cloud_message->header.stamp.toSec();

        static int global_frame_index = 0;
    Frame* frame = new Frame(cloud_time,
                             global_frame_index,
                             image_handler->image_intensity,
                             image_handler->cloud_track);  

        FRAME_HANDLER.addFrame(frame, 1);
        global_frame_index++;
    
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

    MASK = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < IMAGE_HEIGHT; ++i)
        for (int j = 0; j < IMAGE_WIDTH; ++j)
        //cut off left and right edge
            if (j < IMAGE_CROP || j > IMAGE_WIDTH - IMAGE_CROP)
                MASK.at<uchar>(i,j) = 0;

    //set up publishers
    pub_match_img  = n.advertise<sensor_msgs::Image>                ("loop_detector/image", 1);
    pub_match_msg  = n.advertise<std_msgs::Float64MultiArray>       ("loop_detector/time", 1);
    // pub_bow_img    = n.advertise<sensor_msgs::Image>                ("loop_detector/bow", 1);
    pub_prepnp_img = n.advertise<sensor_msgs::Image>                ("loop_detector/prepnp", 1);
    pub_marker     = n.advertise<visualization_msgs::MarkerArray>   ("loop_detector/marker", 1);
    pub_index      = n.advertise<std_msgs::Int64MultiArray>         ("loop_detector/index", 1);

    
    brief_pattern_file = pkg_path + brief_pattern_file;
    briefExtractor = BriefExtractor(brief_pattern_file);


    image_handler = new ImageHandler();
    cloud_displayer cloudDisplayer;

  
    ros::spin();

    return 0;
}