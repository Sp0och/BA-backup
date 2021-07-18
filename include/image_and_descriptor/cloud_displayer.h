#include "image_and_descriptor/setup.h"
#include "image_and_descriptor/ImageHandler.h"
#include "image_and_descriptor/ORB.h"
#include "image_and_descriptor/ORB_Framehandler.h"
#include "image_and_descriptor/BRISK_Framehandler.h"
#include "image_and_descriptor/KLT.h"
#include "image_and_descriptor/BRISK.h"
/**
 * Class cloud_displayer: create a subscriber that subscribes to the topic of the raw PC and via callback function projects that
 * data and publishes the projections.
 * */
class cloud_displayer{

    public:
    cloud_displayer();
    //callback fct for the cloud.
    void callback_c(const sensor_msgs::PointCloud2::ConstPtr&  cloud_message);

    cv::Mat create_mask();

    private:
    //predeclaration of subscribers and nodehandle
    ros::NodeHandle M_nps;
    ros::Subscriber M_CSub;

    std::string M_CLOUD_TOPIC;
    std::string M_EXTRACTOR; 
    int M_IMAGE_WIDTH;
    int M_IMAGE_HEIGHT;
    int M_IMAGE_CROP;
    int M_IMAGE_SOURCE;
    cv::Mat M_MASK;

    //start time
    double M_START_TIMESTAMP;

    ImageHandler* M_image_handler;
    ORB_Framehandler* M_orb_frame_handler;
    // ORB_Framehandler* M_orb_frame_handler2;
    // ORB_Framehandler* M_orb_frame_handler3;
    BRISK_Framehandler* M_brisk_frame_handler;
    // Framehandler* M_frame_handler2;
    // Framehandler* M_frame_handler3;
    KLT* M_klt;
    //KLT* M_klt1;
    //KLT* M_klt2

    ros::NodeHandle n_ch;
  };