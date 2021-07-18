#include "image_and_descriptor/cloud_displayer.h"




int main(int argc, char **argv){

    ros::init(argc, argv, "cloud_projection");
    ros::NodeHandle n;

    cloud_displayer cloudDisplayer;
  
    ros::spin();
    return 0;
}