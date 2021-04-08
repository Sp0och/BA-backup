#include "ORB.h"


ORB::ORB(const cv::Mat &_image_intensity, 
         const pcl::PointCloud<PointType>::Ptr _cloud){

             image_intensity = _image_intensity;
             cloud = _cloud;
             cv::resize(image, thumbnail, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
             create_keypoints();
}

//copy keypoints_in to the initial point2f vector points_in
void keypointConverter(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2f>& points_in)
{
    points_in.resize(keypoints_in.size());
    for(size_t i = 0; i < keypoints_in.size(); i++)
    {
        points_in[i] = keypoints_in[i].pt;
    }
}
//Convert points to keypoints vector
void keypointConverter(vector<cv::Point2f>& points_in, vector<cv::KeyPoint>& keypoints_in)
{
    keypoints_in.resize(points_in.size());
    for(size_t i = 0; i < points_in.size(); i++)
    {
        cv::KeyPoint key;
        key.pt = points_in[i];
        keypoints_in[i] = key;
    }
}

void create_keypoints(){
    cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES, 1.2f, 8, 1);
    detector->detect(image_intensity,orb_keypoints,MASK);
}














