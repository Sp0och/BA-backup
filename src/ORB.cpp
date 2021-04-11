#include "ORB.h"



//copy keypoints_in to the initial point2f vector points_in
void keypointTransition(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2f>& points_in)
{
    points_in.resize(keypoints_in.size());
    for(size_t i = 0; i < keypoints_in.size(); i++)
    {
        points_in[i] = keypoints_in[i].pt;
    }
}
//Convert points to keypoints vector
void keypointTransition(vector<cv::Point2f>& points_in, vector<cv::KeyPoint>& keypoints_in)
{
    keypoints_in.resize(points_in.size());
    for(size_t i = 0; i < points_in.size(); i++)
    {
        cv::KeyPoint key;
        key.pt = points_in[i];
        keypoints_in[i] = key;
    }
}


void publish_keypoints (ros::Publisher* publisher, cv::Mat& image, const vector<cv::Point2f>& keypoints, const int circle_size,const cv::Scalar line_color){
    cv::cvtColor(image, image, CV_GRAY2RGB);
    for(int i = 0; i < (int)keypoints.size(); i++){
        cv::Point2f cur_pt = keypoints[i] * MATCH_IMAGE_SCALE;
        cv::circle(image,cur_pt,circle_size,line_color);
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher->publish(msg);
}



ORB::ORB(const cv::Mat &_input_image, 
         const pcl::PointCloud<PointType>::Ptr _cloud,
         int _mode){
        mode = _mode;
        assert(mode == 1 || mode == 2 || mode == 3);

        input_image = _input_image.clone();
        cv::resize(input_image, image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
        cv::resize(input_image, input_image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
        cloud = _cloud;

        if(mode == 1)
        KP_pub_intensity = n.advertise<sensor_msgs::Image>("orb_keypoints_intensity", 1);
        else if(mode == 2)
        KP_pub_range = n.advertise<sensor_msgs::Image>("orb_keypoints_range", 1);
        else
        KP_pub_ambient = n.advertise<sensor_msgs::Image>("orb_keypoints_ambient", 1);
        
        create_descriptors();

        

}


/**
 * Stores the ORB keypoints in the vector in the std::vector format
 * @param orb_keypoints_2d is then the vector containing the keypoints
 * */
void ORB::create_descriptors(){
    cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES, 1.2f, 8, 1);
    //store keypoints in orb_keypoints
    detector->detect(image,orb_keypoints,MASK);
    keypointTransition(orb_keypoints,orb_keypoints_2d);
    detector->compute(image,orb_keypoints,orb_descriptors);

    if(mode == 1)
    publish_keypoints(&KP_pub_intensity, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
    else if(mode == 2)
    publish_keypoints(&KP_pub_range, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
    else
    publish_keypoints(&KP_pub_ambient, image,orb_keypoints_2d,5,cv::Scalar(0,255,0));
}

    /* void ORB::real_keypoints(const vector<cv::Point2f>& in_vector, 
                        vector<cv::Point3f>& out_3d,
                        vector<cv::Point2f>& out_2d_norm
                        vector<uchar>& out_status){
        orb_point_3d.resize(in_vector.size());
        orb_point_2d_norm.resize(in_vector.size());

        #pragma omp parallel for num_threads(NUM_THREADS)

        for(int i = 0; i < in_vector.size(); i++){
            int row_index = (int)in_vector[i].y;
            int col_index = (int)in_vector[i].x;
            int index = row_index*IMAGE_WIDTH + col_index;
            orb_point_3d[i].x = cloud;
        }
    } */
















