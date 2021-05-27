#include "BRISK.h"


static void keypointTransition(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2f>& points_in)
{
    points_in.resize(keypoints_in.size());
    for(size_t i = 0; i < keypoints_in.size(); i++)
    {
        points_in[i] = keypoints_in[i].pt;
    }
}
//Convert points to keypoints vector
static void keypointTransition(vector<cv::Point2f>& points_in, vector<cv::KeyPoint>& keypoints_in)
{
    keypoints_in.resize(points_in.size());
    for(size_t i = 0; i < points_in.size(); i++)
    {
        cv::KeyPoint key;
        key.pt = points_in[i];
        keypoints_in[i] = key;
    }
}


static void publish_keypoints (ros::Publisher* publisher, cv::Mat& image, const vector<cv::Point2f>& keypoints, const int circle_size,const cv::Scalar line_color){
    cv::cvtColor(image, image, CV_GRAY2RGB);
    for(int i = 0; i < (int)keypoints.size(); i++){
        cv::Point2f cur_pt = keypoints[i] * MATCH_IMAGE_SCALE;
        cv::circle(image,cur_pt,circle_size,line_color);
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher->publish(msg);
}
template <typename Derived>
static void trimVector_brisk(vector<Derived> &v, vector<bool> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

BRISK::BRISK(const cv::Mat &_input_image, 
        const pcl::PointCloud<PointType>::Ptr _cloud,
        int _mode){
        mode = _mode;
        assert(mode == 1 || mode == 2 || mode == 3);

        input_image = _input_image.clone();
        cv::resize(input_image, image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
        cv::resize(input_image, input_image, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);
        cloud = _cloud;

        if(mode == 1)
        KP_pub_intensity = n.advertise<sensor_msgs::Image>("brisk_keypoints_intensity", 1);
        else if(mode == 2)
        KP_pub_range = n.advertise<sensor_msgs::Image>("brisk_keypoints_range", 1);
        else
        KP_pub_ambient = n.advertise<sensor_msgs::Image>("brisk_keypoints_ambient", 1);

        create_descriptors();
}

void BRISK::create_descriptors(){
    static cv::Ptr<cv::BRISK> detector = cv::BRISK::create(30,3,1.0f);
    detector->detect(image,brisk_keypoints,MASK);
    duplicate_filtering();
    detector->compute(image,brisk_keypoints,brisk_descriptors);
    keypointTransition(brisk_keypoints,brisk_keypoints_2d);
    //put keypoints into vector and create descriptors.
    points_for_ransac();
    if(mode == 1)
    publish_keypoints(&KP_pub_intensity, image,brisk_keypoints_2d,5,cv::Scalar(0,255,0));
    else if(mode == 2)
    publish_keypoints(&KP_pub_range, image,brisk_keypoints_2d,5,cv::Scalar(0,255,0));
    else
    publish_keypoints(&KP_pub_ambient, image,brisk_keypoints_2d,5,cv::Scalar(0,255,0));
}

void BRISK::points_for_ransac(){
    brisk_points_3d.resize(brisk_keypoints_2d.size());
    brisk_point_2d_norm.resize(brisk_keypoints_2d.size());
    vector<bool> status;
    status.resize(brisk_keypoints_2d.size());
    #pragma omp parallel for num_threads(NUM_THREADS)

    for(int i = 0; i < brisk_keypoints_2d.size(); i++){
        int row_index = (int)brisk_keypoints_2d[i].y;
        int col_index = (int)brisk_keypoints_2d[i].x;
        int index = row_index*IMAGE_WIDTH + col_index;
        PointType *pi = &cloud->points[index];

        cv::Point3f p_3d(0.f, 0.f, 0.f);
        cv::Point2f p_2d_n(0.f, 0.f);
        //why? Cut off left edge?
        if (abs(pi->x) < 0.01)
        {
            status[i] = 0;
        } 
        //create the points to fill in camera mode
        else 
        {
            status[i] = 1;
            // lidar -> camera
            p_3d.x = -pi->y;
            p_3d.y = -pi->z;
            p_3d.z = pi->x;
            // normalize to projection plane
            p_2d_n.x = p_3d.x / p_3d.z;
            p_2d_n.y = p_3d.y / p_3d.z;
        }
    }
        
    trimVector_brisk(brisk_points_3d,status);
    trimVector_brisk(brisk_point_2d_norm,status);
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
        trimVector_brisk(brisk_keypoints_2d,duplicate_status);
        trimVector_brisk(brisk_keypoints,duplicate_status);
    }



