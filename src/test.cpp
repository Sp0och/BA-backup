#include "parameters.h"

template <typename Derived>

void trim_vector(vector<Derived> &v, vector<uchar>& status){
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void create_points(std::vector<cv::Point3f>& input_vector, int IW, int IH){
        cv::Point3d point;
    for(size_t i = 0; i < input_vector.size(); i++){
        point.x = rand()%IW;
        point.y = rand()%IH;
        point.z = rand()%1024;
        input_vector[i] = point;
    }
}
void create_points_2d(std::vector<cv::Point2f>& input_vector_2d,std::vector<cv::Point2f>& normed_vector, std::vector<cv::Point3f>& input_vector_3d){
        cv::Point2d point_normed;
        cv::Point2d point;
    for(size_t i = 0; i < input_vector_3d.size(); i++){
        point.x = input_vector_3d[i].x/*  / input_vector_3d[i].z */;
        point.y = input_vector_3d[i].y/*  / input_vector_3d[i].z */;
        point_normed.x = input_vector_3d[i].x / input_vector_3d[i].z;
        point_normed.y = input_vector_3d[i].y / input_vector_3d[i].z;
        if(i % 3 == 0){
        point.y += 30;
        point_normed.y += 30/input_vector_3d[i].z;
        }
        input_vector_2d[i] = point;
        normed_vector[i] = point_normed;
    }
}

  void publish_matches(const ros::Publisher* this_pub,cv::Mat& image, std::vector<cv::Point2f>& sorted_KP_cur,std::vector<cv::Point2f>& KP_P,  int circle_size, cv::Scalar line_color,cv::Scalar line_color2){
    cv::Mat color_image;
    cv::cvtColor(image,color_image,CV_GRAY2RGB);
    //indicate features in new image
    for(int i = 0; i< (int)sorted_KP_cur.size(); i++)
    {
        cv::Point2f cur_pt = sorted_KP_cur[i];
        cv::circle(color_image, cur_pt, circle_size, line_color, 1);
    }
    
    for(int i = 0; i< (int)KP_P.size(); i++)
    {
        cv::Point2f cur_pt = KP_P[i];
        cv::circle(color_image, cur_pt, circle_size, line_color2, 1);
    }
    

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
    this_pub->publish(msg);

}


int main(int argc, char **argv){
    int IMAGE_HEIGHT = 128;
    int IMAGE_WIDTH = 256;
    ros::init(argc, argv, "cloud_projection");
    ros::NodeHandle n;

    ros::Publisher publisher = n.advertise<sensor_msgs::Image>("test_filtered_keypoints", 1);
    ros::Publisher publisher_projected = n.advertise<sensor_msgs::Image>("projected_filtered_keypoints", 1);
    ros::Rate loop_rate(1);

    while(ros::ok()){
    std::vector<cv::Point2f> points_2d(100), points_normed_2d(100);
    std::vector<cv::Point3f> points_3d(100);
    create_points(points_3d, IMAGE_WIDTH, IMAGE_HEIGHT);
    create_points_2d(points_2d,points_normed_2d, points_3d);
    cv::Mat test_image = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0));

    cv::Mat r, rvec, tvec, D, inliers;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    //get the filtered keypoint vectors via inliers
    solvePnPRansac(points_3d, points_normed_2d, K, D, rvec, tvec, false, 100, 0.025, 0.99, inliers);
        
    std::vector<uchar> status;
    status.resize(points_2d.size(), 1);
    for( int i = 0; i < inliers.rows; i++)
    {
        int n = inliers.at<int>(i);
        status[n] = 1;
    }
    trim_vector(points_2d,status);
    trim_vector(points_3d,status);
    ROS_INFO("length of 3d is: %i",points_3d.size());
    ROS_INFO(" length of 2d is: %i", points_2d.size());

    std::vector<cv::Point2f> projected;
    projected.resize(points_3d.size());
    for(size_t i = 0; i<projected.size();i++){
        cv::Point2f point;
        point.x = points_3d[i].x;
        point.y = points_3d[i].y;
        projected[i] = point;
    }
    ROS_INFO("length of 3d projected is: %i",points_3d.size());
    ROS_INFO(" length of 2d is: %i", points_2d.size());

    publish_matches(&publisher,test_image,points_2d,projected,1,cv::Scalar(0,255,0),cv::Scalar(0,0,255));

    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}