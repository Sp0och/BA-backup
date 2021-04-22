#include "parameters.h"

template <typename Derived>

void trim_vector(vector<Derived> &v, vector<int>& status){
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
void create_projected_points_2d(std::vector<cv::Point2f>& normed_vector, std::vector<cv::Point3f>& input_vector_3d, int IW, int IH){
        cv::Point2d point_normed;
    for(size_t i = 0; i < input_vector_3d.size(); i++){
        point_normed.x = input_vector_3d[i].x / input_vector_3d[i].z;
        point_normed.y = input_vector_3d[i].y / input_vector_3d[i].z;
        if(i % 10 == 0){
        point_normed.x = rand()%IW/input_vector_3d[i].z;
        point_normed.y = rand()%IH/input_vector_3d[i].z;
        }
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
        cv::circle(color_image, cur_pt, circle_size, line_color, 2);
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
    // ros::Publisher publisher_projected = n.advertise<sensor_msgs::Image>("projected_filtered_keypoints", 1);
    ros::Rate loop_rate(1);

    while(ros::ok()){

    std::vector<cv::Point2f> points_projected_2d(100);
    std::vector<cv::Point3f> points_3d(100);
    create_points(points_3d, IMAGE_WIDTH, IMAGE_HEIGHT);
    create_projected_points_2d(points_projected_2d, points_3d,IMAGE_WIDTH,IMAGE_HEIGHT);
    cv::Mat test_image = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0));

    cv::Mat r, rvec, tvec, D, inliers;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    //get the filtered keypoint vectors via inliers
    solvePnPRansac(points_3d, points_projected_2d, K, D, rvec, tvec, false, 100, 0.025, 0.99, inliers);
    
    std::vector<int> status;
    status.resize(points_projected_2d.size(), 1);
    for( int i = 0; i < inliers.rows; i++)
    {
        status[i] = inliers.at<int>(i);
    }

    std::vector<cv::Point2f> points_projected_2d_c(100);
    for(int i = 0; i < 100; i++)
    points_projected_2d_c[i] = points_projected_2d[i];
    // std::vector<cv::Point2f> projected_c;
    trim_vector(points_projected_2d_c,status);
    trim_vector(points_3d,status);

    for(int i = 0; i < points_3d.size();i++){
      points_projected_2d[i].x*=200;
      points_projected_2d[i].y*=200;
    }

    publish_matches(&publisher,test_image,points_projected_2d,points_projected_2d_c,1,cv::Scalar(0,255,0),cv::Scalar(0,0,255));

    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}