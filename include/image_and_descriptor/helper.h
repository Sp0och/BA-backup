#pragma once
#include "setup.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  

class helper{

    public:

    helper();

    //Conversion and Trimming

    void keypointTransition(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2d>& points_in);

    template <typename Derived>

    /**
     * Trim a 2D Vector according to a flag vector status
     * */
    void trimVector(vector<Derived>& v,const vector<bool>& status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++){
            if (status[i])
                v[j++] = v[i];
        }
        v.resize(j);
    }

    void trim_matrix(Eigen::MatrixXd& m, vector<bool>& status);

    template <typename Derived1>
    /**
     * Get the 3d coordinates of the keypoints - used in KLT
     * */
    void get_3D_points(vector<Derived1>& points_2D, Eigen::MatrixXd& points_3D, const pcl::PointCloud<PointType>::Ptr PC){
        points_3D.resize(3,points_2D.size());
        std::vector<bool>status(points_2D.size(),1);
        for(size_t i = 0; i < points_2D.size(); i++){
            int row_index = cvRound(points_2D[i].y);
            int col_index = cvRound(points_2D[i].x);
            if(row_index > 127)
            row_index = 127;
            if(row_index < 0)
            row_index = 0;
            int index = row_index*IMAGE_WIDTH + col_index;
            PointType *pi = &PC->points[index];
            if(pi->x == pi->y && pi->y == pi->z && pi->z == 0)
                status.at(i) = 0;
            points_3D(0,i) = pi->x;
            points_3D(1,i) = pi->y;
            points_3D(2,i) = pi->z;
        }
        helper::trimVector(points_2D,status);
        helper::trim_matrix(points_3D,status);
    }

    template <typename Derived2>
    /**
     * Get the 3d coordinates of the keypoints - used in KLT
     * */
    void get_3D_points_adapt_status(vector<Derived2>& points_2D, Eigen::MatrixXd& points_3D, const pcl::PointCloud<PointType>::Ptr PC,std::vector<bool>& status){
        points_3D.resize(3,points_2D.size());
        for(size_t i = 0; i < points_2D.size(); i++){
            unsigned int row_index = cvRound(points_2D.at(i).y);
            unsigned int col_index = cvRound(points_2D.at(i).x);
            if(row_index > 127)
            row_index = 127;
            if(row_index < 0)
            row_index = 0;
            unsigned int index = row_index*IMAGE_WIDTH + col_index;
            const PointType *pi = &PC->points[index];
            if(pi->x == pi->y && pi->y == pi->z && pi->z == 0)
                status.at(i) = 0;
            points_3D(0,i) = pi->x;
            points_3D(1,i) = pi->y;
            points_3D(2,i) = pi->z;
        }
        helper::trimVector(points_2D,status);
        helper::trim_matrix(points_3D,status);
    }
    
    //Filtering

    void RANSAC_filtering(std::vector<cv::Point2d>& sorted_2d_cur, std::vector<cv::Point2d>& sorted_2d_prev, Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD);

    void filtering_3D(Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD, vector<cv::Point2d>& cur,vector<cv::Point2d>& prev);

    void RANSAC_filtering_f(std::vector<cv::Point2f>& sorted_fd_cur, std::vector<cv::Point2f>& sorted_fd_prev, Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD);

    void filtering_3D_f(Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD, vector<cv::Point2f>& cur,vector<cv::Point2f>& prev);

    //Visualization Functions

    template <typename Keypoints>
    /**
     * Visualizing the 2D keypoints
     * */
    void publish_keypoints (ros::Publisher* publisher, const cv::Mat& input_image, const vector<Keypoints>& keypoints, const int circle_size,const cv::Scalar point_color, int image_source){
        cv::Mat color_image;
        cv::cvtColor(input_image, color_image, CV_GRAY2RGB);
        for(int i = 0; i < (int)keypoints.size(); i++){
            cv::Point2d cur_pt = keypoints[i];
            cv::circle(color_image,cur_pt,circle_size,point_color,2);
        }
        if(image_source == 1)
        cv::putText(color_image, "Intensity",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        else if (image_source == 2)
        cv::putText(color_image, "Range",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        else
        cv::putText(color_image, "Ambient",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
        publisher->publish(msg);
    }

    void publish_3D_keypoints(const Eigen::MatrixXd& points,const  ros::Publisher* kp_pc_publisher, ros::Time raw_time);

    void publish_lines_3D(const Eigen::MatrixXd& cur_SVD,const Eigen::MatrixXd& prev_SVD,const  ros::Publisher* line_publisher, ros::Time raw_time);

};