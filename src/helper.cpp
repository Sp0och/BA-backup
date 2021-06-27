#pragma once
#include "../include/parameters.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


//conversions and trimming: 

/**
 * store keypoints in Point2d vector
 * */
static void keypointTransition(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2d>& points_in)
{
    points_in.resize(keypoints_in.size());
    for(size_t i = 0; i < keypoints_in.size(); i++)
    {
        points_in[i] = keypoints_in[i].pt;
    }
}

template <typename Derived>
/**
 * Trim a 2D Vector according to a flag vector status
 * */
static void trimVector(vector<Derived>& v,const vector<bool>& status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++){
        if (status[i])
            v[j++] = v[i];
    }
    v.resize(j);
}


/**
 * Trim Eigen Matrix according to flag vector
 * */
static void trim_matrix(Eigen::MatrixXd& m, vector<bool>& status){
    int j = 0;
    for (int i = 0; i < int(m.cols()); i++)
        if (status[i]){
            m(0,j) = m(0,i);
            m(1,j) = m(1,i);
            m(2,j++) = m(2,i);
        }
    m.conservativeResize(3,j);
}


template <typename Derived1>
/**
 * Get the 3d coordinates of the keypoints - used in KLT
 * */
static void get_3D_points(vector<Derived1>& points_2D, Eigen::MatrixXd& points_3D, const pcl::PointCloud<PointType>::Ptr PC){
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
    trimVector(points_2D,status);
    trim_matrix(points_3D,status);
}

template <typename Derived2>
/**
 * Get the 3d coordinates of the keypoints - used in KLT
 * */
static void get_3D_points_adapt_status(vector<Derived2>& points_2D, Eigen::MatrixXd& points_3D, const pcl::PointCloud<PointType>::Ptr PC,std::vector<bool>& status){
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
    trimVector(points_2D,status);
    trim_matrix(points_3D,status);
}


//Filtering Functions:
/**
 * Apply RANSAC filtering to the point clouds using the find Homography method
 * */
static void RANSAC_filtering(std::vector<cv::Point2d>& sorted_2d_cur, std::vector<cv::Point2d>& sorted_2d_prev, Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD){
        cv::Mat MASK;
        cv::Mat H = cv::findHomography(sorted_2d_cur,sorted_2d_prev,cv::RANSAC,3.0,MASK);
        std::vector<bool> status(MASK.rows,0);
        for(int i = 0; i < MASK.rows;i++)
        status[i] = MASK.at<bool>(i);


        //reject the outliers
        trimVector(sorted_2d_cur,status);  
        trimVector(sorted_2d_prev,status);
        trim_matrix(prev_SVD,status);
        trim_matrix(cur_SVD,status);
}


/**
 * Filter out all 3D points whoose difference in a coordinate dirction is more than half its effective value as well as points that are too close to the origin
 * */
static void filtering_3D(Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD, vector<cv::Point2d>& cur,vector<cv::Point2d>& prev){
    vector<bool> distance_flag(prev_SVD.cols(),1);
    // for(int i = 0; i < prev_SVD.cols();i++){
    //     float p_c_x = cur_SVD(0,i);
    //     float p_c_y = cur_SVD(1,i);
    //     float p_c_z = cur_SVD(2,i);
    //     float p_p_x = prev_SVD(0,i);
    //     float p_p_y = prev_SVD(1,i);
    //     float p_p_z = prev_SVD(2,i);
    //     float dist_x = p_c_x - p_p_x;
    //     float dist_y = p_c_y - p_p_y;
    //     float dist_z = p_c_z - p_p_z;

    //     float mdif = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
    //     float dist_c = sqrt(p_c_x*p_c_x + p_c_y*p_c_y + p_c_z*p_c_z);
    //     float dist_p = sqrt(p_p_x*p_p_x + p_p_y*p_p_y + p_p_z*p_p_z);

    //     if(mdif > MAX_MATCH_DISTANCE || dist_c < MIN_KP_DISTANCE || dist_p < MIN_KP_DISTANCE)
    //         distance_flag.at(i) = 0;
    // }
    //vector product attempt:
    for(int i = 0; i < prev_SVD.cols();i++){
        float p_c_x = cur_SVD(0,i);
        float p_c_y = cur_SVD(1,i);
        float p_c_z = cur_SVD(2,i);
        float p_p_x = prev_SVD(0,i);
        float p_p_y = prev_SVD(1,i);
        float p_p_z = prev_SVD(2,i);
        float dist_x = p_c_x - p_p_x;
        float dist_y = p_c_y - p_p_y;
        float dist_z = p_c_z - p_p_z;

        Eigen::Vector3d match_vector(dist_x,dist_y,dist_z);
        Eigen::Vector3d O_cur(p_c_x,p_c_y,p_c_z);
        Eigen::Vector3d O_prev(p_p_x,p_p_y,p_p_z);
        double dot = fabs(match_vector.dot(O_cur));
        double depth_distance = dot/O_cur.norm();

        float mdif = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
        float dist_c = sqrt(p_c_x*p_c_x + p_c_y*p_c_y + p_c_z*p_c_z);
        float dist_p = sqrt(p_p_x*p_p_x + p_p_y*p_p_y + p_p_z*p_p_z);

        // if((costheta > MAX_COS && mdif > MAX_MATCH_DISTANCE) || dist_c < MIN_KP_DISTANCE || dist_p < MIN_KP_DISTANCE)
        //     distance_flag.at(i) = 0;
        // if((costheta * mdif > MAX_MATCH_DISTANCE) || dist_c < MIN_KP_DISTANCE || dist_p < MIN_KP_DISTANCE)
        //     distance_flag.at(i) = 0;
        if(depth_distance > MAX_DEPTH_DISTANCE)
            distance_flag.at(i) = 0;
    }

    trim_matrix(prev_SVD,distance_flag);
    trim_matrix(cur_SVD,distance_flag);
    trimVector(cur,distance_flag);
    trimVector(prev,distance_flag);
}




/**
 * Apply RANSAC filtering to the point clouds using the find Homography method
 * */
static void RANSAC_filtering_f(std::vector<cv::Point2f>& sorted_fd_cur, std::vector<cv::Point2f>& sorted_fd_prev, Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD){
        cv::Mat MASK;
        cv::Mat H = cv::findHomography(sorted_fd_cur,sorted_fd_prev,cv::RANSAC,3.0,MASK);
        std::vector<bool> status(MASK.rows,0);
        for(int i = 0; i < MASK.rows;i++)
        status[i] = MASK.at<bool>(i);


        //reject the outliers
        trimVector(sorted_fd_cur,status);  
        trimVector(sorted_fd_prev,status);
        trim_matrix(prev_SVD,status);
        trim_matrix(cur_SVD,status);
}

/**
 * Filter out all 3D points whoose difference in a coordinate dirction is more than half its effective value as well as points that are too close to the origin
 * */
static void filtering_3D_f(Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD, vector<cv::Point2f>& cur,vector<cv::Point2f>& prev){
    vector<bool> distance_flag(prev_SVD.cols(),1);
    for(int i = 0; i < prev_SVD.cols();i++){
        float p_c_x = cur_SVD(0,i);
        float p_c_y = cur_SVD(1,i);
        float p_c_z = cur_SVD(2,i);
        float p_p_x = prev_SVD(0,i);
        float p_p_y = prev_SVD(1,i);
        float p_p_z = prev_SVD(2,i);
        float dist_x = p_c_x - p_p_x;
        float dist_y = p_c_y - p_p_y;
        float dist_z = p_c_z - p_p_z;

        Eigen::Vector3d match_vector(dist_x,dist_y,dist_z);
        Eigen::Vector3d O_cur(p_c_x,p_c_y,p_c_z);
        Eigen::Vector3d O_prev(p_p_x,p_p_y,p_p_z);
        double dot = fabs(match_vector.dot(O_cur));
        double depth_distance = dot/O_cur.norm();

        float mdif = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
        float dist_c = sqrt(p_c_x*p_c_x + p_c_y*p_c_y + p_c_z*p_c_z);
        float dist_p = sqrt(p_p_x*p_p_x + p_p_y*p_p_y + p_p_z*p_p_z);

        // if((costheta > MAX_COS && mdif > MAX_MATCH_DISTANCE) || dist_c < MIN_KP_DISTANCE || dist_p < MIN_KP_DISTANCE)
        //     distance_flag.at(i) = 0;
        // if((costheta * mdif > MAX_MATCH_DISTANCE) || dist_c < MIN_KP_DISTANCE || dist_p < MIN_KP_DISTANCE)
        //     distance_flag.at(i) = 0;
        if(depth_distance > MAX_DEPTH_DISTANCE)
            distance_flag.at(i) = 0;
    }

    trim_matrix(prev_SVD,distance_flag);
    trim_matrix(cur_SVD,distance_flag);
    trimVector(cur,distance_flag);
    trimVector(prev,distance_flag);
}

/**
 * Filter out duplicate points
 * */
static void duplicate_point_filtering(vector<cv::Point2d>& point){
    std::vector<bool> duplicate_status;
    cv::Mat dubplicate_mask_p = cv::Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1,cv::Scalar(0));
    for(int i = 0; i < point.size();i++){
        cv::Point2d pt_p = point.at(i);
        cv::Scalar color_p = dubplicate_mask_p.at<uchar>(pt_p);
        //check whether a point has already been used if not draw there to indicate usage
        if(color_p == cv::Scalar(0)){
            cv::circle(dubplicate_mask_p,pt_p,0,cv::Scalar(255),DUPLICATE_FILTERING_SIZE);
            duplicate_status.push_back(1);
        }   
        //If one of the points has already been used delete the whole match
        else{
            duplicate_status.push_back(0);
        }
    }
    trimVector(point,duplicate_status);
}

/**
 * Filter out duplicate points (FLOATs)
 * */
static void duplicate_point_filtering_f(vector<cv::Point2f>& point, Eigen::MatrixXd& points_3D){
    std::vector<bool> duplicate_status;
    cv::Mat dubplicate_mask_p = cv::Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1,cv::Scalar(0));
    for(int i = 0; i < point.size();i++){
        cv::Point2d pt_p = point.at(i);
        cv::Scalar color_p = dubplicate_mask_p.at<uchar>(pt_p);
        //check whether a point has already been used if not draw there to indicate usage
        if(color_p == cv::Scalar(0)){
            cv::circle(dubplicate_mask_p,pt_p,0,cv::Scalar(255),DUPLICATE_FILTERING_SIZE);
            duplicate_status.push_back(1);
        }   
        //If one of the points has already been used delete the whole match
        else{
            duplicate_status.push_back(0);
        }
    }
    trimVector(point,duplicate_status);
    trim_matrix(points_3D,duplicate_status);
}

//Visualization Functions:

template <typename Keypoints>
/**
 * Visualizing the 2D keypoints
 * */
static void publish_keypoints (ros::Publisher* publisher, const cv::Mat& input_image, const vector<Keypoints>& keypoints, const int circle_size,const cv::Scalar line_color, int image_source){
    cv::Mat color_image;
    cv::cvtColor(input_image, color_image, CV_GRAY2RGB);
    for(int i = 0; i < (int)keypoints.size(); i++){
        cv::Point2d cur_pt = keypoints[i];
        cv::circle(color_image,cur_pt,circle_size,line_color,2);
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

/**
 * publish the keypoints as a pc topic
 * */
static void publish_3D_keypoints(const Eigen::MatrixXd& points,const  ros::Publisher* kp_pc_publisher, ros::Time raw_time){
        PointCloud::Ptr msg (new PointCloud);
        msg->header.frame_id = "velodyne";
        msg->width = points.cols();
        msg->height = 1;
        for(size_t i = 0; i < points.cols();i++){
            msg->points.push_back(pcl::PointXYZ(points(0,i),points(1,i),points(2,i)));
        }
        pcl_conversions::toPCL(raw_time, msg->header.stamp);
        kp_pc_publisher->publish(msg);
    }

/**
 * publish 3D connection lines between the keypoint PCs
 * */
static void publish_lines_3D(const Eigen::MatrixXd& cur_SVD,const Eigen::MatrixXd& prev_SVD,const  ros::Publisher* line_publisher, ros::Time raw_time){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "velodyne";
    line_list.header.stamp = raw_time;
    line_list.ns = "connection";
    line_list.id = 1;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;

    for(int i = 0; i < cur_SVD.cols(); i ++){
        geometry_msgs::Point p;
        p.x = cur_SVD(0,i);
        p.y = cur_SVD(1,i);
        p.z = cur_SVD(2,i);
        line_list.points.push_back(p);
        p.x = prev_SVD(0,i);
        p.y = prev_SVD(1,i);
        p.z = prev_SVD(2,i);
        line_list.points.push_back(p);

    }
    line_list.scale.x = 0.01;
    line_list.scale.y = 0.01;
    line_list.scale.z = 0.01;
    line_list.color.g = 1.0f;
    line_list.color.a = 1.0f;

    // line_list.lifetime = ros::Duration();

    line_publisher->publish(line_list);
}