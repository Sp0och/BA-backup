#include "image_and_descriptor/helper.h"


helper::helper(){}

//conversions and trimming: 

/**
 * store keypoints in Point2d vector
 * */
void helper::keypointTransition(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2d>& points_in)
{
    points_in.resize(keypoints_in.size());
    for(size_t i = 0; i < keypoints_in.size(); i++)
    {
        points_in[i] = keypoints_in[i].pt;
    }
}

/**
 * Trim Eigen Matrix according to flag vector
 * */
void helper::trim_matrix(Eigen::MatrixXd& m, vector<bool>& status){
    int j = 0;
    for (int i = 0; i < int(m.cols()); i++)
        if (status[i]){
            m(0,j) = m(0,i);
            m(1,j) = m(1,i);
            m(2,j++) = m(2,i);
        }
    m.conservativeResize(3,j);
}

//Acquisition of 3D information for the 2D key  points

//Filtering Functions:

/**
 * Apply RANSAC filtering to the point clouds using the find Homography method
 * */
void helper::RANSAC_filtering(std::vector<cv::Point2d>& sorted_2d_cur, std::vector<cv::Point2d>& sorted_2d_prev, Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD){
        cv::Mat MASK;
        cv::Mat H = cv::findHomography(sorted_2d_cur,sorted_2d_prev,cv::RANSAC,3.0,MASK);
        std::vector<bool> status(MASK.rows,0);
        for(int i = 0; i < MASK.rows;i++)
        status[i] = MASK.at<bool>(i);


        //reject the outliers
        helper::trimVector(sorted_2d_cur,status);  
        helper::trimVector(sorted_2d_prev,status);
        helper::trim_matrix(prev_SVD,status);
        helper::trim_matrix(cur_SVD,status);
}

/**
 * Filter out all 3D points whoose difference in a coordinate dirction is more than half its effective value as well as points that are too close to the origin
 * */
void helper::filtering_3D(Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD, vector<cv::Point2d>& cur,vector<cv::Point2d>& prev){
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

    helper::trim_matrix(prev_SVD,distance_flag);
    helper::trim_matrix(cur_SVD,distance_flag);
    helper::trimVector(cur,distance_flag);
    helper::trimVector(prev,distance_flag);
}

/**
 * Apply RANSAC filtering to the point clouds using the find Homography method
 * */
void helper::RANSAC_filtering_f(std::vector<cv::Point2f>& sorted_fd_cur, std::vector<cv::Point2f>& sorted_fd_prev, Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD){
        cv::Mat MASK;
        cv::Mat H = cv::findHomography(sorted_fd_cur,sorted_fd_prev,cv::RANSAC,3.0,MASK);
        std::vector<bool> status(MASK.rows,0);
        for(int i = 0; i < MASK.rows;i++)
        status[i] = MASK.at<bool>(i);


        //reject the outliers
        helper::trimVector(sorted_fd_cur,status);  
        helper::trimVector(sorted_fd_prev,status);
        helper::trim_matrix(prev_SVD,status);
        helper::trim_matrix(cur_SVD,status);
}

/**
 * Filter out all 3D points whoose difference in a coordinate dirction is more than half its effective value as well as points that are too close to the origin
 * */
void helper::filtering_3D_f(Eigen::MatrixXd& cur_SVD, Eigen::MatrixXd& prev_SVD, vector<cv::Point2f>& cur,vector<cv::Point2f>& prev){
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

    helper::trim_matrix(prev_SVD,distance_flag);
    helper::trim_matrix(cur_SVD,distance_flag);
    helper::trimVector(cur,distance_flag);
    helper::trimVector(prev,distance_flag);
}

//Visualization Functions:

/**
 * publish the keypoints as a pc topic
 * */
void helper::publish_3D_keypoints(const Eigen::MatrixXd& points,const  ros::Publisher* kp_pc_publisher, ros::Time raw_time){
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
void helper::publish_lines_3D(const Eigen::MatrixXd& cur_SVD,const Eigen::MatrixXd& prev_SVD,const  ros::Publisher* line_publisher, ros::Time raw_time){
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

