
#pragma once
#include "parameters.h"
#include "ORB.h"

using namespace Eigen;

template <typename Derived>

void trim_vector(vector<Derived> &v, vector<bool>& status){
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
void trim_matrix(MatrixXd& m, vector<bool>& status){
    int j = 0;
    for (int i = 0; i < int(m.cols()); i++)
        if (status[i]){
            m(0,j) = m(0,i);
            m(1,j) = m(1,i);
            m(2,j++) = m(2,i);
        }
    m.conservativeResize(3,j);
}
void RANSAC_filtering(std::vector<cv::Point2d>& sorted_2d_cur, std::vector<cv::Point2d>& sorted_2d_prev, MatrixXd& cur_ICP, MatrixXd& prev_ICP){
        cv::Mat MASK;
        cv::Mat H = cv::findHomography(sorted_2d_cur,sorted_2d_prev,cv::RANSAC,3.0,MASK);
        std::vector<bool> status(MASK.rows,0);
        for(int i = 0; i < MASK.rows;i++)
        status[i] = MASK.at<bool>(i);


        //reject the outliers
        trim_vector(sorted_2d_cur,status);  
        trim_vector(sorted_2d_prev,status);
        trim_matrix(prev_ICP,status);
        trim_matrix(cur_ICP,status);
}
void zero_filtering(MatrixXd& cur_ICP, MatrixXd& prev_ICP){
    std::vector<bool> zero_filtering(prev_ICP.cols(),1);
    for(int i = 0; i < prev_ICP.cols();i++){
        if((prev_ICP(0,i) == 0 && prev_ICP(1,i) == 0 && prev_ICP(2,i) == 0)||(cur_ICP(0,i) == 0 && cur_ICP(1,i) == 0 && cur_ICP(2,i) == 0)){
            zero_filtering.at(i) = 0;
        }
    }
    trim_matrix(prev_ICP,zero_filtering);
    trim_matrix(cur_ICP,zero_filtering);
}
/**
 * Filter out all points whoose difference in a coordinate dirction is more than half its effective value
 * */
void distance_filtering(MatrixXd& cur_ICP, MatrixXd& prev_ICP){
    std::vector<bool> distance_flag(prev_ICP.cols(),1);
    for(int i = 0; i < prev_ICP.cols();i++){
        float dist_x = fabs(cur_ICP(0,i)-prev_ICP(0,i));
        float dist_y = fabs(cur_ICP(1,i)-prev_ICP(1,i));
        float dist_z = fabs(cur_ICP(2,i)-prev_ICP(2,i));
        if((dist_x*100)/cur_ICP(0,i) > DISTANCE_THRESHOLD || (dist_y*100)/cur_ICP(1,i) > DISTANCE_THRESHOLD || (dist_z*100)/cur_ICP(2,i) > DISTANCE_THRESHOLD){
            distance_flag.at(i) = 0;
        }
    }
    trim_matrix(prev_ICP,distance_flag);
    trim_matrix(cur_ICP,distance_flag);
}
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



/**
 * Framehandler: creates matches between two consecutive frames, publishes these matches 
 * and uses them to indicate the points to use for ICP, also implements ICP
 */
class Framehandler{

    public:
    //Constructor
    Framehandler(int _mode){
        mode = _mode;
        comp_sum = comp_count = 0;
        coord_H = Vector4d(0,0,0,1);
        RT << 1,0,0,0,
              0,1,0,0,
              0,0,1,0,
              0,0,0,1;
        cur_orb = nullptr;
        prev_orb = nullptr;
        // match_publisher = n_frame.advertise<sensor_msgs::Image>("orb_matches", 1);

        kp_pc_publisher_cur = n_frame.advertise<PointCloud>("Keypoint_Pointcloud_cur", 1);
        kp_pc_publisher_prev = n_frame.advertise<PointCloud>("Keypoint_Pointcloud_prev", 1);

        odom_publisher = n_frame.advertise<nav_msgs::Odometry>("Odometry", 1);
        if(mode == 1)
        intensity_publisher = n_frame.advertise<sensor_msgs::Image>("intensity_matches", 1);
        else if(mode == 2)
        range_publisher = n_frame.advertise<sensor_msgs::Image>("range_matches", 1);
        else
        ambient_publisher = n_frame.advertise<sensor_msgs::Image>("ambient_matches", 1);
    }

    void newIteration(std::shared_ptr<ORB> new_frame){
        if(cur_orb == nullptr){
            cur_orb = new_frame;
        }
        else{
            prev_orb = cur_orb;
            cur_orb = new_frame;
            matches_filtering_motion();
        }
    }

    void matches_filtering_motion(){
        static cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        //create matches
        matcher->match(cur_orb->orb_descriptors,prev_orb->orb_descriptors,matches);
        //sort matches in ascending order (concerning distance)
        std::sort(matches.begin(),matches.end());
        std::vector<cv::Point2d> sorted_2d_cur, sorted_2d_prev;
        MatrixXd prev_ICP(3,matches.size()),cur_ICP(3,matches.size());
        std::cout << "cur keypoints size: " << cur_orb->orb_keypoints_2d.size() << " " << std::endl;
        std::cout << "prev keypoints size: " << prev_orb->orb_keypoints_2d.size() << " " << std::endl;
        std::cout << "match size: " << matches.size() << " " << std::endl;
        //pair the keypoints up according to the matches:
        for (size_t i = 0; i < matches.size(); i++)
        {
            //the match indexes for point association
            int cur_index = matches[i].queryIdx;
            int prev_index = matches[i].trainIdx;
            //create sorted keypoint vectors
            sorted_2d_cur.push_back(cur_orb->orb_keypoints_2d[cur_index]);
            sorted_2d_prev.push_back(prev_orb->orb_keypoints_2d[prev_index]);
            //create sorted 3d keypoint vectors
            cur_ICP(0,i) = cur_orb->orb_points_3d.at(cur_index).x;
            cur_ICP(1,i) = cur_orb->orb_points_3d.at(cur_index).y;
            cur_ICP(2,i) = cur_orb->orb_points_3d.at(cur_index).z;
            prev_ICP(0,i) = prev_orb->orb_points_3d.at(prev_index).x;
            prev_ICP(1,i) = prev_orb->orb_points_3d.at(prev_index).y;
            prev_ICP(2,i) = prev_orb->orb_points_3d.at(prev_index).z;
        }
        matches.clear();
        //Before for the feature quality comparison:
        // unsigned int before = sorted_2d_cur.size();

        //Publish matches before RANSAC filtering:
        // publish_matches(&match_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        
        //Homography RANSAC
        RANSAC_filtering(sorted_2d_cur,sorted_2d_prev,cur_ICP,prev_ICP);
        // std::cout << "size after ransac: " << sorted_2d_cur.size() << " " << std::endl;
        
        // //Second filtering for edge values if values beneath a certain range norm were neglected in the image handler
        // zero_filtering(cur_ICP,prev_ICP);
        
        //Filter out 3D mistakes which look right on 2D:
        distance_filtering(cur_ICP,prev_ICP);
        // std::cout << "size after distance: " << cur_ICP.cols() << " " << std::endl;
        

        //Visualize key point point cloud:
        publish_keypoint_pc(cur_ICP, &kp_pc_publisher_cur);
        publish_keypoint_pc(prev_ICP, &kp_pc_publisher_prev);

        //Feature quality comparison:

        // unsigned int after = sorted_2d_cur.size();
        // comp_sum += (100*after)/before;
        // comp_count++;
        // if(comp_count == 1000){
        //     std::cout << "True positive rate is: " << comp_sum/comp_count << "  " << std::endl;
        //     comp_sum = comp_count = 0;
        // }


        //ICP Here
        if(cur_ICP.size() == prev_ICP.size() && cur_ICP.size() != 0)
            ICP(cur_ICP,prev_ICP);
        else    
            std::cout << "ERROR: 3D Vectors weren't initialized properly" << std::endl;

        //First match display option
        
        if(mode == 1)
        publish_matches_2F(&intensity_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        else if(mode == 2)
        publish_matches_2F(&range_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        else
        publish_matches_2F(&ambient_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        
        //Second match display option

        // if(mode == 1)
        // publish_matches_1F(&intensity_publisher, sorted_2d_cur, sorted_2d_prev,5,true);
        // else if(mode == 2)
        // publish_matches_1F(&range_publisher, sorted_2d_cur, sorted_2d_prev,5,true);
        // else
        // publish_matches_1F(&ambient_publisher, sorted_2d_cur, sorted_2d_prev,5,true);
    
    }


    void publish_odom(Matrix4d& RT){
        nav_msgs::Odometry odom;
        ros::Time current_time, last_time;
        current_time = last_time = ros::Time::now();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        geometry_msgs::Quaternion odom_quat;
        geometry_msgs::PoseWithCovariance odom_pose_w;
        geometry_msgs::Pose odom_pose;
        cv::Point3d pt = cv::Point3d(RT(0,3),RT(1,3),RT(2,3)); 
        odom_pose.position.x = pt.x;
        odom_pose.position.y = pt.y;
        odom_pose.position.z = pt.z;
        odom_pose_w.pose = odom_pose;
        odom.pose = odom_pose_w;
        odom_publisher.publish(odom);
    }

    void publish_keypoint_pc(MatrixXd& cur_ICP, ros::Publisher* kp_pc_publisher){
        PointCloud::Ptr msg (new PointCloud);
        msg->header.frame_id = "velodyne";
        msg->width = cur_ICP.cols();
        msg->height = 1;
        for(size_t i = 0; i < cur_ICP.cols();i++){
            msg->points.push_back(pcl::PointXYZ(cur_ICP(0,i),cur_ICP(1,i),cur_ICP(2,i)));
        }
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        kp_pc_publisher->publish(msg);
    }

    void publish_matches_2F(const ros::Publisher* this_pub, std::vector<cv::Point2d>& sorted_KP_cur, 
    std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, cv::Scalar line_color, bool draw_lines){
    cv::Mat color_img,gray_img;
    const cv::Mat old_img = prev_orb->input_image.clone();
    const cv::Mat new_img = cur_orb->input_image.clone();
    int gap = 1;
    cv::Mat gap_img(gap, new_img.size().width, CV_8UC1, cv::Scalar(255, 255, 255));
    //create colored concatenated image with gap inbetween
    cv::vconcat(new_img,gap_img,gap_img);
    cv::vconcat(gap_img,old_img,gray_img);

    cv::cvtColor(gray_img,color_img,CV_GRAY2RGB);
    //indicate features in new image
    for(int i = 0; i< (int)sorted_KP_cur.size(); i++)
    {
        cv::Point2d cur_pt = sorted_KP_cur[i] * MATCH_IMAGE_SCALE;
        cv::circle(color_img, cur_pt, circle_size*MATCH_IMAGE_SCALE, line_color, MATCH_IMAGE_SCALE*2);
    }
    //indicate features in old image
    for(int i = 0; i< (int)sorted_KP_prev.size(); i++)
    {
        cv::Point2d old_pt = sorted_KP_prev[i] * MATCH_IMAGE_SCALE;
        old_pt.y += new_img.size().height + gap;
        cv::circle(color_img, old_pt, circle_size*MATCH_IMAGE_SCALE, line_color, MATCH_IMAGE_SCALE*2);
    }

    if(draw_lines){
        for (int i = 0; i< (int)sorted_KP_cur.size(); i++)
        {
            cv::Point2d old_pt = sorted_KP_prev[i] * MATCH_IMAGE_SCALE;
            old_pt.y += new_img.size().height + gap;
            cv::line(color_img, sorted_KP_cur[i] * MATCH_IMAGE_SCALE, old_pt, line_color, MATCH_IMAGE_SCALE*2, 8, 0);
        }
    }
    if(mode == 1)
    cv::putText(color_img, "Intensity",   cv::Point2d(300, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else if (mode == 2)
    cv::putText(color_img, "Range",   cv::Point2d(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else
    cv::putText(color_img, "Ambient",   cv::Point2d(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_img).toImageMsg();
    this_pub->publish(msg);

}

    void publish_matches_1F(const ros::Publisher* this_pub, std::vector<cv::Point2d>& sorted_KP_cur, 
    std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, bool draw_lines){
        cv::Mat image = cur_orb->input_image.clone();
        cv::cvtColor(image,image,CV_GRAY2RGB);

        for (int i = 0; i< (int)sorted_KP_cur.size(); i++)
        {
            cv::Point2d old_pt = sorted_KP_prev[i] * MATCH_IMAGE_SCALE;
            cv::Point2d cur_pt = sorted_KP_cur[i] * MATCH_IMAGE_SCALE;
            unsigned int r = rand() % 256;
            unsigned int g = rand() % 256;
            unsigned int b = rand() % 256;
            cv::Scalar circle_col = cv::Scalar(r,g,b);
            cv::circle(image, cur_pt, circle_size*MATCH_IMAGE_SCALE, circle_col, MATCH_IMAGE_SCALE*1);
            cv::line(image, cur_pt * MATCH_IMAGE_SCALE, old_pt, cv::Scalar(255,0,0), MATCH_IMAGE_SCALE*1, 8, 0);
        }
        if(mode == 1)
        cv::putText(image, "Intensity",   cv::Point2d(300, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        else if (mode == 2)
        cv::putText(image, "Range",   cv::Point2d(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        else
        cv::putText(image, "Ambient",   cv::Point2d(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        this_pub->publish(msg);
    }

    void ICP(MatrixXd& cur_ICP,MatrixXd& prev_ICP){
        Vector3d sum_prev(0,0,0);
        Vector3d sum_cur(0,0,0);

        for(int i = 0;i < prev_ICP.cols();i++){
            sum_prev(0) += prev_ICP(0,i);
            sum_prev(1) += prev_ICP(1,i);
            sum_prev(2) += prev_ICP(2,i);
            sum_cur(0) += cur_ICP(0,i);
            sum_cur(1) += cur_ICP(1,i);
            sum_cur(2) += cur_ICP(2,i);
        }
        //create the mean for subtraction
        Vector3d mean_prev = sum_prev/prev_ICP.cols();
        Vector3d mean_cur = sum_cur/cur_ICP.cols();  
        //Subtract center of "mass"
        for(int i = 0; i < prev_ICP.cols();i++){
            prev_ICP(0,i) -= mean_prev(0);
            prev_ICP(1,i) -= mean_prev(1);
            prev_ICP(2,i) -= mean_prev(2);
            cur_ICP(0,i) -= mean_cur(0);
            cur_ICP(1,i) -= mean_cur(1);
            cur_ICP(2,i) -= mean_cur(2);
        }

        MatrixXd W;
        //W is of rank 3, that I checked.
        W = prev_ICP*cur_ICP.transpose();
        // std::cout << "W:  " << std::endl << W << std::endl;
        JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
        auto VT = svd.matrixV().transpose();
        MatrixXd R = svd.matrixU()*VT;
        MatrixXd t = mean_prev - R*mean_cur;
        Matrix4d H;
        H << R,t,0,0,0,1;
        RT = H*RT;
        // std::cout << "The rotation matrix is: " << std::endl << R << std::endl;
        // std::cout << "The translation vector is: " << std::endl << t << std::endl;
        // coord_H = RT*coord_H;
        // std::cout << "The current coordinates are: " << std::endl << RT << std::endl;
        //Call Odom publisher
        publish_odom(RT);
    }
    
    private:

    std::shared_ptr<ORB> cur_orb, prev_orb;
    vector<cv::DMatch> matches; 
    int mode;
    unsigned int comp_sum;
    unsigned int comp_count;

    Vector4d coord_H;
    Matrix4d RT;

    ros::NodeHandle n_frame;
    ros::Publisher match_publisher, range_publisher, ambient_publisher, intensity_publisher, 
    kp_pc_publisher_cur, kp_pc_publisher_prev, odom_publisher;
    


};