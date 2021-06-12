
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


/**
 * Filter out all points whoose difference in a coordinate dirction is more than half its effective value as well as points that are too close to the origin
 * */
void distance_filtering(MatrixXd& cur_ICP, MatrixXd& prev_ICP){
    std::vector<bool> distance_flag(prev_ICP.cols(),1);
    for(int i = 0; i < prev_ICP.cols();i++){
        float p_c_x = cur_ICP(0,i);
        float p_c_y = cur_ICP(1,i);
        float p_c_z = cur_ICP(2,i);
        float p_p_x = prev_ICP(0,i);
        float p_p_y = prev_ICP(1,i);
        float p_p_z = prev_ICP(2,i);
        float dist_x = p_c_x - p_p_x;
        float dist_y = p_c_y - p_p_y;
        float dist_z = p_c_z - p_p_z;

        float mdif = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
        float dist_c = sqrt(p_c_x*p_c_x + p_c_y*p_c_y + p_c_z*p_c_z);
        float dist_p = sqrt(p_p_x*p_p_x + p_p_y*p_p_y + p_p_z*p_p_z);

        if(mdif > MAX_FEATURE_DISTANCE || dist_c < MIN_FEATURE_DISTANCE || dist_p < MIN_FEATURE_DISTANCE)
            distance_flag.at(i) = 0;
    }
    trim_matrix(prev_ICP,distance_flag);
    trim_matrix(cur_ICP,distance_flag);
}
static void double_point_filtering(vector<cv::Point2d>& cur, vector<cv::Point2d>& prev,MatrixXd& curM, MatrixXd& prevM){
    std::vector<bool> duplicate_status;
    cv::Mat dubplicate_mask_c = cv::Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1,cv::Scalar(0));
    cv::Mat dubplicate_mask_p = cv::Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1,cv::Scalar(0));
    for(int i = 0; i < cur.size();i++){
        cv::Point2d pt_c = cur.at(i);
        cv::Point2d pt_p = prev.at(i);
        cv::Scalar color_c = dubplicate_mask_c.at<uchar>(pt_c);
        cv::Scalar color_p = dubplicate_mask_p.at<uchar>(pt_p);
        //check whether a point has already been used if not draw there to indicate usage
        if(color_c == cv::Scalar(0) && color_p == cv::Scalar(0)){
            cv::circle(dubplicate_mask_c,pt_c,0,cv::Scalar(255),1);
            cv::circle(dubplicate_mask_p,pt_p,0,cv::Scalar(255),1);
            duplicate_status.push_back(1);
        }   
        //If one of the points has already been used delete the whole match
        else{
            duplicate_status.push_back(0);
        }
    }
    trim_vector(cur,duplicate_status);
    trim_vector(prev,duplicate_status);
    trim_matrix(curM,duplicate_status);
    trim_matrix(prevM,duplicate_status);
}


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



/**
 * Framehandler: creates matches between two consecutive frames, publishes these matches 
 * and uses them to indicate the points to use for ICP, also implements ICP
 */
class Framehandler{

    public:
    //Constructor
    Framehandler(int _mode,bool START_AT_ZERO){
        OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/OLD_prediction_complete.csv",ios_base::app);
        OUT << "x" << "," << "y" << "," << "z" << "," << "roll"<< "," << "pitch"<< "," << "yaw" << "," << "time" << endl;
        OUT.close(); 
        mode = _mode;
        comp_sum = comp_count = 0;
        if(START_AT_ZERO)
            RT << 1,0,0,0,
              0,1,0,0,
              0,0,1,0,
              0,0,0,1;
        else{
            RT << 0.35182179, -0.92905796,  0.11433606,  0.0,
            0.93259485,  0.33738143, -0.12822098,  0.0,
            0.08054986,  0.15174015,  0.98513281,  0.0,
            0,           0,           0,            1;
        }
        //Their start position
        // RT << 0.35182179, -0.92905796,  0.11433606,  0.0,
        // 0.93259485,  0.33738143, -0.12822098,  0.0,
        // 0.08054986,  0.15174015,  0.98513281,  0.0,
        // 0,           0,           0,            1;
        cur_orb = nullptr;
        prev_orb = nullptr;
        // match_publisher = n_frame.advertise<sensor_msgs::Image>("orb_matches", 1);

        kp_pc_publisher_cur = n_frame.advertise<PointCloud>("Keypoint_Pointcloud_cur", 1);
        kp_pc_publisher_prev = n_frame.advertise<PointCloud>("Keypoint_Pointcloud_prev", 1);
        midpoint_publisher = n_frame.advertise<PointCloud>("KP_PC_Midpoints", 1);
        // raw_sub = n_frame.subscribe(CLOUD_TOPIC,1000,&Framehandler::publish_tf);
        odom_publisher = n_frame.advertise<nav_msgs::Odometry>("Odometry", 1);
        if(mode == 1)
        intensity_publisher = n_frame.advertise<sensor_msgs::Image>("intensity_matches", 1);
        else if(mode == 2)
        range_publisher = n_frame.advertise<sensor_msgs::Image>("range_matches", 1);
        else
        ambient_publisher = n_frame.advertise<sensor_msgs::Image>("ambient_matches", 1);
    }

    void newIteration(std::shared_ptr<ORB> new_frame, ros::Time _raw_time){
        raw_time = _raw_time;
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
        //choose correct order
        bool cur_first = false;
        if(cur_orb->orb_keypoints_2d.size() < prev_orb->orb_keypoints_2d.size()){
            cur_first = true;
            matcher->match(cur_orb->orb_descriptors,prev_orb->orb_descriptors,matches);
        }
        else
            matcher->match(prev_orb->orb_descriptors,cur_orb->orb_descriptors,matches);

        //sort matches in ascending order (concerning distance)
        std::sort(matches.begin(),matches.end());
        std::vector<cv::Point2d> sorted_2d_cur, sorted_2d_prev;
        MatrixXd prev_ICP(3,matches.size()),cur_ICP(3,matches.size());
        // std::cout << "cur keypoints size right before matching: " << cur_orb->orb_keypoints_2d.size() << " " << std::endl;
        // std::cout << "prev keypoints size right before matching: " << prev_orb->orb_keypoints_2d.size() << " " << std::endl;
        // std::cout << "match size: " << matches.size() << " " << std::endl;
        //pair the keypoints up according to the matches:
        for (size_t i = 0; i < matches.size(); i++)
        {
            //the match indexes for point association
            int cur_index;
            int prev_index;
            if(cur_first){
                cur_index = matches[i].queryIdx;
                prev_index = matches[i].trainIdx;
            }
            else{
                cur_index = matches[i].trainIdx;
                prev_index = matches[i].queryIdx;
            }
            
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



        //Publish matches before RANSAC filtering:
        // publish_matches(&match_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        
        //Homography RANSAC
        RANSAC_filtering(sorted_2d_cur,sorted_2d_prev,cur_ICP,prev_ICP);
        // std::cout << "size after ransac: " << sorted_2d_cur.size() << " " << std::endl;
        

        // //Second filtering for edge values if values beneath a certain range norm were neglected in the image handler
        double_point_filtering(sorted_2d_cur,sorted_2d_prev,cur_ICP,prev_ICP);

        //Filter out 3D mistakes which look right on 2D:
        distance_filtering(cur_ICP,prev_ICP);
        // std::cout << "size after distance: " << cur_ICP.cols() << " " << std::endl;
        
        // std::cout << "cur keypoints cols after distance filtering: " << cur_ICP.cols() << " " << std::endl;
        // std::cout << "prev keypoints cols after distance filtering: " << prev_ICP.cols() << " " << std::endl;
        // std::cout << "cur keypoints rows after distance filtering: " << cur_ICP.rows() << " " << std::endl;
        // std::cout << "prev keypoints rows after distance filtering: " << prev_ICP.rows() << " " << std::endl;
       
        //Visualize key point point cloud:
        MatrixXd mid_points = get_midpoints(cur_ICP,prev_ICP);
        publish_keypoint_pc(cur_ICP, &kp_pc_publisher_cur);
        publish_keypoint_pc(prev_ICP, &kp_pc_publisher_prev);
        publish_keypoint_pc(mid_points, &midpoint_publisher);

        //ICP Here
        if(cur_ICP.size() == prev_ICP.size() && cur_ICP.size() != 0)
            ICP(cur_ICP,prev_ICP);
        else    
            std::cout << "ERROR: 3D Vectors weren't initialized properly" << std::endl;

        //publish my estimated transform in between odom and velodyne
        publish_tf();


        //publish odometry message
        // publish_odom();

        //First 2D match display option
        
        if(mode == 1)
        publish_matches_2F(&intensity_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        else if(mode == 2)
        publish_matches_2F(&range_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        else
        publish_matches_2F(&ambient_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        
        //Second 2D match display option

        // if(mode == 1)
        // publish_matches_1F(&intensity_publisher, sorted_2d_cur, sorted_2d_prev,5,true);
        // else if(mode == 2)
        // publish_matches_1F(&range_publisher, sorted_2d_cur, sorted_2d_prev,5,true);
        // else
        // publish_matches_1F(&ambient_publisher, sorted_2d_cur, sorted_2d_prev,5,true);
    
    }


    void store_coordinates(const Vector3d& t, const Matrix3d& R){

        Quaterniond q(R);
        tfScalar xq = q.x();
        tfScalar yq = q.y();
        tfScalar zq = q.z();
        tfScalar wq = q.w();
        //Create tf Quaternion
        tf::Quaternion qtf;
        qtf.setX(xq);
        qtf.setY(yq);
        qtf.setZ(zq);
        qtf.setW(wq);
        tf::Vector3 t_fill = tf::Vector3(t(0),t(1),t(2));
        tf::Transform odom_t_velo = tf::Transform(qtf,t_fill);
        Eigen::Vector3d e1, e2;
        odom_t_velo.getBasis().getRPY(e1[0], e1[1], e1[2]);  //XYZ
        odom_t_velo.getBasis().getRPY(e2[0], e2[1], e2[2], 2);
        //Set smaller rotation solution
        Vector3d ea/*  = R.eulerAngles(0,1,2) */;
        if (e1.norm() < e2.norm())
            ea = e1;
        else
            ea = e2;


        OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/OLD_prediction_complete.csv",ios_base::app);
        OUT << t(0) << ',' << t(1) << ',' << t(2) << ',' << ea(0) << ',' << ea(1) << ',' << ea(2) << "\n";
        OUT.close();

    }

    void publish_tf(){
        tf::TransformBroadcaster odom_t_velo_b;
        //Create Eigen Quaternion
        // Matrix4d RTINV = RT.inverse();
        Matrix3d R = RT.topLeftCorner(3,3);
        Quaterniond q(R);
        tfScalar xq = q.x();
        tfScalar yq = q.y();
        tfScalar zq = q.z();
        tfScalar wq = q.w();
        //Create tf Quaternion
        tf::Quaternion qtf;
        qtf.setX(xq);
        qtf.setY(yq);
        qtf.setZ(zq);
        qtf.setW(wq);

        //Create translational part of transform
        tfScalar x = RT(0,3);
        tfScalar y = RT(1,3);
        tfScalar z = RT(2,3);
        tf::Vector3 t = tf::Vector3(x,y,z);
        // std::cout << "translation: [" << t.getX() << ", " << t.getY() << ", " << t.getZ() << "]" << std::endl;
        // std::cout << "rotation tf: [" << qtf.x() << ", " << qtf.y() << ", " << qtf.z() <<  ", " << qtf.w() << "]" << std::endl;
        tf::Transform odom_t_velo = tf::Transform(qtf,t);
        // cout << "my translation: " << COUNT << "(" << odom_t_velo.getOrigin().getX() << ", " << odom_t_velo.getOrigin().getY() << ", " << odom_t_velo.getOrigin().getZ() << ") " << endl;
        // cout << "my rotation: " << COUNT << "(" << odom_t_velo.getRotation().getX() << ", " << odom_t_velo.getRotation().getY() << ", " << odom_t_velo.getRotation().getZ() << ", " << odom_t_velo.getRotation().getW() << ") " << endl;
        // COUNT++;
        odom_t_velo_b.sendTransform(tf::StampedTransform(odom_t_velo,raw_time,"odom","my_velo"));
    }

    void publish_odom(){
        nav_msgs::Odometry odom;
        //translation
        odom.pose.pose.position.x = RT(0,3);
        odom.pose.pose.position.y = RT(1,3);
        odom.pose.pose.position.z = RT(2,3);
        //Quaternions (rotation)
        Matrix3d R = RT.topLeftCorner(3,3);
        Quaterniond q(R);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.header.frame_id = "odom";
        odom_publisher.publish(odom);
    }


    MatrixXd get_midpoints(MatrixXd& cur_ICP,MatrixXd& prev_ICP){
        MatrixXd midpoints(3,cur_ICP.cols());
        for(int i = 0; i < cur_ICP.cols();i++){
            midpoints(0,i) = 0.5*(cur_ICP(0,i) + prev_ICP(0,i));
            midpoints(1,i) = 0.5*(cur_ICP(1,i) + prev_ICP(1,i));
            midpoints(2,i) = 0.5*(cur_ICP(2,i) + prev_ICP(2,i));
        }
        return midpoints;
    }

    void publish_keypoint_pc(const MatrixXd& cur_ICP,const  ros::Publisher* kp_pc_publisher){
        PointCloud::Ptr msg (new PointCloud);
        msg->header.frame_id = "velodyne";
        msg->width = cur_ICP.cols();
        msg->height = 1;
        for(size_t i = 0; i < cur_ICP.cols();i++){
            msg->points.push_back(pcl::PointXYZ(cur_ICP(0,i),cur_ICP(1,i),cur_ICP(2,i)));
        }
        pcl_conversions::toPCL(raw_time, msg->header.stamp);
        kp_pc_publisher->publish(msg);
    }

    void publish_matches_2F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, cv::Scalar line_color, bool draw_lines){
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
        cv::Point2d cur_pt = sorted_KP_cur[i] * 1;
        cv::circle(color_img, cur_pt, circle_size*1, line_color, 1*2);
    }
    //indicate features in old image
    for(int i = 0; i< (int)sorted_KP_prev.size(); i++)
    {
        cv::Point2d old_pt = sorted_KP_prev[i] * 1;
        old_pt.y += new_img.size().height + gap;
        cv::circle(color_img, old_pt, circle_size*1, line_color, 1*2);
    }

    if(draw_lines){
        for (int i = 0; i< (int)sorted_KP_cur.size(); i++)
        {
            cv::Point2d old_pt = sorted_KP_prev[i] * 1;
            old_pt.y += new_img.size().height + gap;
            cv::line(color_img, sorted_KP_cur[i] * 1, old_pt, line_color, 1*2, 8, 0);
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

    void publish_matches_1F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, bool draw_lines){
        cv::Mat image = cur_orb->input_image.clone();
        cv::cvtColor(image,image,CV_GRAY2RGB);

        for (int i = 0; i< (int)sorted_KP_cur.size(); i++)
        {
            cv::Point2d old_pt = sorted_KP_prev[i] * 1;
            cv::Point2d cur_pt = sorted_KP_cur[i] * 1;
            unsigned int r = rand() % 256;
            unsigned int g = rand() % 256;
            unsigned int b = rand() % 256;
            cv::Scalar circle_col = cv::Scalar(r,g,b);
            cv::circle(image, cur_pt, circle_size*1, circle_col, 1*1);
            cv::line(image, cur_pt * 1, old_pt, cv::Scalar(255,0,0), 1*1, 8, 0);
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
        W.resize(3,3);
        W.setZero();
        //W is of rank 3, that I checked.
        for(int i = 0; i < prev_ICP.cols();i++){
            Vector3d prev(prev_ICP(0,i),prev_ICP(1,i),prev_ICP(2,i));
            Vector3d cur(cur_ICP(0,i),cur_ICP(1,i),cur_ICP(2,i));
            W += cur*prev.transpose();
        }
        // W = prev_ICP*cur_ICP.transpose();
        JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
        auto VT = svd.matrixV().transpose();
        Matrix3d R = svd.matrixU()*VT;
        Vector3d t = mean_cur - R*mean_prev;

        Matrix4d H;
        H << R,t,0,0,0,1;
        RT = H*RT;
        
        // Matrix3d RI = RT.topLeftCorner(3,3);
        // Vector3d tI = RT.topRightCorner(3,1);
        // store_coordinates(tI,RI);
        // std::cout << "The rotation matrix is: " << COUNT++ << "  " << std::endl << R << std::endl;
        // std::cout << "The translation vector is: " << std::endl << t << std::endl;
        // std::cout << "The current coordinates are: " << std::endl << RT << std::endl;
    }
    
    private:

    std::shared_ptr<ORB> cur_orb, prev_orb;
    vector<cv::DMatch> matches; 
    int mode;
    unsigned int comp_sum;
    unsigned int comp_count;

    Matrix4d RT;

    ros::Time raw_time;

    ros::NodeHandle n_frame;
    ros::Subscriber raw_sub;
    ros::Publisher match_publisher, range_publisher, ambient_publisher, intensity_publisher, 
    kp_pc_publisher_cur, kp_pc_publisher_prev, midpoint_publisher, odom_publisher;
    


};