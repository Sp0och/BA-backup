#include "../include/ORB_Framehandler.h"
#include "helper.cpp"

using namespace std;

//core member methods:

ORB_Framehandler::ORB_Framehandler(int _image_source,int START_POSE){
        COUNT = 0;
        filtered_count = 0;
        ransac_filtered_count = 0;
        unfiltered_count = 0;
        std::string config_file;
        n_frame.getParam("parameter_file", config_file);
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        usleep(100);
        fsSettings["num_orb_features"] >> NUM_ORB_FEATURES;
        fsSettings["orb_accuracy"] >> ORB_ACCURACY;
        fsSettings["scale_factor"] >> SCALE_FACTOR;
        fsSettings["levels"] >> LEVELS;

        fsSettings["orb_file_path"] >> FILE_PATH;
        fsSettings["directory"] >> DIRECTORY;

        image_source = _image_source;
        if(START_POSE == 0){
            my_pose << 1,0,0,0,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;
        }
        else if(START_POSE == 1){
            my_pose << -0.98877182,  0.14125801,  0.04874899, -0.03791867,
                    -0.14255745, -0.9894887,  -0.02427929, -0.02220684,
                    0.04480693, -0.03095621,  0.99851592, -0.01088667,
                    0,          0,          0,          1; 
        }
        else{
            my_pose << 0.08959188, -0.99543686,  0.03284438,  0.17298461,
        0.96004952,  0.07753545, -0.26887389,  1.35417363,
        0.26510037,  0.05562115,  0.96261523,  0.42004326,
        0,0,0,1;
        }
        
        cur_orb = nullptr;
        prev_orb = nullptr;

        POINT_COLOR = cv::Scalar(255,0,0);
        LINE_COLOR = cv::Scalar(0,255,0);
        match_publisher = n_frame.advertise<sensor_msgs::Image>("unfiltered_matches", 1);
        ransac_publisher = n_frame.advertise<sensor_msgs::Image>("RANSAC_filtered", 1);

        kp_pc_publisher_cur = n_frame.advertise<PointCloud>("Keypoint_Pointcloud_cur", 1);
        kp_pc_publisher_prev = n_frame.advertise<PointCloud>("Keypoint_Pointcloud_prev", 1);
        pc_distance_publisher_c = n_frame.advertise<PointCloud>("Distance_filtered_3D_c", 1);
        pc_distance_publisher_p = n_frame.advertise<PointCloud>("Distance_filtered_3D_p", 1);
        line_publisher = n_frame.advertise<visualization_msgs::Marker>("connection_lines", 1);
        line_distance_publisher = n_frame.advertise<visualization_msgs::Marker>("connection_lines_distance", 1);
        odom_publisher = n_frame.advertise<nav_msgs::Odometry>("Odometry", 1);
        if(image_source == 1)
            intensity_publisher = n_frame.advertise<sensor_msgs::Image>("intensity_matches", 1);
        else if(image_source == 2)
            range_publisher = n_frame.advertise<sensor_msgs::Image>("range_matches", 1);
        else
            ambient_publisher = n_frame.advertise<sensor_msgs::Image>("ambient_matches", 1);
    }

void ORB_Framehandler::newIteration(const std::shared_ptr<ORB> new_frame, ros::Time _raw_time){
        raw_time = _raw_time;
        //if first iteration
        if(cur_orb == nullptr){
            cur_orb = new_frame;
            //Set up plot files, store the inital pose and feature num and publish the transform of the first frame
            publish_tf();
            if(SHOULD_STORE)
            set_plotting_columns_and_start_pose();
        }
        else{
            prev_orb = cur_orb;
            cur_orb = new_frame;
            matches_filtering_motion();
        }
    }

void ORB_Framehandler::matches_filtering_motion(){
        static cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);

        //create matches
        bool cur_first = false;
        if(cur_orb->orb_keypoints_2d.size() < prev_orb->orb_keypoints_2d.size()){
            cur_first = true;
            matcher->match(cur_orb->orb_descriptors,prev_orb->orb_descriptors,matches);
        }
        else{
            matcher->match(prev_orb->orb_descriptors,cur_orb->orb_descriptors,matches);
        }
        //sort matches in ascending order (concerning distance)
        std::sort(matches.begin(),matches.end());
        std::vector<cv::Point2d> sorted_2d_cur, sorted_2d_prev;
        MatrixXd prev_SVD(3,matches.size()),cur_SVD(3,matches.size());
        //pair the keypoints up according to the matches:
        for (size_t i = 0; i < matches.size(); i++)
        {
            //the match indexes for point association
            int cur_index;
            int prev_index;
            if(cur_first){
                cur_index = matches.at(i).queryIdx;
                prev_index = matches.at(i).trainIdx;
            }
            else{
                cur_index = matches.at(i).trainIdx;
                prev_index = matches.at(i).queryIdx;
            }

            //create sorted keypoint vectors
            sorted_2d_cur.push_back(cur_orb->orb_keypoints_2d.at(cur_index));
            sorted_2d_prev.push_back(prev_orb->orb_keypoints_2d.at(prev_index));
            //create sorted 3d keypoint vectors
            cur_SVD(0,i) = cur_orb->orb_points_3d.at(cur_index).x;
            cur_SVD(1,i) = cur_orb->orb_points_3d.at(cur_index).y;
            cur_SVD(2,i) = cur_orb->orb_points_3d.at(cur_index).z;
            prev_SVD(0,i) = prev_orb->orb_points_3d.at(prev_index).x;
            prev_SVD(1,i) = prev_orb->orb_points_3d.at(prev_index).y;
            prev_SVD(2,i) = prev_orb->orb_points_3d.at(prev_index).z;
        }
        matches.clear();

        //Publish matches before filtering:
        publish_matches_2F(&match_publisher, sorted_2d_cur, sorted_2d_prev,2,POINT_COLOR,LINE_COLOR,true);
        // cout << "right after matching: " << prev_SVD.cols() << " " << endl;
        unfiltered_count += cur_SVD.cols();
        
        if(APPLY_RANSAC_FILTERING){
            RANSAC_filtering(sorted_2d_cur,sorted_2d_prev,cur_SVD,prev_SVD);
            publish_matches_2F(&ransac_publisher, sorted_2d_cur, sorted_2d_prev,2,POINT_COLOR,LINE_COLOR,true);
            // cout << "size after RANSAC: " << prev_SVD.cols() << " "<< endl;
        }
        ransac_filtered_count+=cur_SVD.cols();

        visualizer_3D(cur_SVD,prev_SVD,&pc_distance_publisher_c,&pc_distance_publisher_p,&line_distance_publisher);
        
        if(APPLY_DISTANCE_FILTERING){
            filtering_3D(cur_SVD,prev_SVD, sorted_2d_cur, sorted_2d_prev);
            // cout << "size after distance filtering: " << prev_SVD.cols() << " "<< endl;
        }
        filtered_count += cur_SVD.cols();

        if(SHOULD_STORE)
        store_feature_number(cur_SVD);

        visualizer_3D(cur_SVD,prev_SVD,&kp_pc_publisher_cur,&kp_pc_publisher_prev,&line_publisher);


        //SVD Here
        if(cur_SVD.size() == prev_SVD.size() && cur_SVD.size() != 0)
            SVD(cur_SVD,prev_SVD);
        else    
            std::cout << "ERROR: 3D Vectors weren't initialized properly" << std::endl;

        //publish transformation
        publish_tf();

        //First 2D match display option
        
        if(image_source == 1)
        publish_matches_2F(&intensity_publisher, sorted_2d_cur, sorted_2d_prev,2,POINT_COLOR,LINE_COLOR,true);
        else if(image_source == 2)
        publish_matches_2F(&range_publisher, sorted_2d_cur, sorted_2d_prev,2,POINT_COLOR,LINE_COLOR,true);
        else
        publish_matches_2F(&ambient_publisher, sorted_2d_cur, sorted_2d_prev,2,POINT_COLOR,LINE_COLOR,true);
        
        //Second 2D match display option

        // if(image_source == 1)
        // publish_matches_1F(&intensity_publisher, sorted_2d_cur, sorted_2d_prev,1,cv::Scalar(255,0,0),cv::Scalar(0,255,0),true);
        // else if(image_source == 2)
        // publish_matches_1F(&range_publisher, sorted_2d_cur, sorted_2d_prev,1,cv::Scalar(255,0,0),cv::Scalar(0,255,0),true);
        // else
        // publish_matches_1F(&ambient_publisher, sorted_2d_cur, sorted_2d_prev,1,cv::Scalar(255,0,0),cv::Scalar(0,255,0),true);
        
        COUNT++;
        if(COUNT >= 50){
        cout << "average_unfilterd is: " << 1.0*unfiltered_count/COUNT<< " " << endl;
        cout << "average ransac filtered is: " << 1.0*ransac_filtered_count/COUNT << " " << endl;
        cout << "average_filtered is : " << 1.0*filtered_count/COUNT<< " " << endl;
        cout << "TRUE POSITIVE MATCHING RATE IS: " << 100*(1.0*filtered_count/unfiltered_count)<< " " << endl;
        }
    }

//plotting functions

void ORB_Framehandler::set_plotting_columns_and_start_pose(){

    // store the start pose (either their start or identity)
        Matrix3d R_complete = my_pose.topLeftCorner(3,3);
        Quaterniond q_complete(R_complete);
        tfScalar xqc = q_complete.x();
        tfScalar yqc = q_complete.y();
        tfScalar zqc = q_complete.z();
        tfScalar wqc = q_complete.w();
        tf::Quaternion qtfc;
        qtfc.setX(xqc);
        qtfc.setY(yqc);
        qtfc.setZ(zqc);
        qtfc.setW(wqc);

        tfScalar xc = my_pose(0,3);
        tfScalar yc = my_pose(1,3);
        tfScalar zc = my_pose(2,3);
        tf::Vector3 tc_fill = tf::Vector3(xc,yc,zc);
        tf::Transform odom_t_velo_complete = tf::Transform(qtfc,tc_fill);
        Eigen::Vector3d e1c, e2c;
        odom_t_velo_complete.getBasis().getRPY(e1c[0], e1c[1], e1c[2]);  //XYZ
        odom_t_velo_complete.getBasis().getRPY(e2c[0], e2c[1], e2c[2], 2);

        Vector3d eac/*  = R.eulerAngles(0,1,2) */;
        if (e1c.norm() < e2c.norm())
            eac = e1c;
        else
            eac = e2c;

    string Param = to_string(DUPLICATE_FILTERING_SIZE);
        


    OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/prediction_pose_"+FILE_PATH+".csv",ios_base::app);
    OUT << "x" << "," << "y" << "," << "z" << "," << "roll"<< "," << "pitch"<< "," << "yaw" << "," << "time" << endl;
    OUT << my_pose(0,3) << "," << my_pose(1,3) << "," << my_pose(2,3) << "," << eac(0)<< "," << eac(1)<< "," << eac(2) << "," << raw_time << endl;
    OUT.close(); 
    
    OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/prediction_steps_"+FILE_PATH+".csv",ios_base::app);
    OUT << "x" << "," << "y" << "," << "z" << "," << "roll"<< "," << "pitch"<< "," << "yaw" << "," << "time" << endl;
    OUT.close(); 
    
    OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/feature_number_"+FILE_PATH+".csv",ios_base::app);
    OUT << "num_of_features" "," << "time" << endl;
    OUT.close(); 
}

void ORB_Framehandler::store_coordinates(const Vector3d& t, const Matrix3d& R){

        string Param = to_string(DUPLICATE_FILTERING_SIZE);

        //step changes:
        Quaterniond q(R);
        tfScalar xq = q.x();
        tfScalar yq = q.y();
        tfScalar zq = q.z();
        tfScalar wq = q.w();

        tf::Quaternion qtf;
        qtf.setX(xq);
        qtf.setY(yq);
        qtf.setZ(zq);
        qtf.setW(wq);
        tfScalar x = t(0);
        tfScalar y = t(1);
        tfScalar z = t(2);
        tf::Vector3 t_fill = tf::Vector3(x,y,z);
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
        OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/prediction_steps_"+FILE_PATH+".csv",ios_base::app);
        OUT << t(0) << "," << t(1) << "," << t(2) << "," << ea(0)<< "," << ea(1)<< "," << ea(2) << "," << raw_time <<  endl;
        OUT.close(); 




        // overall
        Matrix3d R_complete = my_pose.topLeftCorner(3,3);
        Quaterniond q_complete(R_complete);
        tfScalar xqc = q_complete.x();
        tfScalar yqc = q_complete.y();
        tfScalar zqc = q_complete.z();
        tfScalar wqc = q_complete.w();
        tf::Quaternion qtfc;
        qtfc.setX(xqc);
        qtfc.setY(yqc);
        qtfc.setZ(zqc);
        qtfc.setW(wqc);

        tfScalar xc = my_pose(0,3);
        tfScalar yc = my_pose(1,3);
        tfScalar zc = my_pose(2,3);
        tf::Vector3 tc_fill = tf::Vector3(xc,yc,zc);
        tf::Transform odom_t_velo_complete = tf::Transform(qtfc,tc_fill);
        Eigen::Vector3d e1c, e2c;
        odom_t_velo_complete.getBasis().getRPY(e1c[0], e1c[1], e1c[2]);  //XYZ
        odom_t_velo_complete.getBasis().getRPY(e2c[0], e2c[1], e2c[2], 2);

        Vector3d eac/*  = R.eulerAngles(0,1,2) */;
        if (e1c.norm() < e2c.norm())
            eac = e1c;
        else
            eac = e2c;
        
        OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/prediction_pose_"+FILE_PATH+".csv",ios_base::app);
        OUT << my_pose(0,3) << "," << my_pose(1,3) << "," << my_pose(2,3) << "," << eac(0)<< "," << eac(1)<< "," << eac(2) << "," << raw_time << endl;
        OUT.close();
    }

void ORB_Framehandler::store_feature_number(const MatrixXd& cur_SVD){
    OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/feature_number_"+FILE_PATH+".csv",ios_base::app);
    OUT << cur_SVD.cols() << "," << raw_time <<  endl;
    OUT.close(); 
}

// publishing functions

void ORB_Framehandler::publish_tf(){
        cout << "TF Publisher " << endl;
        tf::TransformBroadcaster odom_t_velo_b;
        //Create Eigen Quaternion
        Matrix3d R = my_pose.topLeftCorner(3,3);
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
        tfScalar x = my_pose(0,3);
        tfScalar y = my_pose(1,3);
        tfScalar z = my_pose(2,3);
        tf::Vector3 t = tf::Vector3(x,y,z);

        // std::cout << "translation: [" << t.getX() << ", " << t.getY() << ", " << t.getZ() << "]" << std::endl;
        // std::cout << "rotation tf: [" << qtf.x() << ", " << qtf.y() << ", " << qtf.z() <<  ", " << qtf.w() << "]" << std::endl;
        tf::Transform odom_t_velo = tf::Transform(qtf,t);
        // cout << "t: " << t.getX() << ", "<< t.getY() << ", "<< t.getZ() << " " << endl;
        // ros::Duration delay(252940.892);
        // ros::Time now_p_delay = raw_time + delay;
        // cout << "my translation: " << COUNT << "(" << odom_t_velo.getOrigin().getX() << ", " << odom_t_velo.getOrigin().getY() << ", " << odom_t_velo.getOrigin().getZ() << ") " << endl;
        // cout << "my rotation: " << COUNT << "(" << odom_t_velo.getRotation().getX() << ", " << odom_t_velo.getRotation().getY() << ", " << odom_t_velo.getRotation().getZ() << ", " << odom_t_velo.getRotation().getW() << ") " << endl;
        // COUNT++;
        odom_t_velo_b.sendTransform(tf::StampedTransform(odom_t_velo,raw_time,"odom","my_velo"));
    }

void ORB_Framehandler::publish_odom(){
        nav_msgs::Odometry odom;
        //translation
        odom.pose.pose.position.x = my_pose(0,3);
        odom.pose.pose.position.y = my_pose(1,3);
        odom.pose.pose.position.z = my_pose(2,3);
        //Quaternions (rotation)
        Matrix3d R = my_pose.topLeftCorner(3,3);
        Quaterniond q(R);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.header.frame_id = "odom";
        odom_publisher.publish(odom);
    }

//member visualization functions

void ORB_Framehandler::visualizer_3D(const MatrixXd& cur_SVD, const MatrixXd& prev_SVD,ros::Publisher* cur_publisher,ros::Publisher* prev_publisher,ros::Publisher* line_publisher){
        publish_3D_keypoints(cur_SVD, cur_publisher, raw_time);
        publish_3D_keypoints(prev_SVD, prev_publisher, raw_time);
        publish_lines_3D(cur_SVD, prev_SVD, line_publisher, raw_time);
}

void ORB_Framehandler::publish_matches_2F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, cv::Scalar point_color, cv::Scalar line_color, bool draw_lines){
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
        cv::Point2d cur_pt = sorted_KP_cur[i] ;
        cv::circle(color_img, cur_pt, circle_size, point_color, 2);
    }
    //indicate features in old image
    for(int i = 0; i< (int)sorted_KP_prev.size(); i++)
    {
        cv::Point2d old_pt = sorted_KP_prev[i];
        old_pt.y += new_img.size().height + gap;
        cv::circle(color_img, old_pt, circle_size, point_color, 2);
    }

    if(draw_lines){
        for (int i = 0; i< (int)sorted_KP_cur.size(); i++)
        {
            cv::Point2d old_pt = sorted_KP_prev[i] * 1;
            old_pt.y += new_img.size().height + gap;
            cv::line(color_img, sorted_KP_cur[i] * 1, old_pt, line_color, 1*2, 8, 0);
        }
    }
    if(image_source == 1)
    cv::putText(color_img, "Intensity",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else if (image_source == 2)
    cv::putText(color_img, "Range",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else
    cv::putText(color_img, "Ambient",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_img).toImageMsg();
    this_pub->publish(msg);

}

void ORB_Framehandler::publish_matches_1F(const ros::Publisher* this_pub,const  std::vector<cv::Point2d>& sorted_KP_cur, 
    const std::vector<cv::Point2d>& sorted_KP_prev, int circle_size, cv::Scalar point_color, cv::Scalar line_color, bool draw_lines){
        cv::Mat image = cur_orb->input_image.clone();
        cv::cvtColor(image,image,CV_GRAY2RGB);

        for (int i = 0; i< (int)sorted_KP_cur.size(); i++)
        {
            cv::Point2d old_pt = sorted_KP_prev[i];
            cv::Point2d cur_pt = sorted_KP_cur[i];
            cv::circle(image, cur_pt, circle_size, point_color, 2);
            cv::circle(image, old_pt, circle_size, point_color, 2);
            if(draw_lines)
                cv::line(image, cur_pt * 1, old_pt, line_color, 1, 8, 0);
        }
        if(image_source == 1)
        cv::putText(image, "Intensity",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        else if (image_source == 2)
        cv::putText(image, "Range",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        else
        cv::putText(image, "Ambient",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        this_pub->publish(msg);
    }

//SVD

void ORB_Framehandler::SVD(MatrixXd& cur_SVD,MatrixXd& prev_SVD){
        Vector3d sum_prev(0,0,0);
        Vector3d sum_cur(0,0,0);

        for(int i = 0;i < prev_SVD.cols();i++){
            sum_prev(0) += prev_SVD(0,i);
            sum_prev(1) += prev_SVD(1,i);
            sum_prev(2) += prev_SVD(2,i);
            sum_cur(0) += cur_SVD(0,i);
            sum_cur(1) += cur_SVD(1,i);
            sum_cur(2) += cur_SVD(2,i);
        }
        //create the mean for subtraction
        Vector3d mean_prev = sum_prev/prev_SVD.cols();
        Vector3d mean_cur = sum_cur/cur_SVD.cols();  
        //Subtract center of "mass"
        for(int i = 0; i < prev_SVD.cols();i++){
            prev_SVD(0,i) -= mean_prev(0);
            prev_SVD(1,i) -= mean_prev(1);
            prev_SVD(2,i) -= mean_prev(2);
            cur_SVD(0,i) -= mean_cur(0);
            cur_SVD(1,i) -= mean_cur(1);
            cur_SVD(2,i) -= mean_cur(2);
        }

        MatrixXd W;
        W.resize(3,3);
        W.setZero();
        //W is the sum of the products of each corresponding point
        for(int i = 0; i < prev_SVD.cols();i++){
            Vector3d point_prev(prev_SVD(0,i),prev_SVD(1,i),prev_SVD(2,i));
            Vector3d point_cur(cur_SVD(0,i),cur_SVD(1,i),cur_SVD(2,i));
            auto PCT = point_cur.transpose();
            W += point_prev*PCT;
        }
        
        JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
        auto VT = svd.matrixV().transpose();
        Matrix3d R = svd.matrixU()*VT;
        Vector3d t = mean_prev - R*mean_cur;


        Matrix4d current_iteration;
        current_iteration.setIdentity();
        current_iteration.block<3,3>(0,0) = R;
        current_iteration.block<3,1>(0,3) = t;


        my_pose = my_pose*current_iteration;

        //Storing the plot data
        if(SHOULD_STORE)
        store_coordinates(t,R);

    }

