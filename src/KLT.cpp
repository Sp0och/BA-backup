#include "../include/KLT.h"
#include "helper.cpp"

//class core

KLT::KLT(int _image_source,int START_POSE){

        image_source = _image_source;
        COUNT = MATCH_COUNT = unfiltered_count = ransac_filtered_count = distance_filtered_count = extracted_count =  min_distance_filtered_count = 0;

        std::string config_file;
        n_KLT.getParam("parameter_file", config_file);
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        usleep(100);
        fsSettings["min_klt_features"] >> MIN_KLT_FEATURES;
        //Extraction
        fsSettings["epsilon"] >> EPSILON;
        fsSettings["criteria_reps"] >> CRITERIA_REPS;
        fsSettings["opt_size"] >> OPT_SIZE;
        fsSettings["num_pyramids"] >> NUM_PYRAMIDS;
        //Tracking
        fsSettings["max_klt_features"] >> MAX_KLT_FEATURES;
        fsSettings["blocksize"] >> BLOCKSIZE;
        fsSettings["quality_level"] >> QUALITY_LEVEL;
        fsSettings["min_klt_detection_distance"] >> MIN_KLT_DETECTION_DISTANCE;
        fsSettings["use_harris"] >> USE_HARRIS;

        fsSettings["directory"] >> DIRECTORY;
        fsSettings["klt_file_path"] >> FILE_PATH;

        prev_corners.resize(MAX_KLT_FEATURES);
        cur_corners.resize(MAX_KLT_FEATURES);

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
        
        for(int i = 0; i < 100; i++){
            int r = rand()%256;
            int g = rand()%256;
            int b = rand()%256;
            cv::Scalar color = cv::Scalar(r,g,b);
            color_vector.push_back(color);
        }
        kp_pc_publisher_cur = n_KLT.advertise<PointCloud>("Keypoint_Pointcloud_cur", 1);
        gotten_KP = n_KLT.advertise<PointCloud>("Gotten_KP", 1);
        kp_pc_publisher_prev = n_KLT.advertise<PointCloud>("Keypoint_Pointcloud_prev", 1);
        mid_point_line_publisher = n_KLT.advertise<visualization_msgs::Marker>("connection_lines", 1);
        odom_publisher = n_KLT.advertise<nav_msgs::Odometry>("Odometry", 1);

        extraction_publisher = n_KLT.advertise<sensor_msgs::Image>("extracted_KLT", 1);
        match_publisher = n_KLT.advertise<sensor_msgs::Image>("unfiltered_matches", 1);
        ransac_publisher = n_KLT.advertise<sensor_msgs::Image>("RANSAC_filtered", 1);
        duplicate_publisher = n_KLT.advertise<sensor_msgs::Image>("DUPLICATE_filtered", 1);
        if(image_source == 1)
        pub_KLT_int = n_KLT.advertise<sensor_msgs::Image>("tracked_points_int", 1);
        else if(image_source == 2)
        pub_KLT_ran = n_KLT.advertise<sensor_msgs::Image>("tracked_points_ran", 1);
        else
        pub_KLT_amb = n_KLT.advertise<sensor_msgs::Image>("tracked_points_amb", 1);
    }

void KLT::KLT_Iteration(const cv::Mat& input_image,const pcl::PointCloud<PointType>::Ptr _cloud,ros::Time _raw_time){
    //First iteration
    raw_time = _raw_time;
    if(prev_image.rows == 0){
        prev_image = input_image.clone();
        prev_cloud = _cloud;

        cv::goodFeaturesToTrack(prev_image,prev_corners,MAX_KLT_FEATURES,QUALITY_LEVEL,MIN_KLT_DETECTION_DISTANCE,MASK,BLOCKSIZE,USE_HARRIS,0.04);
        extracted_count+=prev_corners.size();

        get_3D_points(prev_corners,prev_3D_points,prev_cloud);
        min_distance_filtered_count+=prev_corners.size();
        COUNT++;

        publish_extraction(&extraction_publisher,prev_image,prev_corners,1);

        if(SHOULD_STORE)
        set_plotting_columns_and_start_pose();

        publish_tf();
    }
    
    else {
        cur_image = input_image.clone();
        cur_cloud = _cloud;
        
        std::vector<uchar> stat;
        std::vector<float> err;
        static cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), CRITERIA_REPS,EPSILON );
        assert(prev_corners.size()>0);
        //Track points
        cv::calcOpticalFlowPyrLK(prev_image,cur_image,prev_corners,cur_corners,stat,err,
        cv::Size(OPT_SIZE,OPT_SIZE),NUM_PYRAMIDS,criteria);
        std::vector<bool> status(stat.size(),1);
        for(int i = 0; i < stat.size();i++){
            if(stat.at(i) == 0 || cur_corners.at(i).x < IMAGE_CROP || cur_corners.at(i).x > IMAGE_WIDTH-IMAGE_CROP/2 || cur_corners.at(i).y >127)
            status.at(i) = 0;
        }
        get_3D_points_adapt_status(cur_corners,cur_3D_points,cur_cloud,status);
        trimVector(prev_corners,status);
        trim_matrix(prev_3D_points,status);
        
        unfiltered_count += cur_3D_points.cols();
        // publish_tracking_2F(&match_publisher,cur_image,prev_image,cur_corners,prev_corners,2);

        //Filtering
        if(APPLY_RANSAC_FILTERING){
            RANSAC_filtering_f(cur_corners,prev_corners,cur_3D_points,prev_3D_points);
            // publish_tracking(&ransac_publisher,cur_image,cur_corners,prev_corners,2);
            ransac_filtered_count += cur_3D_points.cols();
        }
        if(APPLY_DISTANCE_FILTERING){
            filtering_3D_f(cur_3D_points,prev_3D_points,cur_corners,prev_corners);
            distance_filtered_count+= cur_3D_points.cols();
        }

        //visualization
        // visualizer_3D(cur_3D_points,prev_3D_points);

        if(SHOULD_STORE)
        store_feature_number(cur_3D_points);

        //SVD
        if(cur_3D_points.cols() == prev_3D_points.cols() && cur_3D_points.cols() != 0 )
            SVD(cur_3D_points,prev_3D_points);
        else
            std::cout << "3D Data not set up correctly" << " " << endl;

        publish_tf();

        //match publishing version one
        // if(image_source == 1)
        // publish_tracking(&pub_KLT_int,cur_image,cur_corners,prev_corners,2);
        // else if(image_source == 2)
        // publish_tracking(&pub_KLT_ran,cur_image,cur_corners,prev_corners,2);
        // else
        // publish_tracking(&pub_KLT_amb,cur_image,cur_corners,prev_corners,2);
        
        
        //match publishing version two with both images displayed
        // if(image_source == 1)
        // publish_tracking_2F(&pub_KLT_int,cur_image,prev_image,cur_corners,prev_corners,2);
        // else if(image_source == 2)
        // publish_tracking_2F(&pub_KLT_ran,cur_image,prev_image,cur_corners,prev_corners,2);
        // else
        // publish_tracking_2F(&pub_KLT_amb,cur_image,prev_image,cur_corners,prev_corners,2);
        
        prev_image = cur_image.clone();
        prev_cloud = cur_cloud;
        prev_corners = cur_corners;
        prev_3D_points = cur_3D_points;
        if(prev_corners.size()<MIN_KLT_FEATURES){
            std::cout << "GET NEW CORNERS" << std::endl;
            // std::cout << "size_before" << prev_corners.size() << " " <<::endl;
            cv::goodFeaturesToTrack(prev_image,prev_corners,MAX_KLT_FEATURES,QUALITY_LEVEL,MIN_KLT_DETECTION_DISTANCE,MASK,BLOCKSIZE,USE_HARRIS,0.04);
            extracted_count += prev_corners.size();
            get_3D_points(prev_corners,prev_3D_points,prev_cloud);
            min_distance_filtered_count += prev_corners.size();
            // std::cout << "size_after" << prev_corners.size() << " " <<::endl;
            publish_extraction(&extraction_publisher,prev_image,prev_corners,1);
            COUNT++;
        }
        else{
            // std::cout << "All Goodie" << std::endl;
        }
        MATCH_COUNT++;
        if(MATCH_COUNT >= 50){
        cout << "average extracted is : " << 1.0*extracted_count/COUNT<< " " << endl;
        cout << "average min distance filtered is: " << 1.0*min_distance_filtered_count/COUNT<< " " << endl;
        cout << "average unfiltered matches is: " << 1.0*unfiltered_count/MATCH_COUNT<< " " << endl;
        cout << "average ransac filtered matches is: " << 1.0*ransac_filtered_count/MATCH_COUNT<< " " << endl;
        cout << "average distance filtered matches is: " << 1.0*distance_filtered_count/MATCH_COUNT<< " " << endl;
        cout << "TRUE POSITIVE MATCHING RATE IS: " << 100*(1.0*distance_filtered_count/unfiltered_count)<< " " << endl;
        }
    }
}


//visualization methods:

void KLT::publish_tracking(ros::Publisher* publisher, const cv::Mat& cur_image,
        const vector<cv::Point2f>& cur_keypoints, const vector<cv::Point2f>& prev_keypoints,
        int circle_size){
        cv::Mat image = cur_image.clone();
        cv::cvtColor(image,image,CV_GRAY2RGB);
        for(int i = 0; i < cur_keypoints.size();i++){
            cv::circle(image,cur_keypoints.at(i),circle_size,cv::Scalar(255,0,0),2,8,0);
            cv::circle(image,prev_keypoints.at(i),circle_size,cv::Scalar(255,0,0),2,8,0);
            cv::line(image,cur_keypoints.at(i),prev_keypoints[i],cv::Scalar(0,255,0),1,8,0);
        }
        if(image_source == 1)
        cv::putText(image, "Intensity",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        else if (image_source == 2)
        cv::putText(image, "Range",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        else
        cv::putText(image, "Ambient",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
        "bgr8", image).toImageMsg();
        publisher->publish(msg);
    }

void KLT::publish_tracking_2F(ros::Publisher* publisher, const cv::Mat& cur_image, const cv::Mat& prev_image,const vector<cv::Point2f>& cur_keypoints, const vector<cv::Point2f>& prev_keypoints,
int circle_size){
    cv::Mat color_img,gray_img;
    const cv::Mat old_img = prev_image.clone();
    const cv::Mat new_img = cur_image.clone();
    int gap = 1;
    cv::Mat gap_img(gap, new_img.size().width, CV_8UC1, cv::Scalar(255, 255, 255));
    //create colored concatenated image with gap inbetween
    cv::vconcat(new_img,gap_img,gap_img);
    cv::vconcat(gap_img,old_img,gray_img);

    cv::cvtColor(gray_img,color_img,CV_GRAY2RGB);
    //indicate features in new image
    for(int i = 0; i< (int)cur_corners.size(); i++)
    {
        cv::Point2d cur_pt = cur_corners[i] ;
        cv::circle(color_img, cur_pt, circle_size, cv::Scalar(255,0,0), 2);
    }
    //indicate features in old image
    for(int i = 0; i< (int)prev_corners.size(); i++)
    {
        cv::Point2d old_pt = prev_corners[i];
        old_pt.y += new_img.size().height + gap;
        cv::circle(color_img, old_pt, circle_size, cv::Scalar(255,0,0), 2);
    }

    for (int i = 0; i< (int)cur_corners.size(); i++)
    {
        cv::Point2d old_pt = prev_corners[i] * 1;
        old_pt.y += new_img.size().height + gap;
        cv::line(color_img, cur_corners[i] * 1, old_pt, cv::Scalar(0,255,0), 1*2, 8, 0);
    }
    if(image_source == 1)
    cv::putText(color_img, "Intensity",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else if (image_source == 2)
    cv::putText(color_img, "Range",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else
    cv::putText(color_img, "Ambient",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_img).toImageMsg();
    publisher->publish(msg);

}

    
void KLT::publish_extraction(ros::Publisher* publisher, const cv::Mat& input_image,
    const vector<cv::Point2f>& keypoints,int circle_size){
    cv::Mat image = input_image.clone();
    cv::cvtColor(image,image,CV_GRAY2RGB);
    for(int i = 0; i < keypoints.size();i++){
        cv::circle(image,keypoints.at(i),circle_size,cv::Scalar(0,255,0),2,8,0);
    }
    if(image_source == 1)
    cv::putText(image, "Intensity",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else if (image_source == 2)
    cv::putText(image, "Range",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    else
    cv::putText(image, "Ambient",   cv::Point2d(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
    "bgr8", image).toImageMsg();
    publisher->publish(msg);
}

void KLT::visualizer_3D(const MatrixXd& cur_SVD, const MatrixXd& prev_SVD){
        publish_3D_keypoints(cur_SVD, &kp_pc_publisher_cur, raw_time);
        publish_3D_keypoints(prev_SVD, &kp_pc_publisher_prev, raw_time);
        publish_lines_3D(cur_SVD, prev_SVD, &mid_point_line_publisher, raw_time);
}

void KLT::publish_tf(){
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
        // std::cout << "my pose: " << my_pose << std::endl;
        odom_t_velo_b.sendTransform(tf::StampedTransform(odom_t_velo,raw_time,"odom","my_velo"));
    }

//plotting functions

void KLT::store_feature_number(const MatrixXd& cur_SVD){
    OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/feature_number_"+ FILE_PATH + ".csv",ios_base::app);
    OUT << cur_SVD.cols() << "," << raw_time <<  endl;
    OUT.close(); 
}

void KLT::set_plotting_columns_and_start_pose(){

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
        
    string Param = to_string(CRITERIA_REPS);

    OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/prediction_pose_"+ FILE_PATH + ".csv",ios_base::app);
    OUT << "x" << "," << "y" << "," << "z" << "," << "roll"<< "," << "pitch"<< "," << "yaw" << "," << "time" << endl;
    OUT << my_pose(0,3) << "," << my_pose(1,3) << "," << my_pose(2,3) << "," << eac(0)<< "," << eac(1)<< "," << eac(2) << "," << raw_time << endl;
    OUT.close(); 
    
    OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/prediction_steps_"+ FILE_PATH + ".csv",ios_base::app);
    OUT << "x" << "," << "y" << "," << "z" << "," << "roll"<< "," << "pitch"<< "," << "yaw" << "," << "time" << endl;
    OUT.close(); 
    
    OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/feature_number_"+ FILE_PATH + ".csv",ios_base::app);
    OUT << "num_of_features" "," << "time" << endl;
    OUT.close(); 
}

void KLT::store_coordinates(const Vector3d& t, const Matrix3d& R){

        string Param = to_string(CRITERIA_REPS);

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

        Vector3d e1, e2;
        odom_t_velo.getBasis().getRPY(e1[0], e1[1], e1[2]);  //XYZ
        odom_t_velo.getBasis().getRPY(e2[0], e2[1], e2[2], 2);

        //Set smaller rotation solution
        Vector3d ea/*  = R.eulerAngles(0,1,2) */;
        if (e1.norm() < e2.norm())
            ea = e1;
        else
            ea = e2;
        OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/prediction_steps_"+ FILE_PATH + ".csv",ios_base::app);
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
        Vector3d e1c, e2c;
        odom_t_velo_complete.getBasis().getRPY(e1c[0], e1c[1], e1c[2]);  //XYZ
        odom_t_velo_complete.getBasis().getRPY(e2c[0], e2c[1], e2c[2], 2);

        Vector3d eac/*  = R.eulerAngles(0,1,2) */;
        if (e1c.norm() < e2c.norm())
            eac = e1c;
        else
            eac = e2c;
        
        OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/" + DIRECTORY + "/prediction_pose_"+ FILE_PATH + ".csv",ios_base::app);
        OUT << my_pose(0,3) << "," << my_pose(1,3) << "," << my_pose(2,3) << "," << eac(0)<< "," << eac(1)<< "," << eac(2) << "," << raw_time << endl;
        OUT.close();
    }

//SVD:

void KLT::SVD(const MatrixXd& cur_3D,const MatrixXd& prev_3D){
        MatrixXd cur_SVD = cur_3D;
        MatrixXd prev_SVD = prev_3D;
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

        // cout << "R: " << endl << R << endl;
        // cout << "t: " << endl << t << endl;
        // cout << "my pose: " << my_pose << endl;

        //Storing the plot data
        if(SHOULD_STORE)
        store_coordinates(t,R);

    }
 
