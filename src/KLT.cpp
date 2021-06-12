#include "../include/KLT.h"
#include "helper.cpp"




KLT::KLT(int _mode,int START_POSE){

        mode = _mode;

        if(START_POSE == 0){
            my_pose << 1,0,0,0,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;
        }
        else if(START_POSE == 1){
            my_pose << 0.29824051, -0.94876171,  0.10442136, -0.03153784,
            0.94640945,  0.27973465, -0.16142391,  0.74966328,
            0.12394255,  0.14696851,  0.98134525,  0.08621409,
            0,           0,           0,            1;
        }
        else{
            my_pose << 0.21383611, -0.97496278, -0.06100574,  0.14088768,
        0.92808941,  0.22224938, -0.29875621,  1.44032526,
        0.30483467,  0.00726608,  0.95237757,  0.3799721,
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
        kp_pc_publisher_prev = n_KLT.advertise<PointCloud>("Keypoint_Pointcloud_prev", 1);
        mid_point_line_publisher = n_KLT.advertise<visualization_msgs::Marker>("connection_lines", 1);
        odom_publisher = n_KLT.advertise<nav_msgs::Odometry>("Odometry", 1);

        if(mode == 1)
        pub_KLT_int = n_KLT.advertise<sensor_msgs::Image>("tracked_points_int", 1);
        else if(mode == 2)
        pub_KLT_ran = n_KLT.advertise<sensor_msgs::Image>("tracked_points_ran", 1);
        else
        pub_KLT_amb = n_KLT.advertise<sensor_msgs::Image>("tracked_points_amb", 1);
    }

void KLT::KLT_Iteration(const cv::Mat& input_image,const pcl::PointCloud<PointType>::Ptr _cloud,ros::Time _raw_time){
    //First iteration
    raw_time = _raw_time;
    if(prev_image.rows == 0 || prev_corners.size()==0){
        prev_image = input_image.clone();
        prev_cloud = _cloud;
        cv::goodFeaturesToTrack(prev_image,prev_corners,MAX_KLT_FEATURES,0.1,3,MASK,7,true,0.04);
        get_3D_points(prev_corners,prev_3D_points,prev_cloud);
        publish_tf();
    }
    
    else {
        cur_image = input_image.clone();
        cur_cloud = _cloud;
        std::vector<uchar> stat;
        std::vector<float> err;
        //Create Termination criteria for the KLT method
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
        //create the new features from the old ones
        assert(prev_corners.size()>0);
        //New point locations

        cv::calcOpticalFlowPyrLK(prev_image,cur_image,prev_corners,cur_corners,stat,err,
        cv::Size(21,21),3,criteria);
        std::vector<bool> status(stat.size());
        for(int i = 0; i < stat.size();i++){
            status.at(i) = (stat.at(i)!=0);
        }
        trimVector(cur_corners,status);
        trimVector(prev_corners,status);
        trim_matrix(prev_3D_points,status);
        get_3D_points(cur_corners,cur_3D_points,cur_cloud);

        // RANSAC_filtering_f(cur_corners,prev_corners,cur_3D_points,prev_3D_points);

        filtering_3D_f(cur_3D_points,prev_3D_points,cur_corners,prev_corners);

        visualizer_3D(cur_3D_points,prev_3D_points);


        if(cur_3D_points.cols() == prev_3D_points.cols() && cur_3D_points.cols() != 0 )
            SVD(cur_3D_points,prev_3D_points);
        else
            cout << "3D Data not set up correctly" << " " << endl;

        publish_tf();
        //Publish and adapt prev_corners
        if(mode == 1)
        publish_KLT(&pub_KLT_int,cur_image,cur_corners,prev_corners,2);
        else if(mode == 2)
        publish_KLT(&pub_KLT_ran,cur_image,cur_corners,prev_corners,2);
        else
        publish_KLT(&pub_KLT_amb,cur_image,cur_corners,prev_corners,2);
        
        prev_image = cur_image.clone();
        prev_cloud = cur_cloud;
        prev_corners = cur_corners;
        prev_3D_points = cur_3D_points;
        if(prev_corners.size()<MIN_KLT_FEATURES){
            // std::cout << "GET NEW CORNERS" << std::endl;
            cv::goodFeaturesToTrack(prev_image,prev_corners,MAX_KLT_FEATURES,0.2,7,MASK,7,true,0.04);
            get_3D_points(prev_corners,prev_3D_points,prev_cloud);
            comparison = 1;
        }
        else{
            // std::cout << "All Goodie" << std::endl;
        }
    }
}

void KLT::store_coordinates(const Vector3d& t, const Matrix3d& R){

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
        OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/prediction_steps.csv",ios_base::app);
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
        
        OUT.open("/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/prediction_complete.csv",ios_base::app);
        OUT << my_pose(0,3) << "," << my_pose(1,3) << "," << my_pose(2,3) << "," << eac(0)<< "," << eac(1)<< "," << eac(2) << "," << raw_time << endl;
        OUT.close();
    }

//visualization methods:

void KLT::publish_KLT(ros::Publisher* publisher, cv::Mat& cur_image,
        const vector<cv::Point2f>& cur_keypoints, const vector<cv::Point2f>& prev_keypoints,
        int circle_size){
        cv::Mat image = cur_image;
        cv::cvtColor(image,image,CV_GRAY2RGB);
        for(int i = 0; i < cur_corners.size();i++){
            cv::Scalar line_color = color_vector.at(i);
            cv::circle(image,cur_corners[i],circle_size,line_color,2,8,0);
            cv::line(image,cur_corners[i],prev_corners[i],line_color,1,8,0);
        }
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
        cout << "TF Publisher " << endl;
        cout << my_pose << endl;
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

//SVD:

void KLT::SVD(MatrixXd& cur_SVD,MatrixXd& prev_SVD){
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
        // if(raw_time >= ros::Time(1598537680.405615616))
        store_coordinates(t,R);

        //print to see my pose after this iteration
        // std::cout << "The current coordinates are: " << std::endl << my_pose << std::endl;
    }
 
