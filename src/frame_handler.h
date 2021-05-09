
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
/**
 * Framehandler: creates matches between two consecutive frames, publishes these matches 
 * and uses them to indicate the points to use for ICP, also implements ICP
 */
class Framehandler{

    public:

    Framehandler(int _mode){
        mode = _mode;
        cur_orb = nullptr;
        prev_orb = nullptr;
        // match_publisher = n_frame.advertise<sensor_msgs::Image>("orb_matches", 1);
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
            create_matches();
        }
    }

    void create_matches(){
        static cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        // cv::BFMatcher matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        //create matches
        matcher->match(cur_orb->orb_descriptors,prev_orb->orb_descriptors,matches);
        //sort matches in ascending order (concerning distance)
        std::sort(matches.begin(),matches.end());
        //prefiltering for distance
        for (size_t i = 0; i < matches.size(); i++)
        {
            good_matches.push_back(matches[i]);
            // if(matches[i].distance > matches[0].distance * 3)
            // break;
        }

        matches.clear();
        std::vector<cv::Point2d> sorted_2d_cur, sorted_2d_prev;
        MatrixXd prev_ICP(3,good_matches.size()),cur_ICP(3,good_matches.size());
        //pair the keypoints up according to the matches:
        // #pragma omp parallel for num_threads(NUM_THREADS)
        for (size_t i = 0; i < good_matches.size(); i++)
        {
            //the match indexes for point association
            int cur_index = good_matches[i].queryIdx;
            int prev_index = good_matches[i].trainIdx;

            sorted_2d_cur.push_back(cur_orb->orb_keypoints_2d[cur_index]);
            sorted_2d_prev.push_back(prev_orb->orb_keypoints_2d[prev_index]);
            prev_ICP(0,i) = prev_orb->orb_points_3d.at(prev_index).x;
            prev_ICP(1,i) = prev_orb->orb_points_3d.at(prev_index).y;
            prev_ICP(2,i) = prev_orb->orb_points_3d.at(prev_index).z;
            cur_ICP(0,i) = cur_orb->orb_points_3d.at(cur_index).x;
            cur_ICP(1,i) = cur_orb->orb_points_3d.at(cur_index).y;
            cur_ICP(2,i) = cur_orb->orb_points_3d.at(cur_index).z;
            //Print tests:
            // std::cout << "Cur 2D coord: " << cur_orb->orb_keypoints_2d[cur_index] << std::endl;
            // std::cout << "Prev 2D coord: " << prev_orb->orb_keypoints_2d[prev_index] << std::endl;
            // std::cout << "Prev 3D coord: " << prev_orb->orb_points_3d.at(prev_index) << std::endl;
            // std::cout << "Cur 3D coord: " << cur_orb->orb_points_3d.at(cur_index) << std::endl;
            // std::cout << "Prev 3D matrix coord: " << prev_ICP(0,i) << " " << prev_ICP(1,i) << " " << prev_ICP(2,i) << std::endl;
            // std::cout << "Cur 3D matrix coord: " << cur_ICP(0,i) << " " << cur_ICP(1,i) << " " << cur_ICP(2,i) << std::endl;
        }
        good_matches.clear();
        
        // publish_matches(&match_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        //Homography RANSAC
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
        // for(size_t i = 0; i < 100; i += 10){
        //     // std::cout << "Cur Keypoints after sorting: " << sorted_2d_cur.at(i) << std::endl << "Previous Keypoints after sorting: " << sorted_2d_prev.at(i) << std::endl;
        //     std::cout << "Cur 3D points after sorting: " << cur_ICP(0,i) << " " << cur_ICP(1,i) << " " << cur_ICP(2,i) << std::endl;
        //     std::cout << "Prev 3D points after sorting: " << prev_ICP(0,i) << " " << prev_ICP(1,i) << " " << prev_ICP(2,i) << std::endl;
        // }
        //ICP CALL
        if(cur_ICP.size() == prev_ICP.size() && cur_ICP.size() != 0)
            ICP(cur_ICP,prev_ICP);
        else    
            std::cout << "ERROR: 3D Vectors weren't initialized properly" << std::endl;


        // if(mode == 1)
        // publish_matches_2F(&intensity_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        // else if(mode == 2)
        // publish_matches_2F(&range_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        // else
        // publish_matches_2F(&ambient_publisher, sorted_2d_cur, sorted_2d_prev,5,cv::Scalar(0,255,0),true);
        if(mode == 1)
        publish_matches_1F(&intensity_publisher, sorted_2d_cur, sorted_2d_prev,2,true);
        else if(mode == 2)
        publish_matches_1F(&range_publisher, sorted_2d_cur, sorted_2d_prev,2,true);
        else
        publish_matches_1F(&ambient_publisher, sorted_2d_cur, sorted_2d_prev,2,true);
    
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
        #pragma omp parallel for num_threads(NUM_THREADS)
        for (int i = 0; i< (int)sorted_KP_cur.size(); i++)
        {
            cv::Point2d old_pt = sorted_KP_prev[i] * MATCH_IMAGE_SCALE;
            cv::Point2d cur_pt = sorted_KP_cur[i] * MATCH_IMAGE_SCALE;
            cv::circle(image, cur_pt, circle_size*MATCH_IMAGE_SCALE, cv::Scalar(0,255,0), MATCH_IMAGE_SCALE*2);
            cv::line(image, cur_pt * MATCH_IMAGE_SCALE, old_pt, cv::Scalar(255,0,0), MATCH_IMAGE_SCALE*2, 8, 0);
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
        // #pragma omp parallel for num_threads(NUM_THREADS)
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
        // std::cout << mean_cur << std::endl;   
        // std::cout << std::endl << "Before averaging: " << cur_ICP << std::endl;
        // #pragma omp parallel for num_threads(NUM_THREADS)
        for(int i = 0; i < prev_ICP.cols();i++){
            prev_ICP(0,i) -= mean_prev(0);
            prev_ICP(1,i) -= mean_prev(1);
            prev_ICP(2,i) -= mean_prev(2);
            cur_ICP(0,i) -= mean_cur(0);
            cur_ICP(1,i) -= mean_cur(1);
            cur_ICP(2,i) -= mean_cur(2);
        }

        // Vector3d sum_prev_2(0,0,0);
        // for(int i = 0;i < prev_ICP.cols();i++){
        //     sum_prev_2(0) += prev_ICP(0,i);
        //     sum_prev_2(1) += prev_ICP(1,i);
        //     sum_prev_2(2) += prev_ICP(2,i);
        // }
        // //create the mean for subtraction
        // Vector3d mean_prev_2 = sum_prev_2/prev_ICP.cols(); 
        // std::cout << std::endl << "After averaging: " << mean_prev_2 << std::endl;

        // std::cout << "After Averaging: " << cur_ICP << std::endl;
        // std::cout << "Prev: " << prev_ICP << std::endl;
        MatrixXd W;
        W = cur_ICP*prev_ICP.transpose();
        std::cout << "W:  " << std::endl << W << std::endl;
        JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
        MatrixXd R = svd.matrixU()*svd.matrixV().transpose();
        MatrixXd t = mean_cur - R*mean_prev;
        std::cout << "The rotation matrix is: " << std::endl << R << std::endl;
        std::cout << "The translation vector is: " << std::endl << t << std::endl;
    }

    
    private:

    std::shared_ptr<ORB> cur_orb;
    std::shared_ptr<ORB> prev_orb;
    vector<cv::DMatch> matches, good_matches; 
    int mode;



    ros::Publisher match_publisher;
    ros::Publisher range_publisher;
    ros::Publisher ambient_publisher;
    ros::Publisher intensity_publisher;
    ros::NodeHandle n_frame;
    


};