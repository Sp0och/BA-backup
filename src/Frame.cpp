#include "Frame.h"

template <typename Derived>
//Maintain all values from Derived which have their flag in status set to 1 and delete all others.
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
//copy keypoints_in to the initial point2f vector points_in
void keypointConverter(vector<cv::KeyPoint>& keypoints_in, vector<cv::Point2f>& points_in)
{
    points_in.resize(keypoints_in.size());
    for(size_t i = 0; i < keypoints_in.size(); i++)
    {
        points_in[i] = keypoints_in[i].pt;
    }
}
//Convert points to keypoints vector
void keypointConverter(vector<cv::Point2f>& points_in, vector<cv::KeyPoint>& keypoints_in)
{
    keypoints_in.resize(points_in.size());
    for(size_t i = 0; i < points_in.size(); i++)
    {
        cv::KeyPoint key;
        key.pt = points_in[i];
        keypoints_in[i] = key;
    }
}
/**
 * We mark the features in both the old and new frame and concatenate these then we publish this new double image with the feature matching lines
 * @param this_pub  is the publisher
 * @param time_stamp 
 * @param draw_lines is a bool to determine wether a line should be drawn.
 * @param circle_size defines the size of the circle to be drawn around features
 * @param line_color self-explanatory
 * @param index_old past frame
 * @param image_old current frame
 * @param features_new features of new frame I suppose
 * @param feautures_old features of old frame I suppose.
 * */
void pubMatchedImages(const ros::Publisher* this_pub,
                      const double time_stamp,
                      const bool draw_lines,
                      const int circle_size,
                      const cv::Scalar line_color,
                      const int index_new, const int index_old, 
                      const cv::Mat& image_new, const cv::Mat& image_old,
                      const vector<cv::Point2f>& features_new, const vector<cv::Point2f>& features_old)
{
    if (this_pub->getNumSubscribers() == 0)
        return;

    int gap = 1;
    //create the new frame old fram and gap inbetween
    cv::Mat gap_image(gap, image_new.size().width, CV_8UC1, cv::Scalar(255, 255, 255));
    cv::Mat gray_img, color_img;
    cv::Mat old_img = image_old;
    //vertical concatenation that we later see
    cv::vconcat(image_new, gap_image, gap_image);
    cv::vconcat(gap_image, old_img, gray_img);
    cv::cvtColor(gray_img, color_img, CV_GRAY2RGB);
    // plot features in current frame 
    for(int i = 0; i< (int)features_new.size(); i++)
    {
        cv::Point2f cur_pt = features_new[i] * MATCH_IMAGE_SCALE;
        cv::circle(color_img, cur_pt, circle_size*MATCH_IMAGE_SCALE, line_color, MATCH_IMAGE_SCALE*2);
    }
    // plot features in previous frame
    for(int i = 0; i< (int)features_old.size(); i++)
    {
        cv::Point2f old_pt = features_old[i] * MATCH_IMAGE_SCALE;
        old_pt.y += image_new.size().height + gap;
        cv::circle(color_img, old_pt, circle_size*MATCH_IMAGE_SCALE, line_color, MATCH_IMAGE_SCALE*2);
    }
    // plot lines connecting features
    if (draw_lines)
    {
        for (int i = 0; i< (int)features_new.size(); i++)
        {
            cv::Point2f old_pt = features_old[i] * MATCH_IMAGE_SCALE;
            old_pt.y += image_new.size().height + gap;
            cv::line(color_img, features_new[i] * MATCH_IMAGE_SCALE, old_pt, line_color, MATCH_IMAGE_SCALE*2, 8, 0);
        }
    }
    // plot text
    // int text_height_pos = 20;
    // double text_scale = 0.8 * MATCH_IMAGE_SCALE;
    // int text_thickness = 2 * MATCH_IMAGE_SCALE;
    // cv::Scalar text_color = cv::Scalar(255,0,255);
    // cv::putText(color_img, "Frame: " + to_string(index_new), 
    //             cv::Point2f(5, text_height_pos),
    //             cv::FONT_HERSHEY_SIMPLEX,  text_scale, text_color, text_thickness);
    // cv::putText(color_img, "Frame: " + to_string(index_old),
    //             cv::Point2f(5, text_height_pos + image_new.size().height + gap),
    //             cv::FONT_HERSHEY_SIMPLEX,  text_scale, text_color, text_thickness);
    // publish matched image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_img).toImageMsg();
    msg->header.stamp = ros::Time(time_stamp);
    this_pub->publish(msg);
}





















// create Frame online
Frame::Frame(double _time_stamp, int _index, const cv::Mat &_image_intensity, const pcl::PointCloud<PointType>::Ptr _cloud)
{
    time_stamp = _time_stamp;
    index = _index;
    cloud = _cloud;
    image = _image_intensity.clone();
    image_intensity = _image_intensity.clone();
    cv::resize(image, thumbnail, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);

    #pragma omp parallel sections num_threads(NUM_THREADS)
    {
        #pragma omp section
        computeWindowOrbPoint();
        #pragma omp section
        computeWindowBriefPoint();
        #pragma omp section
        computeSearchOrbPoint();
        #pragma omp section
        computeSearchBriefPoint();
        // #pragma omp section
        // computeBoWPoint();
    }
    //needed for the computes now released
    image_intensity.release();
    if(!DEBUG_IMAGE)
        image.release();
}
//declares ORB descriptor_and_images
void Frame::computeWindowOrbPoint()
{
    if (USE_ORB)
    {
        // ORB features
        vector<uchar> status;
        //Orb constructor for the ORB class
        cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES, 1.2f, 8, 1);
        //get the keypoints
        detector->detect(image_intensity, orb_window_keypoints, MASK);
        //put keypoints into 2d point vector orb_point_2d_uv
        keypointConverter(orb_window_keypoints, orb_point_2d_uv);
        //get 3d and normalized 2d info of the keypoints.
        extractPoints(orb_point_2d_uv, orb_point_3d, orb_point_2d_norm, status);
        //filter the vectors
        reduceVector(orb_point_3d, status);
        reduceVector(orb_point_2d_uv, status);
        reduceVector(orb_point_2d_norm, status);
        reduceVector(orb_window_keypoints, status);
        //get the orb descriptor_and_images for this image's keypoints: orb_window_descriptor_and_images
        detector->compute(image_intensity, orb_window_keypoints, orb_window_descriptor_and_images);
    }
}

void Frame::computeWindowBriefPoint()
{
    if (USE_BRIEF)
    {
        // Corner features
        vector<uchar> status;
        //find most prominent corners in image
        cv::goodFeaturesToTrack(image_intensity, brief_point_2d_uv, NUM_BRI_FEATURES, 0.01, 10, MASK);
        extractPoints(brief_point_2d_uv, brief_point_3d, brief_point_2d_norm, status);
        reduceVector(brief_point_3d, status);
        reduceVector(brief_point_2d_uv, status);
        reduceVector(brief_point_2d_norm, status);
        keypointConverter(brief_point_2d_uv, brief_window_keypoints);
        briefExtractor(image_intensity, brief_window_keypoints, brief_window_descriptor_and_images);
    }
}
//comparison descriptor_and_images ORB
void Frame::computeSearchOrbPoint()
{
    if (USE_ORB)
    {
        // ORB features
        vector<uchar> status;
        cv::Ptr<cv::ORB> detector = cv::ORB::create(NUM_ORB_FEATURES*5, 1.2f, 8, 5);
        detector->detect(image_intensity, search_orb_keypoints, MASK);
        keypointConverter(search_orb_keypoints, search_orb_point_2d_uv);
        extractPoints(search_orb_point_2d_uv, search_orb_point_3d, search_orb_point_2d_norm, status);
        reduceVector(search_orb_point_3d, status);
        reduceVector(search_orb_point_2d_uv, status);
        reduceVector(search_orb_point_2d_norm, status);
        reduceVector(search_orb_keypoints, status);
        detector->compute(image_intensity, search_orb_keypoints, search_orb_descriptor_and_images);
    }
}
//comparison descriptor_and_images BRIEF
void Frame::computeSearchBriefPoint()
{
    if (USE_BRIEF)
    {
        // Corner feautres
        vector<uchar> status;
        cv::goodFeaturesToTrack(image_intensity, search_brief_point_2d_uv, NUM_BRI_FEATURES*5, 0.01, 2, MASK);
        extractPoints(search_brief_point_2d_uv, search_brief_point_3d, search_brief_point_2d_norm, status);
        reduceVector(search_brief_point_3d, status);
        reduceVector(search_brief_point_2d_uv, status);
        reduceVector(search_brief_point_2d_norm, status);
        keypointConverter(search_brief_point_2d_uv, search_brief_keypoints);
        briefExtractor(image_intensity, search_brief_keypoints, search_brief_descriptor_and_images);
    }
}

/* void Frame::computeBoWPoint()
{
    cv::Mat descriptor_and_images;
    vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(2500, 1.2f, 8, 1);
    detector->detectAndCompute(image, MASK, keypoints, descriptor_and_images);

    bow_descriptor_and_images.resize(descriptor_and_images.rows);
    for (int i = 0; i < descriptor_and_images.rows; ++i)
        bow_descriptor_and_images[i] = descriptor_and_images.row(i);

    if (DEBUG_IMAGE)
    {
        cv::Mat imgShow;
        cv::drawKeypoints(image_intensity, keypoints, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
        cv::resize(imgShow, imgShow, cv::Size(), 2, 2);
        cv::imshow("BOW Keypoints", imgShow );
        cv::waitKey(10);
    }
} */
//Find (and draw) connection from the current Frames descriptor_and_images to the last frames using either BRIEF or ORB descriptor_and_images
bool Frame::findConnection(Frame* old_kf)
{
    // visualize ORB features on two matched images from DBoW2
    // pubMatchedImages(&pub_bow_img, time_stamp, false, 1, cv::Scalar(0, 255, 0), index, old_kf->index, thumbnail, old_kf->thumbnail, search_orb_point_2d_uv, old_kf->search_orb_point_2d_uv);

    if (!USE_ORB && !USE_BRIEF)
    {
        ROS_ERROR("No descriptor_and_image is used!");
        return false;
    }
    
    // ORB matching
    if (USE_ORB)
    {
        vector<cv::DMatch> matches, good_matches; 
        cv::BFMatcher matcher = cv::BFMatcher(cv::NORM_HAMMING); // https://docs.opencv.org/3.3.1/d3/da1/classcv_1_1BFMatcher.html
        //get orb descriptor_and_image matches and determine the quality using the hamming distance
        matcher.match(orb_window_descriptor_and_images, old_kf->search_orb_descriptor_and_images, matches);
        //sort in ascending order, neglect all the matches above a certain matching distance.
        std::sort(matches.begin(), matches.end());
        for (size_t i = 0; i < matches.size(); ++i)
        {
            good_matches.push_back(matches[i]);
            if (matches[i].distance > matches[0].distance * 2)
                break;
        }
        //if still too many matches apply ransac
        if ((int)good_matches.size() > MIN_LOOP_FEATURE_NUM)
        {
            vector<uchar> status;
            vector<cv::Point3f> matched_3d;
            vector<cv::Point2f> matched_2d_cur, matched_2d_old, matched_2d_old_norm;

            for (size_t i=0; i < good_matches.size(); i++)
            {
                int cur_index = good_matches[i].queryIdx;
                matched_3d.push_back(orb_point_3d[cur_index]);
                matched_2d_cur.push_back(orb_point_2d_uv[cur_index]);
                
                int old_index = good_matches[i].trainIdx;
                matched_2d_old.push_back(old_kf->search_orb_point_2d_uv[old_index]);
                matched_2d_old_norm.push_back(old_kf->search_orb_point_2d_norm[old_index]);
            }

            pubMatchedImages(&pub_prepnp_img, time_stamp, true, 5, cv::Scalar(0, 255, 0), index, old_kf->index, thumbnail, old_kf->thumbnail, matched_2d_cur, matched_2d_old);
            //apply ransac to determine which matches to remove
            PnPRANSAC(matched_2d_old_norm, matched_3d, status);
            //remove bad matches
            reduceVector(matched_3d, status);
            reduceVector(matched_2d_cur, status);
            reduceVector(matched_2d_old, status);
            reduceVector(matched_2d_old_norm, status);

            if ((int)matched_2d_cur.size() > MIN_LOOP_FEATURE_NUM && distributionValidation(matched_2d_cur, matched_2d_old))
            {
                //publish post pnp image
                pubMatchedImages(&pub_match_img, time_stamp, true, 5, cv::Scalar(0, 255, 0), index, old_kf->index, thumbnail, old_kf->thumbnail, matched_2d_cur, matched_2d_old);
                return true;
            }
        }
    }

    // BRIEF matching
    if (USE_BRIEF)
    {
        vector<uchar> status;
        vector<cv::Point3f> matched_3d;
        vector<cv::Point2f> matched_2d_cur, matched_2d_old, matched_2d_old_norm;

        matched_3d = brief_point_3d;
        matched_2d_cur = brief_point_2d_uv;

        searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, brief_window_descriptor_and_images, old_kf->search_brief_descriptor_and_images, old_kf->search_brief_point_2d_uv, old_kf->search_brief_point_2d_norm);
        reduceVector(matched_3d, status);
        reduceVector(matched_2d_cur, status);
        reduceVector(matched_2d_old, status);
        reduceVector(matched_2d_old_norm, status);

        pubMatchedImages(&pub_prepnp_img, time_stamp, true, 5, cv::Scalar(0, 255, 0), index, old_kf->index, thumbnail, old_kf->thumbnail, matched_2d_cur, matched_2d_old);

        if ((int)matched_2d_cur.size() > MIN_LOOP_FEATURE_NUM)
        {
            status.clear();
            PnPRANSAC(matched_2d_old_norm, matched_3d, status);
            reduceVector(matched_3d, status);
            reduceVector(matched_2d_cur, status);
            reduceVector(matched_2d_old, status);
            reduceVector(matched_2d_old_norm, status);

            if ((int)matched_2d_cur.size() > MIN_LOOP_FEATURE_NUM && distributionValidation(matched_2d_cur, matched_2d_old))
            {
                pubMatchedImages(&pub_match_img, time_stamp, true, 5, cv::Scalar(0, 255, 0), index, old_kf->index, thumbnail, old_kf->thumbnail, matched_2d_cur, matched_2d_old);
                return true;
            }
        }
    }

    return false;
}


void Frame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                         const std::vector<cv::Point3f> &matched_3d,
                         std::vector<uchar> &status)
{
    cv::Mat r, rvec, tvec, D, inliers;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);

    solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, tvec, false, 100, 0.025, 0.99, inliers);

    status.resize(matched_2d_old_norm.size(), 0);
    for( int i = 0; i < inliers.rows; i++)
    {
        int n = inliers.at<int>(i);
        status[n] = 1;
    }
}
/**
 * Take the image projections from the image_handler to find keypoints (x,y) then use the pointcloud vector from the image_handler to get the 3d and normalized 2d coordinates and store them.
 * @param in_point_2d_uv the detected keypoints (either orb or brief)
 * @param out_point_3d vector containing the 3d info of the keypoints
 * @param out_point_2d_norm vector containing the normalized 2d information of the keypoints
 * @param flag vector to know which values to keep and which to delete
 * */
void Frame::extractPoints(const vector<cv::Point2f>& in_point_2d_uv, 
                        vector<cv::Point3f>& out_point_3d,
                        vector<cv::Point2f>& out_point_2d_norm,
                        vector<uchar>& out_status)
{
    assert(cloud->size() > 0);
    //resize the output vectors to the initial keypoint vector
    out_point_3d.resize(in_point_2d_uv.size());
    out_point_2d_norm.resize(in_point_2d_uv.size());
    out_status.resize(in_point_2d_uv.size());

    #pragma omp parallel for num_threads(NUM_THREADS)
    //iterate through found keypoints
    for (size_t i = 0; i < in_point_2d_uv.size(); ++i)
    {
        //rounding float to integer to get row and column ids
        int col_id = cvRound(in_point_2d_uv[i].x);
        int row_id = cvRound(in_point_2d_uv[i].y);
        int index = row_id * IMAGE_WIDTH + col_id;
        //pointer to the point in the pointcloud at the location of the keypoint
        PointType *pi = &cloud->points[index];

        cv::Point3f p_3d(0.f, 0.f, 0.f);
        cv::Point2f p_2d_n(0.f, 0.f);
        //why? cutting off left edge?
        if (abs(pi->x) < 0.01)
        {
            out_status[i] = 0;
        } 
        else 
        {
            //bring points into camera mode
            out_status[i] = 1;
            // lidar -> camera
            p_3d.x = -pi->y;
            p_3d.y = -pi->z;
            p_3d.z = pi->x;
            // normalize to projection plane
            p_2d_n.x = p_3d.x / p_3d.z;
            p_2d_n.y = p_3d.y / p_3d.z;
        }
        
        //set points into 3d and normed 2d vectors
        out_point_3d[i] = p_3d;
        out_point_2d_norm[i] = p_2d_n;
    }
}
//returns whether there is a good match of a current window desriptor with one of the old descriptor_and_images and assigns this best match to best_match and its norm to best_match_norm
bool Frame::searchInAera(const BRIEF::bitset window_descriptor_and_image,
                            const std::vector<BRIEF::bitset> &descriptor_and_images_old,
                            const std::vector<cv::Point2f> &keypoints_old,
                            const std::vector<cv::Point2f> &keypoints_old_norm,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm)
{
    cv::Point2f best_pt;
    int bestDist = 128;
    int bestIndex = -1;
    //iterate through all descriptor_and_images and find the closest match
    for(int i = 0; i < (int)descriptor_and_images_old.size(); i++)
    {
        int dis = HammingDis(window_descriptor_and_image, descriptor_and_images_old[i]);
        if(dis < bestDist)
        {
            bestDist = dis;
            bestIndex = i;
        }
    }
    //If a single good match has been found return true.
    if (bestIndex != -1 && bestDist < 80)
    {
        best_match = keypoints_old[bestIndex];
        best_match_norm = keypoints_old_norm[bestIndex];
        return true;
    }
    else
        return false;
}
//go through all current descriptor_and_images and look for matches with the old ones.
void Frame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
                                std::vector<cv::Point2f> &matched_2d_old_norm,
                                std::vector<uchar> &status,
                                const std::vector<BRIEF::bitset> &descriptor_and_images_now,
                                const std::vector<BRIEF::bitset> &descriptor_and_images_old,
                                const std::vector<cv::Point2f> &keypoints_old,
                                const std::vector<cv::Point2f> &keypoints_old_norm)
{
    status.resize(descriptor_and_images_now.size());
    matched_2d_old.resize(descriptor_and_images_now.size());
    matched_2d_old_norm.resize(descriptor_and_images_now.size());
    //multithread: go through the descriptor_and_images of the current Frame and check for each if there is a match by using searchInArea
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(size_t i = 0; i < descriptor_and_images_now.size(); i++)
    {
        cv::Point2f pt(0.f, 0.f);
        cv::Point2f pt_norm(0.f, 0.f);
        if (searchInAera(descriptor_and_images_now[i], descriptor_and_images_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
            status[i] = 1;
        else
            status[i] = 0;
        //assign matches
        matched_2d_old[i] = pt;
        matched_2d_old_norm[i] = pt_norm;
    }
}
//obvious
int Frame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    //use binary XOR to determine the hamming distance
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}
//check if covariance matrices of the two frames are more different in a direction than a certain threshold
bool Frame::distributionValidation(const vector<cv::Point2f>& new_point_2d_uv, const vector<cv::Point2f>& old_point_2d_uv)
{
    if (new_point_2d_uv.empty())
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    for (size_t i = 0; i < new_point_2d_uv.size(); ++i)
    {
        pcl::PointXYZ p;
        p.x = new_point_2d_uv[i].x;
        p.y = new_point_2d_uv[i].y;
        p.z = 0;
        new_cloud->push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    for (size_t i = 0; i < old_point_2d_uv.size(); ++i)
    {
        pcl::PointXYZ p;
        p.x = old_point_2d_uv[i].x;
        p.y = old_point_2d_uv[i].y;
        p.z = 0;
        old_cloud->push_back(p);
    }

    Eigen::Vector4f new_xyz_centroid;
    Eigen::Matrix3f new_covariance_matrix;
    pcl::compute3DCentroid(*new_cloud, new_xyz_centroid);
    pcl::computeCovarianceMatrix(*new_cloud, new_xyz_centroid, new_covariance_matrix); 

    Eigen::Vector4f old_xyz_centroid;
    Eigen::Matrix3f old_covariance_matrix;
    pcl::compute3DCentroid(*old_cloud, old_xyz_centroid);
    pcl::computeCovarianceMatrix(*old_cloud, old_xyz_centroid, old_covariance_matrix);

    float new_cov_x = sqrt(new_covariance_matrix(0,0));
    float new_cov_y = sqrt(new_covariance_matrix(1,1));
    float old_cov_x = sqrt(old_covariance_matrix(0,0));
    float old_cov_y = sqrt(old_covariance_matrix(1,1));
    float cov_x_diff = abs(new_cov_x - old_cov_x);
    float cov_y_diff = abs(new_cov_y - old_cov_y);
    if (cov_x_diff > 3 * std::min(new_cov_x, old_cov_x) || cov_y_diff > 0.75 * std::min(new_cov_y, old_cov_y))
    {
        return false;
    }

    return true;
}

void Frame::freeMemory()
{
    // these points are not used again
    brief_point_3d.clear(); brief_point_3d.shrink_to_fit();
    brief_point_2d_uv.clear(); brief_point_2d_uv.shrink_to_fit();
    brief_point_2d_norm.clear(); brief_point_2d_norm.shrink_to_fit();
    brief_window_keypoints.clear(); brief_window_keypoints.shrink_to_fit();
    brief_window_descriptor_and_images.clear(); brief_window_descriptor_and_images.shrink_to_fit();

    search_brief_point_3d.clear(); search_brief_point_3d.shrink_to_fit();
    search_brief_keypoints.clear(); search_brief_keypoints.shrink_to_fit();

    orb_point_3d.clear(); orb_point_3d.shrink_to_fit();
    orb_point_2d_uv.clear(); orb_point_2d_uv.shrink_to_fit();
    orb_point_2d_norm.clear(); orb_point_2d_norm.shrink_to_fit();
    orb_window_keypoints.clear(); orb_window_keypoints.shrink_to_fit();
    orb_window_descriptor_and_images.release();

    search_orb_point_3d.clear(); search_orb_point_3d.shrink_to_fit();
    search_orb_keypoints.clear(); search_orb_keypoints.shrink_to_fit();

    // bow_descriptor_and_images.clear(); bow_descriptor_and_images.shrink_to_fit();
}