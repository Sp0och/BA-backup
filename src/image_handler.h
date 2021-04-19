#include "parameters.h"
//Create images from the laserscan
class ImageHandler
{
public:

    ros::NodeHandle nh;

    ros::Publisher pub_image;
    ros::Publisher pub_intensity;
    //pictures
    cv::Mat image_range;
    cv::Mat image_noise;
    cv::Mat image_intensity;
    //pointcloud
    pcl::PointCloud<PointType>::Ptr cloud_track;
    //class constructor: reset the pointcloud and declare a publisher
    ImageHandler()
    {
        cloud_track.reset(new pcl::PointCloud<PointType>());
        cloud_track->resize(IMAGE_HEIGHT * IMAGE_WIDTH);
        //creates the publisher publishing the image
        pub_image  = nh.advertise<sensor_msgs::Image>("image_stack", 1);
        pub_intensity  = nh.advertise<sensor_msgs::Image>("intensity_image", 1);
    }

    void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        // convert cloud
        pcl::PointCloud<PointOuster>::Ptr laser_cloud(new pcl::PointCloud<PointOuster>());
        //create the laser_cloud pointcloud from cloud_msg 
        pcl::fromROSMsg(*cloud_msg, *laser_cloud);
        //make sure the size of the laser_cloud is scalable with the image size
        assert((int)laser_cloud->size() % IMAGE_HEIGHT * IMAGE_WIDTH == 0);

        // reset images -> three images of the same size and type, all empty i suppose
        image_range = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0));
        image_noise = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0));
        image_intensity = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, cv::Scalar(0));

        #pragma omp parallel for num_threads(NUM_THREADS)
        //create the images from the pointcloud
        for (int u = 0; u < IMAGE_HEIGHT; u++) 
        {
            for (int v = 0; v < IMAGE_WIDTH; v++) 
            {   
                const auto& pt = laser_cloud->points[u * IMAGE_WIDTH + v];

                // extract sensor data into images
                float range = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
                float noise = pt.noise;
                float intensity = pt.intensity;

                // limit to (0~255)
                noise = std::min(noise, 255.0f);
                intensity = std::min(intensity, 255.0f);

                // update all images
                image_range.at<uint8_t>(u, v) = std::min(range * 20, 255.0f);
                image_noise.at<uint8_t>(u, v) = noise;
                image_intensity.at<uint8_t>(u, v) = intensity;

                // fill our pointcloud
                PointType* p = &cloud_track->points[u * IMAGE_WIDTH + v];

                if (range >= 0.1)
                {
                    p->x = pt.x;
                    p->y = pt.y;
                    p->z = pt.z;
                    p->intensity = intensity;
                }
                else
                {
                    p->x = p->y = p->z = p->intensity = 0;
                }
            }
        }
        if(pub_intensity.getNumSubscribers()!=0){
            // option 1: display intensity image
            cv::Mat intensity_visualization = image_intensity.clone();
            cv::cvtColor(intensity_visualization, intensity_visualization, CV_GRAY2RGB);
            pubImage(&pub_intensity, intensity_visualization, cloud_msg->header, "bgr8");
        }
        if (pub_image.getNumSubscribers() != 0)
        {

            // option 2: display all images from available lidar channels
            cv::Mat image_visualization;
            //concatenate image_noise, image_intensity and image_range vertically
            cv::vconcat(image_noise, image_intensity, image_visualization);
            cv::vconcat(image_visualization, image_range, image_visualization);
            cv::cvtColor(image_visualization, image_visualization, CV_GRAY2RGB);
            cv::putText(image_visualization, "Ambient",   cv::Point2d(5, 20 + IMAGE_HEIGHT*0), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
            cv::putText(image_visualization, "Intensity", cv::Point2d(5, 20 + IMAGE_HEIGHT*1), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
            cv::putText(image_visualization, "Range",     cv::Point2d(5, 20 + IMAGE_HEIGHT*2), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
            pubImage(&pub_image, image_visualization, cloud_msg->header, "bgr8");
        }
        // static tf in case tf between base_link and lidar is missing
        static tf::TransformBroadcaster tf_base_to_lidar;
        static tf::Transform base_to_lidar = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tf_base_to_lidar.sendTransform(tf::StampedTransform(base_to_lidar, cloud_msg->header.stamp, "base_link", "velodyne"));
    }

    void pubImage(ros::Publisher *this_pub, const cv::Mat& this_image, std_msgs::Header this_header, string image_format)
    {
        static cv_bridge::CvImage bridge;
        bridge.header = this_header;
        bridge.encoding = image_format;
        bridge.image = this_image;
        this_pub->publish(bridge.toImageMsg());
    }
};