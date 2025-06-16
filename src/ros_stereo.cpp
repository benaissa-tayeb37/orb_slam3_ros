/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc
* Modified for ROS 2 Humble
*
*/

#include "orb_slam3_ros/common.hpp"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <memory>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        
        // Setup subscribers for left and right images
        // Convert QoS settings to rmw_qos_profile_t
        auto qos_profile = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort).get_rmw_qos_profile();
        left_sub_.subscribe(node_, "/camera/left/image_raw", qos_profile);
        right_sub_.subscribe(node_, "/camera/right/image_raw", qos_profile);
        
        // Setup synchronizer
        sync_ = std::make_shared<Sync>(SyncPolicy(10), left_sub_, right_sub_);
        
        // Use a simple callback function approach for message_filters
        using std::placeholders::_1;
        using std::placeholders::_2;
        sync_->registerCallback(std::bind(&ImageGrabber::GrabStereoCallback, this, _1, _2));
        
        RCLCPP_INFO(node_->get_logger(), "Stereo ImageGrabber initialized");
    }
    
    // Callback method with signature matching what the synchronizer expects
    void GrabStereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft, 
                           const sensor_msgs::msg::Image::ConstSharedPtr& msgRight)
    {
        GrabStereo(msgLeft, msgRight);
    }

    void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft, 
                   const sensor_msgs::msg::Image::ConstSharedPtr& msgRight)
    {
        rclcpp::Time msg_time = msgLeft->header.stamp;
        
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
        try
        {
            cv_ptrLeft = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::MONO8);
            cv_ptrRight = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // ORB-SLAM3 runs in TrackStereo()
        Sophus::SE3f Tcw = pSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, rclcpp::Time(msg_time).seconds());

        // Publish pose and TF if tracking succeeded
        if (!Tcw.matrix().isIdentity())
        {
            // Publish pose using global publisher
            publish_camera_pose(Tcw, msg_time);
            
            // Publish TF
            publish_tf_transform(Tcw, world_frame_id, cam_frame_id, msg_time);
            
            // Publish additional data
            publish_topics(msg_time);
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Message filter subscribers and synchronizer
    typedef message_filters::Subscriber<sensor_msgs::msg::Image> ImageSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    
    ImageSubscriber left_sub_;
    ImageSubscriber right_sub_;
    std::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("orb_slam3");
    RCLCPP_INFO(node->get_logger(), "Starting ORB-SLAM3 ROS 2 Stereo Node");

    if (argc > 1)
    {
        RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are ignored.");
    }

    std::string node_name = node->get_name();

    // Declare and get parameters
    node->declare_parameter("voc_file", "file_not_set");
    node->declare_parameter("settings_file", "file_not_set");
    node->declare_parameter("world_frame_id", "map");
    node->declare_parameter("cam_frame_id", "camera");
    node->declare_parameter("enable_pangolin", true);

    std::string voc_file = node->get_parameter("voc_file").as_string();
    std::string settings_file = node->get_parameter("settings_file").as_string();

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "Please provide voc_file and settings_file in the launch file");       
        rclcpp::shutdown();
        return 1;
    }

    world_frame_id = node->get_parameter("world_frame_id").as_string();
    cam_frame_id = node->get_parameter("cam_frame_id").as_string();
    bool enable_pangolin = node->get_parameter("enable_pangolin").as_bool();

    // Create SLAM system
    sensor_type = ORB_SLAM3::System::STEREO;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);
    
    // Create image transport and publishers
    image_transport::ImageTransport image_transport(node);
    setup_publishers(node, image_transport, node_name);
    setup_services(node, node_name);

    // Create the ImageGrabber using the same node
    auto grabber = std::make_shared<ImageGrabber>(node);
    
    RCLCPP_INFO(node->get_logger(), "ORB-SLAM3 ROS 2 Stereo Node is running");

    // Spin
    rclcpp::spin(node);

    // Stop all threads
    pSLAM->Shutdown();
    rclcpp::shutdown();

    return 0;
}