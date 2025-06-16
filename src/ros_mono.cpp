/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
* Modified for ROS 2 Humble
*
*/

#include "orb_slam3_ros/common.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>

using namespace std;

class ImageGrabber 
{
public:
    ImageGrabber(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        
        subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, 
            std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));
            
        RCLCPP_INFO(node_->get_logger(), "ImageGrabber initialized");
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // ORB-SLAM3 runs in TrackMonocular()
        Sophus::SE3f Tcw = pSLAM->TrackMonocular(cv_ptr->image, rclcpp::Time(msg->header.stamp).seconds());

        rclcpp::Time msg_time = msg->header.stamp;

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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("orb_slam3");
    RCLCPP_INFO(node->get_logger(), "Starting ORB-SLAM3 ROS 2 Node");

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
    sensor_type = ORB_SLAM3::System::MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);
    
    // Create image transport and publishers
    image_transport::ImageTransport image_transport(node);
    setup_publishers(node, image_transport, node_name);
    setup_services(node, node_name);

    // Create the ImageGrabber using the same node
    auto grabber = std::make_shared<ImageGrabber>(node);
    
    RCLCPP_INFO(node->get_logger(), "ORB-SLAM3 ROS 2 Node is running");

    // Spin
    rclcpp::spin(node);

    // Stop all threads
    pSLAM->Shutdown();
    rclcpp::shutdown();

    return 0;
}