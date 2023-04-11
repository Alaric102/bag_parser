#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <string_view>

namespace {

constexpr std::string_view IMAGE_TOPIC_PARAM = "image_topic";
constexpr std::string_view CINFO_TOPIC_PARAM = "cinfo_topic";
constexpr std::string_view OUTPUT_PATH_PARAM = "output_path";
constexpr size_t MESSAGE_QUEUE = 10; 

void make_cinfo_parameters(const std::string& topic, ros::NodeHandle& nh, const ros::Duration& timeout = ros::Duration(0))
{
    auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, nh, timeout);
    if (msg == nullptr){
        ROS_INFO_STREAM("Message timeout: " + topic);
        throw std::invalid_argument("Message timeout: " + topic);
    }
    ROS_INFO_STREAM(msg->height << " x " << msg->width);
}

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_STREAM("image received");
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_parser_node");
    ros::NodeHandle private_nh("~");

    std::string image_topic_name, cinfo_topic_name, output_path;
    private_nh.param<std::string>(std::string(CINFO_TOPIC_PARAM), cinfo_topic_name, "");
    make_cinfo_parameters(cinfo_topic_name, private_nh);

    private_nh.param<std::string>(std::string(IMAGE_TOPIC_PARAM), image_topic_name, "");
    private_nh.param<std::string>(std::string(IMAGE_TOPIC_PARAM), output_path, "");
    ROS_INFO_STREAM("Image topic" << image_topic_name);
    const auto sub = private_nh.subscribe(image_topic_name, MESSAGE_QUEUE, image_cb);
    
    ros::Rate rate(10);
    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}