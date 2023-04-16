#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <string_view>
#include <experimental/filesystem>
namespace image_logger{

constexpr std::string_view IMAGE_TOPIC_PARAM = "image_topic";
constexpr std::string_view CINFO_TOPIC_PARAM = "cinfo_topic";
constexpr std::string_view OUTPUT_PATH_PARAM = "output_path";
constexpr size_t MESSAGE_QUEUE = 10; 

namespace fs = std::experimental::filesystem;

class ImageLogger {
public:
    ImageLogger();

    void run() const;

private:
    void image_cb_(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle private_nh_;
    ros::Subscriber image_sub_;
    std::string image_topic_name_, cinfo_topic_name_;
    fs::path output_path_;

    size_t image_counter_ = 0;
};

};
