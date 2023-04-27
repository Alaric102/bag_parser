#pragma once
#include <data_logger/data_logger.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <string_view>

#include <opencv2/core/mat.hpp>

namespace ros_data_logger{

constexpr std::string_view IMAGE_TOPIC_PARAM = "image_topic";
constexpr std::string_view CINFO_TOPIC_PARAM = "cinfo_topic";
constexpr std::string_view OUTPUT_PATH_PARAM = "output_path";
constexpr size_t MESSAGE_QUEUE = 10; 

class BaseLoggerHandler{
public:
    BaseLoggerHandler();
    
    void run() const;
protected:
    ros::NodeHandle private_nh_;
    ros::Subscriber subcriber_;
};

class CloudLoggerHandler final : public BaseLoggerHandler {
public:
    CloudLoggerHandler();
private:
    void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr& msg);
};

class ImageLoggerHandler final : public BaseLoggerHandler {
public:
    ImageLoggerHandler();
private:
    void image_cb_(const sensor_msgs::ImageConstPtr& msg);

private:
    std::string image_topic_name_;
};

}