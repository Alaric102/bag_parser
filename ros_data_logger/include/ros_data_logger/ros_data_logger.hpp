#pragma once
#include <data_logger/data_logger.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <string_view>
#include <memory>

#include <opencv2/core/mat.hpp>

namespace ros_data_logger{

class BaseLoggerHandler{
public:
    BaseLoggerHandler();
    
    void run() const;
protected:
    ros::NodeHandle private_nh_;
    ros::Subscriber subcriber_;
    std::string output_path_name_;
};

class CloudLoggerHandler final : public BaseLoggerHandler {
public:
    CloudLoggerHandler();
private:
    void cloud_cb_(const sensor_msgs::PointCloud2ConstPtr& msg);
private:
    std::unique_ptr<data_logger::CloudLogger> logger_;
};

class ImageLoggerHandler final : public BaseLoggerHandler {
public:
    ImageLoggerHandler();
private:
    void image_cb_(const sensor_msgs::ImageConstPtr& msg);
private:
    std::unique_ptr<data_logger::ImageLogger> logger_;
};

}