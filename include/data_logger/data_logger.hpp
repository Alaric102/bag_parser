#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <string_view>
#include <experimental/filesystem>

#include <opencv2/core/mat.hpp>

namespace data_logger{

constexpr std::string_view IMAGE_TOPIC_PARAM = "image_topic";
constexpr std::string_view CINFO_TOPIC_PARAM = "cinfo_topic";
constexpr std::string_view OUTPUT_PATH_PARAM = "output_path";
constexpr size_t MESSAGE_QUEUE = 10; 

namespace fs = std::experimental::filesystem;

struct CameraParameters{
    cv::Mat camera_mat, dist_mat, proj_mat;
};

class BaseLogger{
public:
    BaseLogger();
    
    void run() const;

    fs::path get_output_path() const;
    
protected:
    ros::NodeHandle private_nh_;
    ros::Subscriber subcriber_;
    size_t data_counter = 0;
private:
    fs::path output_path_;
};

class CloudLogger final : BaseLogger {
public:
    CloudLogger();
};

class ImageLogger final : public BaseLogger {
public:
    ImageLogger();
private:
    void image_cb_(const sensor_msgs::ImageConstPtr& msg);

private:
    std::string image_topic_name_;
};

}