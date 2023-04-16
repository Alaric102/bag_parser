#include "data_logger/data_logger.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <string_view>
#include <experimental/filesystem>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

namespace image_logger{

void make_cinfo_parameters(const std::string& topic, ros::NodeHandle& nh, const ros::Duration& timeout = ros::Duration(0))
{
    auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, nh, timeout);
    if (msg == nullptr){
        ROS_ERROR_STREAM("Message timeout: " + topic);
        throw std::invalid_argument("Message timeout: " + topic);
    }
    ROS_INFO_STREAM(msg->height << " x " << msg->width);
}

fs::path make_save_path(const std::string& folder)
{
    std::error_code err;
    auto help_create = [&err](const fs::path& path){
        if (fs::exists(path, err)){
            return;
        }
        if (!fs::create_directories(path, err)){
            ROS_ERROR_STREAM("Can't create '" << path.string() << "' folder. " << err.message());
            throw std::invalid_argument("Invalid path. Line " + std::to_string(__LINE__));
        }
        ROS_INFO_STREAM("Created '" << path.string() << "' folder");
    };

    const fs::path root_path(folder);
    help_create(root_path);

    const auto new_folder_name = "IMAGES_" + std::to_string(std::distance(root_path.begin(), root_path.end()) + 1);
    const auto new_save_path = root_path / new_folder_name;
    help_create(new_save_path);
    return new_save_path;
}

ImageLogger::ImageLogger():
private_nh_(ros::NodeHandle("~"))
{
    std::string output_path_name;
    private_nh_.param<std::string>(std::string(OUTPUT_PATH_PARAM), output_path_name, "");
    output_path_ = make_save_path(output_path_name);
    ROS_INFO_STREAM("Save folder: " << output_path_.string());

    private_nh_.param<std::string>(std::string(CINFO_TOPIC_PARAM), cinfo_topic_name_, "");
    ROS_INFO_STREAM("Camera info topic" << cinfo_topic_name_);
    make_cinfo_parameters(cinfo_topic_name_, private_nh_);

    private_nh_.param<std::string>(std::string(IMAGE_TOPIC_PARAM), image_topic_name_, "");
    ROS_INFO_STREAM("Image topic: " << image_topic_name_);
    image_sub_ = private_nh_.subscribe(image_topic_name_, MESSAGE_QUEUE, &ImageLogger::image_cb_, this);
}

void ImageLogger::run() const
{
    ros::Rate rate(10);
    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}

void ImageLogger::image_cb_(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr image_ptr;
    try{
        image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        const auto file_path = output_path_ / (std::to_string(image_counter_++) + ".png");
        if (cv::imwrite(file_path.string(), image_ptr->image)){
            ROS_INFO_STREAM("Saved: " << file_path);
        } else {
            ROS_WARN_STREAM("Can't save image.");
        }
        image_ptr->image;
    } catch (const cv_bridge::Exception& e){
        ROS_ERROR_STREAM("cv_bridge error: " << e.what());
        return;
    }
    
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_parser_node");

    image_logger::ImageLogger logger;
    logger.run();
}