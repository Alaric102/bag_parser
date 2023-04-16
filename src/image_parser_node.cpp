#include "data_logger/data_logger.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <string_view>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/emitter.h>

namespace image_logger{

CameraParameters make_cinfo_parameters(const std::string& topic, ros::NodeHandle& nh, const ros::Duration& timeout = ros::Duration(0))
{
    auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, nh, timeout);
    if (msg == nullptr){
        ROS_ERROR_STREAM("Message timeout: " + topic);
        throw std::invalid_argument("Message timeout: " + topic);
    }
    CameraParameters params;
    params.camera_mat = cv::Mat(3, 3, CV_64F, (void*)msg->K.data(), 3*sizeof(double));
    params.proj_mat = cv::Mat(1, 4, CV_64F, (void*)msg->P.data(), 4*sizeof(double));
    params.dist_mat = cv::Mat(1, msg->D.size(), CV_64F, (void*)msg->D.data(), msg->D.size()*sizeof(double));
    return params;
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

    size_t folder_num = 0;
    for (const auto & entry : fs::directory_iterator(root_path)){
        ++folder_num;
    }
    const auto new_folder_name = "IMAGES_" + std::to_string(folder_num);
    const auto new_save_path = root_path / new_folder_name;
    help_create(new_save_path);
    return new_save_path;
}

inline YAML::Emitter& operator<<(YAML::Emitter& emitter, const cv::Mat& mat) {
    emitter << YAML::Flow;
    emitter << YAML::BeginSeq;
    for (auto it = mat.begin<double>(); it != mat.end<double>(); ++it ) {
        emitter << *it;
    }
    return emitter << YAML::EndSeq;;
}

bool save_camera_parameters(const CameraParameters& params, const std::string& file_name)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "camera_matrix";
    out << params.camera_mat;

    out << YAML::Key << "projection_matrix";
    out << params.proj_mat;

    out << YAML::Key << "distortion_matrix";
    out << params.dist_mat;

    try {
        std::ofstream fout(file_name);
        fout << out.c_str();
        fout.close();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to write emitter: " << e.what());
        return false;
    }
    return true;
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
    const auto params = make_cinfo_parameters(cinfo_topic_name_, private_nh_);
    const auto params_path = output_path_ / "params.yaml";
    if (!save_camera_parameters(params, params_path.string())){
        ROS_WARN_STREAM("Can't save camera parameters.");
    };

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