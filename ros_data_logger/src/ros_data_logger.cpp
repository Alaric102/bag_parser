#include "ros_data_logger/ros_data_logger.hpp"
#include <data_logger/data_logger.hpp>

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

namespace ros_data_logger{

static inline data_logger::CameraParameters make_cinfo_parameters(const std::string& topic, ros::NodeHandle& nh, const ros::Duration& timeout = ros::Duration(0))
{
    auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, nh, timeout);
    if (msg == nullptr){
        ROS_ERROR_STREAM("Message timeout: " + topic);
        throw std::invalid_argument("Message timeout: " + topic);
    }
    data_logger::CameraParameters params;
    params.camera_mat = cv::Mat(3, 3, CV_64F, (void*)msg->K.data(), 3*sizeof(double));
    params.proj_mat = cv::Mat(1, 4, CV_64F, (void*)msg->P.data(), 4*sizeof(double));
    params.dist_mat = cv::Mat(1, msg->D.size(), CV_64F, (void*)msg->D.data(), msg->D.size()*sizeof(double));
    return params;
}

BaseLoggerHandler::BaseLoggerHandler(): 
    private_nh_(ros::NodeHandle("~"))
{
    std::string output_path_name;
    private_nh_.param<std::string>(std::string(OUTPUT_PATH_PARAM), output_path_name, "");
    // output_path_ = make_save_path(output_path_name);
    // ROS_INFO_STREAM("Save folder: " << output_path_.string());
}

// fs::path BaseLoggerHandler::get_output_path() const
// {
//     return output_path_;
// }

void BaseLoggerHandler::run() const
{
    ros::Rate rate(10);
    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}

ImageLoggerHandler::ImageLoggerHandler():
    BaseLoggerHandler()
{
    std::string cinfo_topic;
    private_nh_.param<std::string>(std::string(CINFO_TOPIC_PARAM), cinfo_topic, "");
    const auto params = make_cinfo_parameters(cinfo_topic, private_nh_);

    // const auto params_path = get_output_path() / "params.yaml";
    // if (!save_camera_parameters(params, params_path.string())){
    //     ROS_WARN_STREAM("Can't save camera parameters.");
    // };

    private_nh_.param<std::string>(std::string(IMAGE_TOPIC_PARAM), image_topic_name_, "");
    ROS_INFO_STREAM("Image topic: " << image_topic_name_);
    subcriber_ = private_nh_.subscribe(image_topic_name_, MESSAGE_QUEUE, &ImageLoggerHandler::image_cb_, this);
}

void ImageLoggerHandler::image_cb_(const sensor_msgs::ImageConstPtr& msg)
{
    // cv_bridge::CvImagePtr image_ptr;
    // try{
    //     image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    //     const auto file_path = get_output_path() / (std::to_string(data_counter++) + ".png");
    //     if (cv::imwrite(file_path.string(), image_ptr->image)){
    //         ROS_INFO_STREAM("Saved: " << file_path);
    //     } else {
    //         ROS_WARN_STREAM("Can't save image.");
    //     }
    //     image_ptr->image;
    // } catch (const cv_bridge::Exception& e){
    //     ROS_ERROR_STREAM("cv_bridge error: " << e.what());
    //     return;
    // }
}

CloudLoggerHandler::CloudLoggerHandler() : BaseLoggerHandler()
{
    
}

void CloudLoggerHandler::cloud_cb_(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}

}