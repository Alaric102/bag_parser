#include "ros_data_logger/ros_data_logger.hpp"
#include <data_logger/data_logger.hpp>

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

namespace ros_data_logger{

static constexpr std::string_view IMAGE_TOPIC_PARAM = "image_topic";
static constexpr std::string_view CINFO_TOPIC_PARAM = "cinfo_topic";
static constexpr std::string_view CLOUD_TOPIC_PARAM = "cloud_topic";
static constexpr std::string_view OUTPUT_PATH_PARAM = "output_path";
static constexpr size_t MESSAGE_QUEUE = 10; 

static inline data_logger::CameraParameters make_cinfo_parameters(const std::string& topic, ros::NodeHandle& nh, const ros::Duration& timeout = ros::Duration(0))
{
    ROS_INFO_STREAM("Wait message from: " << topic);
    auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, nh, timeout);
    if (msg == nullptr){
        ROS_ERROR_STREAM("Message timeout: " + topic);
        throw std::invalid_argument("Message timeout: " + topic);
    }
    data_logger::CameraParameters params;
    params.camera_mat = cv::Mat(3, 3, CV_64F, (void*)msg->K.data(), 3*sizeof(double));
    params.proj_mat = cv::Mat(3, 4, CV_64F, (void*)msg->P.data(), 4*sizeof(double));
    params.dist_mat = cv::Mat(1, msg->D.size(), CV_64F, (void*)msg->D.data(), msg->D.size()*sizeof(double));
    return params;
}

BaseLoggerHandler::BaseLoggerHandler(): 
    private_nh_(ros::NodeHandle("~"))
{
    private_nh_.param<std::string>(std::string(OUTPUT_PATH_PARAM), output_path_name_, "");
}

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
    logger_ = std::make_unique<data_logger::ImageLogger>(data_logger::ImageLogger(output_path_name_));

    std::string cinfo_topic;
    private_nh_.param<std::string>(std::string(CINFO_TOPIC_PARAM), cinfo_topic, "");
    const auto params = make_cinfo_parameters(cinfo_topic, private_nh_);    
    if (!logger_->save_cinfo(params)){
        ROS_WARN_STREAM("Can't save camera parameters.");
    }

    std::string image_topic;
    private_nh_.param<std::string>(std::string(IMAGE_TOPIC_PARAM), image_topic, "");
    ROS_INFO_STREAM("Image topic: " << image_topic);
    subcriber_ = private_nh_.subscribe(image_topic, MESSAGE_QUEUE, &ImageLoggerHandler::image_cb_, this);
}

void ImageLoggerHandler::image_cb_(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr image_ptr;
    try{
        image_ptr = cv_bridge::toCvShare(msg, "bgr8");
        if (logger_->save_image(image_ptr->image)){
            ROS_INFO_STREAM("Saved image");
        } else {
            ROS_WARN_STREAM("Can't save image.");
        }
    } catch (const cv_bridge::Exception& e){
        ROS_ERROR_STREAM("cv_bridge error: " << e.what());
        return;
    }
}

CloudLoggerHandler::CloudLoggerHandler() : BaseLoggerHandler()
{
    logger_ = std::make_unique<data_logger::CloudLogger>(data_logger::CloudLogger(output_path_name_));

    std::string cloud_topic;
    private_nh_.param<std::string>(std::string(CLOUD_TOPIC_PARAM), cloud_topic, "");
    ROS_INFO_STREAM("Cloud topic: " << cloud_topic);
    subcriber_ = private_nh_.subscribe(cloud_topic, MESSAGE_QUEUE, &CloudLoggerHandler::cloud_cb_, this);
}

void CloudLoggerHandler::cloud_cb_(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    if ( logger_->save_cloud(cloud) ){
        ROS_INFO_STREAM("Saved cloud");
    } else {
        ROS_WARN_STREAM("Can't save cloud.");
    }
}

}