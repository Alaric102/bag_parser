#include "ros_data_logger/ros_data_logger.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_parser_node");
    ros_data_logger::CloudLoggerHandler logger;
    logger.run();
}