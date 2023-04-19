#include "data_logger/data_logger.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_parser_node");
    data_logger::ImageLogger logger;
    logger.run();
}