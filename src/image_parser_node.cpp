#include <ros/ros.h>

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "image_parser_node");
    ros::NodeHandle nh();
    ros::NodeHandle pnh("~");

    ros::Rate rate(10);
    
    while (ros::ok()){
        ros::spinOnce();
        ROS_INFO_STREAM("Spin");
        rate.sleep();
    }
}