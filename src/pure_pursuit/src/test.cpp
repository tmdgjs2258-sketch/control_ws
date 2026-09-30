#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_pursuit_node");

    ros::NodeHandle nh;

    ROS_INFO("Pure Pursuit Start");

    ros::spin();

    return 0;
}