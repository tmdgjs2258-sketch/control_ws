#include "pure_pursuit/pure_pursuit.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pure_pursuit;

    ROS_INFO_STREAM("pure_pursuit_node ready");

    ros::spin();

    return 0;
}