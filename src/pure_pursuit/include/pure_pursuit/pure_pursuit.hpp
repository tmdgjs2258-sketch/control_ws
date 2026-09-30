#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <algorithm>
// #include <dynamic_reconfigure/server.h>
// #include <pure_pursuit/PurePursuitConfig.h>

class PurePursuit
{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher path_pub_;
    ros::Publisher path_pub_L_;
    ros::Subscriber pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher target_pub_; // 목표점 
    ros::Publisher trajectory_pub_; //거북이궤적

    geometry_msgs::Point goal_; // 도착점
    ros::Timer timer_;

    tf2_ros::TransformBroadcaster tf_broadcaster_; // 거북이 TF
    nav_msgs::Path path_L;
    std::vector<geometry_msgs::Point> path_; // 경로
    std::vector<geometry_msgs::Point> waypoints_; // 퍼블리쉬 포인트 저장용
    nav_msgs::Path trajectory_; 

    double x_;
    double y_;
    double theta_;
    double Ld_;
    double v_;
    int target_idx_ = 0;

    int prev_nearest_idx_ = 0;

    void poseCallback(const turtlesim::Pose::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
    
    void control_pp();
    void control_s();
    void control_app();
    void control_rpp();
    void generatePath_C();
    void generatePath_LC();
    void generatePath_L();
    void publishPath();
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void publishTargetPoint(const geometry_msgs::Point& target_p);
    void publishTurtleTF();
    // void reconfigureCallback(pure_pursuit::PurePursuitConfig &config, uint32_t level);

    // dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig> server_;
    // dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig>::CallbackType f_;
    
public:
    PurePursuit();
};