#ifndef EXAMPLE_NAV_BASE
#define EXAMPLE_NAV_BASE


#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <example/WaypointNavigationAction.h>

#include <actionlib/server/simple_action_server.h>

#include <vector>
#include <deque>
#include <string.h>


namespace example {


class NavBase
{
  protected:
    double position_tol, heading_tol, linear_vel_des, linear_vel_inc, angular_vel_des, angular_vel_inc;

    ros::NodeHandle nh, pnh;
    ros::Subscriber odom_sub;
    ros::Publisher path_pub, vel_pub, goal_pub;

    std::string world_frame;
    geometry_msgs::Twist vel_cmd;

    geometry_msgs::PoseStamped robot_pose;
    bool received_robot_pose;

    // path of waypoints to follow
    std::deque<geometry_msgs::PoseStamped> waypoints;

    // action server
    actionlib::SimpleActionServer<example::WaypointNavigationAction> nav_server;

    // methods
    void publishPath() const;
    void planPath();

 public:
    NavBase(std::string, ros::NodeHandle&, ros::NodeHandle&);

    void odomCB(const nav_msgs::Odometry::ConstPtr&);
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr&);
    void goalCB();
    void preemptCB();
};


} // namespace example


#endif
