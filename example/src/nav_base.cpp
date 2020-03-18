#include <example/nav_base.h>

#include <nodelet/nodelet.h>
#include <ros/console.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <queue>
#include <cmath>


namespace example {


#define NB_INFO(fmt, ...) ROS_INFO("[NavBase] " fmt, ##__VA_ARGS__)
#define NB_WARN(fmt, ...) ROS_WARN("[NavBase] " fmt, ##__VA_ARGS__)
#define NB_ERROR(fmt, ...) ROS_ERROR("[NavBase] " fmt, ##__VA_ARGS__)
#define NB_FATAL(fmt, ...) ROS_FATAL("[NavBase] " fmt, ##__VA_ARGS__)
#define NB_DEBUG(fmt, ...) ROS_DEBUG("[NavBase] " fmt, ##__VA_ARGS__)


template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    NB_FATAL("failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


NavBase::NavBase(std::string name, ros::NodeHandle& nh_, ros::NodeHandle& pnh_):
  nav_server(nh_, name, false),
  nh(nh_),
  pnh(pnh_),
  received_robot_pose(false)
{
  nav_server.registerGoalCallback(std::bind(&NavBase::goalCB, this));
  nav_server.registerPreemptCallback(std::bind(&NavBase::preemptCB, this));
  nav_server.start();

  getParamStrict(pnh, "world_frame", world_frame);
  getParamStrict(pnh, "position_tol", position_tol);
  getParamStrict(pnh, "heading_tol", heading_tol);
  getParamStrict(pnh, "linear_vel_des", linear_vel_des);
  getParamStrict(pnh, "angular_vel_des", angular_vel_des);
  getParamStrict(pnh, "linear_vel_inc", linear_vel_inc);
  getParamStrict(pnh, "angular_vel_inc", angular_vel_inc);

  // set nodelet verbosity to debug if desired
  bool debug = false;
  if (pnh.getParam("debug", debug) && debug)
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
      ros::console::notifyLoggerLevelsChanged();

  odom_sub = nh.subscribe("odom", 10, &NavBase::odomCB, this);
  path_pub = nh.advertise<nav_msgs::Path>("path", 10);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("current_goal", 10);
}

void NavBase::goalCB()
{
  // abort any existing goals
  if (nav_server.isActive()) {
    NB_INFO("goal aborted");
    nav_server.setAborted();
  }

  auto action_goal = nav_server.acceptNewGoal();
  NB_INFO("accepted new goal");

  // cannot plan without the current position of the quad
  if (!received_robot_pose) {
    NB_ERROR("no pose! aborting goal");
    nav_server.setAborted();
    return;
  }

  if (nav_server.isPreemptRequested()) {
    NB_INFO("goal preempted");
    nav_server.setPreempted();
    return;
  }

  for (auto& pose : action_goal->waypoints) {
    geometry_msgs::PoseStamped wp;
    wp.header = action_goal->header;
    wp.pose = pose;
    waypoints.push_back(wp);
  }

  // visualize path in rviz
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = world_frame;
  path_msg.header.stamp = ros::Time::now();
  path_msg.poses.push_back(robot_pose);
  path_msg.poses.insert(path_msg.poses.end(), waypoints.begin(), waypoints.end());
  path_pub.publish(path_msg);

  NB_DEBUG("waypoints are:");
  for (const auto& wp : waypoints)
    NB_DEBUG("{%7.2f, %7.2f, %7.2f}", wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
}


void NavBase::preemptCB()
{
  if (nav_server.isActive()) {
    NB_INFO("goal aborted");
    nav_server.setAborted();
  } else {
    NB_INFO("goal preempted");
    nav_server.setPreempted();
  }
}


void NavBase::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::PoseStamped ps_msg;
  ps_msg.header = msg->header;
  ps_msg.pose = msg->pose.pose;
  geometry_msgs::PoseStamped::ConstPtr ptr(new geometry_msgs::PoseStamped(ps_msg));
  poseCB(ptr);
}


// actual tracking of the waypoints happens here
void NavBase::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  received_robot_pose = true;
  robot_pose = *msg;

  if (waypoints.empty())
    return;

  geometry_msgs::PoseStamped curr_wp = waypoints.front();
  curr_wp.pose.orientation.x = 0.0;
  curr_wp.pose.orientation.y = 0.0;
  curr_wp.pose.orientation.z = 0.0;
  curr_wp.pose.orientation.w = 1.0;
  geometry_msgs::Point robot_pt = robot_pose.pose.position;

  goal_pub.publish(curr_wp);

  NB_DEBUG("");

  // heading error

  double yaw_d = atan2(curr_wp.pose.position.y - robot_pt.y,
                       curr_wp.pose.position.x - robot_pt.x);
  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  double yaw_error = yaw_d - yaw;
  if (yaw_error > M_PI)
    yaw_error -= 2.0*M_PI;
  if (yaw_error < -M_PI)
    yaw_error += 2.0*M_PI;

  // position error

  double pos_error = sqrt(pow(curr_wp.pose.position.x - robot_pt.x, 2.0) +
                          pow(curr_wp.pose.position.y - robot_pt.y, 2.0));

  // velocity command with acceleration limits

  /*
  double angular_vel = yaw_error*angular_vel_des;
  if (abs(angular_vel - vel_cmd.angular.z) > angular_vel_inc)
    vel_cmd.angular.z += sgn(angular_vel)*angular_vel_inc;
  */
  vel_cmd.angular.z = yaw_error*angular_vel_des;
  if (abs(vel_cmd.angular.z) > angular_vel_des)
    vel_cmd.angular.z = sgn(vel_cmd.angular.z) * angular_vel_des;

  if (abs(yaw_error) < heading_tol) {
    vel_cmd.linear.x += linear_vel_inc;
    if (vel_cmd.linear.x > linear_vel_des)
      vel_cmd.linear.x = linear_vel_des;
  } else {
    vel_cmd.linear.x = 0.0;
  }

  // acceleration limits

  NB_DEBUG("yaw_error: %6.3f, vel_cmd.angular.z: %6.3f", yaw_error, vel_cmd.angular.z);
  NB_DEBUG("pos_error: %6.3f,  vel_cmd.linear.x: %6.3f", pos_error, vel_cmd.linear.x);
  vel_pub.publish(vel_cmd);

  // waypoint update

  if (pos_error < position_tol && !waypoints.empty()) {
    NB_DEBUG("reached waypoint: {%.2f, %.2f, %.2f}", curr_wp.pose.position.x, curr_wp.pose.position.y, curr_wp.pose.position.z);
    waypoints.pop_front();
    NB_DEBUG("next waypoint: {%.2f, %.2f, %.2f}", waypoints.front().pose.position.x, waypoints.front().pose.position.y, waypoints.front().pose.position.z);
    if (waypoints.empty()) {
      NB_DEBUG("setting goal to succeeded");
      nav_server.setSucceeded();
    }
  }
}


} // namespace example
