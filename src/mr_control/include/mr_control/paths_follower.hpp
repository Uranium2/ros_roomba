#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>

#include "mr_control/Goal.h"

using coords = std::tuple<float, float, float>;

class PathsFollower
{
public:
  PathsFollower(ros::NodeHandle &nh, const ros::NodeHandle &nh_p);

private:
  void loadPath();
  void controlLoop(const ros::TimerEvent &event);
  void print_path();
  float distance(float x1, float y1, float x2, float y2);
  float errror_lat();
  void get_closest_point();
  bool goalCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  bool stopCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  void updateControlPose();
  double norm_angle(double val);
  void cb_speed_ratio(const geometry_msgs::Twist::ConstPtr &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;

  float max_tolerance_goal_norm_;
  float max_tolerance_goal_angle_;

  float speed_ratio_;

  // full filename and starting point
  std::string path_;
  std::vector<coords> path_to_follow_;

  ros::ServiceServer srv_;
  ros::ServiceServer stop_srv_;
  float radius_ = 0.1;
  float length_ = 0.46;
  float d_ = 999.99;
  int last_node_seen_ = -1;
  int next_node_to_check_ = 0;
  double error_lat_;
  ros::Timer timer_;

  ros::Publisher pub_left_;
  ros::Publisher pub_right_;
  ros::Publisher pub_point_;
  ros::Publisher pub_path_;
  ros::Publisher pub_path_poses_;


  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener odom_to_map_listener_;
  coords pose_;
};
