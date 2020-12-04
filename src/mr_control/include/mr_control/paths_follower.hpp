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
  bool goalCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  bool stopCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  void updateControlPose();
  void cb_speed_ratio(const geometry_msgs::Twist::ConstPtr &msg);

  float distance(float x1, float y1, float x2, float y2);
  void get_closest_point();
  void update_next_point_to_visit(double dist, double epsilon_distance);

  double norm_angle(double val);
  double compute_yaw_angle(double dx, double dy);
  void errror_lat(double dx, double dy, double yaw_point);
  void error_angle(double yaw_pose, double yaw_point);

  double PID(double now, double kp, double ki, double kd, double dt, double error);

  double v_ = 1; // vitesse lineaire
  double w_ = 0.0; // vitesse angulaire
  
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
  int next_node_to_check_ = 0;
  double error_lat_;
  double error_angle_;
  double last_time_;
  double proportional_;
  double integral_ = 0;
  double derivative_;
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
