#include <dirent.h>
#include <sys/types.h>
#include <ostream>
#include <chrono>
#include <thread>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <ros/package.h>
#include <angles/angles.h>

#include "mr_control/paths_follower.hpp"
#include "mr_control/csv_reader.hpp"

bool must_stop = false;
bool same_stop_sign = false;

template <typename T1>
void yawToQuatRos(const double &yaw, T1 &q)
{
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(yaw * 0.5);
  q.w = cos(yaw * 0.5);
}

template <typename T1>
double quatRosToYaw(const T1 &q)
{
  // yaw (z-axis rotation)
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return atan2(siny_cosp, cosy_cosp);
}

std::pair<nav_msgs::Path, geometry_msgs::PoseArray> rosPathConverter(const std::vector<coords> &data)
{
  nav_msgs::Path path_to_publish;
  geometry_msgs::PoseArray pose_array_to_publish;
  path_to_publish.header.frame_id = "map";
  pose_array_to_publish.header.frame_id = "map";
  for (auto &&d : data)
  {
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = std::get<0>(d);
    ps.pose.position.y = std::get<1>(d);
    ps.pose.position.z = 0.0;
    yawToQuatRos(std::get<2>(d), ps.pose.orientation);
    path_to_publish.poses.push_back(ps);
    pose_array_to_publish.poses.push_back(ps.pose);
  }
  return {path_to_publish, pose_array_to_publish};
}

std::vector<coords> pathConverter(const std::vector<std::vector<std::string>> &raw_data)
{
  std::vector<coords> res(raw_data.size());
  std::size_t i = 0;
  for (auto &&line : raw_data)
    res[i++] = {stod(line[0]), stod(line[1]), stod(line[2])};
  return res;
}

PathsFollower::PathsFollower(ros::NodeHandle &nh, const ros::NodeHandle &nh_p)
    : nh_(nh), nh_p_(nh_p), path_(""), odom_to_map_listener_(tfBuffer_)
{
  timer_ = nh.createTimer(ros::Duration(0.1), &PathsFollower::controlLoop, this, false, false);
  srv_ = nh_p_.advertiseService("Goal", &PathsFollower::goalCB, this);
  stop_srv_ = nh_p_.advertiseService("Stop", &PathsFollower::stopCB, this);
  pub_left_ = nh_.advertise<std_msgs::Float64>("/gazebo/lwheel_traction_controller/command", 1);
  pub_right_ = nh_.advertise<std_msgs::Float64>("/gazebo/rwheel_traction_controller/command", 1);
  pub_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/paths_follower/tracked_point", 1);
  pub_path_ = nh_.advertise<nav_msgs::Path>("/path", 1, true);
  pub_path_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/path_poses", 1, true);
  is_stop_sign = nh_.subscribe("/is_stop_sign", 1, stopSignCallback);
  loadPath();
}

void stopSignCallback(const std_msgs::Bool::ConstPtr& is_stop_sign_value)
{
    const bool is_stop_sign_bool = is_stop_sign_value->data;

    if (is_stop_sign_bool && !same_stop_sign) {
        must_stop = true;
        same_stop_sign = true;
    } else if (!is_stop_sign_bool && same_stop_sign) {
        same_stop_sign = false;
    }
}

void PathsFollower::updateControlPose()
{
  try
  {
    auto &&ts = tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));
    pose_ = {ts.transform.translation.x, ts.transform.translation.y, quatRosToYaw(ts.transform.rotation)};
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
}

float PathsFollower::distance(float x1, float y1, float x2, float y2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) * 1.0);
}

void PathsFollower::print_path()
{
  for (int i = 0; i < path_to_follow_.size(); i++)
  {
    ROS_INFO_STREAM("POS: " << i);
    ROS_INFO_STREAM("X: " << std::get<0>(path_to_follow_[i]));
    ROS_INFO_STREAM("Y: " << std::get<1>(path_to_follow_[i]));
    ROS_INFO_STREAM("Z: " << std::get<2>(path_to_follow_[i]));
  }
}

void PathsFollower::get_closest_point()
{
  updateControlPose();
  float x1 = std::get<0>(pose_);
  float y1 = std::get<1>(pose_);

  double d_min = distance(std::get<0>(path_to_follow_[0]),
                   std::get<1>(path_to_follow_[0]),
                   std::get<0>(path_to_follow_[1]),
                   std::get<1>(path_to_follow_[1]));

  for (int i = 1; i < path_to_follow_.size(); i++)
  {
    float x2 = std::get<0>(path_to_follow_[i]);
    float y2 = std::get<1>(path_to_follow_[i]);
    float d = distance(x1, y1, x2, y2);
    if (d < d_min)
    {
      d_min = d;
      next_node_to_check_ = i;
    }
  }
  ROS_INFO_STREAM(d_min);
  ROS_INFO_STREAM(next_node_to_check_);
}

void PathsFollower::update_next_point_to_visit(double dist, double epsilon_distance)
{
  if (dist < epsilon_distance)
  {
    ROS_INFO_STREAM("NEXT POINT IN PATH UPDATED : " << next_node_to_check_);
    next_node_to_check_++;
  }
}

void PathsFollower::loadPath()
{
  auto path = ros::package::getPath("mr_control");
  std::ostringstream ss;
  ss << path << "/path/path_to_follow.path";
  ROS_INFO("Loading path from: %s", ss.str().c_str());
  path_ = ss.str();
  CSVReader reader{path_};
  path_to_follow_ = pathConverter(reader.getData());
  auto to_publish = rosPathConverter(path_to_follow_);
  pub_path_.publish(to_publish.first);
  pub_path_poses_.publish(to_publish.second);
  print_path();
}

bool PathsFollower::goalCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  updateControlPose();

  // TODO start to follow the path
  get_closest_point();
  timer_.start();
  last_time_ = ros::Time::now().toSec();
  integral_ = 0;
  w_ = 0;

  return true;
}

bool PathsFollower::stopCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  path_ = "";
  timer_.stop();
  std_msgs::Float64 stop;
  stop.data = 0.0;
  pub_left_.publish(stop);
  pub_right_.publish(stop);
  ROS_ERROR("aborting current path");
  return true;
}

double PathsFollower::norm_angle(double val)
{
  return angles::normalize_angle(val);
}

double PathsFollower::compute_yaw_angle(double dx, double dy)
{
  return norm_angle(std::atan2(dy, dx));
}

void PathsFollower::errror_lat(double dx, double dy, double yaw_point)
{
  const double s_yaw = std::sin(yaw_point);
  const double c_yaw = std::cos(yaw_point);
  error_lat_ = dx * s_yaw - dy * c_yaw;
}

void PathsFollower::error_angle(double yaw_pose, double yaw_point)
{
  error_angle_ = norm_angle(yaw_pose - yaw_point);
}


double PathsFollower::PID(double now, double kp, double ki, double kd, double dt, double error) {
  proportional_ = kp * error;
  integral_ += ki * error * dt;
  derivative_ = -kd * error / dt;
  return proportional_ + integral_ + derivative_;
}

std_msgs::Float64 PathsFollower::doubleToMsgs(double value) {
    std_msgs::Float64 message;
    message.data = value;
    return message;
}

void PathsFollower::stopAtStopSign(double w_first)
{
    int waiting_seconds = 5;
    int braking_intervals_milliseconds = 50;
    double decay_factor = 0.9;
    double target_v = (2 * v_ + length_ * w_first) / (2 * radius_);

    while (target_v > 0.1) {
        pub_left_.publish(doubleToMsgs(target_v));
        pub_right_.publish(doubleToMsgs(target_v));
        target_v *= decay_factor;
        std::this_thread::sleep_for(std::chrono::milliseconds(braking_intervals_milliseconds));
    }
    pub_left_.publish(doubleToMsgs(0.0));
    pub_right_.publish(doubleToMsgs(0.0));
    std::this_thread::sleep_for(std::chrono::seconds(waiting_seconds));
    must_stop = false;
}

void PathsFollower::controlLoop(const ros::TimerEvent &event)
{
  updateControlPose(); // get localization in pose_

  double now = ros::Time::now().toSec();
  double dt = now - last_time_;
  

  double x1 = std::get<0>(pose_);
  double y1 = std::get<1>(pose_);
  double x2 = std::get<0>(path_to_follow_[next_node_to_check_]);
  double y2 = std::get<1>(path_to_follow_[next_node_to_check_]);
  double dx = x1 - x2;
  double dy = y1 - y2;
  double dist = distance(x1, y1, x2, y2);
  double yaw_pose = norm_angle(std::get<2>(pose_));
  double yaw_point = norm_angle(std::get<2>(path_to_follow_[next_node_to_check_]));

  errror_lat(dx, dy, yaw_point);
  error_angle(yaw_pose, yaw_point);

  double w_lat = PID(now, -2, -0.1, -0.05, dt, error_lat_);
  double w_angle = PID(now, 1.7, 0, 0, dt, error_angle_);

  w_ = w_lat + w_angle;

  if (must_stop) {
     stopAtStopSign(w_);
  } else {
    double phi1 = (2 * v_ + length_ * w_) / (2 * radius_);
    double phi2 = (2 * v_ - length_ * w_) / (2 * radius_);
    pub_left_.publish(doubleToMsgs(phi1));
    pub_right_.publish(doubleToMsgs(phi2));
  }
  update_next_point_to_visit(dist, 0.40);
  last_time_ = now;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_follower");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");
  PathsFollower PathsFollower(nh, nh_p);

  ros::spin();

  return 0;
}
