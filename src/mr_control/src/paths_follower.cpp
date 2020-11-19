#include <dirent.h>
#include <sys/types.h>
#include <ostream>
#include <chrono>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <ros/package.h>
#include <angles/angles.h>

#include "mr_control/paths_follower.hpp"
#include "mr_control/csv_reader.hpp"

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
  loadPath();
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

float PathsFollower::distance(float x1, float y1, float x2, float y2){
  return std::sqrt(std::pow(x2 - x1, 2) +  
                std::pow(y2 - y1, 2) * 1.0); 
}

void PathsFollower::print_path(){
  for (int i = 0; i < path_to_follow_.size(); i++)
  {
    ROS_INFO_STREAM("POS: " << i);
    ROS_INFO_STREAM("X: " << std::get<0>(path_to_follow_[i]));
    ROS_INFO_STREAM("Y: " << std::get<1>(path_to_follow_[i]));
    ROS_INFO_STREAM("Z: " << std::get<2>(path_to_follow_[i]));
  }
}

void PathsFollower::get_closest_point(){
  d_ = std::numeric_limits<float>::max();
  updateControlPose();
  float x1 = std::get<0>(pose_);
  float y1 = std::get<1>(pose_);

  for (int i = last_node_seen_ + 1; i < std::min(int(path_to_follow_.size()), i + 5); i++)
  {
    float x2 = std::get<0>(path_to_follow_[i]);
    float y2 = std::get<1>(path_to_follow_[i]);
    float d = distance(x1,y1, x2, y2);
    if (d < d_) {
      d_ = d;
      next_node_to_check_ = i;
    }
  }
  ROS_INFO_STREAM(d_);
  ROS_INFO_STREAM(next_node_to_check_);
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
  get_closest_point();
}

bool PathsFollower::goalCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  updateControlPose();

  // TODO start to follow the path
  timer_.start();

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

double PathsFollower::norm_angle(double val) {
  return angles::normalize_angle(val);
}

float PathsFollower::errror_lat()
{
  float x1 = std::get<0>(pose_);
  float y1 = std::get<1>(pose_);
  ROS_INFO_STREAM("x1: " << x1);
  ROS_INFO_STREAM("y1: " << y1);
  float x2 = std::get<0>(path_to_follow_[next_node_to_check_]);
  float y2 = std::get<1>(path_to_follow_[next_node_to_check_]);
  double dx = x1 - x2;
  double dy = y1 - y2;
  ROS_INFO_STREAM("x2: " << x2);
  ROS_INFO_STREAM("y2: " << y2);
  double path_yaw_angle = norm_angle(std::atan2(dy, dx));
  ROS_INFO_STREAM(path_yaw_angle);

  const double c_yaw = std::cos(path_yaw_angle);
  const double s_yaw = std::sin(path_yaw_angle);
  error_lat_ = dx * s_yaw - dy * s_yaw;
  ROS_INFO_STREAM("Error lat : " << error_lat_);
}

void PathsFollower::controlLoop(const ros::TimerEvent &event)
{
  updateControlPose(); // get localization in pose_
  // ROS_INFO_STREAM("x: " << std::get<0>(pose_) << " y: " << std::get<1>(pose_) << " yaw: " << std::get<2>(pose_));

  auto v = 0.0; // vitesse lineaire
  auto w = 0.5; // vitesse angulaire
  get_closest_point();
  errror_lat();
  

  auto phi1 = (2 * v + length_ * w) / (2 * radius_);
  auto phi2 = (2 * v - length_ * w) / (2 * radius_);
  std_msgs::Float64 data_left;
  data_left.data = phi1;
  std_msgs::Float64 data_right;
  data_right.data = phi2;
  pub_left_.publish(data_left);
  pub_right_.publish(data_right);
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
