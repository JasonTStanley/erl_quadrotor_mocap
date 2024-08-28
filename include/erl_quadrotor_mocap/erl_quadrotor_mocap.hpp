#ifndef ERL_QUADROTOR_MOCAP_NODE_HPP_
#define ERL_QUADROTOR_MOCAP_NODE_HPP_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <deque>
#include <Eigen/Geometry>
#include "filter.hpp"  

namespace erl_quadrotor_mocap {

class MocapNode {
 public:
  MocapNode(ros::NodeHandle &nh);

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);

 private:
  void publishTransform(const geometry_msgs::Pose &pose, const std_msgs::Header &header, const std::string &child_frame_id);
  Eigen::Vector3d averageVector(const std::deque<Eigen::Vector3d>& buffer) const;

  Filter filter_;  
  ros::Publisher odom_pub_;
  ros::Publisher motor_speed_pub_;
  ros::Subscriber pose_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::string child_frame_id_;
  bool publish_tf_;
  std::deque<Eigen::Vector3d> position_buffer_, orientation_buffer_, linear_vel_buffer_, angular_vel_buffer_;
  int average_samples_; 
};

}  // namespace erl_quadrotor_mocap

#endif  // ERL_QUADROTOR_MOCAP_NODE_HPP_
