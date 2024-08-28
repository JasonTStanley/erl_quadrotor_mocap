#ifndef ERL_QUADROTOR_MOCAP_NODE_HPP_
#define ERL_QUADROTOR_MOCAP_NODE_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>
#include <mocap_optitrack_interfaces/msg/rigid_body_array.hpp>

#include <deque>
#include <eigen3/Eigen/Geometry>
#include "filter.hpp"  

#include <rclcpp/rclcpp.hpp>

namespace erl_quadrotor_mocap {

class MocapNode : public rclcpp::Node{ 
 public:
  MocapNode();
  void rigidBodyCallback(const mocap_optitrack_interfaces::msg::RigidBodyArray & rigid_body_msg);
  void publishPose(const geometry_msgs::msg::PoseStamped & pose_msg);

 private:
  void publishTransform(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header, const std::string &child_frame_id);
  Eigen::Vector3d averageVector(const std::deque<Eigen::Vector3d>& buffer) const;

  Filter filter_;  
  // ros::Publisher odom_pub_;
  // ros::Subscriber pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr named_poses_pub_;
  rclcpp::Subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr pose_sub_;
  // tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::string child_frame_id_;
  bool publish_tf_;
  std::deque<Eigen::Vector3d> position_buffer_, orientation_buffer_, linear_vel_buffer_, angular_vel_buffer_;
  int average_samples_; 
};

}  // namespace erl_quadrotor_mocap

#endif  // ERL_QUADROTOR_MOCAP_NODE_HPP_
