#include "erl_quadrotor_mocap/erl_quadrotor_mocap.hpp"
#include <Eigen/Geometry>

namespace erl_quadrotor_mocap {

MocapNode::MocapNode(ros::NodeHandle &nh) {
  double max_accel, update_frequency, dt;
  nh.param("max_accel", max_accel, 5.0);
  nh.param("publish_tf", publish_tf_, false);
  nh.param<std::string>("child_frame_id", child_frame_id_, "base_link");
  nh.param("update_frequency", update_frequency, 100.0);
  nh.param("average_samples", average_samples_, 10);
  ROS_ASSERT(update_frequency > 0.0);
  dt = 1.0 / update_frequency;

  Filter::State_t proc_noise_diag;
  proc_noise_diag << 0.5 * max_accel * dt * dt, 0.5 * max_accel * dt * dt, 0.5 * max_accel * dt * dt, max_accel * dt, max_accel * dt, max_accel * dt;
  proc_noise_diag = proc_noise_diag.array().square();
  Filter::Measurement_t meas_noise_diag;
  meas_noise_diag << 1e-4, 1e-4, 1e-4;
  meas_noise_diag = meas_noise_diag.array().square();
  filter_.initialize(Filter::State_t::Zero(), 0.01 * Filter::ProcessCov_t::Identity(), proc_noise_diag.asDiagonal(), meas_noise_diag.asDiagonal());

  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
  motor_speed_pub_ = nh.advertise<mav_msgs::Actuators>("motor_speed", 10);
  pose_sub_ = nh.subscribe("pose", 10, &MocapNode::poseCallback, this, ros::TransportHints().tcpNoDelay());
}

void MocapNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
  static ros::Time t_last_proc = pose_msg->header.stamp;
  double dt = (pose_msg->header.stamp - t_last_proc).toSec();
  t_last_proc = pose_msg->header.stamp;

  filter_.processUpdate(dt);
  Filter::Measurement_t meas(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
  filter_.measurementUpdate(meas, dt);

  const auto state = filter_.getState();
  const auto proc_noise = filter_.getProcessNoise();

  nav_msgs::Odometry odom_msg;
  odom_msg.header = pose_msg->header;
  odom_msg.child_frame_id = child_frame_id_;
  odom_msg.pose.pose.position = pose_msg->pose.position;
  odom_msg.pose.pose.orientation = pose_msg->pose.orientation;

  if (linear_vel_buffer_.size() >= average_samples_) linear_vel_buffer_.pop_front();
  linear_vel_buffer_.emplace_back(state(3), state(4), state(5));

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom_msg.pose.covariance[6 * i + j] = proc_noise(i, j);
      odom_msg.twist.covariance[6 * i + j] = proc_noise(3 + i, 3 + j);
    }
  }

  static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
  Eigen::Matrix3d R(Eigen::Quaterniond(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z));
  if (dt > 1e-6) {
    const auto R_dot = (R - R_prev) / dt;
    const auto w_hat = R.transpose() * R_dot;
    odom_msg.twist.twist.angular.x = w_hat(2, 1);
    odom_msg.twist.twist.angular.y = w_hat(0, 2);
    odom_msg.twist.twist.angular.z = w_hat(1, 0);

    if (angular_vel_buffer_.size() >= average_samples_) angular_vel_buffer_.pop_front();
    angular_vel_buffer_.emplace_back(w_hat(2, 1), w_hat(0, 2), w_hat(1, 0));
  }
  R_prev = R;

  auto eigenToGeometryMsg = [](const Eigen::Vector3d& vec) {
    geometry_msgs::Vector3 msg;
    msg.x = vec.x();
    msg.y = vec.y();
    msg.z = vec.z();
    return msg;
  };

  odom_msg.twist.twist.linear = eigenToGeometryMsg(averageVector(linear_vel_buffer_));
  odom_msg.twist.twist.angular = eigenToGeometryMsg(averageVector(angular_vel_buffer_));

  odom_pub_.publish(odom_msg);

  if (publish_tf_) publishTransform(odom_msg.pose.pose, odom_msg.header, odom_msg.child_frame_id);

  mav_msgs::Actuators msg;
  msg.header = pose_msg->header;
  msg.angular_velocities = std::vector<double>(4, 0.0); 
  motor_speed_pub_.publish(msg);
}

void MocapNode::publishTransform(const geometry_msgs::Pose &pose, const std_msgs::Header &header, const std::string &child_frame_id) {
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header = header;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation.x = pose.position.x;
  transform_stamped.transform.translation.y = pose.position.y;
  transform_stamped.transform.translation.z = pose.position.z;
  transform_stamped.transform.rotation = pose.orientation;

  tf_broadcaster_.sendTransform(transform_stamped);
}

Eigen::Vector3d MocapNode::averageVector(const std::deque<Eigen::Vector3d>& buffer) const {
  Eigen::Vector3d avg_vector(0, 0, 0);
  for (const auto& vec : buffer) avg_vector += vec;
  return avg_vector / buffer.size();
}

}  // namespace erl_quadrotor_mocap

int main(int argc, char **argv) {
  ros::init(argc, argv, "erl_quadrotor_mocap");
  ros::NodeHandle nh("~");

  try {
    erl_quadrotor_mocap::MocapNode mocap_node(nh);
    ros::spin();
  } catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    return 1;
  }
  return 0;
}
