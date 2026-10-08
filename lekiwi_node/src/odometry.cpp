#include "lekiwi_node/odometry.hpp"

#include <array>
#include <cmath>

#include "tf2/LinearMath/Quaternion.h"

namespace lekiwi_node
{
namespace
{
std::array<double, 36> commandBasedPoseCovariance()
{
  std::array<double, 36> covariance{};
  covariance[0] = 0.05;
  covariance[7] = 0.05;
  covariance[14] = 1.0e3;
  covariance[21] = 1.0e3;
  covariance[28] = 1.0e3;
  covariance[35] = 0.10;
  return covariance;
}

std::array<double, 36> commandBasedTwistCovariance()
{
  std::array<double, 36> covariance{};
  covariance[0] = 0.10;
  covariance[7] = 0.10;
  covariance[14] = 1.0e3;
  covariance[21] = 1.0e3;
  covariance[28] = 1.0e3;
  covariance[35] = 0.20;
  return covariance;
}
}  // namespace

void Odometry::reset()
{
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
}

void Odometry::update(const geometry_msgs::msg::Twist & twist, const double dt)
{
  const double cos_yaw = std::cos(yaw_);
  const double sin_yaw = std::sin(yaw_);

  x_ += (twist.linear.x * cos_yaw - twist.linear.y * sin_yaw) * dt;
  y_ += (twist.linear.x * sin_yaw + twist.linear.y * cos_yaw) * dt;
  yaw_ += twist.angular.z * dt;
}

void Odometry::updateDelta(const BodyPoseDelta & delta)
{
  const double cos_yaw = std::cos(yaw_);
  const double sin_yaw = std::sin(yaw_);
  x_ += cos_yaw * delta.dx - sin_yaw * delta.dy;
  y_ += sin_yaw * delta.dx + cos_yaw * delta.dy;
  yaw_ += delta.dyaw;
}

nav_msgs::msg::Odometry Odometry::toMessage(
  const rclcpp::Time & stamp,
  const std::string & odom_frame,
  const std::string & base_frame,
  const geometry_msgs::msg::Twist & twist) const
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odom_frame;
  msg.child_frame_id = base_frame;
  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw_);
  msg.pose.pose.orientation.x = orientation.x();
  msg.pose.pose.orientation.y = orientation.y();
  msg.pose.pose.orientation.z = orientation.z();
  msg.pose.pose.orientation.w = orientation.w();
  msg.pose.covariance = commandBasedPoseCovariance();
  msg.twist.twist = twist;
  msg.twist.covariance = commandBasedTwistCovariance();
  return msg;
}

geometry_msgs::msg::TransformStamped Odometry::toTransform(
  const rclcpp::Time & stamp,
  const std::string & odom_frame,
  const std::string & base_frame) const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = odom_frame;
  transform.child_frame_id = base_frame;
  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.translation.z = 0.0;

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw_);
  transform.transform.rotation.x = orientation.x();
  transform.transform.rotation.y = orientation.y();
  transform.transform.rotation.z = orientation.z();
  transform.transform.rotation.w = orientation.w();
  return transform;
}

}  // namespace lekiwi_node
