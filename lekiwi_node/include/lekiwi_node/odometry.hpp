#ifndef LEKIWI_NODE__ODOMETRY_HPP_
#define LEKIWI_NODE__ODOMETRY_HPP_

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lekiwi_node/omni_kinematics.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"

namespace lekiwi_node
{

class Odometry
{
public:
  void reset();
  void update(const geometry_msgs::msg::Twist & twist, double dt);
  void updateDelta(const BodyPoseDelta & delta);

  nav_msgs::msg::Odometry toMessage(
    const rclcpp::Time & stamp,
    const std::string & odom_frame,
    const std::string & base_frame,
    const geometry_msgs::msg::Twist & twist) const;

  geometry_msgs::msg::TransformStamped toTransform(
    const rclcpp::Time & stamp,
    const std::string & odom_frame,
    const std::string & base_frame) const;

private:
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

}  // namespace lekiwi_node

#endif  // LEKIWI_NODE__ODOMETRY_HPP_
