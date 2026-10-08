#include "lekiwi_node/omni_kinematics.hpp"

namespace lekiwi_node
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSqrt3 = 1.73205080756887729353;

double wheelSpeed(
  const double wheel_angle,
  const double linear_x,
  const double linear_y,
  const double angular_z,
  const double wheel_radius,
  const double base_radius)
{
  return
    (-std::sin(wheel_angle) * linear_x + std::cos(wheel_angle) * linear_y +
    base_radius * angular_z) /
    wheel_radius;
}

WheelCommand toRawWheelSpeeds(
  const double linear_x,
  const double linear_y,
  const double angular_z,
  const double wheel_radius,
  const double base_radius)
{
  const double robot_lateral = -linear_y;
  return WheelCommand{
    wheelSpeed(
      2.0 * kPi / 3.0, linear_x, robot_lateral, angular_z, wheel_radius, base_radius),
    wheelSpeed(0.0, linear_x, robot_lateral, angular_z, wheel_radius, base_radius),
    wheelSpeed(
      4.0 * kPi / 3.0, linear_x, robot_lateral, angular_z, wheel_radius, base_radius)};
}
}  // namespace

OmniKinematics::OmniKinematics(
  const double wheel_radius,
  const double base_radius,
  const double max_wheel_speed,
  const double kinematics_frame_x,
  const double kinematics_frame_y,
  const double kinematics_frame_yaw)
: wheel_radius_(wheel_radius),
  base_radius_(base_radius),
  max_wheel_speed_(max_wheel_speed),
  kinematics_frame_x_(kinematics_frame_x),
  kinematics_frame_y_(kinematics_frame_y),
  kinematics_frame_yaw_(kinematics_frame_yaw)
{
}

BodyTwist OmniKinematics::toKinematicsFrame(const BodyTwist & twist) const
{
  const double c = std::cos(kinematics_frame_yaw_);
  const double s = std::sin(kinematics_frame_yaw_);
  // Velocity at the offset kinematics origin, then expressed in its axes.
  const double x = twist.linear_x - twist.angular_z * kinematics_frame_y_;
  const double y = twist.linear_y + twist.angular_z * kinematics_frame_x_;
  return BodyTwist{c * x + s * y, -s * x + c * y, twist.angular_z};
}

BodyTwist OmniKinematics::fromKinematicsFrame(const BodyTwist & twist) const
{
  const double c = std::cos(kinematics_frame_yaw_);
  const double s = std::sin(kinematics_frame_yaw_);
  return BodyTwist{
    c * twist.linear_x - s * twist.linear_y + twist.angular_z * kinematics_frame_y_,
    s * twist.linear_x + c * twist.linear_y - twist.angular_z * kinematics_frame_x_,
    twist.angular_z};
}

BodyTwist OmniKinematics::scaleToWheelEnvelope(const BodyTwist & twist) const
{
  const BodyTwist motor_twist = toKinematicsFrame(twist);
  const WheelCommand raw = toRawWheelSpeeds(
    motor_twist.linear_x, motor_twist.linear_y, motor_twist.angular_z,
    wheel_radius_, base_radius_);
  const double peak = std::max({std::abs(raw.left), std::abs(raw.back), std::abs(raw.right)});
  const double scale = peak > max_wheel_speed_ ? max_wheel_speed_ / peak : 1.0;
  return BodyTwist{
    twist.linear_x * scale, twist.linear_y * scale, twist.angular_z * scale};
}

WheelCommand OmniKinematics::toWheelSpeeds(
  const double linear_x,
  const double linear_y,
  const double angular_z) const
{
  const BodyTwist effective = scaleToWheelEnvelope(BodyTwist{linear_x, linear_y, angular_z});
  const BodyTwist motor_twist = toKinematicsFrame(effective);
  const WheelCommand raw = toRawWheelSpeeds(
    motor_twist.linear_x, motor_twist.linear_y, motor_twist.angular_z,
    wheel_radius_, base_radius_);
  return WheelCommand{
    std::clamp(raw.left, -max_wheel_speed_, max_wheel_speed_),
    std::clamp(raw.back, -max_wheel_speed_, max_wheel_speed_),
    std::clamp(raw.right, -max_wheel_speed_, max_wheel_speed_)};
}

BodyTwist OmniKinematics::toBodyTwist(const WheelCommand & command) const
{
  const double left = command.left * wheel_radius_;
  const double back = command.back * wheel_radius_;
  const double right = command.right * wheel_radius_;
  const double angular_z = (left + back + right) / (3.0 * base_radius_);

  return fromKinematicsFrame(BodyTwist{
    (right - left) / kSqrt3,
    -(back - (base_radius_ * angular_z)),
    angular_z});
}

BodyPoseDelta OmniKinematics::toBodyDelta(const WheelPositionDelta & delta) const
{
  const double left = delta.left * wheel_radius_;
  const double back = delta.back * wheel_radius_;
  const double right = delta.right * wheel_radius_;
  const double dyaw = (left + back + right) / (3.0 * base_radius_);

  const double dx = (right - left) / kSqrt3;
  const double dy = -(back - (base_radius_ * dyaw));
  const double c = std::cos(kinematics_frame_yaw_);
  const double s = std::sin(kinematics_frame_yaw_);
  // Conjugate the finite pose increment: R * delta + t - R(dyaw) * t.
  // The finite rotation term avoids drift of the offset origin during turns.
  const double turn_c = std::cos(dyaw);
  const double turn_s = std::sin(dyaw);
  return BodyPoseDelta{
    c * dx - s * dy + (1.0 - turn_c) * kinematics_frame_x_ +
    turn_s * kinematics_frame_y_,
    s * dx + c * dy - turn_s * kinematics_frame_x_ +
    (1.0 - turn_c) * kinematics_frame_y_,
    dyaw};
}

}  // namespace lekiwi_node
