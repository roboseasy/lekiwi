#ifndef LEKIWI_NODE__OMNI_KINEMATICS_HPP_
#define LEKIWI_NODE__OMNI_KINEMATICS_HPP_

#include <algorithm>
#include <cmath>

namespace lekiwi_node
{

struct WheelCommand
{
  double left{0.0};
  double back{0.0};
  double right{0.0};
};

struct BodyTwist
{
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

struct WheelPositionDelta
{
  double left{0.0};   // rad
  double back{0.0};   // rad
  double right{0.0};  // rad
};

struct BodyPoseDelta
{
  double dx{0.0};    // body-frame m
  double dy{0.0};    // body-frame m
  double dyaw{0.0};  // rad
};

class OmniKinematics
{
public:
  OmniKinematics() = default;
  OmniKinematics(
    double wheel_radius, double base_radius, double max_wheel_speed,
    double kinematics_frame_x = 0.0, double kinematics_frame_y = 0.0,
    double kinematics_frame_yaw = 0.0);

  BodyTwist scaleToWheelEnvelope(const BodyTwist & twist) const;
  WheelCommand toWheelSpeeds(double linear_x, double linear_y, double angular_z) const;
  BodyTwist toBodyTwist(const WheelCommand & command) const;
  BodyPoseDelta toBodyDelta(const WheelPositionDelta & delta) const;

private:
  BodyTwist toKinematicsFrame(const BodyTwist & twist) const;
  BodyTwist fromKinematicsFrame(const BodyTwist & twist) const;

  double wheel_radius_{0.05};
  double base_radius_{0.13647};
  double max_wheel_speed_{20.0};
  // Original motor-kinematics origin expressed in the current robot base frame.
  double kinematics_frame_x_{0.0};
  double kinematics_frame_y_{0.0};
  double kinematics_frame_yaw_{0.0};
};

}  // namespace lekiwi_node

#endif  // LEKIWI_NODE__OMNI_KINEMATICS_HPP_
