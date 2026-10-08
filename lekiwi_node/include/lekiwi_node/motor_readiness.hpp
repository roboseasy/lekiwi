#ifndef LEKIWI_NODE__MOTOR_READINESS_HPP_
#define LEKIWI_NODE__MOTOR_READINESS_HPP_

#include "lekiwi_node/omni_kinematics.hpp"

namespace lekiwi_node
{

enum class MotorSafetyState
{
  kDisarmed,
  kWaitingForEncoder,
  kReady,
  kFaultLatched,
};

class MotorReadinessLatch
{
public:
  void onPowerDisabled();
  bool onPowerEnabled(bool hardware_connected);
  void onFreshEncoderSample();
  void onEncoderTimeout();
  void onBackendFault();
  bool ready(bool real_hardware, bool hardware_connected, bool power_enabled) const;
  bool armed() const;
  bool faultLatched() const;
  bool permits(const WheelCommand & command) const;

private:
  MotorSafetyState state_{MotorSafetyState::kDisarmed};
};

}  // namespace lekiwi_node

#endif  // LEKIWI_NODE__MOTOR_READINESS_HPP_
