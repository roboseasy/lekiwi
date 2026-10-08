#include "lekiwi_node/motor_readiness.hpp"

namespace lekiwi_node
{

void MotorReadinessLatch::onPowerDisabled()
{
  state_ = MotorSafetyState::kDisarmed;
}

bool MotorReadinessLatch::onPowerEnabled(const bool hardware_connected)
{
  if (!hardware_connected) {
    state_ = MotorSafetyState::kFaultLatched;
    return false;
  }
  if (state_ == MotorSafetyState::kFaultLatched) {
    return false;
  }
  state_ = MotorSafetyState::kWaitingForEncoder;
  return true;
}

void MotorReadinessLatch::onFreshEncoderSample()
{
  if (state_ == MotorSafetyState::kWaitingForEncoder) {
    state_ = MotorSafetyState::kReady;
  }
}

void MotorReadinessLatch::onEncoderTimeout()
{
  if (state_ == MotorSafetyState::kReady) {
    state_ = MotorSafetyState::kFaultLatched;
  }
}

void MotorReadinessLatch::onBackendFault()
{
  state_ = MotorSafetyState::kFaultLatched;
}

bool MotorReadinessLatch::ready(
  const bool real_hardware,
  const bool hardware_connected,
  const bool power_enabled) const
{
  return state_ == MotorSafetyState::kReady && real_hardware && hardware_connected && power_enabled;
}

bool MotorReadinessLatch::armed() const
{
  return state_ == MotorSafetyState::kWaitingForEncoder || state_ == MotorSafetyState::kReady;
}

bool MotorReadinessLatch::faultLatched() const
{
  return state_ == MotorSafetyState::kFaultLatched;
}

bool MotorReadinessLatch::permits(const WheelCommand & command) const
{
  const bool stopped = command.left == 0.0 && command.back == 0.0 && command.right == 0.0;
  return stopped || state_ == MotorSafetyState::kReady;
}

}  // namespace lekiwi_node
