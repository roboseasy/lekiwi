#include "lekiwi_node/motor_safety_coordinator.hpp"

#include <utility>

#include "lekiwi_node/base_controller.hpp"

namespace lekiwi_node
{

MotorSafetyCoordinator::MotorSafetyCoordinator(
  BaseMotorInterface & motor_interface,
  const bool real_hardware,
  const bool encoder_feedback_required,
  const double encoder_read_period,
  const double encoder_sample_timeout,
  const rclcpp::Time & initial_time,
  NowFunction now_function)
: motor_interface_(motor_interface),
  real_hardware_(real_hardware),
  encoder_feedback_required_(encoder_feedback_required),
  encoder_read_period_(encoder_read_period),
  encoder_sample_timeout_(encoder_sample_timeout),
  last_encoder_attempt_time_(initial_time),
  now_function_(std::move(now_function))
{
  if (real_hardware_ && motor_interface_.isConnected() &&
    motor_interface_.isMotorPowerEnabled())
  {
    if (motor_readiness_.onPowerEnabled(true)) {
      encoder_wait_start_time_ = initial_time;
    }
  }
  last_readiness_decision_ = motorReady();
}

MotorSafetyUpdate MotorSafetyCoordinator::beforeCommand(const rclcpp::Time & attempt_time)
{
  const bool ready_before = last_readiness_decision_;
  MotorSafetyUpdate update;
  const auto finish = [this, ready_before](MotorSafetyUpdate result) {
    last_readiness_decision_ = motorReady();
    result.readiness_changed = ready_before != last_readiness_decision_;
    return result;
  };

  const auto cycle_time = now_function_();
  update.safety_time = cycle_time;
  const auto sample_age = [this](const rclcpp::Time & time) {
    const auto & start = last_encoder_success_time_ ?
      last_encoder_success_time_ : encoder_wait_start_time_;
    return start ? (time - *start).seconds() : 0.0;
  };
  if (encoder_feedback_required_ && motor_readiness_.armed() && stopMotorOnEncoderTimeout(
      motor_interface_,
      motor_readiness_,
      encoder_wait_start_time_,
      last_encoder_success_time_,
      cycle_time,
      encoder_sample_timeout_))
  {
    recovery_disarmed_ = false;
    update.encoder_timeout = true;
    update.encoder_timeout_stage = EncoderTimeoutStage::kBeforeRead;
    update.encoder_sample_age_seconds = sample_age(cycle_time);
    update.consecutive_failed_reads = consecutive_failed_reads_;
    return finish(update);
  }

  if (encoder_feedback_required_ &&
    last_encoder_attempt_time_ &&
    (attempt_time - *last_encoder_attempt_time_).seconds() >= encoder_read_period_)
  {
    last_encoder_attempt_time_ = attempt_time;
    std::optional<WheelFeedbackSample> read_result;
    try {
      read_result = motor_interface_.readWheelFeedback();
    } catch (...) {
      update.encoder_read_status = EncoderReadStatus::kException;
    }
    if (update.encoder_read_status != EncoderReadStatus::kException) {
      update.encoder_read_status = read_result ?
        EncoderReadStatus::kSample : EncoderReadStatus::kNoSample;
    }
    const auto completion_time = now_function_();
    update.safety_time = completion_time;
    update.encoder_read_duration_seconds = (completion_time - cycle_time).seconds();
    if (!read_result) {
      ++consecutive_failed_reads_;
    }
    if (motor_readiness_.armed() && stopMotorOnEncoderTimeout(
        motor_interface_,
        motor_readiness_,
        encoder_wait_start_time_,
        last_encoder_success_time_,
        completion_time,
        encoder_sample_timeout_))
    {
      recovery_disarmed_ = false;
      update.encoder_timeout = true;
      update.encoder_timeout_stage = EncoderTimeoutStage::kAfterRead;
      update.encoder_sample_age_seconds = sample_age(completion_time);
      update.consecutive_failed_reads = consecutive_failed_reads_;
      return finish(update);
    }
    if (updateEncoderFeedbackOnSuccessfulRead(
        read_result,
        completion_time,
        &last_encoder_wheel_speeds_,
        &last_encoder_success_time_))
    {
      update.wheel_position_delta = read_result->wheel_position_delta;
      consecutive_failed_reads_ = 0;
      encoder_wait_start_time_.reset();
      motor_readiness_.onFreshEncoderSample();
    }
  }

  if (motor_readiness_.armed() &&
    (!motor_interface_.isConnected() || !motor_interface_.isMotorPowerEnabled()))
  {
    motor_readiness_.onBackendFault();
    recovery_disarmed_ = false;
  }
  return finish(update);
}

void MotorSafetyCoordinator::sendWheelCommand(const WheelCommand & command)
{
  const bool armed_before_write = motor_readiness_.armed();
  motor_interface_.writeWheelSpeeds(motor_readiness_.permits(command) ? command : WheelCommand{});
  if (armed_before_write &&
    (!motor_interface_.isConnected() || !motor_interface_.isMotorPowerEnabled()))
  {
    motor_readiness_.onBackendFault();
    recovery_disarmed_ = false;
  }
}

std::optional<WheelCommand> MotorSafetyCoordinator::freshEncoderWheelSpeeds(
  const rclcpp::Time & current_time) const
{
  if (!last_encoder_success_time_) {
    return std::nullopt;
  }
  return selectFreshEncoderWheelSpeeds(
    last_encoder_wheel_speeds_,
    (current_time - *last_encoder_success_time_).seconds(),
    encoder_sample_timeout_);
}

std_srvs::srv::SetBool::Response MotorSafetyCoordinator::setMotorPower(
  const bool enabled,
  const rclcpp::Time & request_time)
{
  const auto finish = [this](std_srvs::srv::SetBool::Response response) {
    last_readiness_decision_ = motorReady();
    return response;
  };
  if (!real_hardware_) {
    return finish(setMotorPowerForService(motor_interface_, enabled));
  }

  std_srvs::srv::SetBool::Response response;
  if (!enabled) {
    const bool success = motor_interface_.setMotorPower(false);
    if (success) {
      motor_readiness_.onPowerDisabled();
      recovery_disarmed_ = true;
      encoder_wait_start_time_.reset();
      last_encoder_wheel_speeds_.reset();
      last_encoder_success_time_.reset();
      last_encoder_attempt_time_.reset();
      consecutive_failed_reads_ = 0;
    } else {
      motor_readiness_.onBackendFault();
      recovery_disarmed_ = false;
    }
    response.success = success;
    response.message = success ? "motor power disabled" : "failed to disable motor power";
    return finish(response);
  }

  if (!motorPowerEnableAllowed(motor_readiness_, recovery_disarmed_)) {
    response.success = false;
    response.message = "disable motor power before recovering a latched fault";
    return finish(response);
  }

  if (!motor_interface_.isConnected()) {
    motor_interface_.connect();
  }
  if (!motor_interface_.isConnected() || !motor_interface_.setMotorPower(true)) {
    motor_interface_.stop();
    (void)motor_interface_.setMotorPower(false);
    motor_readiness_.onBackendFault();
    recovery_disarmed_ = false;
    response.success = false;
    response.message = "failed to connect and enable motor power";
    return finish(response);
  }

  if (!motor_readiness_.onPowerEnabled(true)) {
    recovery_disarmed_ = false;
    response.success = false;
    response.message = "failed to arm encoder readiness check";
    return finish(response);
  }

  recovery_disarmed_ = false;
  encoder_wait_start_time_ = request_time;
  last_encoder_wheel_speeds_.reset();
  last_encoder_success_time_.reset();
  consecutive_failed_reads_ = 0;
  last_encoder_attempt_time_ = request_time;
  response.success = true;
  response.message = "motor power enabled; waiting for fresh encoder feedback";
  return finish(response);
}

bool MotorSafetyCoordinator::motorReady() const
{
  return motor_readiness_.ready(
    real_hardware_,
    motor_interface_.isConnected(),
    motor_interface_.isMotorPowerEnabled());
}

bool MotorSafetyCoordinator::faultLatched() const
{
  return motor_readiness_.faultLatched();
}

rclcpp::QoS makeMotorReadyQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace lekiwi_node
