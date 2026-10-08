#ifndef LEKIWI_NODE__MOTOR_SAFETY_COORDINATOR_HPP_
#define LEKIWI_NODE__MOTOR_SAFETY_COORDINATOR_HPP_

#include <functional>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "lekiwi_node/motor_readiness.hpp"

namespace lekiwi_node
{

class BaseMotorInterface;

enum class EncoderReadStatus { kNotAttempted, kSample, kNoSample, kException };
enum class EncoderTimeoutStage { kNone, kBeforeRead, kAfterRead };

struct MotorSafetyUpdate
{
  rclcpp::Time safety_time{0, 0};
  bool readiness_changed{false};
  bool encoder_timeout{false};
  EncoderTimeoutStage encoder_timeout_stage{EncoderTimeoutStage::kNone};
  EncoderReadStatus encoder_read_status{EncoderReadStatus::kNotAttempted};
  double encoder_sample_age_seconds{0.0};
  double encoder_read_duration_seconds{0.0};
  unsigned int consecutive_failed_reads{0};
  std::optional<WheelPositionDelta> wheel_position_delta;
};

class MotorSafetyCoordinator
{
public:
  using NowFunction = std::function<rclcpp::Time()>;

  MotorSafetyCoordinator(
    BaseMotorInterface & motor_interface,
    bool real_hardware,
    bool encoder_feedback_required,
    double encoder_read_period,
    double encoder_sample_timeout,
    const rclcpp::Time & initial_time,
    NowFunction now_function);

  MotorSafetyUpdate beforeCommand(const rclcpp::Time & attempt_time);
  void sendWheelCommand(const WheelCommand & command);
  std::optional<WheelCommand> freshEncoderWheelSpeeds(const rclcpp::Time & current_time) const;
  std_srvs::srv::SetBool::Response setMotorPower(
    bool enabled,
    const rclcpp::Time & request_time);
  bool motorReady() const;
  bool faultLatched() const;

private:
  BaseMotorInterface & motor_interface_;
  bool real_hardware_;
  bool encoder_feedback_required_;
  double encoder_read_period_;
  double encoder_sample_timeout_;
  std::optional<rclcpp::Time> encoder_wait_start_time_;
  std::optional<rclcpp::Time> last_encoder_attempt_time_;
  std::optional<rclcpp::Time> last_encoder_success_time_;
  std::optional<WheelCommand> last_encoder_wheel_speeds_;
  unsigned int consecutive_failed_reads_{0};
  bool recovery_disarmed_{false};
  bool last_readiness_decision_{false};
  MotorReadinessLatch motor_readiness_;
  NowFunction now_function_;
};

rclcpp::QoS makeMotorReadyQos();

}  // namespace lekiwi_node

#endif  // LEKIWI_NODE__MOTOR_SAFETY_COORDINATOR_HPP_
