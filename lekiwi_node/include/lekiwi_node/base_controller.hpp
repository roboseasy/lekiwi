#ifndef LEKIWI_NODE__BASE_CONTROLLER_HPP_
#define LEKIWI_NODE__BASE_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "lekiwi_node/motor_readiness.hpp"
#include "lekiwi_node/odometry.hpp"
#include "lekiwi_node/omni_kinematics.hpp"

namespace lekiwi_node
{

struct BaseControllerParameters
{
  double wheel_radius{0.05};
  double base_radius{0.13647};
  // Must match lekiwi_description/config/base_frame.yaml.
  double kinematics_frame_x{0.007587843843};
  double kinematics_frame_y{-0.010776931094};
  double kinematics_frame_yaw{2.094379865161};
  double max_linear_x{0.11};
  double max_linear_y{0.1};
  double max_linear_speed{0.11};
  double max_angular_z{0.75};
  double max_wheel_speed{2.3009711818284617};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};
  double cmd_vel_timeout{0.3};
  double tf_time_offset{0.0};
  std::string odom_source{"encoder"};
  double encoder_read_period{0.1};
  double encoder_sample_timeout{0.5};
  std::string motor_backend{"mock"};
  std::string serial_port{"/dev/ttyACM0"};
  bool enable_motor_write{false};
  // Model joints left/back/right occupy physical rear/right/left positions.
  std::vector<int64_t> wheel_ids{8, 9, 7};
  std::vector<double> wheel_directions{1.0, 1.0, 1.0};
  int64_t baud_rate{1000000};
  double speed_tick_scale{651.8986469044033};
  int64_t speed_tick_limit{1500};
  int64_t encoder_tick_deadband{50};
  std::string encoder_feedback_source{"position"};
  double encoder_position_ticks_per_revolution{4096.0};
  double encoder_odom_scale{1.0};
  int64_t encoder_position_tick_deadband{0};
  bool torque_enable{false};
  bool log_encoder_reads{false};
  bool monitor_motor_health{false};
  double motor_min_voltage{11.5};
  double motor_max_voltage{12.6};
};

enum class BaseMotorBackendKind
{
  kMockLogging,
  kFeetechHardware,
};

class BaseMotorInterface;
class MotorSafetyCoordinator;

struct WheelFeedbackSample
{
  WheelCommand wheel_speeds{};
  std::optional<WheelPositionDelta> wheel_position_delta;
};

BaseMotorBackendKind selectBaseMotorBackend(const BaseControllerParameters & params);
void validateBaseControllerParameters(const BaseControllerParameters & params);
void validateFeetechMotorParameters(const BaseControllerParameters & params);
geometry_msgs::msg::Twist limitBodyTwist(
  const geometry_msgs::msg::Twist & command,
  double max_linear_x,
  double max_linear_y,
  double max_linear_speed,
  double max_angular_z,
  const OmniKinematics & kinematics);
geometry_msgs::msg::Twist selectOdometryTwist(
  const geometry_msgs::msg::Twist & command,
  bool has_command,
  double command_age,
  double command_timeout);
geometry_msgs::msg::Twist selectCommandAtTime(
  const geometry_msgs::msg::Twist & command,
  bool has_command,
  const rclcpp::Time & last_command_time,
  const rclcpp::Time & current_time,
  double command_timeout);
geometry_msgs::msg::Twist selectOdometryUpdateTwist(
  const geometry_msgs::msg::Twist & command,
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const std::string & odom_source,
  const OmniKinematics & kinematics);
void updateOdometryForSource(
  Odometry & odometry,
  const std::string & odom_source,
  const std::string & encoder_feedback_source,
  const geometry_msgs::msg::Twist & selected_twist,
  double control_dt,
  const std::optional<WheelPositionDelta> & wheel_position_delta,
  const OmniKinematics & kinematics);
bool shouldRefreshEncoderSample(
  const std::string & odom_source,
  double encoder_sample_age,
  double encoder_read_period);
std::optional<WheelCommand> selectFreshEncoderWheelSpeeds(
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  double encoder_sample_age,
  double encoder_sample_timeout);
bool updateEncoderFeedbackOnSuccessfulRead(
  const std::optional<WheelFeedbackSample> & read_result,
  const rclcpp::Time & success_time,
  std::optional<WheelCommand> * last_sample,
  std::optional<rclcpp::Time> * last_success_time);
bool encoderFeedbackTimedOut(
  const std::optional<rclcpp::Time> & encoder_wait_start_time,
  const std::optional<rclcpp::Time> & last_success_time,
  const rclcpp::Time & current_time,
  double encoder_sample_timeout);
bool motorPowerEnableAllowed(
  const MotorReadinessLatch & motor_readiness,
  bool recovery_disarmed);
bool stopMotorOnEncoderTimeout(
  BaseMotorInterface & motor_interface,
  MotorReadinessLatch & motor_readiness,
  const std::optional<rclcpp::Time> & encoder_wait_start_time,
  const std::optional<rclcpp::Time> & last_success_time,
  const rclcpp::Time & current_time,
  double encoder_sample_timeout);
WheelCommand selectJointStateWheelSpeeds(
  const WheelCommand & command_wheel_speeds,
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const std::string & odom_source);
sensor_msgs::msg::JointState makeBaseWheelJointState(
  const WheelCommand & wheel_speeds,
  const rclcpp::Time & stamp);
std_srvs::srv::Trigger::Response resetOdometryForService(Odometry & odometry);
std_srvs::srv::SetBool::Response setMotorPowerForService(
  BaseMotorInterface & motor_interface,
  bool enabled);
rclcpp::Time offsetTransformStamp(const rclcpp::Time & stamp, double offset_seconds);

class BaseMotorInterface
{
public:
  explicit BaseMotorInterface(double max_wheel_speed);
  virtual ~BaseMotorInterface() = default;
  virtual void connect() = 0;
  virtual void shutdown() noexcept;
  virtual bool isConnected() const;
  virtual std::optional<WheelFeedbackSample> readWheelFeedback();
  virtual bool setMotorPower(bool enabled);
  bool isMotorPowerEnabled() const;
  void writeWheelSpeeds(const WheelCommand & command);
  void stop();

protected:
  virtual void writeClampedWheelSpeeds(const WheelCommand & command) = 0;
  void setMotorPowerState(bool enabled);

private:
  double max_wheel_speed_;
  bool motor_power_enabled_{true};
};

void stopMotorInterfaceForShutdown(BaseMotorInterface * motor_interface) noexcept;

class LoggingMotorInterface : public BaseMotorInterface
{
public:
  LoggingMotorInterface(rclcpp::Logger logger, double max_wheel_speed);
  void connect() override;
  bool isConnected() const override;

private:
  void writeClampedWheelSpeeds(const WheelCommand & command) override;

  rclcpp::Logger logger_;
};

class BaseController : public rclcpp::Node
{
public:
  explicit BaseController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BaseController() override;

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onControlTimer();
  void onResetOdometry(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onMotorPower(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void publishMotorReady();
  bool usesRealMotorHardware() const;
  BaseControllerParameters loadParameters();
  void startMotorInterface();
  void createRosInterfaces();

  BaseControllerParameters params_;
  OmniKinematics kinematics_;
  Odometry odometry_;
  geometry_msgs::msg::Twist current_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_update_time_;
  bool has_cmd_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_ready_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odometry_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motor_power_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<BaseMotorInterface> motor_interface_;
  std::unique_ptr<MotorSafetyCoordinator> motor_safety_;
};

}  // namespace lekiwi_node

#endif  // LEKIWI_NODE__BASE_CONTROLLER_HPP_
