#include "lekiwi_node/base_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "lekiwi_node/feetech_motor_interface.hpp"
#include "lekiwi_node/motor_safety_coordinator.hpp"

namespace lekiwi_node
{
namespace
{
constexpr auto kControlPeriod = std::chrono::milliseconds(20);
constexpr int64_t kFeetechMaxServoId = 253;
constexpr int64_t kFeetechMaxVelocityMagnitude = 32767;

void validateWheelIdsParameter(const std::vector<int64_t> & wheel_ids)
{
  if (wheel_ids.size() != 3) {
    throw std::invalid_argument("wheel_ids must contain exactly 3 values");
  }
  for (std::size_t i = 0; i < wheel_ids.size(); ++i) {
    if (wheel_ids[i] < 1 || wheel_ids[i] > kFeetechMaxServoId) {
      throw std::invalid_argument("wheel_ids must be in the range [1, 253]");
    }
    for (std::size_t j = i + 1; j < wheel_ids.size(); ++j) {
      if (wheel_ids[i] == wheel_ids[j]) {
        throw std::invalid_argument("wheel_ids must not contain duplicate values");
      }
    }
  }
}

std::array<int, 3> wheelIdsFromParameter(const std::vector<int64_t> & wheel_ids)
{
  validateWheelIdsParameter(wheel_ids);
  return {
    static_cast<int>(wheel_ids[0]),
    static_cast<int>(wheel_ids[1]),
    static_cast<int>(wheel_ids[2])};
}

void validateWheelDirectionsParameter(const std::vector<double> & wheel_directions)
{
  if (wheel_directions.size() != 3) {
    throw std::invalid_argument("wheel_directions must contain exactly 3 values");
  }
  for (const double wheel_direction : wheel_directions) {
    if (!std::isfinite(wheel_direction)) {
      throw std::invalid_argument("wheel_directions must be finite");
    }
    if (wheel_direction == 0.0) {
      throw std::invalid_argument("wheel_directions must not contain zero");
    }
  }
}

std::array<double, 3> wheelDirectionsFromParameter(const std::vector<double> & wheel_directions)
{
  validateWheelDirectionsParameter(wheel_directions);
  return {wheel_directions[0], wheel_directions[1], wheel_directions[2]};
}

int intFromParameter(const int64_t value, const char * parameter_name)
{
  if (value < static_cast<int64_t>(std::numeric_limits<int>::min()) ||
    value > static_cast<int64_t>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument(std::string(parameter_name) + " must fit in int");
  }
  return static_cast<int>(value);
}

FeetechMotorConfig makeFeetechMotorConfig(const BaseControllerParameters & params)
{
  FeetechMotorConfig config;
  config.serial_port = params.serial_port;
  config.wheel_ids = wheelIdsFromParameter(params.wheel_ids);
  config.wheel_directions = wheelDirectionsFromParameter(params.wheel_directions);
  config.max_wheel_speed = params.max_wheel_speed;
  config.enable_motor_write = params.enable_motor_write;
  config.baud_rate = intFromParameter(params.baud_rate, "baud_rate");
  config.speed_tick_scale = params.speed_tick_scale;
  config.speed_tick_limit = intFromParameter(params.speed_tick_limit, "speed_tick_limit");
  config.encoder_tick_deadband = intFromParameter(
    params.encoder_tick_deadband, "encoder_tick_deadband");
  config.encoder_feedback_source = params.encoder_feedback_source;
  config.encoder_read_period = params.encoder_read_period;
  config.encoder_sample_timeout = params.encoder_sample_timeout;
  config.position_ticks_per_revolution = intFromParameter(
    static_cast<int64_t>(params.encoder_position_ticks_per_revolution),
    "encoder_position_ticks_per_revolution");
  config.encoder_odom_scale = params.encoder_odom_scale;
  config.encoder_position_tick_deadband = intFromParameter(
    params.encoder_position_tick_deadband, "encoder_position_tick_deadband");
  config.torque_enable = params.torque_enable;
  config.log_encoder_reads = params.log_encoder_reads;
  config.monitor_motor_health = params.monitor_motor_health;
  config.motor_min_voltage = params.motor_min_voltage;
  config.motor_max_voltage = params.motor_max_voltage;
  return config;
}

geometry_msgs::msg::Twist toRosTwist(const BodyTwist & twist)
{
  geometry_msgs::msg::Twist ros_twist;
  ros_twist.linear.x = twist.linear_x;
  ros_twist.linear.y = twist.linear_y;
  ros_twist.angular.z = twist.angular_z;
  return ros_twist;
}

bool isFiniteTwist(const geometry_msgs::msg::Twist & twist)
{
  return
    std::isfinite(twist.linear.x) &&
    std::isfinite(twist.linear.y) &&
    std::isfinite(twist.linear.z) &&
    std::isfinite(twist.angular.x) &&
    std::isfinite(twist.angular.y) &&
    std::isfinite(twist.angular.z);
}

bool twistsEqual(
  const geometry_msgs::msg::Twist & lhs,
  const geometry_msgs::msg::Twist & rhs)
{
  return
    lhs.linear.x == rhs.linear.x &&
    lhs.linear.y == rhs.linear.y &&
    lhs.linear.z == rhs.linear.z &&
    lhs.angular.x == rhs.angular.x &&
    lhs.angular.y == rhs.angular.y &&
    lhs.angular.z == rhs.angular.z;
}
}  // namespace

BaseMotorInterface::BaseMotorInterface(double max_wheel_speed)
: max_wheel_speed_(max_wheel_speed)
{
  if (!std::isfinite(max_wheel_speed_) || max_wheel_speed_ <= 0.0) {
    throw std::invalid_argument("max_wheel_speed must be positive and finite");
  }
}

void BaseMotorInterface::writeWheelSpeeds(const WheelCommand & command)
{
  if (!motor_power_enabled_) {
    return;
  }

  const WheelCommand clamped{
    std::clamp(command.left, -max_wheel_speed_, max_wheel_speed_),
    std::clamp(command.back, -max_wheel_speed_, max_wheel_speed_),
    std::clamp(command.right, -max_wheel_speed_, max_wheel_speed_)};
  writeClampedWheelSpeeds(clamped);
}

void BaseMotorInterface::stop()
{
  writeWheelSpeeds(WheelCommand{});
}

void BaseMotorInterface::shutdown() noexcept
{
  try {
    stop();
  } catch (...) {
  }
}

std::optional<WheelFeedbackSample> BaseMotorInterface::readWheelFeedback()
{
  return std::nullopt;
}

bool BaseMotorInterface::isConnected() const
{
  return false;
}

bool BaseMotorInterface::setMotorPower(const bool enabled)
{
  if (!enabled) {
    stop();
  }
  motor_power_enabled_ = enabled;
  return true;
}

bool BaseMotorInterface::isMotorPowerEnabled() const
{
  return motor_power_enabled_;
}

void BaseMotorInterface::setMotorPowerState(const bool enabled)
{
  motor_power_enabled_ = enabled;
}

void stopMotorInterfaceForShutdown(BaseMotorInterface * motor_interface) noexcept
{
  if (!motor_interface) {
    return;
  }

  motor_interface->shutdown();
}

LoggingMotorInterface::LoggingMotorInterface(rclcpp::Logger logger, double max_wheel_speed)
: BaseMotorInterface(max_wheel_speed),
  logger_(logger)
{
}

void LoggingMotorInterface::connect()
{
}

bool LoggingMotorInterface::isConnected() const
{
  return false;
}

void LoggingMotorInterface::writeClampedWheelSpeeds(const WheelCommand & command)
{
  RCLCPP_DEBUG(
    logger_, "wheel command left=%.3f back=%.3f right=%.3f",
    command.left, command.back, command.right);
}

BaseMotorBackendKind selectBaseMotorBackend(const BaseControllerParameters & params)
{
  if (params.motor_backend == "feetech" && params.enable_motor_write) {
    return BaseMotorBackendKind::kFeetechHardware;
  }
  return BaseMotorBackendKind::kMockLogging;
}

void validateBaseControllerParameters(const BaseControllerParameters & params)
{
  for (const double value : {
      params.kinematics_frame_x, params.kinematics_frame_y, params.kinematics_frame_yaw})
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("kinematics frame transform must be finite");
    }
  }
  const std::array<std::pair<double, const char *>, 4> body_limits{{
      {params.max_linear_x, "max_linear_x"},
      {params.max_linear_y, "max_linear_y"},
      {params.max_linear_speed, "max_linear_speed"},
      {params.max_angular_z, "max_angular_z"}}};
  for (const auto & [value, name] : body_limits) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(std::string(name) + " must be positive and finite");
    }
  }

  const double required_linear_wheel_speed = params.max_linear_speed / params.wheel_radius;
  const double required_yaw_wheel_speed =
    (params.base_radius + std::hypot(params.kinematics_frame_x, params.kinematics_frame_y)) *
    params.max_angular_z / params.wheel_radius;
  if (required_linear_wheel_speed > params.max_wheel_speed ||
    required_yaw_wheel_speed > params.max_wheel_speed)
  {
    throw std::invalid_argument("individual body velocity limit exceeds max_wheel_speed");
  }

  if (!std::isfinite(params.encoder_read_period) || params.encoder_read_period <= 0.0) {
    throw std::invalid_argument("encoder_read_period must be positive and finite");
  }
  if (!std::isfinite(params.encoder_sample_timeout) || params.encoder_sample_timeout <= 0.0) {
    throw std::invalid_argument("encoder_sample_timeout must be positive and finite");
  }
  if (params.encoder_feedback_source == "position") {
    if (params.encoder_position_tick_deadband != 0) {
      throw std::invalid_argument(
        "encoder_position_tick_deadband is deprecated and must be zero for position feedback");
    }
    constexpr double kTimingTolerance = 1e-12;
    if (3.0 * params.encoder_read_period >
      params.encoder_sample_timeout + kTimingTolerance)
    {
      throw std::invalid_argument(
        "position feedback requires 3 * encoder_read_period <= encoder_sample_timeout");
    }
  }
}

geometry_msgs::msg::Twist limitBodyTwist(
  const geometry_msgs::msg::Twist & command,
  const double max_linear_x,
  const double max_linear_y,
  const double max_linear_speed,
  const double max_angular_z,
  const OmniKinematics & kinematics)
{
  geometry_msgs::msg::Twist limited;
  if (!isFiniteTwist(command)) {
    return limited;
  }

  double scale = 1.0;
  scale = std::min(scale, max_linear_x / std::max(std::abs(command.linear.x), 1e-12));
  scale = std::min(scale, max_linear_y / std::max(std::abs(command.linear.y), 1e-12));
  scale = std::min(
    scale,
    max_linear_speed / std::max(std::hypot(command.linear.x, command.linear.y), 1e-12));
  limited.linear.x = command.linear.x * scale;
  limited.linear.y = command.linear.y * scale;
  limited.angular.z = std::clamp(command.angular.z, -max_angular_z, max_angular_z);
  const BodyTwist effective = kinematics.scaleToWheelEnvelope(
    BodyTwist{limited.linear.x, limited.linear.y, limited.angular.z});
  limited.linear.x = effective.linear_x;
  limited.linear.y = effective.linear_y;
  limited.angular.z = effective.angular_z;
  return limited;
}

void validateFeetechMotorParameters(const BaseControllerParameters & params)
{
  validateWheelIdsParameter(params.wheel_ids);
  validateWheelDirectionsParameter(params.wheel_directions);

  if (params.baud_rate <= 0) {
    throw std::invalid_argument("baud_rate must be positive");
  }
  (void)intFromParameter(params.baud_rate, "baud_rate");

  if (params.speed_tick_limit < 0 || params.speed_tick_limit > kFeetechMaxVelocityMagnitude) {
    throw std::invalid_argument("speed_tick_limit must be between 0 and 32767");
  }
  (void)intFromParameter(params.speed_tick_limit, "speed_tick_limit");
  if (params.encoder_tick_deadband < 0 ||
    params.encoder_tick_deadband > kFeetechMaxVelocityMagnitude)
  {
    throw std::invalid_argument("encoder_tick_deadband must be between 0 and 32767");
  }
  (void)intFromParameter(params.encoder_tick_deadband, "encoder_tick_deadband");
  if (params.encoder_feedback_source != "speed" && params.encoder_feedback_source != "position") {
    throw std::invalid_argument("encoder_feedback_source must be 'speed' or 'position'");
  }
  if (!std::isfinite(params.encoder_position_ticks_per_revolution) ||
    params.encoder_position_ticks_per_revolution <= 0.0 ||
    params.encoder_position_ticks_per_revolution > static_cast<double>(std::numeric_limits<int>::max()) ||
    std::trunc(params.encoder_position_ticks_per_revolution) !=
    params.encoder_position_ticks_per_revolution)
  {
    throw std::invalid_argument(
      "encoder_position_ticks_per_revolution must be a positive finite integer");
  }
  if (!std::isfinite(params.encoder_odom_scale) || params.encoder_odom_scale <= 0.0) {
    throw std::invalid_argument("encoder_odom_scale must be positive and finite");
  }
  if (params.encoder_position_tick_deadband < 0 ||
    params.encoder_position_tick_deadband >=
    static_cast<int64_t>(params.encoder_position_ticks_per_revolution / 2.0))
  {
    throw std::invalid_argument(
      "encoder_position_tick_deadband must be non-negative and less than half a revolution");
  }
  (void)intFromParameter(params.encoder_position_tick_deadband, "encoder_position_tick_deadband");

  if (!std::isfinite(params.speed_tick_scale) || params.speed_tick_scale <= 0.0) {
    throw std::invalid_argument("speed_tick_scale must be positive and finite");
  }
  if (!std::isfinite(params.max_wheel_speed) || params.max_wheel_speed <= 0.0) {
    throw std::invalid_argument("max_wheel_speed must be positive and finite");
  }

  const double max_speed_ticks = params.speed_tick_scale * params.max_wheel_speed;
  if (!std::isfinite(max_speed_ticks) ||
    max_speed_ticks > static_cast<double>(kFeetechMaxVelocityMagnitude) ||
    max_speed_ticks > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument(
      "speed_tick_scale * max_wheel_speed must fit Feetech sign-magnitude velocity range");
  }
}

geometry_msgs::msg::Twist selectOdometryTwist(
  const geometry_msgs::msg::Twist & command,
  const bool has_command,
  const double command_age,
  const double command_timeout)
{
  if (!has_command || command_age > command_timeout) {
    return geometry_msgs::msg::Twist{};
  }
  return command;
}

geometry_msgs::msg::Twist selectCommandAtTime(
  const geometry_msgs::msg::Twist & command,
  const bool has_command,
  const rclcpp::Time & last_command_time,
  const rclcpp::Time & current_time,
  const double command_timeout)
{
  return selectOdometryTwist(
    command,
    has_command,
    (current_time - last_command_time).seconds(),
    command_timeout);
}

geometry_msgs::msg::Twist selectOdometryUpdateTwist(
  const geometry_msgs::msg::Twist & command,
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const std::string & odom_source,
  const OmniKinematics & kinematics)
{
  if (odom_source == "encoder") {
    if (!encoder_wheel_speeds.has_value()) {
      return geometry_msgs::msg::Twist{};
    }
    return toRosTwist(kinematics.toBodyTwist(*encoder_wheel_speeds));
  }
  return command;
}

void updateOdometryForSource(
  Odometry & odometry,
  const std::string & odom_source,
  const std::string & encoder_feedback_source,
  const geometry_msgs::msg::Twist & selected_twist,
  const double control_dt,
  const std::optional<WheelPositionDelta> & wheel_position_delta,
  const OmniKinematics & kinematics)
{
  if (odom_source == "encoder") {
    if (encoder_feedback_source == "position" && wheel_position_delta) {
      odometry.updateDelta(kinematics.toBodyDelta(*wheel_position_delta));
    } else if (encoder_feedback_source == "speed") {
      odometry.update(selected_twist, control_dt);
    }
    return;
  }
  odometry.update(selected_twist, control_dt);
}

bool shouldRefreshEncoderSample(
  const std::string & odom_source,
  const double encoder_sample_age,
  const double encoder_read_period)
{
  if (odom_source != "encoder") {
    return false;
  }
  return encoder_sample_age >= encoder_read_period;
}

std::optional<WheelCommand> selectFreshEncoderWheelSpeeds(
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const double encoder_sample_age,
  const double encoder_sample_timeout)
{
  if (!encoder_wheel_speeds.has_value() || encoder_sample_age > encoder_sample_timeout) {
    return std::nullopt;
  }
  return encoder_wheel_speeds;
}

bool updateEncoderFeedbackOnSuccessfulRead(
  const std::optional<WheelFeedbackSample> & read_result,
  const rclcpp::Time & success_time,
  std::optional<WheelCommand> * last_sample,
  std::optional<rclcpp::Time> * last_success_time)
{
  if (!read_result || last_sample == nullptr || last_success_time == nullptr) {
    return false;
  }
  *last_sample = read_result->wheel_speeds;
  *last_success_time = success_time;
  return true;
}

bool encoderFeedbackTimedOut(
  const std::optional<rclcpp::Time> & encoder_wait_start_time,
  const std::optional<rclcpp::Time> & last_success_time,
  const rclcpp::Time & current_time,
  const double encoder_sample_timeout)
{
  const auto & deadline_start = last_success_time ? last_success_time : encoder_wait_start_time;
  return deadline_start &&
         (current_time - *deadline_start).seconds() > encoder_sample_timeout;
}

bool motorPowerEnableAllowed(
  const MotorReadinessLatch & motor_readiness,
  const bool recovery_disarmed)
{
  return !motor_readiness.faultLatched() || recovery_disarmed;
}

bool stopMotorOnEncoderTimeout(
  BaseMotorInterface & motor_interface,
  MotorReadinessLatch & motor_readiness,
  const std::optional<rclcpp::Time> & encoder_wait_start_time,
  const std::optional<rclcpp::Time> & last_success_time,
  const rclcpp::Time & current_time,
  const double encoder_sample_timeout)
{
  if (!encoderFeedbackTimedOut(
      encoder_wait_start_time,
      last_success_time,
      current_time,
      encoder_sample_timeout))
  {
    return false;
  }

  try {
    motor_interface.stop();
  } catch (...) {
  }
  try {
    (void)motor_interface.setMotorPower(false);
  } catch (...) {
  }
  motor_readiness.onBackendFault();
  return true;
}

WheelCommand selectJointStateWheelSpeeds(
  const WheelCommand & command_wheel_speeds,
  const std::optional<WheelCommand> & encoder_wheel_speeds,
  const std::string & odom_source)
{
  if (odom_source == "encoder" && encoder_wheel_speeds.has_value()) {
    return *encoder_wheel_speeds;
  }
  return command_wheel_speeds;
}

sensor_msgs::msg::JointState makeBaseWheelJointState(
  const WheelCommand & wheel_speeds,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = {"left_wheel_joint", "back_wheel_joint", "right_wheel_joint"};
  msg.position = {0.0, 0.0, 0.0};
  msg.velocity = {wheel_speeds.left, wheel_speeds.back, wheel_speeds.right};
  return msg;
}

std_srvs::srv::Trigger::Response resetOdometryForService(Odometry & odometry)
{
  odometry.reset();

  std_srvs::srv::Trigger::Response response;
  response.success = true;
  response.message = "odometry reset";
  return response;
}

std_srvs::srv::SetBool::Response setMotorPowerForService(
  BaseMotorInterface & motor_interface,
  const bool enabled)
{
  std_srvs::srv::SetBool::Response response;
  response.success = motor_interface.setMotorPower(enabled);
  if (response.success) {
    response.message = enabled ? "motor power enabled" : "motor power disabled";
  } else {
    response.message = enabled ? "failed to enable motor power" : "failed to disable motor power";
  }
  return response;
}

rclcpp::Time offsetTransformStamp(const rclcpp::Time & stamp, const double offset_seconds)
{
  return stamp + rclcpp::Duration::from_seconds(offset_seconds);
}

BaseController::BaseController(const rclcpp::NodeOptions & options)
: Node("lekiwi_node", options),
  params_(loadParameters()),
  kinematics_(
    params_.wheel_radius, params_.base_radius, params_.max_wheel_speed,
    params_.kinematics_frame_x, params_.kinematics_frame_y, params_.kinematics_frame_yaw),
  last_cmd_time_(now()),
  last_update_time_(now())
{
  validateBaseControllerParameters(params_);

  if (params_.motor_backend == "feetech") {
    if (!params_.enable_motor_write) {
      RCLCPP_WARN(
        get_logger(),
        "motor_backend is 'feetech' but enable_motor_write is false; Feetech serial probe will "
        "not run; using mock/logging backend");
    }
  } else if (params_.motor_backend != "mock") {
    RCLCPP_WARN(
      get_logger(),
      "unknown motor_backend '%s'; hardware interface will not be selected; using mock/logging "
      "backend",
      params_.motor_backend.c_str());
  }
  if (params_.odom_source != "command" && params_.odom_source != "encoder") {
    RCLCPP_WARN(
      get_logger(),
      "unknown odom_source '%s'; falling back to command odometry",
      params_.odom_source.c_str());
  }

  const auto backend = selectBaseMotorBackend(params_);
  if (backend == BaseMotorBackendKind::kMockLogging) {
    motor_interface_ = std::make_unique<LoggingMotorInterface>(get_logger(), params_.max_wheel_speed);
  } else if (backend == BaseMotorBackendKind::kFeetechHardware) {
    validateFeetechMotorParameters(params_);
    motor_interface_ = std::make_unique<FeetechMotorInterface>(
      makeFeetechMotorConfig(params_));
  }
  startMotorInterface();
  const rclcpp::Time initial_time = now();
  motor_safety_ = std::make_unique<MotorSafetyCoordinator>(
    *motor_interface_,
    usesRealMotorHardware(),
    usesRealMotorHardware() || params_.odom_source == "encoder",
    params_.encoder_read_period,
    params_.encoder_sample_timeout,
    initial_time,
    [this]() { return now(); });
  createRosInterfaces();
}

BaseController::~BaseController()
{
  if (control_timer_) {
    control_timer_->cancel();
  }
  stopMotorInterfaceForShutdown(motor_interface_.get());
}

void BaseController::startMotorInterface()
{
  if (motor_interface_) {
    motor_interface_->connect();
  }
}

void BaseController::createRosInterfaces()
{
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
  joint_state_pub_ =
    create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SystemDefaultsQoS());
  auto qos = makeMotorReadyQos();
  motor_ready_pub_ = create_publisher<std_msgs::msg::Bool>("motor_ready", qos);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { onCmdVel(msg); });
  reset_odometry_srv_ = create_service<std_srvs::srv::Trigger>(
    "reset_odometry",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      onResetOdometry(request, response);
    });
  motor_power_srv_ = create_service<std_srvs::srv::SetBool>(
    "motor_power",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      onMotorPower(request, response);
    });
  publishMotorReady();
  control_timer_ = create_wall_timer(kControlPeriod, [this]() { onControlTimer(); });
}

void BaseController::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const auto limited = limitBodyTwist(
    *msg,
    params_.max_linear_x,
    params_.max_linear_y,
    params_.max_linear_speed,
    params_.max_angular_z,
    kinematics_);
  if (!isFiniteTwist(*msg)) {
    RCLCPP_WARN(get_logger(), "received non-finite cmd_vel; commanding zero velocity");
  } else if (!twistsEqual(*msg, limited)) {
    RCLCPP_WARN(get_logger(), "cmd_vel exceeded body or wheel velocity limits and was limited");
  }
  current_cmd_ = limited;
  last_cmd_time_ = now();
  has_cmd_ = true;
}

void BaseController::onControlTimer()
{
  const rclcpp::Time current_time = now();
  const double dt = std::max(0.0, (current_time - last_update_time_).seconds());
  last_update_time_ = current_time;

  const auto safety_update = motor_safety_->beforeCommand(current_time);
  const rclcpp::Time safety_time = safety_update.safety_time;
  if (safety_update.readiness_changed) {
    publishMotorReady();
  }
  if (safety_update.encoder_timeout) {
    const char * stage = safety_update.encoder_timeout_stage == EncoderTimeoutStage::kBeforeRead ?
      "before_read" : "after_read";
    const char * read_status = "not_attempted";
    if (safety_update.encoder_read_status == EncoderReadStatus::kSample) {
      read_status = "sample";
    } else if (safety_update.encoder_read_status == EncoderReadStatus::kNoSample) {
      read_status = "no_sample";
    } else if (safety_update.encoder_read_status == EncoderReadStatus::kException) {
      read_status = "exception";
    }
    RCLCPP_ERROR(
      get_logger(),
      "encoder feedback timed out; motor power disabled and fault latched "
      "(stage=%s sample_age=%.3fs read_status=%s read_duration=%.3fs failed_reads=%u)",
      stage, safety_update.encoder_sample_age_seconds, read_status,
      safety_update.encoder_read_duration_seconds, safety_update.consecutive_failed_reads);
  }
  const geometry_msgs::msg::Twist command = selectCommandAtTime(
    current_cmd_, has_cmd_, last_cmd_time_, safety_time, params_.cmd_vel_timeout);
  const WheelCommand command_wheel_speeds =
    kinematics_.toWheelSpeeds(command.linear.x, command.linear.y, command.angular.z);

  motor_safety_->sendWheelCommand(command_wheel_speeds);
  const auto encoder_wheel_speeds = motor_safety_->freshEncoderWheelSpeeds(safety_time);
  const geometry_msgs::msg::Twist odom_twist =
    selectOdometryUpdateTwist(command, encoder_wheel_speeds, params_.odom_source, kinematics_);

  updateOdometryForSource(
    odometry_,
    params_.odom_source,
    params_.encoder_feedback_source,
    odom_twist,
    dt,
    safety_update.wheel_position_delta,
    kinematics_);
  odom_pub_->publish(
    odometry_.toMessage(current_time, params_.odom_frame, params_.base_frame, odom_twist));
  joint_state_pub_->publish(
    makeBaseWheelJointState(
      selectJointStateWheelSpeeds(command_wheel_speeds, encoder_wheel_speeds, params_.odom_source),
      current_time));
  tf_broadcaster_->sendTransform(
    odometry_.toTransform(
      offsetTransformStamp(current_time, params_.tf_time_offset),
      params_.odom_frame,
      params_.base_frame));
}

void BaseController::onResetOdometry(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  *response = resetOdometryForService(odometry_);
}

void BaseController::onMotorPower(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  *response = motor_safety_->setMotorPower(request->data, now());
  publishMotorReady();
}

void BaseController::publishMotorReady()
{
  if (!motor_ready_pub_) {
    return;
  }
  std_msgs::msg::Bool message;
  message.data = motor_safety_->motorReady();
  motor_ready_pub_->publish(message);
}

bool BaseController::usesRealMotorHardware() const
{
  return selectBaseMotorBackend(params_) == BaseMotorBackendKind::kFeetechHardware;
}

BaseControllerParameters BaseController::loadParameters()
{
  BaseControllerParameters params;
  params.monitor_motor_health = declare_parameter<bool>("monitor_motor_health", false);
  params.motor_min_voltage = declare_parameter<double>("motor_min_voltage", params.motor_min_voltage);
  params.motor_max_voltage = declare_parameter<double>("motor_max_voltage", params.motor_max_voltage);
  params.wheel_radius = declare_parameter<double>("wheel_radius", params.wheel_radius);
  params.base_radius = declare_parameter<double>("base_radius", params.base_radius);
  params.kinematics_frame_x =
    declare_parameter<double>("kinematics_frame_x", params.kinematics_frame_x);
  params.kinematics_frame_y =
    declare_parameter<double>("kinematics_frame_y", params.kinematics_frame_y);
  params.kinematics_frame_yaw =
    declare_parameter<double>("kinematics_frame_yaw", params.kinematics_frame_yaw);
  params.max_linear_x = declare_parameter<double>("max_linear_x", params.max_linear_x);
  params.max_linear_y = declare_parameter<double>("max_linear_y", params.max_linear_y);
  params.max_linear_speed =
    declare_parameter<double>("max_linear_speed", params.max_linear_speed);
  params.max_angular_z = declare_parameter<double>("max_angular_z", params.max_angular_z);
  params.max_wheel_speed = declare_parameter<double>("max_wheel_speed", params.max_wheel_speed);
  params.odom_frame = declare_parameter<std::string>("odom_frame", params.odom_frame);
  params.base_frame = declare_parameter<std::string>("base_frame", params.base_frame);
  params.cmd_vel_timeout = declare_parameter<double>("cmd_vel_timeout", params.cmd_vel_timeout);
  params.tf_time_offset = declare_parameter<double>("tf_time_offset", params.tf_time_offset);
  params.odom_source = declare_parameter<std::string>("odom_source", params.odom_source);
  params.encoder_read_period =
    declare_parameter<double>("encoder_read_period", params.encoder_read_period);
  params.encoder_sample_timeout =
    declare_parameter<double>("encoder_sample_timeout", params.encoder_sample_timeout);
  params.motor_backend = declare_parameter<std::string>("motor_backend", params.motor_backend);
  params.serial_port = declare_parameter<std::string>("serial_port", params.serial_port);
  params.enable_motor_write =
    declare_parameter<bool>("enable_motor_write", params.enable_motor_write);
  params.wheel_ids = declare_parameter<std::vector<int64_t>>("wheel_ids", params.wheel_ids);
  params.wheel_directions =
    declare_parameter<std::vector<double>>("wheel_directions", params.wheel_directions);
  params.baud_rate = declare_parameter<int64_t>("baud_rate", params.baud_rate);
  params.speed_tick_scale = declare_parameter<double>("speed_tick_scale", params.speed_tick_scale);
  params.speed_tick_limit = declare_parameter<int64_t>("speed_tick_limit", params.speed_tick_limit);
  params.encoder_tick_deadband =
    declare_parameter<int64_t>("encoder_tick_deadband", params.encoder_tick_deadband);
  params.encoder_feedback_source =
    declare_parameter<std::string>("encoder_feedback_source", params.encoder_feedback_source);
  params.encoder_position_ticks_per_revolution =
    declare_parameter<double>(
      "encoder_position_ticks_per_revolution",
      params.encoder_position_ticks_per_revolution);
  params.encoder_odom_scale =
    declare_parameter<double>("encoder_odom_scale", params.encoder_odom_scale);
  params.encoder_position_tick_deadband =
    declare_parameter<int64_t>(
      "encoder_position_tick_deadband",
      params.encoder_position_tick_deadband);
  params.torque_enable = declare_parameter<bool>("torque_enable", params.torque_enable);
  params.log_encoder_reads =
    declare_parameter<bool>("log_encoder_reads", params.log_encoder_reads);
  return params;
}

}  // namespace lekiwi_node
