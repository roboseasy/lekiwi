#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

#include "lekiwi_node/base_controller.hpp"
#include "lekiwi_node/odometry.hpp"
#include "lekiwi_node/omni_kinematics.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

std::string readSourceFile(const std::string & relative_path)
{
  std::ifstream stream(std::string(LEKIWI_NODE_SOURCE_DIR) + "/" + relative_path);
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

}  // namespace

TEST(BaseControllerBuildContractTest, DeclaresStdMsgsDependency)
{
  const auto cmake = readSourceFile("CMakeLists.txt");
  const auto package_xml = readSourceFile("package.xml");

  EXPECT_NE(cmake.find("find_package(std_msgs REQUIRED)"), std::string::npos);
  EXPECT_NE(cmake.find("  std_msgs\n"), std::string::npos);
  EXPECT_NE(package_xml.find("<depend>std_msgs</depend>"), std::string::npos);
}

TEST(BaseControllerBuildContractTest, DescribesProtocolBackendWithoutHardwareAcceptanceClaim)
{
  const auto package_xml = readSourceFile("package.xml");

  EXPECT_NE(
    package_xml.find(
      "<description>LeKiwi base control node with mock logging and protocol-level Feetech motor "
      "backends.</description>"),
    std::string::npos);
  EXPECT_EQ(package_xml.find("skeleton"), std::string::npos);
  EXPECT_EQ(package_xml.find("hardware-accepted"), std::string::npos);
}

TEST(BaseControllerPublisherContractTest, MotorReadyUsesTransientLocalReliableQos)
{
  const auto controller_source = readSourceFile("src/base_controller.cpp");
  const auto coordinator_source = readSourceFile("src/motor_safety_coordinator.cpp");

  EXPECT_NE(
    coordinator_source.find("rclcpp::QoS(1).reliable().transient_local()"),
    std::string::npos);
  EXPECT_NE(
    controller_source.find("auto qos = makeMotorReadyQos()"),
    std::string::npos);
  EXPECT_NE(
    controller_source.find("create_publisher<std_msgs::msg::Bool>(\"motor_ready\", qos)"),
    std::string::npos);
}

TEST(BaseControllerSerialContractTest, PosixReadTimeoutIsFiveMilliseconds)
{
  const auto source = readSourceFile("src/feetech_motor_interface.cpp");

  EXPECT_NE(source.find("constexpr int kSerialReadTimeoutMs = 5;"), std::string::npos);
}

TEST(BaseControllerSafetyWiringContractTest, UsesCoordinatorForCycleServiceAndPublication)
{
  const auto source = readSourceFile("src/base_controller.cpp");

  EXPECT_NE(source.find("motor_safety_->beforeCommand(current_time)"), std::string::npos);
  EXPECT_NE(source.find("motor_safety_->sendWheelCommand(command_wheel_speeds)"), std::string::npos);
  EXPECT_NE(source.find("motor_safety_->freshEncoderWheelSpeeds(safety_time)"), std::string::npos);
  EXPECT_NE(source.find("motor_safety_->setMotorPower(request->data, now())"), std::string::npos);
  EXPECT_NE(source.find("message.data = motor_safety_->motorReady()"), std::string::npos);
}

TEST(OmniKinematicsTest, ZeroTwistProducesStoppedWheelCommands)
{
  const lekiwi_node::OmniKinematics kinematics;
  const auto command = kinematics.toWheelSpeeds(0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(command.left, 0.0);
  EXPECT_DOUBLE_EQ(command.back, 0.0);
  EXPECT_DOUBLE_EQ(command.right, 0.0);
}

TEST(BaseControllerBackendTest, FeetechWithoutMotorWriteUsesMockLoggingBackend)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = false;

  EXPECT_EQ(
    lekiwi_node::BaseMotorBackendKind::kMockLogging,
    lekiwi_node::selectBaseMotorBackend(params));
}

TEST(BaseControllerBackendTest, FeetechWithMotorWriteUsesFeetechHardwareBackend)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = true;

  EXPECT_EQ(
    lekiwi_node::BaseMotorBackendKind::kFeetechHardware,
    lekiwi_node::selectBaseMotorBackend(params));
}

TEST(BaseControllerCommandLimitTest, RejectsNonFiniteTwistAsZero)
{
  const lekiwi_node::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  geometry_msgs::msg::Twist command;
  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  command.angular.z = 0.2;
  const auto limited = lekiwi_node::limitBodyTwist(
    command, 0.26, 0.15, 0.26, 1.82, kinematics);
  EXPECT_DOUBLE_EQ(limited.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(limited.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(limited.angular.z, 0.0);
}

TEST(BaseControllerCommandLimitTest, PreservesDirectionWhileApplyingAxisAndPlanarLimits)
{
  const lekiwi_node::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.52;
  command.linear.y = 0.20;
  const auto limited = lekiwi_node::limitBodyTwist(
    command, 0.26, 0.15, 0.26, 1.82, kinematics);
  EXPECT_LE(std::abs(limited.linear.x), 0.26);
  EXPECT_LE(std::abs(limited.linear.y), 0.15);
  EXPECT_LE(std::hypot(limited.linear.x, limited.linear.y), 0.26 + 1e-12);
  EXPECT_NEAR(limited.linear.x / limited.linear.y, command.linear.x / command.linear.y, 1e-12);
  EXPECT_DOUBLE_EQ(limited.angular.z, 0.0);
}

TEST(BaseControllerCommandLimitTest, ClampsWaffleAngularLimit)
{
  const lekiwi_node::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  geometry_msgs::msg::Twist command;
  command.angular.z = 3.64;

  const auto limited = lekiwi_node::limitBodyTwist(
    command, 0.26, 0.15, 0.26, 1.82, kinematics);

  EXPECT_NEAR(limited.angular.z, 1.82, 1e-12);
}

TEST(BaseControllerCommandLimitTest, CombinedBodyCommandUsesUniformWheelScale)
{
  constexpr double kWheelRadius = 0.0508;
  constexpr double kBaseRadius = 0.115;
  constexpr double kMaxWheelSpeed = 5.4;
  const lekiwi_node::OmniKinematics kinematics(
    kWheelRadius, kBaseRadius, kMaxWheelSpeed);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.26 * std::sqrt(3.0) * 0.5;
  command.linear.y = 0.26 * 0.5;
  command.angular.z = 1.82;
  const double expected_peak_wheel_speed =
    (std::sqrt(3.0) * 0.5 * command.linear.x + 0.5 * command.linear.y +
    kBaseRadius * command.angular.z) / kWheelRadius;
  const double expected_scale = kMaxWheelSpeed / expected_peak_wheel_speed;

  const auto limited = lekiwi_node::limitBodyTwist(
    command, 0.26, 0.15, 0.26, 1.82, kinematics);

  ASSERT_LT(expected_scale, 1.0);
  EXPECT_NEAR(limited.linear.x, command.linear.x * expected_scale, 1e-12);
  EXPECT_NEAR(limited.linear.y, command.linear.y * expected_scale, 1e-12);
  EXPECT_NEAR(limited.angular.z, command.angular.z * expected_scale, 1e-12);

  const auto odom_twist = lekiwi_node::selectOdometryUpdateTwist(
    limited, std::nullopt, "command", kinematics);
  EXPECT_NEAR(odom_twist.linear.x, limited.linear.x, 1e-12);
  EXPECT_NEAR(odom_twist.linear.y, limited.linear.y, 1e-12);
  EXPECT_NEAR(odom_twist.angular.z, limited.angular.z, 1e-12);
}

TEST(BaseControllerOdometryTest, TimeoutCommandProducesZeroOdomTwist)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;

  const auto active_twist =
    lekiwi_node::selectOdometryTwist(command, true, 0.29, 0.3);
  EXPECT_DOUBLE_EQ(active_twist.linear.x, command.linear.x);
  EXPECT_DOUBLE_EQ(active_twist.linear.y, command.linear.y);
  EXPECT_DOUBLE_EQ(active_twist.angular.z, command.angular.z);

  const auto timeout_twist =
    lekiwi_node::selectOdometryTwist(command, true, 0.31, 0.3);
  EXPECT_DOUBLE_EQ(timeout_twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(timeout_twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(timeout_twist.angular.z, 0.0);
}

TEST(BaseControllerOdometrySourceTest, CommandSourceUsesCommandTwist)
{
  const lekiwi_node::OmniKinematics kinematics;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;

  const auto selected = lekiwi_node::selectOdometryUpdateTwist(
    command, lekiwi_node::WheelCommand{1.0, 2.0, 3.0}, "command", kinematics);

  EXPECT_DOUBLE_EQ(selected.linear.x, command.linear.x);
  EXPECT_DOUBLE_EQ(selected.linear.y, command.linear.y);
  EXPECT_DOUBLE_EQ(selected.angular.z, command.angular.z);
}

TEST(BaseControllerOdometrySourceTest, EncoderSourceUsesBodyTwistFromWheelSpeeds)
{
  const lekiwi_node::OmniKinematics kinematics;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;
  const auto wheel_speeds = kinematics.toWheelSpeeds(0.12, -0.04, 0.2);

  const auto selected = lekiwi_node::selectOdometryUpdateTwist(
    command, wheel_speeds, "encoder", kinematics);
  const auto expected = kinematics.toBodyTwist(wheel_speeds);

  EXPECT_NEAR(selected.linear.x, expected.linear_x, 1e-12);
  EXPECT_NEAR(selected.linear.y, expected.linear_y, 1e-12);
  EXPECT_NEAR(selected.angular.z, expected.angular_z, 1e-12);
}

TEST(BaseControllerOdometrySourceTest, EncoderSourceReadFailureReturnsZeroTwist)
{
  const lekiwi_node::OmniKinematics kinematics;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;

  const auto selected = lekiwi_node::selectOdometryUpdateTwist(
    command, std::nullopt, "encoder", kinematics);

  EXPECT_DOUBLE_EQ(selected.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(selected.angular.z, 0.0);
}

TEST(BaseControllerOdometrySourceTest, EncoderDeltaChangesPoseOnlyOnItsReadCycle)
{
  const lekiwi_node::OmniKinematics kinematics;
  lekiwi_node::Odometry odometry;
  const lekiwi_node::WheelPositionDelta wheel_delta{0.1, -0.2, 0.3};
  geometry_msgs::msg::Twist smoothed_twist;
  smoothed_twist.linear.x = 0.7;
  smoothed_twist.linear.y = -0.4;
  smoothed_twist.angular.z = 0.6;

  lekiwi_node::updateOdometryForSource(
    odometry, "encoder", "position", smoothed_twist, 0.02, wheel_delta, kinematics);
  const auto after_delta = odometry.toMessage(
    rclcpp::Time(0, 0), "odom", "base_footprint", smoothed_twist);

  for (int cycle = 0; cycle < 2; ++cycle) {
    lekiwi_node::updateOdometryForSource(
      odometry, "encoder", "position", smoothed_twist, 0.02, std::nullopt, kinematics);
  }
  const auto after_holds = odometry.toMessage(
    rclcpp::Time(0, 40000000), "odom", "base_footprint", smoothed_twist);

  EXPECT_DOUBLE_EQ(after_holds.pose.pose.position.x, after_delta.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(after_holds.pose.pose.position.y, after_delta.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(after_holds.pose.pose.orientation.z, after_delta.pose.pose.orientation.z);
  EXPECT_DOUBLE_EQ(after_holds.pose.pose.orientation.w, after_delta.pose.pose.orientation.w);
  EXPECT_DOUBLE_EQ(after_holds.twist.twist.linear.x, smoothed_twist.linear.x);
  EXPECT_DOUBLE_EQ(after_holds.twist.twist.linear.y, smoothed_twist.linear.y);
  EXPECT_DOUBLE_EQ(after_holds.twist.twist.angular.z, smoothed_twist.angular.z);
}

TEST(BaseControllerOdometrySourceTest, SpeedFeedbackEncoderSourceIntegratesWheelDerivedTwist)
{
  const lekiwi_node::OmniKinematics kinematics;
  lekiwi_node::Odometry odometry;
  geometry_msgs::msg::Twist encoder_twist;
  encoder_twist.linear.x = 0.12;
  encoder_twist.linear.y = -0.04;
  encoder_twist.angular.z = 0.2;

  lekiwi_node::updateOdometryForSource(
    odometry, "encoder", "speed", encoder_twist, 0.5, std::nullopt, kinematics);
  const auto msg = odometry.toMessage(
    rclcpp::Time(0, 0), "odom", "base_footprint", encoder_twist);

  EXPECT_NEAR(msg.pose.pose.position.x, 0.06, 1e-12);
  EXPECT_NEAR(msg.pose.pose.position.y, -0.02, 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.z, std::sin(0.1 / 2.0), 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.w, std::cos(0.1 / 2.0), 1e-12);
}

TEST(BaseControllerOdometrySourceTest, EncoderPoseUsesRawNetTickDeltaNotSmoothedEventSum)
{
  constexpr double kTicksPerRevolution = 4096.0;
  constexpr double kPi = 3.14159265358979323846;
  const lekiwi_node::OmniKinematics kinematics;
  lekiwi_node::Odometry odometry;
  const double angle_per_tick = 2.0 * kPi / kTicksPerRevolution;
  const lekiwi_node::WheelPositionDelta raw_net_delta{-5.0 * angle_per_tick, 0.0, 0.0};
  const auto expected = kinematics.toBodyDelta(raw_net_delta);
  geometry_msgs::msg::Twist larger_smoothed_window_twist;
  larger_smoothed_window_twist.linear.x = 8.0;
  larger_smoothed_window_twist.linear.y = -6.0;
  larger_smoothed_window_twist.angular.z = 4.0;

  lekiwi_node::updateOdometryForSource(
    odometry,
    "encoder",
    "position",
    larger_smoothed_window_twist,
    0.02,
    raw_net_delta,
    kinematics);
  const auto msg = odometry.toMessage(
    rclcpp::Time(0, 0), "odom", "base_footprint", larger_smoothed_window_twist);

  EXPECT_NEAR(msg.pose.pose.position.x, expected.dx, 1e-12);
  EXPECT_NEAR(msg.pose.pose.position.y, expected.dy, 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.z, std::sin(expected.dyaw / 2.0), 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.w, std::cos(expected.dyaw / 2.0), 1e-12);
  EXPECT_DOUBLE_EQ(msg.twist.twist.linear.x, larger_smoothed_window_twist.linear.x);
}

TEST(BaseControllerOdometrySourceTest, CommandSourceIntegratesTwistAndIgnoresEncoderDelta)
{
  const lekiwi_node::OmniKinematics kinematics;
  lekiwi_node::Odometry odometry;
  geometry_msgs::msg::Twist command_twist;
  command_twist.linear.x = 0.2;
  command_twist.linear.y = -0.1;
  command_twist.angular.z = 0.3;

  lekiwi_node::updateOdometryForSource(
    odometry,
    "command",
    "speed",
    command_twist,
    0.5,
    lekiwi_node::WheelPositionDelta{10.0, 20.0, 30.0},
    kinematics);
  const auto msg = odometry.toMessage(
    rclcpp::Time(0, 0), "odom", "base_footprint", command_twist);

  EXPECT_NEAR(msg.pose.pose.position.x, 0.1, 1e-12);
  EXPECT_NEAR(msg.pose.pose.position.y, -0.05, 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.z, std::sin(0.15 / 2.0), 1e-12);
  EXPECT_NEAR(msg.pose.pose.orientation.w, std::cos(0.15 / 2.0), 1e-12);
}

TEST(BaseControllerJointStateTest, BuildsWheelJointStateWithUrdfWheelJointNamesAndVelocities)
{
  const auto msg = lekiwi_node::makeBaseWheelJointState(
    lekiwi_node::WheelCommand{1.0, -2.0, 3.0},
    rclcpp::Time(12, 340000000));

  EXPECT_EQ(msg.header.stamp, rclcpp::Time(12, 340000000));
  EXPECT_EQ(
    msg.name,
    (std::vector<std::string>{"left_wheel_joint", "back_wheel_joint", "right_wheel_joint"}));
  EXPECT_EQ(msg.position, (std::vector<double>{0.0, 0.0, 0.0}));
  EXPECT_EQ(msg.velocity, (std::vector<double>{1.0, -2.0, 3.0}));
  EXPECT_TRUE(msg.effort.empty());
}

TEST(BaseControllerJointStateTest, EncoderSourceUsesFreshEncoderSpeedsForJointState)
{
  const auto selected = lekiwi_node::selectJointStateWheelSpeeds(
    lekiwi_node::WheelCommand{1.0, 2.0, 3.0},
    lekiwi_node::WheelCommand{-1.0, -2.0, -3.0},
    "encoder");

  EXPECT_DOUBLE_EQ(selected.left, -1.0);
  EXPECT_DOUBLE_EQ(selected.back, -2.0);
  EXPECT_DOUBLE_EQ(selected.right, -3.0);
}

TEST(BaseControllerJointStateTest, EncoderSourceFallsBackToCommandSpeedsWithoutEncoderSample)
{
  const auto selected = lekiwi_node::selectJointStateWheelSpeeds(
    lekiwi_node::WheelCommand{1.0, 2.0, 3.0},
    std::nullopt,
    "encoder");

  EXPECT_DOUBLE_EQ(selected.left, 1.0);
  EXPECT_DOUBLE_EQ(selected.back, 2.0);
  EXPECT_DOUBLE_EQ(selected.right, 3.0);
}

TEST(BaseControllerServiceTest, ResetOdometryHelperResetsPoseAndReportsSuccess)
{
  lekiwi_node::Odometry odometry;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.angular.z = 0.5;
  odometry.update(twist, 1.0);

  const auto response = lekiwi_node::resetOdometryForService(odometry);
  const auto msg = odometry.toMessage(rclcpp::Time(0, 0), "odom", "base_footprint", twist);

  EXPECT_TRUE(response.success);
  EXPECT_FALSE(response.message.empty());
  EXPECT_DOUBLE_EQ(msg.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(msg.pose.pose.orientation.w, 1.0);
}

TEST(OdometryTest, AppliesBodyPoseDeltaExactlyOnceInWorldFrame)
{
  lekiwi_node::Odometry odometry;
  odometry.updateDelta({0.0, 0.0, M_PI_2});
  odometry.updateDelta({1.0, 0.25, 0.0});

  const auto msg = odometry.toMessage(
    rclcpp::Time(1, 0), "odom", "base_footprint", geometry_msgs::msg::Twist{});
  EXPECT_NEAR(msg.pose.pose.position.x, -0.25, 1e-12);
  EXPECT_NEAR(msg.pose.pose.position.y, 1.0, 1e-12);
}

TEST(BaseControllerOdometrySourceTest, EncoderReadIsThrottledByConfiguredPeriod)
{
  EXPECT_FALSE(lekiwi_node::shouldRefreshEncoderSample("encoder", 0.0, 0.1));
  EXPECT_FALSE(lekiwi_node::shouldRefreshEncoderSample("encoder", 0.05, 0.1));
  EXPECT_TRUE(lekiwi_node::shouldRefreshEncoderSample("encoder", 0.1, 0.1));
  EXPECT_FALSE(lekiwi_node::shouldRefreshEncoderSample("command", 0.2, 0.1));
}

TEST(BaseControllerOdometrySourceTest, StaleEncoderSampleIsIgnored)
{
  const lekiwi_node::WheelCommand wheel_speeds{1.0, 2.0, 3.0};

  const auto fresh = lekiwi_node::selectFreshEncoderWheelSpeeds(wheel_speeds, 0.19, 0.2);
  ASSERT_TRUE(fresh.has_value());
  EXPECT_DOUBLE_EQ(fresh->left, wheel_speeds.left);
  EXPECT_DOUBLE_EQ(fresh->back, wheel_speeds.back);
  EXPECT_DOUBLE_EQ(fresh->right, wheel_speeds.right);

  EXPECT_FALSE(lekiwi_node::selectFreshEncoderWheelSpeeds(wheel_speeds, 0.21, 0.2).has_value());
  EXPECT_FALSE(lekiwi_node::selectFreshEncoderWheelSpeeds(std::nullopt, 0.0, 0.2).has_value());
}

TEST(BaseControllerEncoderTimingTest, FailedAttemptLeavesLastSuccessfulSampleTimeUnchanged)
{
  std::optional<lekiwi_node::WheelCommand> last_sample;
  std::optional<rclcpp::Time> last_success_time;
  const rclcpp::Time successful_attempt(10, 0);

  EXPECT_TRUE(lekiwi_node::updateEncoderFeedbackOnSuccessfulRead(
      lekiwi_node::WheelFeedbackSample{
        lekiwi_node::WheelCommand{1.0, 2.0, 3.0},
        lekiwi_node::WheelPositionDelta{4.0, 5.0, 6.0}},
      successful_attempt,
      &last_sample,
      &last_success_time));
  ASSERT_TRUE(last_success_time.has_value());
  EXPECT_EQ(*last_success_time, successful_attempt);

  EXPECT_FALSE(lekiwi_node::updateEncoderFeedbackOnSuccessfulRead(
      std::nullopt,
      rclcpp::Time(20, 0),
      &last_sample,
      &last_success_time));
  ASSERT_TRUE(last_success_time.has_value());
  EXPECT_EQ(*last_success_time, successful_attempt);
  ASSERT_TRUE(last_sample.has_value());
  EXPECT_DOUBLE_EQ(last_sample->left, 1.0);
  EXPECT_DOUBLE_EQ(last_sample->back, 2.0);
  EXPECT_DOUBLE_EQ(last_sample->right, 3.0);
}

TEST(BaseControllerEncoderTimingTest, PostReadTimeDetectsTimeoutCrossedDuringRead)
{
  const std::optional<rclcpp::Time> wait_start{rclcpp::Time(9, 700000000)};
  const std::optional<rclcpp::Time> last_success{rclcpp::Time(9, 800000000)};

  EXPECT_FALSE(lekiwi_node::encoderFeedbackTimedOut(
      wait_start, last_success, rclcpp::Time(10, 0), 0.3));
  EXPECT_TRUE(lekiwi_node::encoderFeedbackTimedOut(
      wait_start, last_success, rclcpp::Time(10, 200000000), 0.3));
  EXPECT_FALSE(lekiwi_node::encoderFeedbackTimedOut(
      wait_start, std::nullopt, rclcpp::Time(10, 0), 0.3));
  EXPECT_TRUE(lekiwi_node::encoderFeedbackTimedOut(
      wait_start, std::nullopt, rclcpp::Time(10, 1), 0.3));
  EXPECT_FALSE(lekiwi_node::encoderFeedbackTimedOut(
      std::nullopt, std::nullopt, rclcpp::Time(10, 200000000), 0.3));
}

TEST(BaseControllerCommandTimingTest, PostReadTimeStopsCommandThatExpiresDuringRead)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.1;
  const rclcpp::Time last_command_time(9, 800000000);

  const auto before_read = lekiwi_node::selectCommandAtTime(
    command, true, last_command_time, rclcpp::Time(10, 0), 0.3);
  const auto after_read = lekiwi_node::selectCommandAtTime(
    command, true, last_command_time, rclcpp::Time(10, 200000000), 0.3);

  EXPECT_DOUBLE_EQ(before_read.linear.x, 0.1);
  EXPECT_DOUBLE_EQ(after_read.linear.x, 0.0);
}

TEST(BaseControllerOdometrySourceTest, UnknownSourceUsesCommandTwist)
{
  const lekiwi_node::OmniKinematics kinematics;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.linear.y = -0.1;
  command.angular.z = 0.3;

  const auto selected = lekiwi_node::selectOdometryUpdateTwist(
    command, std::nullopt, "wheel_encoder", kinematics);

  EXPECT_DOUBLE_EQ(selected.linear.x, command.linear.x);
  EXPECT_DOUBLE_EQ(selected.linear.y, command.linear.y);
  EXPECT_DOUBLE_EQ(selected.angular.z, command.angular.z);
}

TEST(BaseControllerOdometryTest, TfTimeOffsetOnlyFutureDatesTransformStamp)
{
  const rclcpp::Time stamp(10, 100000000);

  EXPECT_EQ(lekiwi_node::offsetTransformStamp(stamp, 0.0), stamp);
  EXPECT_EQ(lekiwi_node::offsetTransformStamp(stamp, 0.05), rclcpp::Time(10, 150000000));
}

TEST(OdometryMessageTest, CommandBasedOdomUsesExplicitNonZeroCovariance)
{
  lekiwi_node::Odometry odometry;
  geometry_msgs::msg::Twist twist;
  const auto msg = odometry.toMessage(rclcpp::Time(0, 0), "odom", "base_footprint", twist);

  EXPECT_EQ(msg.header.frame_id, "odom");
  EXPECT_EQ(msg.child_frame_id, "base_footprint");
  EXPECT_GT(msg.pose.covariance[0], 0.0);
  EXPECT_GT(msg.pose.covariance[7], 0.0);
  EXPECT_GT(msg.pose.covariance[35], 0.0);
  EXPECT_GT(msg.twist.covariance[0], 0.0);
  EXPECT_GT(msg.twist.covariance[7], 0.0);
  EXPECT_GT(msg.twist.covariance[35], 0.0);
  EXPECT_TRUE(std::all_of(
    msg.pose.covariance.begin(), msg.pose.covariance.end(),
    [](const double value) { return std::isfinite(value); }));
  EXPECT_TRUE(std::all_of(
    msg.twist.covariance.begin(), msg.twist.covariance.end(),
    [](const double value) { return std::isfinite(value); }));
}

TEST(BaseControllerParameterValidationTest, AcceptsValidFeetechHardwareParameters)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = true;

  EXPECT_NO_THROW(lekiwi_node::validateFeetechMotorParameters(params));
}

TEST(BaseControllerParameterValidationTest, PublicDefaultsDisableDeprecatedPositionDeadband)
{
  const lekiwi_node::BaseControllerParameters params;

  EXPECT_EQ(params.odom_source, "encoder");
  EXPECT_EQ(params.encoder_position_tick_deadband, 0);
}

TEST(BaseControllerParameterValidationTest, RejectsInvalidEncoderTimingParameters)
{
  lekiwi_node::BaseControllerParameters params;

  params.encoder_read_period = 0.0;
  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);

  params.encoder_read_period = 0.1;
  params.encoder_sample_timeout = -0.1;
  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);

  params.encoder_sample_timeout = std::numeric_limits<double>::infinity();
  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsDeprecatedPositionDeadbandForPositionFeedback)
{
  lekiwi_node::BaseControllerParameters params;
  params.encoder_feedback_source = "position";
  params.encoder_position_tick_deadband = 1;

  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, AcceptsDeprecatedPositionDeadbandForSpeedFeedback)
{
  lekiwi_node::BaseControllerParameters params;
  params.encoder_feedback_source = "speed";
  params.encoder_position_tick_deadband = 4;

  EXPECT_NO_THROW(lekiwi_node::validateBaseControllerParameters(params));
}

TEST(BaseControllerParameterValidationTest, RejectsPositionFeedbackWindowShorterThanThreeReads)
{
  lekiwi_node::BaseControllerParameters params;
  params.encoder_feedback_source = "position";
  params.encoder_read_period = 0.1001;
  params.encoder_sample_timeout = 0.3;

  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, AcceptsPositionFeedbackWindowAtThreeReadBoundary)
{
  lekiwi_node::BaseControllerParameters params;
  params.encoder_feedback_source = "position";
  params.encoder_read_period = 0.1;
  params.encoder_sample_timeout = 0.3;

  EXPECT_NO_THROW(lekiwi_node::validateBaseControllerParameters(params));
}

TEST(BaseControllerParameterValidationTest, DoesNotApplyPositionWindowTimingToSpeedFeedback)
{
  lekiwi_node::BaseControllerParameters params;
  params.encoder_feedback_source = "speed";
  params.encoder_read_period = 0.1001;
  params.encoder_sample_timeout = 0.3;

  EXPECT_NO_THROW(lekiwi_node::validateBaseControllerParameters(params));
}

TEST(BaseControllerParameterValidationTest, RejectsInvalidBodyVelocityEnvelope)
{
  lekiwi_node::BaseControllerParameters params;

  params.max_linear_x = 0.0;
  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);

  params.max_linear_x = 0.18;
  params.max_linear_y = std::numeric_limits<double>::infinity();
  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);

  params.max_linear_y = 0.15;
  params.max_linear_speed = -0.18;
  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);

  params.max_linear_speed = 0.18;
  params.max_angular_z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, AcceptsReferenceGeometryAndVelocityLimit)
{
  lekiwi_node::BaseControllerParameters params;
  EXPECT_DOUBLE_EQ(params.wheel_radius, 0.05);
  EXPECT_DOUBLE_EQ(params.base_radius, 0.13647);
  EXPECT_NEAR(params.speed_tick_scale, 4096.0 / (2.0 * std::acos(-1.0)), 1e-10);
  EXPECT_NEAR(params.max_wheel_speed * params.speed_tick_scale, 1500.0, 1e-9);
  EXPECT_NO_THROW(lekiwi_node::validateBaseControllerParameters(params));
}

TEST(BaseControllerParameterValidationTest, RejectsIndividuallyInfeasibleAxis)
{
  lekiwi_node::BaseControllerParameters params;
  params.max_linear_speed = 0.56;
  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);
  params.max_linear_speed = 0.26;
  params.max_angular_z = 5.0;
  EXPECT_THROW(lekiwi_node::validateBaseControllerParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsWheelIdsOutsideServoRange)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = true;

  params.wheel_ids = {7, 0, 9};
  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);

  params.wheel_ids = {7, 254, 9};
  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsDuplicateWheelIds)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = true;
  params.wheel_ids = {7, 7, 9};

  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsZeroWheelDirection)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = true;
  params.wheel_directions = {1.0, 0.0, -1.0};

  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsInt64ValuesOutsideIntOrBackendRange)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = true;

  params.baud_rate = static_cast<int64_t>(std::numeric_limits<int>::max()) + 1;
  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);

  params.baud_rate = 1000000;
  params.speed_tick_limit = static_cast<int64_t>(std::numeric_limits<int>::max()) + 1;
  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);

  params.speed_tick_limit = 32768;
  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);

  params.speed_tick_limit = 1000;
  params.encoder_tick_deadband = static_cast<int64_t>(std::numeric_limits<int>::max()) + 1;
  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);

  params.encoder_tick_deadband = 32768;
  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsFractionalPositionTicksPerRevolution)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = true;
  params.encoder_position_ticks_per_revolution = 4096.5;

  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);
}

TEST(BaseControllerParameterValidationTest, RejectsNonPositiveOrNonFiniteEncoderOdomScale)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = true;

  for (const double invalid_scale : {
      0.0,
      -1.0,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    params.encoder_odom_scale = invalid_scale;
    EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);
  }
}

TEST(BaseControllerParameterValidationTest, RejectsVelocityScaleOutsideSignMagnitudeRange)
{
  lekiwi_node::BaseControllerParameters params;
  params.motor_backend = "feetech";
  params.enable_motor_write = true;
  params.max_wheel_speed = 3.0;
  params.speed_tick_scale = 20000.0;

  EXPECT_THROW(lekiwi_node::validateFeetechMotorParameters(params), std::invalid_argument);
}
