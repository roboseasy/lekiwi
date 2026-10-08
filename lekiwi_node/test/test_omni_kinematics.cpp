#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "lekiwi_node/base_controller.hpp"
#include "lekiwi_node/omni_kinematics.hpp"

namespace
{
constexpr double kTolerance = 1.0e-9;

}  // namespace

TEST(OmniKinematicsFrameTest, ReferenceCalibrationRoundTripsSixAxesWithinRawLimit)
{
  const lekiwi_node::BaseControllerParameters p;
  const lekiwi_node::OmniKinematics kinematics(
    p.wheel_radius, p.base_radius, p.max_wheel_speed,
    p.kinematics_frame_x, p.kinematics_frame_y, p.kinematics_frame_yaw);
  for (const auto & input : std::array<lekiwi_node::BodyTwist, 6>{
      lekiwi_node::BodyTwist{0.10, 0, 0}, {-0.10, 0, 0},
      {0, 0.10, 0}, {0, -0.10, 0}, {0, 0, 0.75}, {0, 0, -0.75}})
  {
    const auto wheels = kinematics.toWheelSpeeds(input.linear_x, input.linear_y, input.angular_z);
    for (const double speed : {wheels.left, wheels.back, wheels.right}) {
      EXPECT_LE(std::abs(speed * p.speed_tick_scale), p.speed_tick_limit + kTolerance);
    }
    const auto actual = kinematics.toBodyTwist(wheels);
    EXPECT_NEAR(actual.linear_x, input.linear_x, kTolerance);
    EXPECT_NEAR(actual.linear_y, input.linear_y, kTolerance);
    EXPECT_NEAR(actual.angular_z, input.angular_z, kTolerance);
    const auto delta = kinematics.toBodyDelta({wheels.left, wheels.back, wheels.right});
    // Finite displacement is not instantaneous velocity at an offset origin.
    if (input.angular_z == 0.0) {
      EXPECT_NEAR(delta.dx, input.linear_x, kTolerance);
      EXPECT_NEAR(delta.dy, input.linear_y, kTolerance);
    }
    EXPECT_NEAR(delta.dyaw, input.angular_z, kTolerance);
  }
}

TEST(OmniKinematicsTest, ForwardLateralAndYawTwistsProduceImageWheelSignPatterns)
{
  const lekiwi_node::OmniKinematics kinematics;

  const auto forward = kinematics.toWheelSpeeds(0.1, 0.0, 0.0);
  EXPECT_LT(forward.left, 0.0);
  EXPECT_NEAR(forward.back, 0.0, kTolerance);
  EXPECT_GT(forward.right, 0.0);

  const auto lateral = kinematics.toWheelSpeeds(0.0, 0.1, 0.0);
  EXPECT_GT(lateral.left, 0.0);
  EXPECT_LT(lateral.back, 0.0);
  EXPECT_GT(lateral.right, 0.0);

  const auto yaw = kinematics.toWheelSpeeds(0.0, 0.0, 0.1);
  EXPECT_GT(yaw.left, 0.0);
  EXPECT_GT(yaw.back, 0.0);
  EXPECT_GT(yaw.right, 0.0);
}

TEST(OmniKinematicsTest, InverseKinematicsRestoresNominalBodyTwist)
{
  const lekiwi_node::OmniKinematics kinematics;
  const lekiwi_node::BodyTwist expected{0.2, -0.1, 0.4};

  const auto command =
    kinematics.toWheelSpeeds(expected.linear_x, expected.linear_y, expected.angular_z);
  const auto actual = kinematics.toBodyTwist(command);

  EXPECT_NEAR(actual.linear_x, expected.linear_x, kTolerance);
  EXPECT_NEAR(actual.linear_y, expected.linear_y, kTolerance);
  EXPECT_NEAR(actual.angular_z, expected.angular_z, kTolerance);
}

TEST(OmniKinematicsTest, WheelPositionDeltaProducesUnclampedBodyPoseDelta)
{
  constexpr double wheel_radius = 0.05;
  constexpr double base_radius = 0.125;
  const lekiwi_node::OmniKinematics kinematics(wheel_radius, base_radius, 0.1);
  const lekiwi_node::WheelPositionDelta wheel_delta{4.0, -2.0, 7.0};

  const double left = wheel_delta.left * wheel_radius;
  const double back = wheel_delta.back * wheel_radius;
  const double right = wheel_delta.right * wheel_radius;
  const double expected_dyaw = (left + back + right) / (3.0 * base_radius);
  const double expected_dx = (right - left) / std::sqrt(3.0);
  const double expected_dy = -(back - base_radius * expected_dyaw);

  const auto actual = kinematics.toBodyDelta(wheel_delta);

  EXPECT_NEAR(actual.dx, expected_dx, kTolerance);
  EXPECT_NEAR(actual.dy, expected_dy, kTolerance);
  EXPECT_NEAR(actual.dyaw, expected_dyaw, kTolerance);
  EXPECT_GT(actual.dyaw, 0.1);
}

TEST(OmniKinematicsTest, Nav2VelocityEnvelopeDoesNotSaturateFivePointFourRadPerSecondWheels)
{
  const lekiwi_node::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  const double x = -0.18 * std::sqrt(3.0) / 2.0;
  const double y = 0.18 / 2.0;
  const auto wheels = kinematics.toWheelSpeeds(x, y, 0.8);
  const auto restored = kinematics.toBodyTwist(wheels);
  EXPECT_NEAR(restored.linear_x, x, 1e-9);
  EXPECT_NEAR(restored.linear_y, y, 1e-9);
  EXPECT_NEAR(restored.angular_z, 0.8, 1e-9);
  EXPECT_LE(
    std::max({std::abs(wheels.left), std::abs(wheels.back), std::abs(wheels.right)}), 5.4);
}

TEST(OmniKinematicsTest, WaffleSingleAxisLimitsDoNotScale)
{
  const lekiwi_node::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  for (const auto & input : std::array<lekiwi_node::BodyTwist, 3>{
      lekiwi_node::BodyTwist{0.26, 0.0, 0.0},
      lekiwi_node::BodyTwist{0.0, 0.15, 0.0},
      lekiwi_node::BodyTwist{0.0, 0.0, 1.82}}) {
    const auto actual = kinematics.scaleToWheelEnvelope(input);
    EXPECT_NEAR(actual.linear_x, input.linear_x, 1e-12);
    EXPECT_NEAR(actual.linear_y, input.linear_y, 1e-12);
    EXPECT_NEAR(actual.angular_z, input.angular_z, 1e-12);
  }
}

TEST(OmniKinematicsTest, CombinedWaffleCommandUsesOneWheelEnvelopeScale)
{
  const lekiwi_node::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  const lekiwi_node::BodyTwist input{0.26, 0.0, 1.82};
  const auto effective = kinematics.scaleToWheelEnvelope(input);
  const auto wheels = kinematics.toWheelSpeeds(
    input.linear_x, input.linear_y, input.angular_z);
  const double scale = effective.linear_x / input.linear_x;
  EXPECT_LT(scale, 1.0);
  EXPECT_NEAR(effective.angular_z, input.angular_z * scale, 1e-12);
  EXPECT_NEAR(
    std::max({std::abs(wheels.left), std::abs(wheels.back), std::abs(wheels.right)}),
    5.4, 1e-12);
  const auto restored = kinematics.toBodyTwist(wheels);
  EXPECT_NEAR(restored.linear_x, effective.linear_x, 1e-12);
  EXPECT_NEAR(restored.angular_z, effective.angular_z, 1e-12);
}

TEST(OmniKinematicsTest, CombinedWaffleSignCombinationsStayBounded)
{
  const lekiwi_node::OmniKinematics kinematics(0.0508, 0.115, 5.4);
  for (const double x_sign : {-1.0, 1.0}) {
    for (const double y_sign : {-1.0, 1.0}) {
      for (const double yaw_sign : {-1.0, 1.0}) {
        const auto wheels = kinematics.toWheelSpeeds(
          x_sign * 0.26, y_sign * 0.15, yaw_sign * 1.82);
        EXPECT_LE(
          std::max({std::abs(wheels.left), std::abs(wheels.back), std::abs(wheels.right)}),
          5.4);
      }
    }
  }
}

TEST(OmniKinematicsFrameTest, NinetyDegreeBaseRotationPreservesMotorWheelOrdering)
{
  const lekiwi_node::OmniKinematics kinematics(0.05, 0.125, 20.0, 0.0, 0.0, std::acos(-1.0) / 2.0);
  const auto command = kinematics.toWheelSpeeds(0.1, 0.0, 0.0);
  EXPECT_NEAR(command.left, -1.0, kTolerance);
  EXPECT_NEAR(command.back, 2.0, kTolerance);
  EXPECT_NEAR(command.right, -1.0, kTolerance);
  const auto body = kinematics.toBodyTwist(command);
  EXPECT_NEAR(body.linear_x, 0.1, kTolerance);
  EXPECT_NEAR(body.linear_y, 0.0, kTolerance);
  EXPECT_NEAR(body.angular_z, 0.0, kTolerance);
}

TEST(OmniKinematicsFrameTest, ClassroomForwardUsesNewFrontDirection)
{
  const lekiwi_node::OmniKinematics kinematics(
    0.0508, 0.115, 5.4, 0.007587843843, -0.010776931094, 2.094379865161);
  const auto wheels = kinematics.toWheelSpeeds(0.1, 0.0, 0.0);
  // The left-labelled wheel is now at the rear of the arm-forward base frame.
  EXPECT_NEAR(wheels.left, 0.0, 4.0e-5);
  EXPECT_GT(wheels.back, 0.0);
  EXPECT_LT(wheels.right, 0.0);
}

TEST(OmniKinematicsFrameTest, DefaultForwardLeavesObservedRearMotorEightStationary)
{
  // 2026-09-21: turning only the physical rear wheel changed encoder ID 8.
  const lekiwi_node::BaseControllerParameters params;
  const lekiwi_node::OmniKinematics kinematics(
    params.wheel_radius, params.base_radius, params.max_wheel_speed,
    params.kinematics_frame_x, params.kinematics_frame_y, params.kinematics_frame_yaw);
  const auto rear = std::find(params.wheel_ids.begin(), params.wheel_ids.end(), 8);
  ASSERT_NE(rear, params.wheel_ids.end());
  const auto index = static_cast<std::size_t>(rear - params.wheel_ids.begin());
  for (const double vx : {-0.03, 0.03}) {
    const auto command = kinematics.toWheelSpeeds(vx, 0.0, 0.0);
    const std::array<double, 3> speeds{command.left, command.back, command.right};
    EXPECT_NEAR(speeds.at(index), 0.0, 1.0e-5);
  }
}

TEST(OmniKinematicsFrameTest, PhysicalWheelIdsMatchCommandsAndEncoderTranslation)
{
  const lekiwi_node::BaseControllerParameters params;
  const lekiwi_node::OmniKinematics kinematics(
    params.wheel_radius, params.base_radius, params.max_wheel_speed,
    params.kinematics_frame_x, params.kinematics_frame_y, params.kinematics_frame_yaw);
  auto physical_speeds = [&params](const lekiwi_node::WheelCommand & command) {
      const std::array<double, 3> slots{command.left, command.back, command.right};
      std::array<double, 3> by_id{};  // Physical left 7, rear 8, right 9.
      for (std::size_t i = 0; i < slots.size(); ++i) {
        by_id.at(params.wheel_ids.at(i) - 7) = slots[i];
      }
      return by_id;
    };
  const auto forward = physical_speeds(kinematics.toWheelSpeeds(0.03, 0.0, 0.0));
  EXPECT_LT(forward[0], -0.1);
  EXPECT_NEAR(forward[1], 0.0, 1.0e-5);
  EXPECT_GT(forward[2], 0.1);
  const auto lateral = physical_speeds(kinematics.toWheelSpeeds(0.0, 0.03, 0.0));
  EXPECT_GT(lateral[0], 0.1);
  EXPECT_LT(lateral[1], -0.1);
  EXPECT_GT(lateral[2], 0.1);
  const auto yaw = physical_speeds(kinematics.toWheelSpeeds(0.0, 0.0, 0.1));
  for (const double speed : yaw) {EXPECT_GT(speed, 0.1);}

  // Independent encoder input: left rolls forward, rear stays still, right
  // rolls forward. This must advance body X even if the command path is unused.
  const std::array<double, 3> physical_angles{-1.0, 0.0, 1.0};
  const auto delta = kinematics.toBodyDelta({
      physical_angles.at(params.wheel_ids[0] - 7),
      physical_angles.at(params.wheel_ids[1] - 7),
      physical_angles.at(params.wheel_ids[2] - 7)});
  EXPECT_GT(delta.dx, 0.05);
  EXPECT_NEAR(delta.dy, 0.0, 1.0e-6);
  EXPECT_NEAR(delta.dyaw, 0.0, 1.0e-9);
}

TEST(OmniKinematicsFrameTest, OffsetOriginRoundTripsTranslationAndYawWithinWheelEnvelope)
{
  const lekiwi_node::OmniKinematics kinematics(
    0.0508, 0.115, 5.4, 0.007587843843, -0.010776931094, 2.094379865161);
  for (const double x : {-0.26, 0.0, 0.26}) {
    for (const double y : {-0.15, 0.0, 0.15}) {
      for (const double yaw : {-1.82, 0.0, 1.82}) {
        const auto expected = kinematics.scaleToWheelEnvelope({x, y, yaw});
        const auto wheels = kinematics.toWheelSpeeds(x, y, yaw);
        const auto actual = kinematics.toBodyTwist(wheels);
        EXPECT_NEAR(actual.linear_x, expected.linear_x, kTolerance);
        EXPECT_NEAR(actual.linear_y, expected.linear_y, kTolerance);
        EXPECT_NEAR(actual.angular_z, expected.angular_z, kTolerance);
        EXPECT_LE(std::max({std::abs(wheels.left), std::abs(wheels.back), std::abs(wheels.right)}), 5.4);
      }
    }
  }
}

TEST(OmniKinematicsFrameTest, PureMotorOriginRotationAccountsForBaseOriginLeverArm)
{
  const lekiwi_node::OmniKinematics kinematics(0.05, 0.125, 20.0, 0.1, -0.2, 0.7);
  // Equal wheel speeds give a unit yaw rate at the original motor origin.
  const auto body = kinematics.toBodyTwist({2.5, 2.5, 2.5});
  EXPECT_NEAR(body.linear_x, -0.2, kTolerance);
  EXPECT_NEAR(body.linear_y, -0.1, kTolerance);
  EXPECT_NEAR(body.angular_z, 1.0, kTolerance);
}

TEST(OmniKinematicsFrameTest, FiniteEncoderRotationMovesOffsetOriginAlongExactArc)
{
  const double half_pi = std::acos(-1.0) / 2.0;
  const lekiwi_node::OmniKinematics kinematics(0.05, 0.125, 0.1, 1.0, 0.0, 0.7);
  const double wheel_angle = half_pi * 0.125 / 0.05;
  const auto delta = kinematics.toBodyDelta({wheel_angle, wheel_angle, wheel_angle});
  EXPECT_NEAR(delta.dx, 1.0, kTolerance);
  EXPECT_NEAR(delta.dy, -1.0, kTolerance);
  EXPECT_NEAR(delta.dyaw, half_pi, kTolerance);
}
