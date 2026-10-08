#include <gtest/gtest.h>

#include "lekiwi_node/base_controller.hpp"
#include "lekiwi_node/motor_readiness.hpp"

TEST(MotorReadinessLatchTest, DisarmedPermitsOnlyZeroCommand)
{
  lekiwi_node::MotorReadinessLatch latch;

  EXPECT_FALSE(latch.permits(lekiwi_node::WheelCommand{1.0, 0.0, 0.0}));
  EXPECT_TRUE(latch.permits(lekiwi_node::WheelCommand{}));
}

TEST(MotorReadinessLatchTest, FreshEncoderSampleMakesConnectedPoweredHardwareReady)
{
  lekiwi_node::MotorReadinessLatch latch;

  EXPECT_TRUE(latch.onPowerEnabled(true));
  EXPECT_FALSE(latch.ready(true, true, true));
  latch.onFreshEncoderSample();

  EXPECT_TRUE(latch.ready(true, true, true));
  EXPECT_TRUE(latch.permits(lekiwi_node::WheelCommand{1.0, 0.0, 0.0}));
}

TEST(MotorReadinessLatchTest, EncoderTimeoutLatchesFaultAndBlocksMotion)
{
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));
  latch.onFreshEncoderSample();

  latch.onEncoderTimeout();

  EXPECT_TRUE(latch.faultLatched());
  EXPECT_FALSE(latch.permits(lekiwi_node::WheelCommand{1.0, 0.0, 0.0}));
  EXPECT_TRUE(latch.permits(lekiwi_node::WheelCommand{}));
}

TEST(MotorReadinessLatchTest, PowerCycleThroughDisarmedClearsFaultButRequiresFreshSample)
{
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));
  latch.onFreshEncoderSample();
  latch.onEncoderTimeout();
  ASSERT_TRUE(latch.faultLatched());

  latch.onPowerDisabled();
  EXPECT_TRUE(latch.onPowerEnabled(true));
  EXPECT_FALSE(latch.ready(true, true, true));
  latch.onFreshEncoderSample();

  EXPECT_TRUE(latch.ready(true, true, true));
}

TEST(MotorReadinessLatchTest, FaultCannotBeRearmedWithoutPowerDisable)
{
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));
  latch.onBackendFault();

  EXPECT_FALSE(latch.onPowerEnabled(true));
  EXPECT_TRUE(latch.faultLatched());
}

TEST(MotorReadinessRecoveryTest, LatchedFaultRequiresExplicitRecoveryDisarmGate)
{
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));
  latch.onFreshEncoderSample();
  latch.onEncoderTimeout();

  EXPECT_FALSE(lekiwi_node::motorPowerEnableAllowed(latch, false));
  latch.onPowerDisabled();
  EXPECT_TRUE(lekiwi_node::motorPowerEnableAllowed(latch, true));
  ASSERT_TRUE(latch.onPowerEnabled(true));
  EXPECT_FALSE(latch.ready(true, true, true));
  latch.onFreshEncoderSample();
  EXPECT_TRUE(latch.ready(true, true, true));
}

TEST(MotorReadinessLatchTest, DisconnectedOrNonHardwareBackendNeverReportsReady)
{
  lekiwi_node::MotorReadinessLatch latch;

  EXPECT_FALSE(latch.onPowerEnabled(false));
  EXPECT_FALSE(latch.ready(true, false, true));

  latch.onPowerDisabled();
  ASSERT_TRUE(latch.onPowerEnabled(true));
  latch.onFreshEncoderSample();
  EXPECT_FALSE(latch.ready(false, true, true));
  EXPECT_FALSE(latch.ready(true, true, false));
}
