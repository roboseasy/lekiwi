#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "lekiwi_node/base_controller.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace
{

struct RecordingMotorState
{
  bool connected{false};
  std::vector<lekiwi_node::WheelCommand> writes;
};

class SafetyOrderingMotorInterface final : public lekiwi_node::BaseMotorInterface
{
public:
  SafetyOrderingMotorInterface()
  : lekiwi_node::BaseMotorInterface(3.0)
  {
  }

  void connect() override {}
  bool isConnected() const override { return true; }
  bool setMotorPower(bool enabled) override
  {
    events.push_back(enabled ? "power_on" : "power_off");
    setMotorPowerState(enabled);
    return true;
  }

  std::vector<std::string> events;

private:
  void writeClampedWheelSpeeds(const lekiwi_node::WheelCommand & command) override
  {
    events.push_back(
      command.left == 0.0 && command.back == 0.0 && command.right == 0.0 ? "zero" : "motion");
  }
};

class ThrowingSafetyMotorInterface final : public lekiwi_node::BaseMotorInterface
{
public:
  ThrowingSafetyMotorInterface(bool throw_on_zero, bool throw_on_power_off)
  : lekiwi_node::BaseMotorInterface(3.0),
    throw_on_zero_(throw_on_zero),
    throw_on_power_off_(throw_on_power_off)
  {
  }

  void connect() override {}
  bool isConnected() const override { return true; }
  bool setMotorPower(bool enabled) override
  {
    events.push_back(enabled ? "power_on" : "power_off");
    if (!enabled && throw_on_power_off_) {
      throw std::runtime_error("power off failed");
    }
    setMotorPowerState(enabled);
    return true;
  }

  std::vector<std::string> events;

private:
  void writeClampedWheelSpeeds(const lekiwi_node::WheelCommand & command) override
  {
    const bool zero = command.left == 0.0 && command.back == 0.0 && command.right == 0.0;
    events.push_back(zero ? "zero" : "motion");
    if (zero && throw_on_zero_) {
      throw std::runtime_error("zero failed");
    }
  }

  bool throw_on_zero_;
  bool throw_on_power_off_;
};

class RecordingMotorInterface final : public lekiwi_node::BaseMotorInterface
{
public:
  explicit RecordingMotorInterface(
    double max_wheel_speed,
    std::shared_ptr<RecordingMotorState> state = std::make_shared<RecordingMotorState>())
  : lekiwi_node::BaseMotorInterface(max_wheel_speed),
    state_(std::move(state))
  {
  }

  void connect() override { state_->connected = true; }

  const RecordingMotorState & state() const { return *state_; }

private:
  void writeClampedWheelSpeeds(const lekiwi_node::WheelCommand & command) override
  {
    state_->writes.push_back(command);
  }

  std::shared_ptr<RecordingMotorState> state_;
};

class ThrowingMotorInterface final : public lekiwi_node::BaseMotorInterface
{
public:
  ThrowingMotorInterface()
  : lekiwi_node::BaseMotorInterface(3.0)
  {
  }

  void connect() override {}

private:
  void writeClampedWheelSpeeds(const lekiwi_node::WheelCommand &) override
  {
    throw std::runtime_error("write failed");
  }
};

class ShutdownRecordingMotorInterface final : public lekiwi_node::BaseMotorInterface
{
public:
  ShutdownRecordingMotorInterface()
  : lekiwi_node::BaseMotorInterface(3.0)
  {
  }

  void connect() override {}

  void shutdown() noexcept override { shutdown_called = true; }

  bool shutdown_called{false};

private:
  void writeClampedWheelSpeeds(const lekiwi_node::WheelCommand &) override
  {
    throw std::runtime_error("stop should not be called directly");
  }
};

class FailingMotorPowerInterface final : public lekiwi_node::BaseMotorInterface
{
public:
  FailingMotorPowerInterface()
  : lekiwi_node::BaseMotorInterface(3.0)
  {
  }

  void connect() override {}
  bool setMotorPower(bool) override { return false; }

private:
  void writeClampedWheelSpeeds(const lekiwi_node::WheelCommand &) override {}
};

}  // namespace

TEST(BaseMotorInterfaceContractTest, RejectsNonPositiveMaxWheelSpeed)
{
  EXPECT_THROW(RecordingMotorInterface(0.0), std::invalid_argument);
  EXPECT_THROW(RecordingMotorInterface(-1.0), std::invalid_argument);
}

TEST(BaseMotorInterfaceContractTest, RejectsNonFiniteMaxWheelSpeed)
{
  EXPECT_THROW(
    RecordingMotorInterface(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(
    RecordingMotorInterface(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(BaseMotorInterfaceContractTest, ConnectMustBeExplicit)
{
  RecordingMotorInterface backend(3.0);

  EXPECT_FALSE(backend.state().connected);
  EXPECT_FALSE(backend.isConnected());

  backend.connect();

  EXPECT_TRUE(backend.state().connected);
  EXPECT_FALSE(backend.isConnected());
}

TEST(BaseMotorInterfaceContractTest, DefaultFeedbackReadReturnsNoSample)
{
  RecordingMotorInterface backend(3.0);

  EXPECT_FALSE(backend.readWheelFeedback().has_value());
}

TEST(BaseMotorInterfaceContractTest, WriteWheelSpeedsClampsEveryWheel)
{
  RecordingMotorInterface backend(3.0);

  backend.writeWheelSpeeds({9.0, -4.0, 2.0});

  ASSERT_EQ(backend.state().writes.size(), 1u);
  EXPECT_DOUBLE_EQ(backend.state().writes.back().left, 3.0);
  EXPECT_DOUBLE_EQ(backend.state().writes.back().back, -3.0);
  EXPECT_DOUBLE_EQ(backend.state().writes.back().right, 2.0);
}

TEST(BaseMotorInterfaceContractTest, StopWritesZeroForEveryWheel)
{
  RecordingMotorInterface backend(3.0);

  backend.writeWheelSpeeds({1.0, 2.0, 3.0});
  backend.stop();

  ASSERT_EQ(backend.state().writes.size(), 2u);
  EXPECT_DOUBLE_EQ(backend.state().writes.back().left, 0.0);
  EXPECT_DOUBLE_EQ(backend.state().writes.back().back, 0.0);
  EXPECT_DOUBLE_EQ(backend.state().writes.back().right, 0.0);
}

TEST(BaseMotorInterfaceContractTest, MotorPowerTracksStateAndStopsWhenDisabled)
{
  RecordingMotorInterface backend(3.0);

  EXPECT_TRUE(backend.isMotorPowerEnabled());

  backend.setMotorPower(false);

  EXPECT_FALSE(backend.isMotorPowerEnabled());
  ASSERT_EQ(backend.state().writes.size(), 1u);
  EXPECT_DOUBLE_EQ(backend.state().writes.back().left, 0.0);
  EXPECT_DOUBLE_EQ(backend.state().writes.back().back, 0.0);
  EXPECT_DOUBLE_EQ(backend.state().writes.back().right, 0.0);

  backend.setMotorPower(true);

  EXPECT_TRUE(backend.isMotorPowerEnabled());
  EXPECT_EQ(backend.state().writes.size(), 1u);
}

TEST(BaseControllerMotorInterfaceTest, MotorPowerServiceHelperReportsSuccess)
{
  RecordingMotorInterface backend(3.0);

  const auto response = lekiwi_node::setMotorPowerForService(backend, false);

  EXPECT_TRUE(response.success);
  EXPECT_FALSE(response.message.empty());
  EXPECT_FALSE(backend.isMotorPowerEnabled());
}

TEST(BaseControllerMotorInterfaceTest, MotorPowerServiceHelperReportsFailure)
{
  FailingMotorPowerInterface backend;

  const auto response = lekiwi_node::setMotorPowerForService(backend, true);

  EXPECT_FALSE(response.success);
  EXPECT_NE(response.message.find("failed"), std::string::npos);
  EXPECT_EQ(response.message.find("enabled"), std::string::npos);
}

TEST(BaseControllerMotorInterfaceTest, ShutdownHelperStopsMotorInterfaceBeforeTeardown)
{
  const auto state = std::make_shared<RecordingMotorState>();
  RecordingMotorInterface backend(3.0, state);

  lekiwi_node::stopMotorInterfaceForShutdown(&backend);

  ASSERT_FALSE(state->writes.empty());
  EXPECT_DOUBLE_EQ(state->writes.back().left, 0.0);
  EXPECT_DOUBLE_EQ(state->writes.back().back, 0.0);
  EXPECT_DOUBLE_EQ(state->writes.back().right, 0.0);
}

TEST(BaseControllerMotorInterfaceTest, ShutdownHelperUsesShutdownApi)
{
  ShutdownRecordingMotorInterface backend;

  lekiwi_node::stopMotorInterfaceForShutdown(&backend);

  EXPECT_TRUE(backend.shutdown_called);
}

TEST(BaseControllerMotorInterfaceTest, ShutdownHelperDoesNotThrowWhenStopFails)
{
  ThrowingMotorInterface backend;

  EXPECT_NO_THROW(lekiwi_node::stopMotorInterfaceForShutdown(&backend));
}

TEST(BaseControllerMotorSafetyTest, StaleFeedbackWritesZeroBeforePowerOffAndLatchesFault)
{
  SafetyOrderingMotorInterface backend;
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));
  latch.onFreshEncoderSample();

  EXPECT_TRUE(lekiwi_node::stopMotorOnEncoderTimeout(
      backend,
      latch,
      std::nullopt,
      rclcpp::Time(9, 0),
      rclcpp::Time(10, 0),
      0.3));

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.faultLatched());
  EXPECT_FALSE(backend.isMotorPowerEnabled());
}

TEST(BaseControllerMotorSafetyTest, ExpiredWaitWritesZeroBeforePowerOffAndLatchesFault)
{
  SafetyOrderingMotorInterface backend;
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));

  EXPECT_TRUE(lekiwi_node::stopMotorOnEncoderTimeout(
      backend,
      latch,
      rclcpp::Time(9, 0),
      std::nullopt,
      rclcpp::Time(10, 0),
      0.3));

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.faultLatched());
  EXPECT_FALSE(backend.isMotorPowerEnabled());
}

TEST(BaseControllerMotorSafetyTest, FreshFeedbackDoesNotStopOrPowerOff)
{
  SafetyOrderingMotorInterface backend;
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));
  latch.onFreshEncoderSample();

  EXPECT_FALSE(lekiwi_node::stopMotorOnEncoderTimeout(
      backend,
      latch,
      std::nullopt,
      rclcpp::Time(9, 900000000),
      rclcpp::Time(10, 0),
      0.3));

  EXPECT_TRUE(backend.events.empty());
  EXPECT_FALSE(latch.faultLatched());
}

TEST(BaseControllerMotorSafetyTest, ZeroExceptionStillAttemptsPowerOffAndLatchesFault)
{
  ThrowingSafetyMotorInterface backend(true, false);
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));

  EXPECT_NO_THROW({
    EXPECT_TRUE(lekiwi_node::stopMotorOnEncoderTimeout(
        backend,
        latch,
        rclcpp::Time(9, 0),
        std::nullopt,
        rclcpp::Time(10, 0),
        0.3));
  });

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.faultLatched());
}

TEST(BaseControllerMotorSafetyTest, PowerOffExceptionStillLatchesFault)
{
  ThrowingSafetyMotorInterface backend(false, true);
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));

  EXPECT_NO_THROW({
    EXPECT_TRUE(lekiwi_node::stopMotorOnEncoderTimeout(
        backend,
        latch,
        rclcpp::Time(9, 0),
        std::nullopt,
        rclcpp::Time(10, 0),
        0.3));
  });

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.faultLatched());
}

TEST(BaseControllerMotorSafetyTest, ZeroAndPowerOffExceptionsStillLatchFault)
{
  ThrowingSafetyMotorInterface backend(true, true);
  lekiwi_node::MotorReadinessLatch latch;
  ASSERT_TRUE(latch.onPowerEnabled(true));

  EXPECT_NO_THROW({
    EXPECT_TRUE(lekiwi_node::stopMotorOnEncoderTimeout(
        backend,
        latch,
        rclcpp::Time(9, 0),
        std::nullopt,
        rclcpp::Time(10, 0),
        0.3));
  });

  EXPECT_EQ(backend.events, (std::vector<std::string>{"zero", "power_off"}));
  EXPECT_TRUE(latch.faultLatched());
}
