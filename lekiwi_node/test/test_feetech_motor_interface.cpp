#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lekiwi_node/base_controller.hpp"
#include "lekiwi_node/feetech_motor_interface.hpp"

namespace
{

lekiwi_node::FeetechMotorConfig makeConfig(bool enable_motor_write)
{
  lekiwi_node::FeetechMotorConfig config;
  config.serial_port = "/dev/test-feetech";
  config.wheel_ids = {7, 8, 9};
  config.wheel_directions = {1.0, 1.0, -1.0};
  config.max_wheel_speed = 3.0;
  config.enable_motor_write = enable_motor_write;
  config.baud_rate = 115200;
  config.speed_tick_scale = 10.0;
  config.speed_tick_limit = 25;
  config.encoder_tick_deadband = 0;
  config.encoder_feedback_source = "speed";
  config.encoder_read_period = 0.1;
  config.encoder_sample_timeout = 0.3;
  config.position_ticks_per_revolution = 4096;
  config.encoder_position_tick_deadband = 4;
  config.torque_enable = true;
  config.log_encoder_reads = false;
  return config;
}

uint8_t checksum(const std::vector<uint8_t> & packet_without_checksum)
{
  uint8_t sum = 0;
  for (std::size_t i = 2; i < packet_without_checksum.size(); ++i) {
    sum = static_cast<uint8_t>(sum + packet_without_checksum[i]);
  }
  return static_cast<uint8_t>(~sum);
}

std::vector<uint8_t> makeWriteAckStatusPacket(int id, uint8_t error = 0)
{
  std::vector<uint8_t> packet{
    0xff,
    0xff,
    static_cast<uint8_t>(id),
    2,
    error};
  packet.push_back(checksum(packet));
  return packet;
}

struct FakeSerialState
{
  int factory_count{0};
  int open_count{0};
  std::string opened_port;
  int opened_baud_rate{0};
  bool open_result{true};
  int fail_write_index{-1};
  int fail_read_index{-1};
  std::set<int> throw_write_indices;
  std::set<int> throw_read_indices;
  bool throw_on_close{false};
  int write_exception_count{0};
  int read_exception_count{0};
  int close_exception_count{0};
  int read_count{0};
  int close_count{0};
  int unicast_write_count{0};
  bool unread_write_ack_collision{false};
  std::optional<std::vector<uint8_t>> pending_write_ack;
  std::map<int, std::vector<uint8_t>> write_ack_overrides;
  std::map<int, std::vector<uint8_t>> torque_read_overrides;
  std::map<int, std::deque<int>> voltage_reads;
  std::vector<std::string> io_events;
  std::vector<std::vector<uint8_t>> writes;
  std::deque<std::vector<uint8_t>> reads;
};

class FakeSerialPort final : public lekiwi_node::FeetechSerialPort
{
public:
  explicit FakeSerialPort(FakeSerialState * state)
  : state_(state)
  {
  }

  bool open(const std::string & serial_port, int baud_rate) override
  {
    ++state_->open_count;
    state_->opened_port = serial_port;
    state_->opened_baud_rate = baud_rate;
    return state_->open_result;
  }

  bool write(const std::vector<uint8_t> & bytes) override
  {
    state_->writes.push_back(bytes);
    if (state_->throw_write_indices.count(static_cast<int>(state_->writes.size())) != 0u) {
      ++state_->write_exception_count;
      throw std::runtime_error("injected serial write exception");
    }
    if (state_->fail_write_index == static_cast<int>(state_->writes.size())) {
      return false;
    }

    if (bytes.size() == 8u && bytes[4] == 2u && bytes[5] == 62u && bytes[6] == 1u) {
      auto & readings = state_->voltage_reads[bytes[2]];
      if (!readings.empty()) {
        const int voltage = readings.front();
        readings.pop_front();
        std::vector<uint8_t> reply{0xff, 0xff, bytes[2], 3, 0,
          static_cast<uint8_t>(voltage)};
        reply.push_back(checksum(reply));
        state_->reads.push_front(reply);
      }
    }
    const bool is_torque_read =
      bytes.size() == 8u && bytes[4] == 2u && bytes[5] == 40u && bytes[6] == 1u;
    if (is_torque_read) {
      const auto override = state_->torque_read_overrides.find(bytes[2]);
      if (override != state_->torque_read_overrides.end()) {
        state_->reads.push_front(override->second);
      } else {
        std::vector<uint8_t> reply{0xff, 0xff, bytes[2], 3, 0, 0};
        reply.push_back(checksum(reply));
        state_->reads.push_front(reply);
      }
    }

    const bool is_unicast_write =
      bytes.size() >= 5u && bytes[0] == 0xff && bytes[1] == 0xff &&
      bytes[2] != 0xfe && bytes[4] == 3u;
    if (is_unicast_write) {
      if (state_->pending_write_ack.has_value()) {
        state_->unread_write_ack_collision = true;
        return false;
      }

      ++state_->unicast_write_count;
      state_->io_events.push_back("write:" + std::to_string(bytes[2]));
      const auto override = state_->write_ack_overrides.find(state_->unicast_write_count);
      if (override == state_->write_ack_overrides.end()) {
        state_->pending_write_ack = makeWriteAckStatusPacket(bytes[2]);
      } else if (!override->second.empty()) {
        state_->pending_write_ack = override->second;
      }
    }
    return true;
  }

  bool read(std::vector<uint8_t> * bytes, std::size_t size) override
  {
    ++state_->read_count;
    if (state_->throw_read_indices.count(state_->read_count) != 0u) {
      ++state_->read_exception_count;
      throw std::runtime_error("injected serial read exception");
    }
    if (state_->fail_read_index == state_->read_count) {
      return false;
    }
    if (bytes == nullptr || size == 0) {
      return false;
    }

    if (state_->pending_write_ack.has_value()) {
      auto & pending_ack = *state_->pending_write_ack;
      if (pending_ack.size() >= 3u) {
        state_->io_events.push_back("read:" + std::to_string(pending_ack[2]));
      }
      const std::size_t read_size = std::min(size, pending_ack.size());
      bytes->assign(
        pending_ack.begin(), pending_ack.begin() + static_cast<std::ptrdiff_t>(read_size));
      pending_ack.erase(
        pending_ack.begin(), pending_ack.begin() + static_cast<std::ptrdiff_t>(read_size));
      if (pending_ack.empty()) {
        state_->pending_write_ack.reset();
      }
      return true;
    }

    if (state_->reads.empty()) {
      return false;
    }

    while (!state_->reads.empty() && state_->reads.front().empty()) {
      state_->reads.pop_front();
    }
    if (state_->reads.empty()) {
      return false;
    }

    auto & next_read = state_->reads.front();
    const std::size_t read_size = std::min(size, next_read.size());
    bytes->assign(next_read.begin(), next_read.begin() + static_cast<std::ptrdiff_t>(read_size));
    next_read.erase(next_read.begin(), next_read.begin() + static_cast<std::ptrdiff_t>(read_size));
    if (next_read.empty()) {
      state_->reads.pop_front();
    }
    return true;
  }

  void close() override
  {
    ++state_->close_count;
    if (state_->throw_on_close) {
      ++state_->close_exception_count;
      throw std::runtime_error("injected serial close exception");
    }
  }

private:
  FakeSerialState * state_;
};

lekiwi_node::FeetechSerialPortFactory makeFactory(FakeSerialState * state)
{
  return [state]() {
    ++state->factory_count;
    return std::make_unique<FakeSerialPort>(state);
  };
}

class ManualSteadyClock
{
public:
  std::chrono::steady_clock::time_point now() const { return now_; }

  void advance(const double seconds)
  {
    now_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
  }

private:
  std::chrono::steady_clock::time_point now_{};
};

std::optional<lekiwi_node::WheelCommand> readWheelSpeeds(
  lekiwi_node::FeetechMotorInterface & backend)
{
  const auto feedback = backend.readWheelFeedback();
  if (!feedback) {
    return std::nullopt;
  }
  return feedback->wheel_speeds;
}

const std::vector<uint8_t> kWheel7SpeedModePacket{0xff, 0xff, 7, 4, 3, 33, 1, 207};
const std::vector<uint8_t> kWheel8SpeedModePacket{0xff, 0xff, 8, 4, 3, 33, 1, 206};
const std::vector<uint8_t> kWheel9SpeedModePacket{0xff, 0xff, 9, 4, 3, 33, 1, 205};
const std::vector<uint8_t> kZeroGoalSpeedSyncPacket{
  0xff, 0xff, 0xfe, 13, 0x83, 46, 2,
  7, 0, 0,
  8, 0, 0,
  9, 0, 0,
  0x29};
const std::vector<uint8_t> kWheel7TorqueEnablePacket{0xff, 0xff, 7, 4, 3, 40, 1, 200};
const std::vector<uint8_t> kWheel8TorqueEnablePacket{0xff, 0xff, 8, 4, 3, 40, 1, 199};
const std::vector<uint8_t> kWheel9TorqueEnablePacket{0xff, 0xff, 9, 4, 3, 40, 1, 198};
const std::vector<uint8_t> kWheel7TorqueDisablePacket{0xff, 0xff, 7, 4, 3, 40, 0, 201};
const std::vector<uint8_t> kWheel8TorqueDisablePacket{0xff, 0xff, 8, 4, 3, 40, 0, 200};
const std::vector<uint8_t> kWheel9TorqueDisablePacket{0xff, 0xff, 9, 4, 3, 40, 0, 199};
const std::vector<uint8_t> kWheel7PresentSpeedReadPacket{0xff, 0xff, 7, 4, 2, 58, 2, 182};
const std::vector<uint8_t> kWheel8PresentSpeedReadPacket{0xff, 0xff, 8, 4, 2, 58, 2, 181};
const std::vector<uint8_t> kWheel9PresentSpeedReadPacket{0xff, 0xff, 9, 4, 2, 58, 2, 180};
const std::vector<uint8_t> kWheel7PresentPositionReadPacket{0xff, 0xff, 7, 4, 2, 56, 2, 184};
const std::vector<uint8_t> kWheel8PresentPositionReadPacket{0xff, 0xff, 8, 4, 2, 56, 2, 183};
const std::vector<uint8_t> kWheel9PresentPositionReadPacket{0xff, 0xff, 9, 4, 2, 56, 2, 182};

std::vector<uint8_t> makePresentSpeedStatusPacket(int id, uint16_t encoded_speed)
{
  std::vector<uint8_t> packet{
    0xff,
    0xff,
    static_cast<uint8_t>(id),
    4,
    0,
    static_cast<uint8_t>(encoded_speed & 0xff),
    static_cast<uint8_t>((encoded_speed >> 8) & 0xff)};
  packet.push_back(checksum(packet));
  return packet;
}

std::vector<uint8_t> makeUnsupportedLengthStatusPacket(int id)
{
  std::vector<uint8_t> packet{
    0xff,
    0xff,
    static_cast<uint8_t>(id),
    3,
    0,
    0};
  packet.push_back(checksum(packet));
  return packet;
}

std::vector<uint8_t> makePresentPositionStatusPacket(int id, uint16_t position_ticks)
{
  return makePresentSpeedStatusPacket(id, position_ticks);
}

void queuePositionFrame(
  FakeSerialState * state,
  const std::array<int, 3> & position_ticks)
{
  state->reads.push_back(makePresentPositionStatusPacket(7, position_ticks[0]));
  state->reads.push_back(makePresentPositionStatusPacket(8, position_ticks[1]));
  state->reads.push_back(makePresentPositionStatusPacket(9, position_ticks[2]));
}

lekiwi_node::FeetechMotorConfig makePositionConfig()
{
  auto config = makeConfig(true);
  config.encoder_feedback_source = "position";
  config.encoder_read_period = 0.1;
  config.position_ticks_per_revolution = 4096;
  config.encoder_position_tick_deadband = 0;
  return config;
}

}  // namespace

TEST(FeetechMotorConfigTest, PublicDefaultsMatchConfiguredBaseControllerCeiling)
{
  const lekiwi_node::FeetechMotorConfig config;

  EXPECT_DOUBLE_EQ(config.max_wheel_speed, 2.3009711818284617);
  EXPECT_EQ(config.speed_tick_limit, 1500);
  EXPECT_DOUBLE_EQ(config.encoder_sample_timeout, 0.5);
  EXPECT_EQ(config.encoder_position_tick_deadband, 0);
  EXPECT_FALSE(config.torque_enable);
}

TEST(FeetechMotorConfigTest, RejectsDeprecatedPositionDeadbandForPositionFeedback)
{
  auto config = makePositionConfig();
  config.encoder_position_tick_deadband = 1;

  EXPECT_THROW((void)lekiwi_node::FeetechMotorInterface(config), std::invalid_argument);
}

TEST(FeetechMotorConfigTest, AcceptsDeprecatedPositionDeadbandForSpeedFeedback)
{
  auto config = makeConfig(false);
  ASSERT_EQ(config.encoder_feedback_source, "speed");
  ASSERT_EQ(config.encoder_position_tick_deadband, 4);

  EXPECT_NO_THROW((void)lekiwi_node::FeetechMotorInterface(config));
}

TEST(FeetechMotorConfigTest, RejectsPositionFeedbackWindowShorterThanThreeReads)
{
  auto config = makePositionConfig();
  config.encoder_read_period = 0.1001;
  config.encoder_sample_timeout = 0.3;

  EXPECT_THROW((void)lekiwi_node::FeetechMotorInterface(config), std::invalid_argument);
}

TEST(FeetechMotorConfigTest, AcceptsPositionFeedbackWindowAtThreeReadBoundary)
{
  auto config = makePositionConfig();
  config.encoder_read_period = 0.1;
  config.encoder_sample_timeout = 0.3;

  EXPECT_NO_THROW((void)lekiwi_node::FeetechMotorInterface(config));
}

TEST(FeetechMotorConfigTest, DoesNotApplyPositionWindowTimingToSpeedFeedback)
{
  auto config = makeConfig(false);
  config.encoder_feedback_source = "speed";
  config.encoder_read_period = 0.1001;
  config.encoder_sample_timeout = 0.3;

  EXPECT_NO_THROW((void)lekiwi_node::FeetechMotorInterface(config));
}

TEST(FeetechMotorConfigTest, RejectsInvalidEncoderSampleTimeout)
{
  auto config = makeConfig(false);
  config.encoder_sample_timeout = -0.1;
  EXPECT_THROW((void)lekiwi_node::FeetechMotorInterface(config), std::invalid_argument);

  config.encoder_sample_timeout = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)lekiwi_node::FeetechMotorInterface(config), std::invalid_argument);
}

TEST(FeetechBackendSelectionTest, SelectsFeetechOnlyWhenBackendAndMotorWriteAreEnabled)
{
  lekiwi_node::BaseControllerParameters params;

  params.motor_backend = "mock";
  params.enable_motor_write = false;
  EXPECT_EQ(
    lekiwi_node::BaseMotorBackendKind::kMockLogging,
    lekiwi_node::selectBaseMotorBackend(params));

  params.motor_backend = "feetech";
  params.enable_motor_write = false;
  EXPECT_EQ(
    lekiwi_node::BaseMotorBackendKind::kMockLogging,
    lekiwi_node::selectBaseMotorBackend(params));

  params.motor_backend = "unknown";
  params.enable_motor_write = true;
  EXPECT_EQ(
    lekiwi_node::BaseMotorBackendKind::kMockLogging,
    lekiwi_node::selectBaseMotorBackend(params));

  params.motor_backend = "feetech";
  params.enable_motor_write = true;
  EXPECT_EQ(
    lekiwi_node::BaseMotorBackendKind::kFeetechHardware,
    lekiwi_node::selectBaseMotorBackend(params));
}

TEST(FeetechMotorInterfaceTest, DisabledMotorWriteDoesNotOpenSerialPort)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(false), makeFactory(&state));

  backend.connect();

  EXPECT_EQ(state.factory_count, 0);
  EXPECT_EQ(state.open_count, 0);
  EXPECT_FALSE(backend.hasSerialPortProbeSucceeded());
  EXPECT_FALSE(backend.isConnected());
}

TEST(FeetechMotorInterfaceTest, ConnectConsumesEachWriteAckBeforeNextUnicastWrite)
{
  FakeSerialState state;
  auto config = makeConfig(true);
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

  backend.connect();

  ASSERT_TRUE(backend.isConnected());
  EXPECT_FALSE(state.unread_write_ack_collision);
  EXPECT_EQ(
    state.io_events,
    (std::vector<std::string>{
      "write:7", "read:7", "write:8", "read:8", "write:9", "read:9",
      "write:7", "read:7", "write:8", "read:8", "write:9", "read:9"}));
  EXPECT_EQ(state.read_count, 9);
}

TEST(FeetechMotorInterfaceTest, ConnectRejectsInvalidWriteAck)
{
  auto bad_header = makeWriteAckStatusPacket(7);
  bad_header[0] = 0;

  auto bad_declared_length = makeWriteAckStatusPacket(7);
  bad_declared_length[3] = 3;
  bad_declared_length.back() = checksum(
    std::vector<uint8_t>(bad_declared_length.begin(), bad_declared_length.end() - 1));

  auto bad_checksum = makeWriteAckStatusPacket(7);
  bad_checksum.back() ^= 0x01;

  const std::vector<std::pair<std::string, std::vector<uint8_t>>> cases{
    {"missing ACK", {}},
    {"partial ACK", {0xff, 0xff, 7}},
    {"bad header", bad_header},
    {"bad declared length", bad_declared_length},
    {"wrong ID", makeWriteAckStatusPacket(8)},
    {"non-zero error", makeWriteAckStatusPacket(7, 1)},
    {"bad checksum", bad_checksum},
  };

  for (const auto & [name, response] : cases) {
    SCOPED_TRACE(name);
    FakeSerialState state;
    state.write_ack_overrides[1] = response;
    auto config = makeConfig(true);
    config.torque_enable = false;
    lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

    backend.connect();

    EXPECT_FALSE(backend.isConnected());
    EXPECT_EQ(state.close_count, 1);
    EXPECT_FALSE(state.unread_write_ack_collision);
    EXPECT_EQ(
      std::find(state.writes.begin(), state.writes.end(), kWheel7SpeedModePacket),
      state.writes.end());
    EXPECT_EQ(
      std::find(state.writes.begin(), state.writes.end(), kWheel8SpeedModePacket),
      state.writes.end());
    EXPECT_EQ(
      std::find(state.writes.begin(), state.writes.end(), kWheel9SpeedModePacket),
      state.writes.end());
    EXPECT_EQ(
      std::find(state.writes.begin(), state.writes.end(), kWheel7TorqueEnablePacket),
      state.writes.end());
  }
}

TEST(FeetechMotorInterfaceTest, SerialWriteFailureDoesNotReadAckOrContinueSpeedModeWrites)
{
  FakeSerialState state;
  state.fail_write_index = 7;
  auto config = makeConfig(true);
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

  backend.connect();

  EXPECT_FALSE(backend.isConnected());
  EXPECT_EQ(state.close_count, 1);
  EXPECT_EQ(state.read_count, 12);
  EXPECT_EQ(
    std::find(state.writes.begin(), state.writes.end(), kWheel8SpeedModePacket),
    state.writes.end());
  EXPECT_EQ(
    std::find(state.writes.begin(), state.writes.end(), kWheel9SpeedModePacket),
    state.writes.end());
}

TEST(FeetechMotorInterfaceTest, EnableConsumesEachWriteAckAndBroadcastZeroConsumesNone)
{
  FakeSerialState state;
  auto config = makeConfig(true);
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));
  backend.connect();
  ASSERT_TRUE(backend.isConnected());
  state.writes.clear();
  state.read_count = 0;
  state.unicast_write_count = 0;
  state.io_events.clear();

  ASSERT_TRUE(backend.setMotorPower(true));

  EXPECT_TRUE(backend.isMotorPowerEnabled());
  EXPECT_FALSE(state.unread_write_ack_collision);
  EXPECT_EQ(state.read_count, 3);
  EXPECT_EQ(
    state.io_events,
    (std::vector<std::string>{
      "write:7", "read:7", "write:8", "read:8", "write:9", "read:9"}));
  ASSERT_EQ(state.writes.size(), 4u);
  EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
}

TEST(FeetechMotorInterfaceTest, TorqueEnabledConnectDisarmsBeforeModeZeroAndEnableWrites)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();

  EXPECT_EQ(state.factory_count, 1);
  EXPECT_EQ(state.open_count, 1);
  EXPECT_EQ(state.opened_port, "/dev/test-feetech");
  EXPECT_EQ(state.opened_baud_rate, 115200);
  EXPECT_TRUE(backend.hasSerialPortProbeSucceeded());
  EXPECT_TRUE(backend.isConnected());
  EXPECT_EQ(state.close_count, 0);
  ASSERT_EQ(state.writes.size(), 13u);
  EXPECT_EQ(state.writes[0], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[1], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[2], kWheel9TorqueDisablePacket);
  EXPECT_EQ(state.writes[6], kWheel7SpeedModePacket);
  EXPECT_EQ(state.writes[7], kWheel8SpeedModePacket);
  EXPECT_EQ(state.writes[8], kWheel9SpeedModePacket);
  EXPECT_EQ(state.writes[9], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[10], kWheel7TorqueEnablePacket);
  EXPECT_EQ(state.writes[11], kWheel8TorqueEnablePacket);
  EXPECT_EQ(state.writes[12], kWheel9TorqueEnablePacket);
}

TEST(FeetechMotorInterfaceTest, TorqueEnabledInitialDisarmFailureClosesWithoutLaterInitWrites)
{
  FakeSerialState state;
  state.fail_write_index = 2;
  auto config = makeConfig(true);
  ASSERT_TRUE(config.torque_enable);
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

  backend.connect();

  EXPECT_TRUE(backend.hasSerialPortProbeSucceeded());
  EXPECT_FALSE(backend.isConnected());
  EXPECT_TRUE(backend.isMotorPowerEnabled());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 10u);
  EXPECT_EQ(state.writes[0], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[1], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[2], kWheel9TorqueDisablePacket);
  EXPECT_EQ(state.writes[3], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[4], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[5], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[6], kWheel9TorqueDisablePacket);
  EXPECT_EQ(
    std::find(state.writes.begin(), state.writes.end(), kWheel7SpeedModePacket),
    state.writes.end());
  EXPECT_EQ(
    std::find(state.writes.begin(), state.writes.end(), kWheel7TorqueEnablePacket),
    state.writes.end());
}

TEST(FeetechMotorInterfaceTest, NonZeroWheelCommandWritesSyncGoalSpeedPacket)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  state.writes.clear();

  backend.writeWheelSpeeds(lekiwi_node::WheelCommand{1.2, -0.7, 20.0});

  ASSERT_EQ(state.writes.size(), 1u);
  EXPECT_EQ(
    state.writes[0],
    (std::vector<uint8_t>{
      0xff, 0xff, 0xfe, 13, 0x83, 46, 2,
      7, 12, 0,
      8, 7, 0x80,
      9, 25, 0x80,
      0xfd}));
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsWritesPresentSpeedReadsAndDecodesResponses)
{
  FakeSerialState state;
  state.reads.push_back(makePresentSpeedStatusPacket(7, 12));
  state.reads.push_back(makePresentSpeedStatusPacket(8, 0x8007));
  state.reads.push_back(makePresentSpeedStatusPacket(9, 0x8005));
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  state.writes.clear();

  const auto speeds = readWheelSpeeds(backend);

  ASSERT_TRUE(speeds.has_value());
  EXPECT_DOUBLE_EQ(speeds->left, 1.2);
  EXPECT_DOUBLE_EQ(speeds->back, -0.7);
  EXPECT_DOUBLE_EQ(speeds->right, 0.5);
  ASSERT_EQ(state.writes.size(), 3u);
  EXPECT_EQ(state.writes[0], kWheel7PresentSpeedReadPacket);
  EXPECT_EQ(state.writes[1], kWheel8PresentSpeedReadPacket);
  EXPECT_EQ(state.writes[2], kWheel9PresentSpeedReadPacket);
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsStoresRawPresentSpeedDiagnosticSamplesWhenEnabled)
{
  FakeSerialState state;
  state.reads.push_back(makePresentSpeedStatusPacket(7, 0x0032));
  state.reads.push_back(makePresentSpeedStatusPacket(8, 0x8032));
  state.reads.push_back(makePresentSpeedStatusPacket(9, 0));
  auto config = makeConfig(true);
  config.log_encoder_reads = true;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

  backend.connect();

  const auto speeds = readWheelSpeeds(backend);

  ASSERT_TRUE(speeds.has_value());
  const auto samples = backend.lastPresentSpeedSamples();
  ASSERT_EQ(samples.size(), 3u);
  EXPECT_EQ(samples[0].wheel_id, 7);
  EXPECT_EQ(samples[0].raw_low_byte, 0x32);
  EXPECT_EQ(samples[0].raw_high_byte, 0x00);
  EXPECT_EQ(samples[0].raw_signed_ticks, 50);
  EXPECT_EQ(samples[0].filtered_signed_ticks, 50);
  EXPECT_DOUBLE_EQ(samples[0].wheel_speed, 5.0);
  EXPECT_EQ(samples[1].wheel_id, 8);
  EXPECT_EQ(samples[1].raw_low_byte, 0x32);
  EXPECT_EQ(samples[1].raw_high_byte, 0x80);
  EXPECT_EQ(samples[1].raw_signed_ticks, -50);
  EXPECT_EQ(samples[1].filtered_signed_ticks, -50);
  EXPECT_DOUBLE_EQ(samples[1].wheel_speed, -5.0);
  EXPECT_EQ(samples[2].wheel_id, 9);
  EXPECT_EQ(samples[2].raw_signed_ticks, 0);
  EXPECT_EQ(samples[2].filtered_signed_ticks, 0);
  EXPECT_DOUBLE_EQ(samples[2].wheel_speed, -0.0);
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsAppliesEncoderTickDeadbandBeforeScaling)
{
  FakeSerialState state;
  state.reads.push_back(makePresentSpeedStatusPacket(7, 0x0032));
  state.reads.push_back(makePresentSpeedStatusPacket(8, 0x8032));
  state.reads.push_back(makePresentSpeedStatusPacket(9, 0x0033));
  auto config = makeConfig(true);
  config.encoder_tick_deadband = 50;
  config.log_encoder_reads = true;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

  backend.connect();

  const auto feedback = backend.readWheelFeedback();

  ASSERT_TRUE(feedback.has_value());
  EXPECT_FALSE(feedback->wheel_position_delta.has_value());
  EXPECT_DOUBLE_EQ(feedback->wheel_speeds.left, 0.0);
  EXPECT_DOUBLE_EQ(feedback->wheel_speeds.back, 0.0);
  EXPECT_DOUBLE_EQ(feedback->wheel_speeds.right, -5.1);
  const auto samples = backend.lastPresentSpeedSamples();
  ASSERT_EQ(samples.size(), 3u);
  EXPECT_EQ(samples[0].raw_signed_ticks, 50);
  EXPECT_EQ(samples[0].filtered_signed_ticks, 0);
  EXPECT_EQ(samples[1].raw_signed_ticks, -50);
  EXPECT_EQ(samples[1].filtered_signed_ticks, 0);
  EXPECT_EQ(samples[2].raw_signed_ticks, 51);
  EXPECT_EQ(samples[2].filtered_signed_ticks, 51);
}

TEST(FeetechMotorInterfaceTest, FirstPositionFrameSeedsBaselineAndReturnsNoSample)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  queuePositionFrame(&state, {1000, 2000, 3000});
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();

  EXPECT_FALSE(backend.readWheelFeedback().has_value());
}

TEST(FeetechMotorInterfaceTest, PositionFeedbackSkipsConfiguredWriteAcksBeforeExpectedStatus)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  state.reads.push_back(makeWriteAckStatusPacket(7));
  state.reads.push_back(makeWriteAckStatusPacket(8));
  state.reads.push_back(makeWriteAckStatusPacket(9));
  queuePositionFrame(&state, {1000, 2000, 3000});
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  state.writes.clear();

  EXPECT_FALSE(backend.readWheelFeedback().has_value());

  ASSERT_EQ(state.writes.size(), 3u);
  EXPECT_EQ(state.writes[0], kWheel7PresentPositionReadPacket);
  EXPECT_EQ(state.writes[1], kWheel8PresentPositionReadPacket);
  EXPECT_EQ(state.writes[2], kWheel9PresentPositionReadPacket);
}

TEST(FeetechMotorInterfaceTest, PositionFeedbackRejectsNonZeroErrorWriteAck)
{
  FakeSerialState state;
  state.reads.push_back(makeWriteAckStatusPacket(7, 1));
  queuePositionFrame(&state, {1000, 2000, 3000});
  lekiwi_node::FeetechMotorInterface backend(makePositionConfig(), makeFactory(&state));
  backend.connect();
  state.read_count = 0;
  state.writes.clear();

  EXPECT_FALSE(backend.readWheelFeedback().has_value());
  EXPECT_EQ(state.read_count, 6);
  ASSERT_EQ(state.writes.size(), 1u);
  EXPECT_EQ(state.writes[0], kWheel7PresentPositionReadPacket);
}

TEST(FeetechMotorInterfaceTest, SecondPositionFrameReturnsWindowSpeedAndCurrentRawDelta)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  queuePositionFrame(&state, {1000, 2000, 3000});
  queuePositionFrame(&state, {1010, 1990, 3010});
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  clock.advance(0.1);

  const auto sample = backend.readWheelFeedback();

  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->wheel_position_delta.has_value());
  const double angle_per_tick = 2.0 * M_PI / 4096.0;
  EXPECT_NEAR(sample->wheel_speeds.left, 10.0 * angle_per_tick / 0.3, 1e-12);
  EXPECT_NEAR(sample->wheel_speeds.back, -10.0 * angle_per_tick / 0.3, 1e-12);
  EXPECT_NEAR(sample->wheel_speeds.right, -10.0 * angle_per_tick / 0.3, 1e-12);
  EXPECT_NEAR(sample->wheel_position_delta->left, 10.0 * angle_per_tick, 1e-12);
  EXPECT_NEAR(sample->wheel_position_delta->back, -10.0 * angle_per_tick, 1e-12);
  EXPECT_NEAR(sample->wheel_position_delta->right, -10.0 * angle_per_tick, 1e-12);
}

TEST(FeetechMotorInterfaceTest, StationaryEncoderDitherStaysSlowAndDoesNotAccumulateOneSidedDelta)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  const std::array<int, 6> positions{1000, 1005, 1000, 994, 999, 1000};
  for (const int position : positions) {
    queuePositionFrame(&state, {position, position, position});
  }
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  double accumulated_left_delta = 0.0;

  for (std::size_t i = 1; i < positions.size(); ++i) {
    clock.advance(0.1);
    const auto sample = backend.readWheelFeedback();
    ASSERT_TRUE(sample.has_value());
    ASSERT_TRUE(sample->wheel_position_delta.has_value());
    EXPECT_LE(std::abs(sample->wheel_speeds.left), 0.05);
    EXPECT_LE(std::abs(sample->wheel_speeds.back), 0.05);
    EXPECT_LE(std::abs(sample->wheel_speeds.right), 0.05);
    accumulated_left_delta += sample->wheel_position_delta->left;
  }
  EXPECT_NEAR(accumulated_left_delta, 0.0, 1e-12);
}

TEST(FeetechMotorInterfaceTest, MonotonicSingleTicksAreAllPreservedInPositionDeltas)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  for (int position = 1000; position <= 1010; ++position) {
    queuePositionFrame(&state, {position, position, position});
  }
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  const double angle_per_tick = 2.0 * M_PI / 4096.0;
  double total_delta = 0.0;

  for (int i = 0; i < 10; ++i) {
    clock.advance(0.1);
    const auto sample = backend.readWheelFeedback();
    ASSERT_TRUE(sample.has_value());
    ASSERT_TRUE(sample->wheel_position_delta.has_value());
    EXPECT_NEAR(sample->wheel_position_delta->left, angle_per_tick, 1e-12);
    total_delta += sample->wheel_position_delta->left;
  }
  EXPECT_NEAR(total_delta, 10.0 * angle_per_tick, 1e-12);
}

TEST(FeetechMotorInterfaceTest, AlternatingCadencePreservesSixtySecondDeltaAndActualWindowTime)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  constexpr int kIntervals = 546;
  for (int position = 1000; position <= 1000 + kIntervals; ++position) {
    queuePositionFrame(&state, {position, position, position});
  }
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  const double angle_per_tick = 2.0 * M_PI / 4096.0;
  double total_delta = 0.0;
  std::array<double, 3> last_intervals{};
  lekiwi_node::WheelFeedbackSample last_sample;

  for (int i = 0; i < kIntervals; ++i) {
    const double elapsed = i % 2 == 0 ? 0.10 : 0.12;
    last_intervals[static_cast<std::size_t>(i) % last_intervals.size()] = elapsed;
    clock.advance(elapsed);
    const auto sample = backend.readWheelFeedback();
    ASSERT_TRUE(sample.has_value());
    ASSERT_TRUE(sample->wheel_position_delta.has_value());
    total_delta += sample->wheel_position_delta->left;
    last_sample = *sample;
  }

  const double window_time = last_intervals[0] + last_intervals[1] + last_intervals[2];
  EXPECT_NEAR(total_delta, kIntervals * angle_per_tick, 1e-10);
  EXPECT_NEAR(last_sample.wheel_speeds.left, 3.0 * angle_per_tick / window_time, 1e-12);
}

TEST(FeetechMotorInterfaceTest, ThreeFixedReadsAfterMotionMakeWindowVelocityExactlyZero)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  for (const int position : {1000, 1010, 1010, 1010, 1010}) {
    queuePositionFrame(&state, {position, position, position});
  }
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  std::optional<lekiwi_node::WheelFeedbackSample> sample;
  for (int i = 0; i < 4; ++i) {
    clock.advance(0.1);
    sample = backend.readWheelFeedback();
    ASSERT_TRUE(sample.has_value());
  }
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.left, 0.0);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.back, 0.0);
  EXPECT_DOUBLE_EQ(sample->wheel_speeds.right, -0.0);
}

TEST(FeetechMotorInterfaceTest, PositionDeltaUsesShortestWrapInBothDirections)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  queuePositionFrame(&state, {4090, 5, 1000});
  queuePositionFrame(&state, {5, 4090, 1000});
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  clock.advance(0.1);
  const auto sample = backend.readWheelFeedback();

  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->wheel_position_delta.has_value());
  const double angle_per_tick = 2.0 * M_PI / 4096.0;
  EXPECT_NEAR(sample->wheel_position_delta->left, 11.0 * angle_per_tick, 1e-12);
  EXPECT_NEAR(sample->wheel_position_delta->back, -11.0 * angle_per_tick, 1e-12);
  EXPECT_DOUBLE_EQ(sample->wheel_position_delta->right, -0.0);
}

TEST(FeetechMotorInterfaceTest, WheelDirectionAndOdomScaleApplyToSpeedAndDelta)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  queuePositionFrame(&state, {1000, 1000, 1000});
  queuePositionFrame(&state, {1010, 1010, 1010});
  auto config = makePositionConfig();
  config.wheel_directions = {2.0, -2.0, 0.5};
  config.encoder_odom_scale = 0.5;
  lekiwi_node::FeetechMotorInterface backend(
    config, makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  clock.advance(0.1);
  const auto sample = backend.readWheelFeedback();

  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->wheel_position_delta.has_value());
  const double angle_per_tick = 2.0 * M_PI / 4096.0;
  const std::array<double, 3> directions{2.0, -2.0, 0.5};
  const std::array<double, 3> speeds{
    sample->wheel_speeds.left, sample->wheel_speeds.back, sample->wheel_speeds.right};
  const std::array<double, 3> deltas{
    sample->wheel_position_delta->left,
    sample->wheel_position_delta->back,
    sample->wheel_position_delta->right};
  for (std::size_t i = 0; i < directions.size(); ++i) {
    const double expected_delta = 10.0 * angle_per_tick / directions[i] * 0.5;
    EXPECT_NEAR(deltas[i], expected_delta, 1e-12);
    EXPECT_NEAR(speeds[i], expected_delta / 0.3, 1e-12);
  }
}

TEST(FeetechMotorInterfaceTest, FailedWheelReadPreservesPositionBaselineAndWindow)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  queuePositionFrame(&state, {1000, 1000, 1000});
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  ASSERT_FALSE(backend.readWheelFeedback().has_value());

  state.reads.push_back(makePresentPositionStatusPacket(7, 1010));
  auto invalid_wheel_8_packet = makePresentPositionStatusPacket(8, 1010);
  invalid_wheel_8_packet.back() ^= 0x01;
  state.reads.push_back(std::move(invalid_wheel_8_packet));
  clock.advance(0.1);
  EXPECT_FALSE(backend.readWheelFeedback().has_value());
  queuePositionFrame(&state, {1020, 1020, 1020});
  clock.advance(0.1);
  const auto sample = backend.readWheelFeedback();

  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->wheel_position_delta.has_value());
  const double angle_per_tick = 2.0 * M_PI / 4096.0;
  EXPECT_NEAR(sample->wheel_position_delta->left, 20.0 * angle_per_tick, 1e-12);
  EXPECT_NEAR(sample->wheel_speeds.left, 20.0 * angle_per_tick / 0.4, 1e-12);
}

TEST(FeetechMotorInterfaceTest, ConnectEnableAndSuccessfulDisableResetPositionEstimator)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  queuePositionFrame(&state, {1000, 1000, 1000});
  queuePositionFrame(&state, {1010, 1010, 1010});
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  clock.advance(0.1);
  ASSERT_TRUE(backend.readWheelFeedback().has_value());

  ASSERT_TRUE(backend.setMotorPower(true));
  queuePositionFrame(&state, {1020, 1020, 1020});
  clock.advance(0.1);
  EXPECT_FALSE(backend.readWheelFeedback().has_value());
  queuePositionFrame(&state, {1030, 1030, 1030});
  clock.advance(0.1);
  ASSERT_TRUE(backend.readWheelFeedback().has_value());

  ASSERT_TRUE(backend.setMotorPower(false));
  queuePositionFrame(&state, {1040, 1040, 1040});
  clock.advance(0.1);
  EXPECT_FALSE(backend.readWheelFeedback().has_value());
  queuePositionFrame(&state, {1050, 1050, 1050});
  clock.advance(0.1);
  ASSERT_TRUE(backend.readWheelFeedback().has_value());

  backend.connect();
  queuePositionFrame(&state, {1060, 1060, 1060});
  clock.advance(0.1);
  EXPECT_FALSE(backend.readWheelFeedback().has_value());
}

TEST(FeetechMotorInterfaceTest, ShutdownAndWriteFaultCleanupResetPositionEstimator)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  queuePositionFrame(&state, {1000, 1000, 1000});
  queuePositionFrame(&state, {1010, 1010, 1010});
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  clock.advance(0.1);
  ASSERT_TRUE(backend.readWheelFeedback().has_value());

  backend.shutdown();
  backend.connect();
  queuePositionFrame(&state, {1020, 1020, 1020});
  clock.advance(0.1);
  EXPECT_FALSE(backend.readWheelFeedback().has_value());
  queuePositionFrame(&state, {1030, 1030, 1030});
  clock.advance(0.1);
  ASSERT_TRUE(backend.readWheelFeedback().has_value());

  state.writes.clear();
  state.fail_write_index = 1;
  clock.advance(0.1);
  EXPECT_FALSE(backend.readWheelFeedback().has_value());
  EXPECT_FALSE(backend.isConnected());
  state.fail_write_index = -1;
  backend.connect();
  queuePositionFrame(&state, {1040, 1040, 1040});
  clock.advance(0.1);
  EXPECT_FALSE(backend.readWheelFeedback().has_value());
}

TEST(FeetechMotorInterfaceTest, FailedDisableResetsPositionEstimatorAfterDisconnect)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  lekiwi_node::FeetechMotorInterface backend(
    makePositionConfig(), makeFactory(&state), [&clock]() { return clock.now(); });
  backend.connect();
  queuePositionFrame(&state, {1000, 1000, 1000});
  queuePositionFrame(&state, {1010, 1010, 1010});
  ASSERT_FALSE(backend.readWheelFeedback().has_value());
  clock.advance(0.1);
  ASSERT_TRUE(backend.readWheelFeedback().has_value());

  state.writes.clear();
  state.fail_write_index = 2;
  EXPECT_FALSE(backend.setMotorPower(false));
  EXPECT_FALSE(backend.isConnected());
  state.fail_write_index = -1;
  backend.connect();
  queuePositionFrame(&state, {1020, 1020, 1020});
  clock.advance(0.1);
  EXPECT_FALSE(backend.readWheelFeedback().has_value());
  queuePositionFrame(&state, {1030, 1030, 1030});
  clock.advance(0.1);
  EXPECT_TRUE(backend.readWheelFeedback().has_value());
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsRecoversValidPacketAfterGarbagePrefix)
{
  FakeSerialState state;
  const auto wheel7_status = makePresentSpeedStatusPacket(7, 12);
  state.reads.push_back({
    0x00,
    0x7e,
    0xff,
    wheel7_status[0],
    wheel7_status[1],
    wheel7_status[2],
    wheel7_status[3],
    wheel7_status[4]});
  state.reads.push_back({
    wheel7_status[5],
    wheel7_status[6],
    wheel7_status[7]});
  state.reads.push_back(makePresentSpeedStatusPacket(8, 0x8007));
  state.reads.push_back(makePresentSpeedStatusPacket(9, 0x8005));
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  state.writes.clear();

  const auto speeds = readWheelSpeeds(backend);

  ASSERT_TRUE(speeds.has_value());
  EXPECT_DOUBLE_EQ(speeds->left, 1.2);
  EXPECT_DOUBLE_EQ(speeds->back, -0.7);
  EXPECT_DOUBLE_EQ(speeds->right, 0.5);
  ASSERT_EQ(state.writes.size(), 3u);
  EXPECT_EQ(state.writes[0], kWheel7PresentSpeedReadPacket);
  EXPECT_EQ(state.writes[1], kWheel8PresentSpeedReadPacket);
  EXPECT_EQ(state.writes[2], kWheel9PresentSpeedReadPacket);
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsRejectsUnsupportedLengthStatusFromConfiguredWheel)
{
  FakeSerialState state;
  state.reads.push_back(makeUnsupportedLengthStatusPacket(7));
  state.reads.push_back(makePresentSpeedStatusPacket(7, 12));
  state.reads.push_back(makePresentSpeedStatusPacket(8, 0x8007));
  state.reads.push_back(makePresentSpeedStatusPacket(9, 0x8005));
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  state.read_count = 0;
  state.writes.clear();

  EXPECT_FALSE(readWheelSpeeds(backend).has_value());
  EXPECT_EQ(state.read_count, 7);
  ASSERT_EQ(state.writes.size(), 1u);
  EXPECT_EQ(state.writes[0], kWheel7PresentSpeedReadPacket);
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsStopsBeforeExpectedPacketAfterWrongIdStatusPacket)
{
  FakeSerialState state;
  state.reads.push_back(makePresentSpeedStatusPacket(8, 99));
  state.reads.push_back(makePresentSpeedStatusPacket(7, 12));
  state.reads.push_back(makePresentSpeedStatusPacket(8, 0x8007));
  state.reads.push_back(makePresentSpeedStatusPacket(9, 0x8005));
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  state.read_count = 0;
  state.writes.clear();

  EXPECT_FALSE(readWheelSpeeds(backend).has_value());
  EXPECT_EQ(state.read_count, 8);
  ASSERT_EQ(state.writes.size(), 1u);
  EXPECT_EQ(state.writes[0], kWheel7PresentSpeedReadPacket);
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsReturnsNulloptWhenNoValidStatusPacketIsFound)
{
  FakeSerialState state;
  state.reads.push_back({0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07});
  state.reads.push_back(makePresentSpeedStatusPacket(8, 12));
  state.reads.push_back(makePresentSpeedStatusPacket(7, 12));
  state.reads.back().back() ^= 0x01;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();

  EXPECT_FALSE(readWheelSpeeds(backend).has_value());
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsStopsPresentSpeedReadAfterPerPacketReadCallBudget)
{
  FakeSerialState state;
  state.reads.push_back(std::vector<uint8_t>(32, 0x00));
  state.reads.push_back(makePresentSpeedStatusPacket(7, 12));
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  state.read_count = 0;
  state.writes.clear();

  EXPECT_FALSE(readWheelSpeeds(backend).has_value());
  EXPECT_EQ(state.read_count, 32);
  ASSERT_EQ(state.writes.size(), 1u);
  EXPECT_EQ(state.writes[0], kWheel7PresentSpeedReadPacket);
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsReturnsNulloptOnChecksumFailureOrIdMismatch)
{
  FakeSerialState checksum_failure_state;
  checksum_failure_state.reads.push_back(makePresentSpeedStatusPacket(7, 12));
  checksum_failure_state.reads.back().back() ^= 0x01;
  lekiwi_node::FeetechMotorInterface checksum_failure_backend(
    makeConfig(true), makeFactory(&checksum_failure_state));

  checksum_failure_backend.connect();

  EXPECT_FALSE(readWheelSpeeds(checksum_failure_backend).has_value());

  FakeSerialState id_mismatch_state;
  id_mismatch_state.reads.push_back(makePresentSpeedStatusPacket(8, 12));
  lekiwi_node::FeetechMotorInterface id_mismatch_backend(makeConfig(true), makeFactory(&id_mismatch_state));

  id_mismatch_backend.connect();

  EXPECT_FALSE(readWheelSpeeds(id_mismatch_backend).has_value());
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsReturnsNulloptOnSerialReadFailure)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  state.writes.clear();

  EXPECT_FALSE(readWheelSpeeds(backend).has_value());
  ASSERT_EQ(state.writes.size(), 1u);
  EXPECT_EQ(state.writes[0], kWheel7PresentSpeedReadPacket);
  EXPECT_EQ(
    std::find(state.writes.begin(), state.writes.end(), kWheel8PresentSpeedReadPacket),
    state.writes.end());
  EXPECT_EQ(
    std::find(state.writes.begin(), state.writes.end(), kWheel9PresentSpeedReadPacket),
    state.writes.end());
}

TEST(FeetechMotorInterfaceTest, ReadRequestWriteFailureDisarmsAndDisconnectsBackend)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  ASSERT_TRUE(backend.isConnected());
  state.writes.clear();
  state.close_count = 0;
  state.fail_write_index = 1;

  EXPECT_FALSE(readWheelSpeeds(backend).has_value());

  EXPECT_FALSE(backend.isConnected());
  EXPECT_FALSE(backend.isMotorPowerEnabled());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 8u);
  EXPECT_EQ(state.writes[0], kWheel7PresentSpeedReadPacket);
  EXPECT_EQ(state.writes[1], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[2], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[3], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[4], kWheel9TorqueDisablePacket);
}

TEST(FeetechMotorInterfaceTest, ReadWheelSpeedsReturnsNulloptWhenDisconnectedOrDisabled)
{
  FakeSerialState disabled_state;
  lekiwi_node::FeetechMotorInterface disabled_backend(makeConfig(false), makeFactory(&disabled_state));

  disabled_backend.connect();

  EXPECT_FALSE(readWheelSpeeds(disabled_backend).has_value());
  EXPECT_TRUE(disabled_state.writes.empty());

  FakeSerialState disconnected_state;
  disconnected_state.open_result = false;
  lekiwi_node::FeetechMotorInterface disconnected_backend(
    makeConfig(true), makeFactory(&disconnected_state));

  disconnected_backend.connect();

  EXPECT_FALSE(readWheelSpeeds(disconnected_backend).has_value());
  EXPECT_FALSE(disconnected_backend.setMotorPower(true));
}

TEST(FeetechMotorInterfaceTest, ConnectAckReadExceptionClosesAndDoesNotVerifyDisarm)
{
  FakeSerialState state;
  state.throw_read_indices.insert(1);
  auto config = makeConfig(true);
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

  EXPECT_NO_THROW(backend.connect());

  EXPECT_TRUE(backend.hasSerialPortProbeSucceeded());
  EXPECT_FALSE(backend.isConnected());
  EXPECT_EQ(state.close_count, 1);
  EXPECT_FALSE(backend.setMotorPower(false));
  backend.shutdown();
  EXPECT_FALSE(backend.setMotorPower(false));
}

TEST(FeetechMotorInterfaceTest, ConnectInitializationFailuresRequireRestartAfterShutdown)
{
  struct ConnectFaultCase
  {
    const char * name;
    int fail_write_index;
    bool torque_enable;
  };
  const std::array<ConnectFaultCase, 3> cases{{
      {"speed mode", 7, false},
      {"initialization zero", 10, false},
      {"torque enable", 11, true}}};

  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.name);
    FakeSerialState state;
    state.fail_write_index = test_case.fail_write_index;
    auto config = makeConfig(true);
    config.torque_enable = test_case.torque_enable;
    lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

    backend.connect();

    EXPECT_FALSE(backend.isConnected());
    EXPECT_FALSE(backend.setMotorPower(false));
    backend.shutdown();
    EXPECT_FALSE(backend.setMotorPower(false));
  }
}

TEST(FeetechMotorInterfaceTest, TorqueEnableWriteAndReadExceptionsAttemptCompleteCleanup)
{
  {
    FakeSerialState state;
    auto config = makeConfig(true);
    config.torque_enable = false;
    lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));
    backend.connect();
    ASSERT_TRUE(backend.isConnected());
    state.writes.clear();
    state.read_count = 0;
    state.unicast_write_count = 0;
    state.close_count = 0;
    state.throw_write_indices.insert(2);

    EXPECT_FALSE(backend.setMotorPower(true));

    EXPECT_FALSE(backend.isConnected());
    EXPECT_FALSE(backend.isMotorPowerEnabled());
    EXPECT_EQ(state.close_count, 1);
    ASSERT_EQ(state.writes.size(), 9u);
    EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
    EXPECT_EQ(state.writes[1], kWheel7TorqueEnablePacket);
    EXPECT_EQ(state.writes[2], kZeroGoalSpeedSyncPacket);
    EXPECT_EQ(state.writes[3], kWheel7TorqueDisablePacket);
    EXPECT_EQ(state.writes[4], kWheel8TorqueDisablePacket);
    EXPECT_EQ(state.writes[5], kWheel9TorqueDisablePacket);
  }

  {
    FakeSerialState state;
    auto config = makeConfig(true);
    config.torque_enable = false;
    lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));
    backend.connect();
    ASSERT_TRUE(backend.isConnected());
    state.writes.clear();
    state.read_count = 0;
    state.unicast_write_count = 0;
    state.close_count = 0;
    state.throw_read_indices.insert(1);

    EXPECT_FALSE(backend.setMotorPower(true));

    EXPECT_FALSE(backend.isConnected());
    EXPECT_TRUE(backend.isMotorPowerEnabled());
    EXPECT_EQ(state.close_count, 1);
    ASSERT_EQ(state.writes.size(), 6u);
    EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
    EXPECT_EQ(state.writes[1], kWheel7TorqueEnablePacket);
    EXPECT_EQ(state.writes[2], kZeroGoalSpeedSyncPacket);
    EXPECT_EQ(state.writes[3], kWheel7TorqueDisablePacket);
    EXPECT_EQ(state.writes[4], kWheel8TorqueDisablePacket);
    EXPECT_EQ(state.writes[5], kWheel9TorqueDisablePacket);
  }
}

TEST(FeetechMotorInterfaceTest, FailedExplicitDisableRemainsUnverifiedAfterCleanup)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));
  backend.connect();
  ASSERT_TRUE(backend.isConnected());
  ASSERT_TRUE(backend.isMotorPowerEnabled());
  state.writes.clear();
  state.read_count = 0;
  state.unicast_write_count = 0;
  state.close_count = 0;
  state.write_ack_overrides[2] = {};

  EXPECT_FALSE(backend.setMotorPower(false));

  EXPECT_FALSE(backend.isConnected());
  EXPECT_TRUE(backend.isMotorPowerEnabled());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 11u);
  EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[1], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[2], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[3], kWheel9TorqueDisablePacket);
  EXPECT_EQ(state.writes[4], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[5], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[6], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[7], kWheel9TorqueDisablePacket);
  EXPECT_FALSE(backend.setMotorPower(false));
  backend.shutdown();
  EXPECT_FALSE(backend.setMotorPower(false));
}

TEST(FeetechMotorInterfaceTest, ShutdownContainsEveryCleanupExceptionAndAttemptsLaterStages)
{
  for (const int write_index : {1, 2, 3, 4}) {
    SCOPED_TRACE("write exception index " + std::to_string(write_index));
    FakeSerialState state;
    auto config = makeConfig(true);
    config.torque_enable = false;
    lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));
    backend.connect();
    ASSERT_TRUE(backend.isConnected());
    state.writes.clear();
    state.read_count = 0;
    state.unicast_write_count = 0;
    state.close_count = 0;
    state.throw_write_indices.insert(write_index);

    EXPECT_NO_THROW(backend.shutdown());

    EXPECT_EQ(state.write_exception_count, 1);
    EXPECT_EQ(state.close_count, 1);
    ASSERT_EQ(state.writes.size(), write_index == 1 ? 7u : 4u);
    EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
    EXPECT_EQ(state.writes[1], kWheel7TorqueDisablePacket);
    EXPECT_EQ(state.writes[2], kWheel8TorqueDisablePacket);
    EXPECT_EQ(state.writes[3], kWheel9TorqueDisablePacket);
  }

  for (const int read_index : {1, 2, 3}) {
    SCOPED_TRACE("read exception index " + std::to_string(read_index));
    FakeSerialState state;
    auto config = makeConfig(true);
    config.torque_enable = false;
    lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));
    backend.connect();
    ASSERT_TRUE(backend.isConnected());
    state.writes.clear();
    state.read_count = 0;
    state.unicast_write_count = 0;
    state.close_count = 0;
    state.throw_read_indices.insert(read_index);

    EXPECT_NO_THROW(backend.shutdown());

    EXPECT_EQ(state.read_exception_count, 1);
    EXPECT_EQ(state.read_count, read_index);
    EXPECT_EQ(state.close_count, 1);
    ASSERT_EQ(state.writes.size(), 4u);
    EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
    EXPECT_EQ(state.writes[1], kWheel7TorqueDisablePacket);
    EXPECT_EQ(state.writes[2], kWheel8TorqueDisablePacket);
    EXPECT_EQ(state.writes[3], kWheel9TorqueDisablePacket);
  }

  FakeSerialState close_state;
  auto close_config = makeConfig(true);
  close_config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface close_backend(close_config, makeFactory(&close_state));
  close_backend.connect();
  ASSERT_TRUE(close_backend.isConnected());
  close_state.writes.clear();
  close_state.read_count = 0;
  close_state.unicast_write_count = 0;
  close_state.close_count = 0;
  close_state.throw_on_close = true;

  EXPECT_NO_THROW(close_backend.shutdown());

  EXPECT_EQ(close_state.close_exception_count, 1);
  EXPECT_EQ(close_state.close_count, 1);
  ASSERT_EQ(close_state.writes.size(), 7u);
}

TEST(FeetechMotorInterfaceTest, VerifiedDisableAllowsDisconnectedOffFastPath)
{
  FakeSerialState initially_disabled_state;
  auto initially_disabled_config = makeConfig(true);
  initially_disabled_config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface initially_disabled_backend(
    initially_disabled_config, makeFactory(&initially_disabled_state));

  initially_disabled_backend.connect();

  ASSERT_TRUE(initially_disabled_backend.isConnected());
  EXPECT_FALSE(initially_disabled_backend.isMotorPowerEnabled());

  FakeSerialState explicit_disable_state;
  lekiwi_node::FeetechMotorInterface explicit_disable_backend(
    makeConfig(true), makeFactory(&explicit_disable_state));
  explicit_disable_backend.connect();
  ASSERT_TRUE(explicit_disable_backend.isMotorPowerEnabled());

  ASSERT_TRUE(explicit_disable_backend.setMotorPower(false));
  EXPECT_FALSE(explicit_disable_backend.isMotorPowerEnabled());
  explicit_disable_backend.shutdown();

  EXPECT_FALSE(explicit_disable_backend.isConnected());
  EXPECT_TRUE(explicit_disable_backend.setMotorPower(false));
}

TEST(FeetechMotorInterfaceTest, ShutdownWritesZeroTorqueDisableAndClosesSerialPort)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  ASSERT_TRUE(backend.isConnected());
  ASSERT_EQ(state.writes.size(), 13u);

  backend.shutdown();

  EXPECT_FALSE(backend.isConnected());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 20u);
  EXPECT_EQ(state.writes[13], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[14], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[15], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[16], kWheel9TorqueDisablePacket);
}

TEST(FeetechMotorInterfaceTest, DisableRejectsInvalidOrPoweredTorqueReadbackDespiteValidAcks)
{
  const std::vector<std::vector<uint8_t>> invalid_statuses{
    {}, {0xff, 0xff, 8, 3, 0, 0},
    {0xfe, 0xff, 8, 3, 0, 0, 244},
    {0xff, 0xff, 7, 3, 0, 0, 245},
    {0xff, 0xff, 8, 2, 0, 0, 245},
    {0xff, 0xff, 8, 3, 1, 0, 243},
    {0xff, 0xff, 8, 3, 0, 0, 243},
    {0xff, 0xff, 8, 3, 0, 1, 243}};
  for (const auto & status : invalid_statuses) {
    SCOPED_TRACE(::testing::PrintToString(status));
    FakeSerialState state;
    lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));
    backend.connect();
    ASSERT_TRUE(backend.isConnected());
    ASSERT_TRUE(backend.isMotorPowerEnabled());
    state.writes.clear();
    state.torque_read_overrides[8] = status;

    EXPECT_FALSE(backend.setMotorPower(false));
    EXPECT_FALSE(backend.isConnected());
    EXPECT_TRUE(backend.isMotorPowerEnabled());
    EXPECT_FALSE(backend.setMotorPower(false));
    // All disable commands precede readback, and a bad ID 8 response must not
    // prevent either the ID 9 check or cleanup's later disable attempts.
    ASSERT_GE(state.writes.size(), 7u);
    EXPECT_EQ(state.writes[1], kWheel7TorqueDisablePacket);
    EXPECT_EQ(state.writes[2], kWheel8TorqueDisablePacket);
    EXPECT_EQ(state.writes[3], kWheel9TorqueDisablePacket);
    for (std::size_t i = 4; i < 7; ++i) {
      ASSERT_EQ(state.writes[i].size(), 8u);
      EXPECT_EQ(state.writes[i][2], i + 3);
      EXPECT_EQ(state.writes[i][4], 2u);
      EXPECT_EQ(state.writes[i][5], 40u);
      EXPECT_EQ(state.writes[i][6], 1u);
    }
  }
}

TEST(FeetechMotorInterfaceTest, ConnectNeverEnablesTorqueWhenDisableReadbackIsPowered)
{
  FakeSerialState state;
  state.torque_read_overrides[8] = {0xff, 0xff, 8, 3, 0, 1, 243};
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));
  backend.connect();
  EXPECT_FALSE(backend.isConnected());
  EXPECT_FALSE(backend.setMotorPower(false));
  EXPECT_TRUE(backend.isMotorPowerEnabled());
  for (const auto & packet : state.writes) {
    EXPECT_NE(packet, kWheel7TorqueEnablePacket);
    EXPECT_NE(packet, kWheel8TorqueEnablePacket);
    EXPECT_NE(packet, kWheel9TorqueEnablePacket);
  }
}

TEST(FeetechMotorInterfaceTest, TorqueReadbackIoFailuresRemainUnverifiedAndContinueLaterWheels)
{
  for (const bool shutdown : {false, true}) {
    for (const std::string fault : {"write failure", "write exception", "read failure", "read exception"}) {
      SCOPED_TRACE(fault + (shutdown ? " during shutdown" : " during explicit disable"));
      FakeSerialState state;
      lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));
      backend.connect();
      ASSERT_TRUE(backend.isConnected());
      state.writes.clear();
      state.read_count = 0;
      // Zero, three disable writes/ACKs, then the first torque read request.
      if (fault == "write failure") {state.fail_write_index = 5;}
      if (fault == "write exception") {state.throw_write_indices.insert(5);}
      if (fault == "read failure") {state.fail_read_index = 4;}
      if (fault == "read exception") {state.throw_read_indices.insert(4);}

      if (shutdown) {
        EXPECT_NO_THROW(backend.shutdown());
      } else {
        EXPECT_FALSE(backend.setMotorPower(false));
      }
      EXPECT_FALSE(backend.isConnected());
      EXPECT_FALSE(backend.setMotorPower(false));
      EXPECT_TRUE(backend.isMotorPowerEnabled());
      EXPECT_EQ(state.close_count, 1);
      ASSERT_GE(state.writes.size(), 7u);
      for (std::size_t i = 4; i < 7; ++i) {
        ASSERT_EQ(state.writes[i].size(), 8u);
        EXPECT_EQ(state.writes[i][2], i + 3);
        EXPECT_EQ(state.writes[i][4], 2u);
        EXPECT_EQ(state.writes[i][5], 40u);
      }
    }
  }
}

TEST(FeetechMotorInterfaceTest, ShutdownDoesNotClaimDisarmedWhenReadbackRemainsPowered)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));
  backend.connect();
  ASSERT_TRUE(backend.isConnected());
  state.torque_read_overrides[8] = {0xff, 0xff, 8, 3, 0, 1, 243};
  EXPECT_NO_THROW(backend.shutdown());
  EXPECT_FALSE(backend.isConnected());
  EXPECT_TRUE(backend.isMotorPowerEnabled());
  EXPECT_FALSE(backend.setMotorPower(false));
  EXPECT_EQ(state.close_count, 1);
}

TEST(FeetechMotorInterfaceTest, MotorPowerWritesTorquePacketsWhenConnected)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  ASSERT_TRUE(backend.isConnected());
  state.writes.clear();

  EXPECT_TRUE(backend.setMotorPower(false));

  EXPECT_FALSE(backend.isMotorPowerEnabled());
  ASSERT_EQ(state.writes.size(), 7u);
  EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[1], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[2], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[3], kWheel9TorqueDisablePacket);

  state.writes.clear();

  EXPECT_TRUE(backend.setMotorPower(true));

  EXPECT_TRUE(backend.isMotorPowerEnabled());
  ASSERT_EQ(state.writes.size(), 4u);
  EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[1], kWheel7TorqueEnablePacket);
  EXPECT_EQ(state.writes[2], kWheel8TorqueEnablePacket);
  EXPECT_EQ(state.writes[3], kWheel9TorqueEnablePacket);
}

TEST(FeetechMotorInterfaceTest, TorqueDisabledConnectDisarmsBeforeModeAndZeroWrites)
{
  FakeSerialState state;
  auto config = makeConfig(true);
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

  backend.connect();

  EXPECT_TRUE(backend.isConnected());
  EXPECT_FALSE(backend.isMotorPowerEnabled());
  ASSERT_EQ(state.writes.size(), 10u);
  EXPECT_EQ(state.writes[0], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[1], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[2], kWheel9TorqueDisablePacket);
  EXPECT_EQ(state.writes[6], kWheel7SpeedModePacket);
  EXPECT_EQ(state.writes[7], kWheel8SpeedModePacket);
  EXPECT_EQ(state.writes[8], kWheel9SpeedModePacket);
  EXPECT_EQ(state.writes[9], kZeroGoalSpeedSyncPacket);

  backend.writeWheelSpeeds(lekiwi_node::WheelCommand{1.0, 0.5, -0.25});

  EXPECT_EQ(state.writes.size(), 10u);
}

TEST(FeetechMotorInterfaceTest, TorqueDisabledConnectFailureCleansUpAndStaysDisconnected)
{
  FakeSerialState state;
  state.fail_write_index = 2;
  auto config = makeConfig(true);
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

  backend.connect();

  EXPECT_TRUE(backend.hasSerialPortProbeSucceeded());
  EXPECT_FALSE(backend.isConnected());
  EXPECT_TRUE(backend.isMotorPowerEnabled());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 10u);
  EXPECT_EQ(state.writes[0], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[1], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[2], kWheel9TorqueDisablePacket);
  EXPECT_EQ(state.writes[3], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[4], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[5], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[6], kWheel9TorqueDisablePacket);
  EXPECT_EQ(
    std::find(state.writes.begin(), state.writes.end(), kWheel7SpeedModePacket),
    state.writes.end());
}

TEST(FeetechMotorInterfaceTest, EnablingMotorPowerWritesZeroGoalSpeedBeforeTorqueEnable)
{
  FakeSerialState state;
  auto config = makeConfig(true);
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));

  backend.connect();
  state.writes.clear();

  EXPECT_TRUE(backend.setMotorPower(true));

  EXPECT_TRUE(backend.isMotorPowerEnabled());
  ASSERT_EQ(state.writes.size(), 4u);
  EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[1], kWheel7TorqueEnablePacket);
  EXPECT_EQ(state.writes[2], kWheel8TorqueEnablePacket);
  EXPECT_EQ(state.writes[3], kWheel9TorqueEnablePacket);
}

TEST(FeetechMotorInterfaceTest, EnableFailureStopsDisablesAndDisconnectsPoweredBackend)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));
  backend.connect();
  ASSERT_TRUE(backend.isConnected());
  ASSERT_TRUE(backend.isMotorPowerEnabled());
  backend.writeWheelSpeeds(lekiwi_node::WheelCommand{1.0, 0.0, 0.0});
  state.writes.clear();
  state.fail_write_index = 1;

  EXPECT_FALSE(backend.setMotorPower(true));

  EXPECT_FALSE(backend.isConnected());
  EXPECT_FALSE(backend.isMotorPowerEnabled());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 8u);
  EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[1], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[2], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[3], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[4], kWheel9TorqueDisablePacket);
}

TEST(FeetechMotorInterfaceTest, TorqueEnableFailureStopsDisablesAndDisconnectsBackend)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));
  backend.connect();
  ASSERT_TRUE(backend.setMotorPower(false));
  ASSERT_FALSE(backend.isMotorPowerEnabled());
  state.writes.clear();
  state.fail_write_index = 2;

  EXPECT_FALSE(backend.setMotorPower(true));

  EXPECT_FALSE(backend.isConnected());
  EXPECT_FALSE(backend.isMotorPowerEnabled());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 9u);
  EXPECT_EQ(state.writes[0], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[1], kWheel7TorqueEnablePacket);
  EXPECT_EQ(state.writes[2], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[3], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[4], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[5], kWheel9TorqueDisablePacket);
}

TEST(FeetechMotorInterfaceTest, DisabledMotorWriteDoesNotWriteWheelCommands)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(false), makeFactory(&state));

  backend.connect();
  backend.writeWheelSpeeds(lekiwi_node::WheelCommand{0.2, 0.0, 0.0});

  EXPECT_TRUE(state.writes.empty());
}

TEST(FeetechMotorInterfaceTest, FailedSerialOpenDoesNotWriteWheelCommands)
{
  FakeSerialState state;
  state.open_result = false;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  backend.writeWheelSpeeds(lekiwi_node::WheelCommand{0.2, 0.0, 0.0});

  EXPECT_EQ(state.factory_count, 1);
  EXPECT_EQ(state.open_count, 1);
  EXPECT_FALSE(backend.hasSerialPortProbeSucceeded());
  EXPECT_FALSE(backend.isConnected());
  EXPECT_TRUE(state.writes.empty());
}

TEST(FeetechMotorInterfaceTest, InitializationWriteFailureLeavesBackendDisconnected)
{
  FakeSerialState state;
  state.fail_write_index = 8;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  backend.writeWheelSpeeds(lekiwi_node::WheelCommand{0.2, 0.0, 0.0});

  EXPECT_TRUE(backend.hasSerialPortProbeSucceeded());
  EXPECT_FALSE(backend.isConnected());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 15u);
  EXPECT_EQ(state.writes[0], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[1], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[2], kWheel9TorqueDisablePacket);
  EXPECT_EQ(state.writes[6], kWheel7SpeedModePacket);
  EXPECT_EQ(state.writes[7], kWheel8SpeedModePacket);
  EXPECT_EQ(state.writes[8], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[9], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[10], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[11], kWheel9TorqueDisablePacket);
}

TEST(FeetechMotorInterfaceTest, TorqueEnableFailureDisablesWheelsBeforeClosing)
{
  FakeSerialState state;
  state.fail_write_index = 11;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  backend.writeWheelSpeeds(lekiwi_node::WheelCommand{0.2, 0.0, 0.0});

  EXPECT_TRUE(backend.hasSerialPortProbeSucceeded());
  EXPECT_FALSE(backend.isConnected());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 18u);
  EXPECT_EQ(state.writes[0], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[1], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[2], kWheel9TorqueDisablePacket);
  EXPECT_EQ(state.writes[6], kWheel7SpeedModePacket);
  EXPECT_EQ(state.writes[7], kWheel8SpeedModePacket);
  EXPECT_EQ(state.writes[8], kWheel9SpeedModePacket);
  EXPECT_EQ(state.writes[9], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[10], kWheel7TorqueEnablePacket);
  EXPECT_EQ(state.writes[11], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[12], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[13], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[14], kWheel9TorqueDisablePacket);
}

TEST(FeetechMotorInterfaceTest, WheelCommandWriteFailureAttemptsZeroAndTorqueOffCleanup)
{
  FakeSerialState state;
  lekiwi_node::FeetechMotorInterface backend(makeConfig(true), makeFactory(&state));

  backend.connect();
  state.writes.clear();
  state.close_count = 0;
  state.fail_write_index = 1;

  backend.writeWheelSpeeds(lekiwi_node::WheelCommand{0.2, 0.0, 0.0});
  backend.writeWheelSpeeds(lekiwi_node::WheelCommand{0.3, 0.0, 0.0});

  EXPECT_FALSE(backend.isConnected());
  EXPECT_EQ(state.close_count, 1);
  ASSERT_EQ(state.writes.size(), 8u);
  EXPECT_EQ(state.writes[1], kZeroGoalSpeedSyncPacket);
  EXPECT_EQ(state.writes[2], kWheel7TorqueDisablePacket);
  EXPECT_EQ(state.writes[3], kWheel8TorqueDisablePacket);
  EXPECT_EQ(state.writes[4], kWheel9TorqueDisablePacket);
}

TEST(FeetechMotorInterfaceTest, RejectsInvalidConfigurationBeforeOpeningSerialPort)
{
  FakeSerialState state;
  auto config = makeConfig(true);
  config.wheel_ids = {7, 7, 9};
  EXPECT_THROW(
    lekiwi_node::FeetechMotorInterface(config, makeFactory(&state)),
    std::invalid_argument);

  config = makeConfig(true);
  config.wheel_ids = {7, 0, 9};
  EXPECT_THROW(
    lekiwi_node::FeetechMotorInterface(config, makeFactory(&state)),
    std::invalid_argument);

  config = makeConfig(true);
  config.speed_tick_limit = 32768;
  EXPECT_THROW(
    lekiwi_node::FeetechMotorInterface(config, makeFactory(&state)),
    std::invalid_argument);

  config = makeConfig(true);
  config.encoder_tick_deadband = 32768;
  EXPECT_THROW(
    lekiwi_node::FeetechMotorInterface(config, makeFactory(&state)),
    std::invalid_argument);

  config = makeConfig(true);
  config.speed_tick_scale = 20000.0;
  EXPECT_THROW(
    lekiwi_node::FeetechMotorInterface(config, makeFactory(&state)),
    std::invalid_argument);

  config = makeConfig(true);
  config.wheel_directions = {1.0, 0.0, -1.0};
  EXPECT_THROW(
    lekiwi_node::FeetechMotorInterface(config, makeFactory(&state)),
    std::invalid_argument);

  for (const double invalid_scale : {
      0.0,
      -1.0,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    config = makeConfig(true);
    config.encoder_odom_scale = invalid_scale;
    EXPECT_THROW(
      lekiwi_node::FeetechMotorInterface(config, makeFactory(&state)),
      std::invalid_argument);
  }

  EXPECT_EQ(state.factory_count, 0);
  EXPECT_EQ(state.open_count, 0);
}

TEST(FeetechMotorHealth, RejectsOutOfRangeVoltageStartWithoutTorqueEnable)
{
  for (const int voltage : {114, 127}) {
    FakeSerialState state;
    auto config = makeConfig(true);
    config.torque_enable = false;
    config.monitor_motor_health = true;
    lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));
    backend.connect();
    state.voltage_reads[7].push_back(voltage);
    EXPECT_FALSE(backend.setMotorPower(true));
    EXPECT_FALSE(backend.isMotorPowerEnabled());
    EXPECT_FALSE(backend.isConnected());
    EXPECT_EQ(std::count(state.writes.begin(), state.writes.end(), kWheel7TorqueEnablePacket), 0);
  }
}

TEST(FeetechMotorHealth, OutOfRangeRunningVoltageDisarmsAndDisconnects)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  auto config = makeConfig(true);
  config.monitor_motor_health = true;
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state), [&]() {return clock.now();});
  backend.connect();
  for (int id : {7, 8, 9}) {state.voltage_reads[id].push_back(117);}
  ASSERT_TRUE(backend.setMotorPower(true));
  clock.advance(0.6);
  state.voltage_reads[7].push_back(114);
  EXPECT_FALSE(backend.readWheelFeedback());
  EXPECT_FALSE(backend.isMotorPowerEnabled());
  EXPECT_FALSE(backend.isConnected());
  EXPECT_NE(std::find(state.writes.begin(), state.writes.end(), kZeroGoalSpeedSyncPacket), state.writes.end());
}

TEST(FeetechMotorHealth, VoltageOnlyReadKeepsNormalFeedbackRunning)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  auto config = makeConfig(true);
  config.monitor_motor_health = true;
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state), [&]() {return clock.now();});
  backend.connect();
  for (int id : {7, 8, 9}) {state.voltage_reads[id].push_back(117);}
  ASSERT_TRUE(backend.setMotorPower(true));
  clock.advance(0.6);
  for (int id : {7, 8, 9}) {state.voltage_reads[id].push_back(117);}
  for (int id : {7, 8, 9}) {state.reads.push_back(makePresentSpeedStatusPacket(id, 0));}
  EXPECT_TRUE(backend.readWheelFeedback());
  EXPECT_TRUE(backend.isMotorPowerEnabled());
  EXPECT_TRUE(backend.isConnected());
  for (const auto & packet : state.writes) {
    if (packet.size() == 8u && packet[4] == 2u && packet[5] == 62u) {
      EXPECT_EQ(packet[6], 1u);
    }
  }
}

TEST(FeetechMotorHealth, MissingHealthFeedbackFailsClosed)
{
  FakeSerialState state;
  ManualSteadyClock clock;
  auto config = makeConfig(true);
  config.monitor_motor_health = true;
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state), [&]() {return clock.now();});
  backend.connect();
  for (int id : {7, 8, 9}) {state.voltage_reads[id].push_back(117);}
  ASSERT_TRUE(backend.setMotorPower(true));
  clock.advance(0.6);
  EXPECT_FALSE(backend.readWheelFeedback());
  EXPECT_FALSE(backend.isMotorPowerEnabled());
}

TEST(FeetechMotorHealth, BadVoltageChecksumRejectsTorqueEnable)
{
  FakeSerialState state;
  auto config = makeConfig(true);
  config.monitor_motor_health = true;
  config.torque_enable = false;
  lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state));
  backend.connect();
  state.reads.push_back({0xff, 0xff, 7, 3, 0, 117, 0});
  EXPECT_FALSE(backend.setMotorPower(true));
  EXPECT_FALSE(backend.isMotorPowerEnabled());
  EXPECT_EQ(std::count(state.writes.begin(), state.writes.end(), kWheel7TorqueEnablePacket), 0);
}

TEST(FeetechMotorHealth, BadThresholdsAreRejected)
{
  FakeSerialState state;
  for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    auto config = makeConfig(true);
    config.motor_min_voltage = bad;
    EXPECT_THROW(lekiwi_node::FeetechMotorInterface(config, makeFactory(&state)), std::invalid_argument);
  }
  auto config = makeConfig(true);
  config.motor_min_voltage = config.motor_max_voltage;
  EXPECT_THROW(lekiwi_node::FeetechMotorInterface(config, makeFactory(&state)), std::invalid_argument);
}

TEST(FeetechMotorHealth, VoltageAndReadFailureStops)
{
  for (const bool low_voltage : {true, false}) {
    FakeSerialState state;
    ManualSteadyClock clock;
    auto config = makeConfig(true);
    config.monitor_motor_health = true;
    config.torque_enable = false;
    lekiwi_node::FeetechMotorInterface backend(config, makeFactory(&state), [&]() {return clock.now();});
    backend.connect();
    for (int id : {7, 8, 9}) {state.voltage_reads[id].push_back(117);}
    ASSERT_TRUE(backend.setMotorPower(true));
    clock.advance(0.6);
    if (low_voltage) {state.voltage_reads[7].push_back(114);}
    EXPECT_FALSE(backend.readWheelFeedback());
    EXPECT_FALSE(backend.isMotorPowerEnabled());
    EXPECT_FALSE(backend.isConnected());
  }
}
