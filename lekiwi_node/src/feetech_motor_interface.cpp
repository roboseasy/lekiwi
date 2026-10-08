#include "lekiwi_node/feetech_motor_interface.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lekiwi_node
{
namespace
{
constexpr uint8_t kBroadcastId = 0xfe;
constexpr uint8_t kInstructionWrite = 0x03;
constexpr uint8_t kInstructionRead = 0x02;
constexpr uint8_t kInstructionSyncWrite = 0x83;
constexpr uint8_t kSmsStsMode = 33;
constexpr uint8_t kSmsStsTorqueEnable = 40;
constexpr uint8_t kSmsStsGoalSpeedLow = 46;
constexpr uint8_t kSmsStsPresentPositionLow = 56;
constexpr uint8_t kSmsStsPresentSpeedLow = 58;
constexpr uint8_t kSmsStsSpeedMode = 1;
constexpr uint8_t kPresentSpeedReadLength = 2;
constexpr uint8_t kPresentPositionReadLength = 2;
constexpr std::size_t kWriteStatusPacketSize = 6;
constexpr std::size_t kPresentVoltageStatusPacketSize = 7;
constexpr std::size_t kPresentSpeedStatusPacketSize = 8;
constexpr std::size_t kReadStatusPacketReadChunkSize = 1;
constexpr std::size_t kReadStatusPacketByteBudget = 32;
constexpr std::size_t kReadStatusPacketReadCallBudget = 32;
constexpr int kSerialReadTimeoutMs = 5;
constexpr int kVelocitySignBit = 15;
constexpr int kMaxServoId = 253;
constexpr int kMaxVelocityMagnitude = (1 << kVelocitySignBit) - 1;
constexpr double kTwoPi = 6.28318530717958647692;

class PosixFeetechSerialPort final : public FeetechSerialPort
{
public:
  ~PosixFeetechSerialPort() override { close(); }

  bool open(const std::string & serial_port, int baud_rate) override
  {
    close();
    fd_ = ::open(serial_port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "failed to open Feetech serial port '%s': %s",
        serial_port.c_str(),
        std::strerror(errno));
      return false;
    }

    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "failed to read serial attributes for '%s': %s",
        serial_port.c_str(),
        std::strerror(errno));
      close();
      return false;
    }

    const auto speed = toBaudConstant(baud_rate);
    if (speed == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "unsupported Feetech serial baud_rate %d",
        baud_rate);
      close();
      return false;
    }

    cfmakeraw(&options);
    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "failed to set serial baud_rate %d: %s",
        baud_rate,
        std::strerror(errno));
      close();
      return false;
    }
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "failed to apply serial attributes for '%s': %s",
        serial_port.c_str(),
        std::strerror(errno));
      close();
      return false;
    }
    return true;
  }

  bool write(const std::vector<uint8_t> & bytes) override
  {
    if (fd_ < 0) {
      return false;
    }

    std::size_t written = 0;
    while (written < bytes.size()) {
      const ssize_t result = ::write(fd_, bytes.data() + written, bytes.size() - written);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(
          rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
          "failed to write Feetech serial packet: %s",
          std::strerror(errno));
        return false;
      }
      if (result == 0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
          "serial write returned 0 bytes");
        return false;
      }
      written += static_cast<std::size_t>(result);
    }
    return true;
  }

  bool read(std::vector<uint8_t> * bytes, std::size_t size) override
  {
    if (fd_ < 0 || bytes == nullptr) {
      return false;
    }

    bytes->clear();
    bytes->reserve(size);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(kSerialReadTimeoutMs);

    while (bytes->size() < size) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        RCLCPP_ERROR(
          rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
          "timed out reading Feetech serial packet");
        return false;
      }

      const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      pollfd poll_fd{};
      poll_fd.fd = fd_;
      poll_fd.events = POLLIN;
      const int poll_result = ::poll(&poll_fd, 1, static_cast<int>(remaining_ms.count()));
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(
          rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
          "failed to poll Feetech serial packet: %s",
          std::strerror(errno));
        return false;
      }
      if (poll_result == 0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
          "timed out reading Feetech serial packet");
        return false;
      }

      const std::size_t old_size = bytes->size();
      bytes->resize(size);
      const ssize_t result = ::read(fd_, bytes->data() + old_size, size - old_size);
      if (result < 0) {
        bytes->resize(old_size);
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(
          rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
          "failed to read Feetech serial packet: %s",
          std::strerror(errno));
        return false;
      }
      if (result == 0) {
        bytes->resize(old_size);
        continue;
      }
      bytes->resize(old_size + static_cast<std::size_t>(result));
    }

    return true;
  }

  void close() override
  {
    if (fd_ >= 0) {
      (void)::close(fd_);
      fd_ = -1;
    }
  }

private:
  static speed_t toBaudConstant(int baud_rate)
  {
    switch (baud_rate) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
#ifdef B460800
      case 460800:
        return B460800;
#endif
#ifdef B500000
      case 500000:
        return B500000;
#endif
#ifdef B576000
      case 576000:
        return B576000;
#endif
#ifdef B921600
      case 921600:
        return B921600;
#endif
#ifdef B1000000
      case 1000000:
        return B1000000;
#endif
#ifdef B1152000
      case 1152000:
        return B1152000;
#endif
      default:
        return 0;
    }
  }

  int fd_{-1};
};

std::unique_ptr<FeetechSerialPort> makePosixSerialPort()
{
  return std::make_unique<PosixFeetechSerialPort>();
}

uint8_t checksum(const std::vector<uint8_t> & packet_without_checksum)
{
  uint8_t sum = 0;
  for (std::size_t i = 2; i < packet_without_checksum.size(); ++i) {
    sum = static_cast<uint8_t>(sum + packet_without_checksum[i]);
  }
  return static_cast<uint8_t>(~sum);
}

std::vector<uint8_t> makeWritePacket(uint8_t id, uint8_t address, std::vector<uint8_t> parameters)
{
  std::vector<uint8_t> packet{
    0xff,
    0xff,
    id,
    static_cast<uint8_t>(parameters.size() + 3),
    kInstructionWrite,
    address};
  packet.insert(packet.end(), parameters.begin(), parameters.end());
  packet.push_back(checksum(packet));
  return packet;
}

std::vector<uint8_t> makeReadPacket(uint8_t id, uint8_t address, uint8_t length)
{
  std::vector<uint8_t> packet{
    0xff,
    0xff,
    id,
    4,
    kInstructionRead,
    address,
    length};
  packet.push_back(checksum(packet));
  return packet;
}

std::vector<uint8_t> makeSyncWritePacket(
  const std::array<int, 3> & ids,
  uint8_t address,
  const std::array<std::array<uint8_t, 2>, 3> & parameters)
{
  constexpr uint8_t kParameterLength = 2;
  std::vector<uint8_t> packet{
    0xff,
    0xff,
    kBroadcastId,
    static_cast<uint8_t>((kParameterLength + 1) * ids.size() + 4),
    kInstructionSyncWrite,
    address,
    kParameterLength};
  for (std::size_t i = 0; i < ids.size(); ++i) {
    packet.push_back(static_cast<uint8_t>(ids[i]));
    packet.push_back(parameters[i][0]);
    packet.push_back(parameters[i][1]);
  }
  packet.push_back(checksum(packet));
  return packet;
}

std::array<uint8_t, 2> toLittleEndianWord(int value)
{
  return {
    static_cast<uint8_t>(value & 0xff),
    static_cast<uint8_t>((value >> 8) & 0xff)};
}

std::array<uint8_t, 2> encodeVelocity(double wheel_speed, double direction, const FeetechMotorConfig & config)
{
  const double directed_speed = wheel_speed * direction;
  const int signed_ticks = static_cast<int>(std::llround(directed_speed * config.speed_tick_scale));
  const int magnitude = std::min(std::abs(signed_ticks), config.speed_tick_limit);
  const int encoded = signed_ticks < 0 ? (magnitude | (1 << kVelocitySignBit)) : magnitude;
  return toLittleEndianWord(encoded);
}

int decodeVelocityTicks(uint8_t low_byte, uint8_t high_byte)
{
  const int encoded = static_cast<int>(low_byte) | (static_cast<int>(high_byte) << 8);
  const int magnitude = encoded & kMaxVelocityMagnitude;
  return (encoded & (1 << kVelocitySignBit)) != 0 ? -magnitude : magnitude;
}

int decodePositionTicks(uint8_t low_byte, uint8_t high_byte)
{
  return static_cast<int>(low_byte) | (static_cast<int>(high_byte) << 8);
}

int wrapPositionDelta(int current_ticks, int previous_ticks, int ticks_per_revolution)
{
  int delta = current_ticks - previous_ticks;
  const int half_revolution = ticks_per_revolution / 2;
  if (delta > half_revolution) {
    delta -= ticks_per_revolution;
  } else if (delta < -half_revolution) {
    delta += ticks_per_revolution;
  }
  return delta;
}

bool isValidReadStatusPacket(
  const std::vector<uint8_t> & packet, uint8_t expected_id, std::size_t expected_size)
{
  if (packet.size() != expected_size) {
    return false;
  }
  if (packet[0] != 0xff || packet[1] != 0xff || packet[2] != expected_id ||
    packet[3] != expected_size - 4u)
  {
    return false;
  }
  if (packet[4] != 0) {
    return false;
  }

  std::vector<uint8_t> packet_without_checksum(packet.begin(), packet.end() - 1);
  return packet.back() == checksum(packet_without_checksum);
}

bool isConfiguredWheelId(uint8_t id, const std::array<int, 3> & wheel_ids)
{
  return std::any_of(
    wheel_ids.begin(), wheel_ids.end(),
    [id](int configured_id) {return configured_id == static_cast<int>(id);});
}

bool isValidWriteStatusPacket(
  const std::vector<uint8_t> & packet,
  const std::array<int, 3> & wheel_ids)
{
  if (packet.size() != kWriteStatusPacketSize ||
    packet[0] != 0xff || packet[1] != 0xff || packet[3] != 2 || packet[4] != 0 ||
    !isConfiguredWheelId(packet[2], wheel_ids))
  {
    return false;
  }
  const std::vector<uint8_t> packet_without_checksum(packet.begin(), packet.end() - 1);
  return packet.back() == checksum(packet_without_checksum);
}

std::optional<std::vector<uint8_t>> readStatusPacket(
  FeetechSerialPort & serial_port,
  uint8_t expected_id,
  const std::array<int, 3> & wheel_ids,
  std::size_t expected_size = kPresentSpeedStatusPacketSize)
{
  std::vector<uint8_t> buffered_bytes;
  buffered_bytes.reserve(kReadStatusPacketByteBudget);

  for (std::size_t bytes_read = 0, read_calls = 0;
    bytes_read < kReadStatusPacketByteBudget &&
    read_calls < kReadStatusPacketReadCallBudget; )
  {
    std::vector<uint8_t> read_bytes;
    ++read_calls;
    if (!serial_port.read(&read_bytes, kReadStatusPacketReadChunkSize) || read_bytes.empty()) {
      return std::nullopt;
    }

    bytes_read += read_bytes.size();
    buffered_bytes.insert(buffered_bytes.end(), read_bytes.begin(), read_bytes.end());
    bool discarded_write_ack = false;
    bool waiting_for_unsupported_configured_packet = false;
    for (std::size_t offset = 0; offset + 1 < buffered_bytes.size(); ++offset) {
      if (buffered_bytes[offset] != 0xff || buffered_bytes[offset + 1] != 0xff) {
        continue;
      }
      if (buffered_bytes.size() - offset < 4u) {
        break;
      }

      const std::size_t packet_size =
        static_cast<std::size_t>(buffered_bytes[offset + 3]) + 4u;
      if (packet_size != kWriteStatusPacketSize &&
        packet_size != kPresentVoltageStatusPacketSize &&
        packet_size != kPresentSpeedStatusPacketSize)
      {
        if (!isConfiguredWheelId(buffered_bytes[offset + 2], wheel_ids)) {
          continue;
        }
        if (buffered_bytes.size() - offset < packet_size) {
          waiting_for_unsupported_configured_packet = true;
          break;
        }
        return std::nullopt;
      }
      if (buffered_bytes.size() - offset < packet_size) {
        break;
      }

      std::vector<uint8_t> candidate(
        buffered_bytes.begin() + static_cast<std::ptrdiff_t>(offset),
        buffered_bytes.begin() + static_cast<std::ptrdiff_t>(offset + packet_size));
      if (packet_size == kWriteStatusPacketSize) {
        if (!isValidWriteStatusPacket(candidate, wheel_ids)) {
          return std::nullopt;
        }
        buffered_bytes.erase(
          buffered_bytes.begin(),
          buffered_bytes.begin() + static_cast<std::ptrdiff_t>(offset + packet_size));
        discarded_write_ack = true;
        break;
      }
      if (!isValidReadStatusPacket(candidate, candidate[2], expected_size) ||
        candidate[2] != expected_id)
      {
        return std::nullopt;
      }
      return candidate;
    }

    if (discarded_write_ack) {
      continue;
    }
    if (waiting_for_unsupported_configured_packet) {
      continue;
    }
    if (buffered_bytes.size() > kPresentSpeedStatusPacketSize - 1) {
      buffered_bytes.erase(
        buffered_bytes.begin(),
        buffered_bytes.end() - static_cast<std::ptrdiff_t>(kPresentSpeedStatusPacketSize - 1));
    }
  }

  return std::nullopt;
}

void validateConfig(const FeetechMotorConfig & config)
{
  for (const double value : {config.motor_min_voltage, config.motor_max_voltage})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("motor health limits must be positive and finite");
    }
  }
  if (config.motor_min_voltage >= config.motor_max_voltage ||
    config.motor_max_voltage > 25.5)
  {
    throw std::invalid_argument("invalid motor health limit ordering or register range");
  }
  if (config.baud_rate <= 0) {
    throw std::invalid_argument("baud_rate must be positive");
  }
  if (!std::isfinite(config.speed_tick_scale) || config.speed_tick_scale <= 0.0) {
    throw std::invalid_argument("speed_tick_scale must be positive and finite");
  }
  if (config.speed_tick_limit < 0 || config.speed_tick_limit > kMaxVelocityMagnitude) {
    throw std::invalid_argument("speed_tick_limit must be between 0 and 32767");
  }
  if (config.encoder_tick_deadband < 0 || config.encoder_tick_deadband > kMaxVelocityMagnitude) {
    throw std::invalid_argument("encoder_tick_deadband must be between 0 and 32767");
  }
  if (config.encoder_feedback_source != "speed" && config.encoder_feedback_source != "position") {
    throw std::invalid_argument("encoder_feedback_source must be 'speed' or 'position'");
  }
  if (!std::isfinite(config.encoder_read_period) || config.encoder_read_period <= 0.0) {
    throw std::invalid_argument("encoder_read_period must be positive and finite");
  }
  if (!std::isfinite(config.encoder_sample_timeout) || config.encoder_sample_timeout <= 0.0) {
    throw std::invalid_argument("encoder_sample_timeout must be positive and finite");
  }
  if (config.encoder_feedback_source == "position") {
    constexpr double kTimingTolerance = 1e-12;
    if (3.0 * config.encoder_read_period >
      config.encoder_sample_timeout + kTimingTolerance)
    {
      throw std::invalid_argument(
        "position feedback requires 3 * encoder_read_period <= encoder_sample_timeout");
    }
  }
  if (config.position_ticks_per_revolution <= 0) {
    throw std::invalid_argument("position_ticks_per_revolution must be positive");
  }
  if (!std::isfinite(config.encoder_odom_scale) || config.encoder_odom_scale <= 0.0) {
    throw std::invalid_argument("encoder_odom_scale must be positive and finite");
  }
  if (config.encoder_position_tick_deadband < 0 ||
    config.encoder_position_tick_deadband >= config.position_ticks_per_revolution / 2)
  {
    throw std::invalid_argument(
      "encoder_position_tick_deadband must be non-negative and less than half a revolution");
  }
  if (config.encoder_feedback_source == "position" &&
    config.encoder_position_tick_deadband != 0)
  {
    throw std::invalid_argument(
      "encoder_position_tick_deadband is deprecated and must be zero for position feedback");
  }
  if (!std::isfinite(config.max_wheel_speed) || config.max_wheel_speed <= 0.0) {
    throw std::invalid_argument("max_wheel_speed must be positive and finite");
  }
  const double max_speed_ticks = config.speed_tick_scale * config.max_wheel_speed;
  if (!std::isfinite(max_speed_ticks) ||
    max_speed_ticks > static_cast<double>(kMaxVelocityMagnitude) ||
    max_speed_ticks > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument(
      "speed_tick_scale * max_wheel_speed must fit Feetech sign-magnitude velocity range");
  }
  for (std::size_t i = 0; i < config.wheel_ids.size(); ++i) {
    if (config.wheel_ids[i] < 1 || config.wheel_ids[i] > kMaxServoId) {
      throw std::invalid_argument("wheel_ids must be in the range [1, 253]");
    }
    for (std::size_t j = i + 1; j < config.wheel_ids.size(); ++j) {
      if (config.wheel_ids[i] == config.wheel_ids[j]) {
        throw std::invalid_argument("wheel_ids must not contain duplicate values");
      }
    }
    if (!std::isfinite(config.wheel_directions[i])) {
      throw std::invalid_argument("wheel_directions must be finite");
    }
    if (config.wheel_directions[i] == 0.0) {
      throw std::invalid_argument("wheel_directions must not contain zero");
    }
  }
}

}  // namespace

FeetechMotorInterface::FeetechMotorInterface(FeetechMotorConfig config)
: FeetechMotorInterface(
    std::move(config), makePosixSerialPort, []() { return std::chrono::steady_clock::now(); })
{
}

FeetechMotorInterface::FeetechMotorInterface(
  FeetechMotorConfig config,
  FeetechSerialPortFactory serial_port_factory)
: FeetechMotorInterface(
    std::move(config), std::move(serial_port_factory),
    []() { return std::chrono::steady_clock::now(); })
{
}

FeetechMotorInterface::FeetechMotorInterface(
  FeetechMotorConfig config,
  FeetechSerialPortFactory serial_port_factory,
  SteadyNowFunction steady_now_function)
: BaseMotorInterface(config.max_wheel_speed),
  config_(std::move(config)),
  serial_port_factory_(std::move(serial_port_factory)),
  steady_now_function_(std::move(steady_now_function))
{
  validateConfig(config_);
}

void FeetechMotorInterface::connect()
{
  last_health_check_.reset();
  last_health_log_.reset();
  resetPositionEstimator();
  connected_ = false;
  serial_port_probe_succeeded_ = false;
  disarm_verified_ = false;

  if (!config_.enable_motor_write) {
    RCLCPP_WARN(
      rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
      "Feetech motor write is disabled; serial port '%s' will not be opened",
      config_.serial_port.c_str());
    return;
  }
  setMotorPowerState(true);

  serial_port_ = serial_port_factory_ ? serial_port_factory_() : nullptr;
  if (!serial_port_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
      "failed to create Feetech serial port backend");
    return;
  }

  if (!serial_port_->open(config_.serial_port, config_.baud_rate)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
      "failed to open Feetech serial port '%s'",
      config_.serial_port.c_str());
    return;
  }
  serial_port_probe_succeeded_ = true;

  if (!tryWriteTorqueDisablePackets()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
      "failed to establish torque-disabled state for one or more Feetech wheels");
    cleanupAfterFault(true);
    return;
  }
  disarm_verified_ = true;
  setMotorPowerState(false);

  for (const int wheel_id : config_.wheel_ids) {
    const auto id = static_cast<uint8_t>(wheel_id);
    bool speed_mode_verified = false;
    try {
      speed_mode_verified = writeUnicastPacketAndExpectAck(
        id, makeWritePacket(id, kSmsStsMode, {kSmsStsSpeedMode}));
    } catch (...) {
    }
    if (!speed_mode_verified)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "failed to set Feetech wheel id %d to speed mode",
        wheel_id);
      cleanupAfterFault(true);
      return;
    }
  }

  if (!tryWriteZeroGoalSpeedSync()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
      "failed to write zero Feetech wheel speed sync packet during initialization");
    cleanupAfterFault(true);
    return;
  }

  if (config_.torque_enable) {
    if (!checkMotorHealth(true)) {
      cleanupAfterFault(false);
      return;
    }
    disarm_verified_ = false;
    setMotorPowerState(true);
    bool torque_enable_verified = false;
    try {
      torque_enable_verified = writeTorqueEnablePackets();
    } catch (...) {
    }
    if (!torque_enable_verified) {
      RCLCPP_ERROR(
        rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "failed to enable torque for one or more Feetech wheels");
      cleanupAfterFault(true);
      return;
    }
  }

  connected_ = true;
}

bool FeetechMotorInterface::hasSerialPortProbeSucceeded() const
{
  return serial_port_probe_succeeded_;
}

bool FeetechMotorInterface::isConnected() const
{
  return connected_;
}

const std::vector<FeetechPresentSpeedSample> & FeetechMotorInterface::lastPresentSpeedSamples() const
{
  return last_present_speed_samples_;
}

std::optional<WheelFeedbackSample> FeetechMotorInterface::readWheelFeedback()
{
  if (!config_.enable_motor_write || !connected_ || !serial_port_) {
    return std::nullopt;
  }

  if (isMotorPowerEnabled() && !checkMotorHealth(false)) {
    cleanupAfterFault(false);
    return std::nullopt;
  }

  last_present_speed_samples_.clear();
  std::array<double, 3> wheel_speeds{};
  if (config_.encoder_feedback_source == "position") {
    std::array<int, 3> current_position_ticks{};
    for (std::size_t i = 0; i < config_.wheel_ids.size(); ++i) {
      const auto id = static_cast<uint8_t>(config_.wheel_ids[i]);
      if (!serial_port_->write(makeReadPacket(id, kSmsStsPresentPositionLow, kPresentPositionReadLength))) {
        cleanupAfterFault(false);
        return std::nullopt;
      }

      const auto status_packet = readStatusPacket(*serial_port_, id, config_.wheel_ids);
      if (!status_packet) {
        return std::nullopt;
      }

      current_position_ticks[i] = decodePositionTicks((*status_packet)[5], (*status_packet)[6]);
    }
    const auto current_position_time = steady_now_function_();

    if (!last_position_ticks_) {
      last_position_ticks_ = current_position_ticks;
      last_position_time_ = current_position_time;
      position_intervals_.push_back(PositionInterval{{0, 0, 0}, config_.encoder_read_period});
      position_intervals_.push_back(PositionInterval{{0, 0, 0}, config_.encoder_read_period});
      return std::nullopt;
    }

    const double elapsed_seconds =
      std::chrono::duration<double>(current_position_time - *last_position_time_).count();
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0) {
      return std::nullopt;
    }

    PositionInterval current_interval;
    current_interval.elapsed_seconds = elapsed_seconds;
    for (std::size_t i = 0; i < config_.wheel_ids.size(); ++i) {
      current_interval.delta_ticks[i] = wrapPositionDelta(
        current_position_ticks[i],
        (*last_position_ticks_)[i],
        config_.position_ticks_per_revolution);
    }

    auto updated_intervals = position_intervals_;
    updated_intervals.push_back(current_interval);
    while (updated_intervals.size() > 3u) {
      updated_intervals.pop_front();
    }

    std::array<int, 3> window_delta_ticks{};
    double window_elapsed_seconds = 0.0;
    for (const auto & interval : updated_intervals) {
      window_elapsed_seconds += interval.elapsed_seconds;
      for (std::size_t i = 0; i < window_delta_ticks.size(); ++i) {
        window_delta_ticks[i] += interval.delta_ticks[i];
      }
    }
    if (!std::isfinite(window_elapsed_seconds) || window_elapsed_seconds <= 0.0) {
      return std::nullopt;
    }

    std::array<double, 3> wheel_position_deltas{};
    for (std::size_t i = 0; i < config_.wheel_ids.size(); ++i) {
      const double direction_and_scale =
        config_.encoder_odom_scale / config_.wheel_directions[i];
      wheel_speeds[i] =
        static_cast<double>(window_delta_ticks[i]) * kTwoPi /
        static_cast<double>(config_.position_ticks_per_revolution) /
        window_elapsed_seconds * direction_and_scale;
      wheel_position_deltas[i] =
        static_cast<double>(current_interval.delta_ticks[i]) * kTwoPi /
        static_cast<double>(config_.position_ticks_per_revolution) * direction_and_scale;
      if (!std::isfinite(wheel_speeds[i]) || !std::isfinite(wheel_position_deltas[i])) {
        return std::nullopt;
      }
      if (config_.log_encoder_reads) {
        RCLCPP_INFO(
          rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
          "Feetech present position wheel_id=%d raw_position_ticks=%d previous_position_ticks=%d "
          "raw_delta_ticks=%d window_delta_ticks=%d window_elapsed_seconds=%.6f wheel_speed=%.6f",
          config_.wheel_ids[i],
          current_position_ticks[i],
          (*last_position_ticks_)[i],
          current_interval.delta_ticks[i],
          window_delta_ticks[i],
          window_elapsed_seconds,
          wheel_speeds[i]);
      }
    }

    position_intervals_ = std::move(updated_intervals);
    last_position_ticks_ = current_position_ticks;
    last_position_time_ = current_position_time;
    return WheelFeedbackSample{
      WheelCommand{wheel_speeds[0], wheel_speeds[1], wheel_speeds[2]},
      WheelPositionDelta{
        wheel_position_deltas[0], wheel_position_deltas[1], wheel_position_deltas[2]}};
  }

  for (std::size_t i = 0; i < config_.wheel_ids.size(); ++i) {
    const auto id = static_cast<uint8_t>(config_.wheel_ids[i]);
    if (!serial_port_->write(makeReadPacket(id, kSmsStsPresentSpeedLow, kPresentSpeedReadLength))) {
      cleanupAfterFault(false);
      return std::nullopt;
    }

    const auto status_packet = readStatusPacket(*serial_port_, id, config_.wheel_ids);
    if (!status_packet) {
      return std::nullopt;
    }

    const int raw_signed_ticks = decodeVelocityTicks((*status_packet)[5], (*status_packet)[6]);
    const int filtered_signed_ticks =
      std::abs(raw_signed_ticks) <= config_.encoder_tick_deadband ? 0 : raw_signed_ticks;
    const double raw_wheel_speed =
      static_cast<double>(filtered_signed_ticks) / config_.speed_tick_scale /
      config_.wheel_directions[i];
    wheel_speeds[i] = raw_wheel_speed * config_.encoder_odom_scale;
    if (config_.log_encoder_reads) {
      last_present_speed_samples_.push_back(
        FeetechPresentSpeedSample{
          config_.wheel_ids[i],
          (*status_packet)[5],
          (*status_packet)[6],
          raw_signed_ticks,
          filtered_signed_ticks,
          wheel_speeds[i]});
      RCLCPP_INFO(
        rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "Feetech present speed wheel_id=%d raw_low=0x%02x raw_high=0x%02x raw_signed_ticks=%d "
        "filtered_signed_ticks=%d wheel_speed=%.6f",
        config_.wheel_ids[i],
        (*status_packet)[5],
        (*status_packet)[6],
        raw_signed_ticks,
        filtered_signed_ticks,
        wheel_speeds[i]);
    }
  }

  return WheelFeedbackSample{
    WheelCommand{wheel_speeds[0], wheel_speeds[1], wheel_speeds[2]}, std::nullopt};
}

bool FeetechMotorInterface::setMotorPower(const bool enabled)
{
  if (!config_.enable_motor_write || !connected_ || !serial_port_) {
    return !enabled && disarm_verified_;
  }

  if (enabled) {
    if (!checkMotorHealth(true)) {
      cleanupAfterFault(false);
      return false;
    }
    resetPositionEstimator();
    disarm_verified_ = false;
    setMotorPowerState(true);
    if (!tryWriteZeroGoalSpeedSync()) {
      cleanupAfterFault(false);
      return false;
    }
    bool success = false;
    try {
      success = writeTorqueEnablePackets();
    } catch (...) {
    }
    if (!success) {
      cleanupAfterFault(false);
    }
    return success;
  }

  const bool zero_succeeded = tryWriteZeroGoalSpeedSync();
  const bool disable_verified = tryWriteTorqueDisablePackets();
  if (zero_succeeded && disable_verified) {
    disarm_verified_ = true;
    setMotorPowerState(false);
    resetPositionEstimator();
    return true;
  }
  disarm_verified_ = false;
  cleanupAfterFault(true);
  return false;
}

bool FeetechMotorInterface::checkMotorHealth(const bool before_enable)
{
  if (!config_.monitor_motor_health) {
    return true;
  }
  const auto current = steady_now_function_();
  if (!before_enable && last_health_check_ &&
    current - *last_health_check_ < std::chrono::milliseconds(500))
  {
    return true;
  }
  if (!serial_port_) {
    return false;
  }
  const bool log_health = before_enable || !last_health_log_ ||
    current - *last_health_log_ >= std::chrono::seconds(1);
  for (const int wheel_id : config_.wheel_ids) {
    if (!serial_port_->write(makeReadPacket(static_cast<uint8_t>(wheel_id), 62u, 1u))) {
      RCLCPP_ERROR(rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "motor voltage read failed id=%d; stopping", wheel_id);
      return false;
    }
    const auto packet = readStatusPacket(
      *serial_port_, static_cast<uint8_t>(wheel_id), config_.wheel_ids,
      kPresentVoltageStatusPacketSize);
    if (!packet) {
      RCLCPP_ERROR(rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "motor voltage read failed id=%d; stopping", wheel_id);
      return false;
    }
    const double voltage = (*packet)[5] / 10.0;
    if (log_health) {
      RCLCPP_INFO(rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "motor voltage id=%d value=%.1f V", wheel_id, voltage);
    }
    if (voltage < config_.motor_min_voltage || voltage > config_.motor_max_voltage) {
      RCLCPP_ERROR(rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "motor supply stop id=%d voltage=%.1f V", wheel_id, voltage);
      return false;
    }
  }
  last_health_check_ = steady_now_function_();
  if (log_health) {
    last_health_log_ = last_health_check_;
  }
  return true;
}

void FeetechMotorInterface::shutdown() noexcept
{
  const bool preserve_unverified_fault = !connected_ && !disarm_verified_;
  resetPositionEstimator();
  connected_ = false;
  if (!serial_port_) {
    return;
  }
  cleanupAfterFault(preserve_unverified_fault);
}

bool FeetechMotorInterface::writeUnicastPacketAndExpectAck(
  const uint8_t expected_id,
  const std::vector<uint8_t> & packet)
{
  if (!serial_port_ || !serial_port_->write(packet)) {
    return false;
  }

  std::vector<uint8_t> ack;
  if (!serial_port_->read(&ack, 6u) || ack.size() != 6u) {
    return false;
  }

  const std::vector<uint8_t> ack_without_checksum(ack.begin(), ack.end() - 1);
  return
    ack[0] == 0xff && ack[1] == 0xff && ack[2] == expected_id &&
    ack[3] == 2u && ack[4] == 0u && ack[5] == checksum(ack_without_checksum);
}

bool FeetechMotorInterface::writeZeroGoalSpeedSync()
{
  if (!serial_port_) {
    return false;
  }
  return serial_port_->write(makeSyncWritePacket(config_.wheel_ids, kSmsStsGoalSpeedLow, {}));
}

bool FeetechMotorInterface::writeTorqueEnablePackets()
{
  if (!serial_port_) {
    return false;
  }

  for (const int wheel_id : config_.wheel_ids) {
    const auto id = static_cast<uint8_t>(wheel_id);
    if (!writeUnicastPacketAndExpectAck(
        id, makeWritePacket(id, kSmsStsTorqueEnable, {1})))
    {
      return false;
    }
  }
  return true;
}

bool FeetechMotorInterface::writeTorqueDisablePackets()
{
  if (!serial_port_) {
    return false;
  }

  bool all_writes_succeeded = true;
  for (const int wheel_id : config_.wheel_ids) {
    const auto id = static_cast<uint8_t>(wheel_id);
    const bool write_succeeded = writeUnicastPacketAndExpectAck(
      id, makeWritePacket(id, kSmsStsTorqueEnable, {0}));
    all_writes_succeeded = all_writes_succeeded && write_succeeded;
  }
  return all_writes_succeeded;
}

bool FeetechMotorInterface::tryWriteZeroGoalSpeedSync() noexcept
{
  try {
    return writeZeroGoalSpeedSync();
  } catch (...) {
    return false;
  }
}

bool FeetechMotorInterface::tryWriteTorqueDisablePackets() noexcept
{
  if (!serial_port_) {
    return false;
  }

  using Clock = std::chrono::steady_clock;
  const auto elapsed_us = [](const Clock::time_point start, const Clock::time_point end) {
      return static_cast<long long>(
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
    };
  std::array<bool, 3> write_acks{};
  std::array<Clock::time_point, 3> ack_times{};
  std::array<int, 3> readback_values{-1, -1, -1};
  std::array<long long, 3> ack_to_read_us{};
  std::array<long long, 3> read_us{};
  bool all_disables_verified = true;
  for (std::size_t i = 0; i < config_.wheel_ids.size(); ++i) {
    const int wheel_id = config_.wheel_ids[i];
    const auto id = static_cast<uint8_t>(wheel_id);
    try {
      write_acks[i] = writeUnicastPacketAndExpectAck(
        id, makeWritePacket(id, kSmsStsTorqueEnable, {0}));
    } catch (...) {
    }
    ack_times[i] = Clock::now();
    all_disables_verified = all_disables_verified && write_acks[i];
  }
  if (!all_disables_verified) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
      "torque-disable write ACK failed: ids=[%d,%d,%d] ack=[%d,%d,%d]",
      config_.wheel_ids[0], config_.wheel_ids[1], config_.wheel_ids[2],
      write_acks[0], write_acks[1], write_acks[2]);
    return false;
  }

  // A write ACK alone does not establish that torque is off. Send all three
  // disable commands before spending time on register readback.
  for (std::size_t i = 0; i < config_.wheel_ids.size(); ++i) {
    const int wheel_id = config_.wheel_ids[i];
    const auto id = static_cast<uint8_t>(wheel_id);
    bool torque_is_off = false;
    std::vector<uint8_t> status;
    bool read_request_sent = false;
    bool response_read = false;
    bool read_exception = false;
    const auto read_start = Clock::now();
    ack_to_read_us[i] = elapsed_us(ack_times[i], read_start);
    try {
      read_request_sent = serial_port_->write(makeReadPacket(id, kSmsStsTorqueEnable, 1u));
      if (read_request_sent) {
        response_read = serial_port_->read(&status, 7u);
      }
      if (read_request_sent && response_read && status.size() == 7u)
      {
        const std::vector<uint8_t> without_checksum(status.begin(), status.end() - 1);
        const bool valid_status =
          status[0] == 0xff && status[1] == 0xff && status[2] == id &&
          status[3] == 3u && status[4] == 0u &&
          status[6] == checksum(without_checksum);
        readback_values[i] = valid_status ? static_cast<int>(status[5]) : -1;
        torque_is_off = readback_values[i] == 0;
      }
    } catch (...) {
      read_exception = true;
    }
    read_us[i] = elapsed_us(read_start, Clock::now());
    if (!torque_is_off) {
      const auto byte = [&status](std::size_t index) {
          return index < status.size() ? static_cast<int>(status[index]) : -1;
        };
      RCLCPP_ERROR(
        rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
        "failed to verify torque-disabled register for Feetech wheel id %d: "
        "request_sent=%d response_read=%d exception=%d bytes=%zu "
        "packet_decimal=[%d,%d,%d,%d,%d,%d,%d] (-1=missing)",
        wheel_id, read_request_sent, response_read, read_exception, status.size(),
        byte(0), byte(1), byte(2), byte(3), byte(4), byte(5), byte(6));
    }
    all_disables_verified = all_disables_verified && torque_is_off;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
    "torque-disable readback: ids=[%d,%d,%d] ack=[%d,%d,%d] value=[%d,%d,%d] "
    "ack_to_read_us=[%lld,%lld,%lld] read_us=[%lld,%lld,%lld] verified=%d",
    config_.wheel_ids[0], config_.wheel_ids[1], config_.wheel_ids[2],
    write_acks[0], write_acks[1], write_acks[2],
    readback_values[0], readback_values[1], readback_values[2],
    ack_to_read_us[0], ack_to_read_us[1], ack_to_read_us[2],
    read_us[0], read_us[1], read_us[2], all_disables_verified);
  return all_disables_verified;
}

void FeetechMotorInterface::tryCloseSerialPort() noexcept
{
  if (!serial_port_) {
    return;
  }
  try {
    serial_port_->close();
  } catch (...) {
  }
}

void FeetechMotorInterface::cleanupAfterFault(const bool preserve_unverified_fault) noexcept
{
  resetPositionEstimator();
  connected_ = false;
  const bool zero_write_succeeded = tryWriteZeroGoalSpeedSync();
  const bool disable_verified = tryWriteTorqueDisablePackets();
  tryCloseSerialPort();
  if (zero_write_succeeded && disable_verified && !preserve_unverified_fault) {
    RCLCPP_INFO(
      rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
      "Feetech cleanup completed: zero_goal_speed_write=1 "
      "torque_disable_verified=1; serial close attempted");
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
      "Feetech fault cleanup completed: zero_goal_speed_write=%d "
      "torque_disable_verified=%d preserve_unverified_fault=%d; serial close attempted",
      zero_write_succeeded, disable_verified, preserve_unverified_fault);
  }

  if (preserve_unverified_fault) {
    disarm_verified_ = false;
    return;
  }

  disarm_verified_ = disable_verified;
  if (disable_verified) {
    setMotorPowerState(false);
  }
}

void FeetechMotorInterface::resetPositionEstimator() noexcept
{
  position_intervals_.clear();
  last_position_ticks_.reset();
  last_position_time_.reset();
}

void FeetechMotorInterface::writeClampedWheelSpeeds(const WheelCommand & command)
{
  if (!config_.enable_motor_write || !connected_ || !serial_port_) {
    return;
  }

  const std::array<double, 3> wheel_speeds{command.left, command.back, command.right};
  std::array<std::array<uint8_t, 2>, 3> speed_parameters{};
  for (std::size_t i = 0; i < wheel_speeds.size(); ++i) {
    speed_parameters[i] = encodeVelocity(wheel_speeds[i], config_.wheel_directions[i], config_);
  }

  if (!serial_port_->write(makeSyncWritePacket(config_.wheel_ids, kSmsStsGoalSpeedLow, speed_parameters))) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lekiwi_node.feetech_motor_interface"),
      "failed to write Feetech wheel speed sync packet; motor backend is now disconnected");
    cleanupAfterFault(false);
  }
}

}  // namespace lekiwi_node
