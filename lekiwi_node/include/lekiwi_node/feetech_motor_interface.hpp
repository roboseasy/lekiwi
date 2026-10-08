#ifndef LEKIWI_NODE__FEETECH_MOTOR_INTERFACE_HPP_
#define LEKIWI_NODE__FEETECH_MOTOR_INTERFACE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lekiwi_node/base_controller.hpp"

namespace lekiwi_node
{

struct FeetechMotorConfig
{
  std::string serial_port{"/dev/ttyACM0"};
  // Model joints left/back/right occupy physical rear/right/left positions.
  std::array<int, 3> wheel_ids{8, 9, 7};
  std::array<double, 3> wheel_directions{1.0, 1.0, 1.0};
  double max_wheel_speed{2.3009711818284617};
  bool enable_motor_write{false};
  int baud_rate{1000000};
  double speed_tick_scale{651.8986469044033};
  int speed_tick_limit{1500};
  int encoder_tick_deadband{50};
  std::string encoder_feedback_source{"position"};
  double encoder_read_period{0.1};
  double encoder_sample_timeout{0.5};
  int position_ticks_per_revolution{4096};
  double encoder_odom_scale{1.0};
  int encoder_position_tick_deadband{0};
  bool torque_enable{false};
  bool log_encoder_reads{false};
  bool monitor_motor_health{false};
  double motor_min_voltage{11.5};
  double motor_max_voltage{12.6};
};

struct FeetechPresentSpeedSample
{
  int wheel_id{0};
  uint8_t raw_low_byte{0};
  uint8_t raw_high_byte{0};
  int raw_signed_ticks{0};
  int filtered_signed_ticks{0};
  double wheel_speed{0.0};
};

class FeetechSerialPort
{
public:
  virtual ~FeetechSerialPort() = default;
  virtual bool open(const std::string & serial_port, int baud_rate) = 0;
  virtual bool write(const std::vector<uint8_t> & bytes) = 0;
  virtual bool read(std::vector<uint8_t> * bytes, std::size_t size) = 0;
  virtual void close() = 0;
};

using FeetechSerialPortFactory = std::function<std::unique_ptr<FeetechSerialPort>()>;
using SteadyNowFunction = std::function<std::chrono::steady_clock::time_point()>;

class FeetechMotorInterface final : public BaseMotorInterface
{
public:
  explicit FeetechMotorInterface(FeetechMotorConfig config);
  FeetechMotorInterface(
    FeetechMotorConfig config,
    FeetechSerialPortFactory serial_port_factory);
  FeetechMotorInterface(
    FeetechMotorConfig config,
    FeetechSerialPortFactory serial_port_factory,
    SteadyNowFunction steady_now_function);

  void connect() override;
  void shutdown() noexcept override;
  bool hasSerialPortProbeSucceeded() const;
  bool isConnected() const override;
  std::optional<WheelFeedbackSample> readWheelFeedback() override;
  bool setMotorPower(bool enabled) override;
  const std::vector<FeetechPresentSpeedSample> & lastPresentSpeedSamples() const;

private:
  struct PositionInterval
  {
    std::array<int, 3> delta_ticks{};
    double elapsed_seconds{0.0};
  };

  void writeClampedWheelSpeeds(const WheelCommand & command) override;
  bool writeUnicastPacketAndExpectAck(
    uint8_t expected_id,
    const std::vector<uint8_t> & packet);
  bool writeZeroGoalSpeedSync();
  bool writeTorqueEnablePackets();
  bool writeTorqueDisablePackets();
  bool tryWriteZeroGoalSpeedSync() noexcept;
  bool tryWriteTorqueDisablePackets() noexcept;
  void tryCloseSerialPort() noexcept;
  void cleanupAfterFault(bool preserve_unverified_fault) noexcept;
  void resetPositionEstimator() noexcept;
  bool checkMotorHealth(bool before_enable);

  FeetechMotorConfig config_;
  FeetechSerialPortFactory serial_port_factory_;
  SteadyNowFunction steady_now_function_;
  std::unique_ptr<FeetechSerialPort> serial_port_;
  std::vector<FeetechPresentSpeedSample> last_present_speed_samples_;
  std::deque<PositionInterval> position_intervals_;
  std::optional<std::array<int, 3>> last_position_ticks_;
  std::optional<std::chrono::steady_clock::time_point> last_position_time_;
  bool serial_port_probe_succeeded_{false};
  bool connected_{false};
  bool disarm_verified_{false};
  std::optional<std::chrono::steady_clock::time_point> last_health_check_;
  std::optional<std::chrono::steady_clock::time_point> last_health_log_;
};

}  // namespace lekiwi_node

#endif  // LEKIWI_NODE__FEETECH_MOTOR_INTERFACE_HPP_
