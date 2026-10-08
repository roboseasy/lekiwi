#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "lekiwi_sensors/bmi160_conversion.hpp"

extern "C"
{
#include "bmi160.h"
}

namespace lekiwi_sensors
{
namespace
{

int g_i2c_fd = -1;
uint8_t g_i2c_address = 0;

int8_t bmi160LinuxRead(
  const uint8_t device_address,
  const uint8_t register_address,
  uint8_t * data,
  const uint16_t length)
{
  if (g_i2c_fd < 0 || data == nullptr || length == 0 || device_address != g_i2c_address) {
    return BMI160_E_COM_FAIL;
  }
  if (::write(g_i2c_fd, &register_address, 1) != 1) {
    return BMI160_E_COM_FAIL;
  }
  return ::read(g_i2c_fd, data, length) == static_cast<ssize_t>(length) ?
         BMI160_OK : BMI160_E_COM_FAIL;
}

int8_t bmi160LinuxWrite(
  const uint8_t device_address,
  const uint8_t register_address,
  uint8_t * data,
  const uint16_t length)
{
  if (g_i2c_fd < 0 || (length > 0 && data == nullptr) || device_address != g_i2c_address) {
    return BMI160_E_COM_FAIL;
  }

  std::vector<uint8_t> packet;
  packet.reserve(static_cast<std::size_t>(length) + 1U);
  packet.push_back(register_address);
  if (length > 0) {
    packet.insert(packet.end(), data, data + length);
  }
  return ::write(g_i2c_fd, packet.data(), packet.size()) ==
         static_cast<ssize_t>(packet.size()) ? BMI160_OK : BMI160_E_COM_FAIL;
}

void bmi160LinuxDelay(const uint32_t milliseconds)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

std::array<int, 3> parseAxisMap(const std::vector<int64_t> & parameter)
{
  if (parameter.size() != 3U) {
    throw std::invalid_argument("axis_map must contain exactly three integers");
  }
  const std::array<int, 3> axis_map{
    static_cast<int>(parameter[0]),
    static_cast<int>(parameter[1]),
    static_cast<int>(parameter[2])};
  if (!isValidAxisMap(axis_map)) {
    throw std::invalid_argument("axis_map must be a signed permutation of [1, 2, 3]");
  }
  return axis_map;
}

void setCovarianceDiagonal(std::array<double, 9> & covariance, const double variance)
{
  covariance.fill(0.0);
  covariance[0] = variance;
  covariance[4] = variance;
  covariance[8] = variance;
}

}  // namespace

class Bmi160Node final : public rclcpp::Node
{
public:
  Bmi160Node()
  : Node("bmi160_node")
  {
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    const int64_t configured_address = declare_parameter<int64_t>("i2c_address", 0x68);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    topic_name_ = declare_parameter<std::string>("topic_name", "imu/data_raw");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    gyro_bias_samples_ = declare_parameter<int64_t>("gyro_bias_samples", 200);
    angular_velocity_variance_ =
      declare_parameter<double>("angular_velocity_variance", 0.0004);
    linear_acceleration_variance_ =
      declare_parameter<double>("linear_acceleration_variance", 0.04);
    axis_map_ = parseAxisMap(
      declare_parameter<std::vector<int64_t>>("axis_map", {1, 2, 3}));

    if (configured_address < 0x08 || configured_address > 0x77) {
      throw std::invalid_argument("i2c_address must be a valid 7-bit I2C address");
    }
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 100.0) {
      throw std::invalid_argument("publish_rate_hz must be in (0, 100]");
    }
    if (gyro_bias_samples_ < 0 || gyro_bias_samples_ > 10000) {
      throw std::invalid_argument("gyro_bias_samples must be in [0, 10000]");
    }
    if (frame_id_.empty() || topic_name_.empty()) {
      throw std::invalid_argument("frame_id and topic_name must not be empty");
    }

    i2c_address_ = static_cast<uint8_t>(configured_address);
    openAndConfigureSensor();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      topic_name_, rclcpp::SensorDataQoS());
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { readAndPublish(); });

    RCLCPP_INFO(
      get_logger(),
      "BMI160 initialized on %s address 0x%02x; frame=%s topic=%s rate=%.1f Hz",
      i2c_device_.c_str(), i2c_address_, frame_id_.c_str(), topic_name_.c_str(), publish_rate_hz_);
    if (gyro_bias_samples_ > 0) {
      RCLCPP_INFO(
        get_logger(),
        "keep the robot stationary while %ld gyro bias samples are collected",
        gyro_bias_samples_);
    }
  }

  ~Bmi160Node() override
  {
    timer_.reset();
    if (i2c_fd_ >= 0) {
      ::close(i2c_fd_);
    }
    if (g_i2c_fd == i2c_fd_) {
      g_i2c_fd = -1;
      g_i2c_address = 0;
    }
  }

private:
  void openAndConfigureSensor()
  {
    if (g_i2c_fd >= 0) {
      throw std::runtime_error("only one BMI160 I2C node may run in a process");
    }

    i2c_fd_ = ::open(i2c_device_.c_str(), O_RDWR | O_CLOEXEC);
    if (i2c_fd_ < 0) {
      throw std::runtime_error(
              "failed to open " + i2c_device_ + ": " + std::strerror(errno));
    }
    if (::ioctl(i2c_fd_, I2C_SLAVE, static_cast<int>(i2c_address_)) < 0) {
      const std::string message =
        "failed to select BMI160 I2C address: " + std::string(std::strerror(errno));
      ::close(i2c_fd_);
      i2c_fd_ = -1;
      throw std::runtime_error(message);
    }

    g_i2c_fd = i2c_fd_;
    g_i2c_address = i2c_address_;
    device_ = {};
    device_.id = i2c_address_;
    device_.intf = BMI160_I2C_INTF;
    device_.read = bmi160LinuxRead;
    device_.write = bmi160LinuxWrite;
    device_.delay_ms = bmi160LinuxDelay;
    device_.read_write_len = 32;

    int8_t result = bmi160_init(&device_);
    if (result != BMI160_OK || device_.chip_id != BMI160_CHIP_ID) {
      throw std::runtime_error("Bosch SensorAPI could not identify a BMI160");
    }

    device_.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    device_.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    device_.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    device_.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    device_.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    device_.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
    device_.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    device_.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
    result = bmi160_set_sens_conf(&device_);
    if (result != BMI160_OK) {
      throw std::runtime_error(
              "Bosch SensorAPI failed to configure BMI160, error " +
              std::to_string(static_cast<int>(result)));
    }
  }

  void readAndPublish()
  {
    bmi160_sensor_data accel{};
    bmi160_sensor_data gyro{};
    const int8_t result = bmi160_get_sensor_data(
      BMI160_BOTH_ACCEL_AND_GYRO, &accel, &gyro, &device_);
    if (result != BMI160_OK) {
      ++consecutive_read_failures_;
      if (consecutive_read_failures_ == 1 || consecutive_read_failures_ % 100 == 0) {
        RCLCPP_ERROR(
          get_logger(), "BMI160 read failed (%d), consecutive failures=%zu",
          static_cast<int>(result), consecutive_read_failures_);
      }
      return;
    }
    consecutive_read_failures_ = 0;

    const auto acceleration = applyAxisMap(
      {
        bmi160AccelerationToMetersPerSecondSquared(accel.x),
        bmi160AccelerationToMetersPerSecondSquared(accel.y),
        bmi160AccelerationToMetersPerSecondSquared(accel.z)},
      axis_map_);
    const auto angular_velocity = applyAxisMap(
      {
        bmi160AngularVelocityToRadiansPerSecond(gyro.x),
        bmi160AngularVelocityToRadiansPerSecond(gyro.y),
        bmi160AngularVelocityToRadiansPerSecond(gyro.z)},
      axis_map_);

    if (bias_samples_collected_ < static_cast<std::size_t>(gyro_bias_samples_)) {
      for (std::size_t i = 0; i < gyro_bias_sum_.size(); ++i) {
        gyro_bias_sum_[i] += angular_velocity[i];
      }
      ++bias_samples_collected_;
      if (bias_samples_collected_ == static_cast<std::size_t>(gyro_bias_samples_)) {
        for (std::size_t i = 0; i < gyro_bias_.size(); ++i) {
          gyro_bias_[i] = gyro_bias_sum_[i] / static_cast<double>(bias_samples_collected_);
        }
        RCLCPP_INFO(
          get_logger(), "BMI160 gyro bias ready: [%.6f, %.6f, %.6f] rad/s",
          gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
      }
      return;
    }

    sensor_msgs::msg::Imu message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;
    message.angular_velocity.x = angular_velocity[0] - gyro_bias_[0];
    message.angular_velocity.y = angular_velocity[1] - gyro_bias_[1];
    message.angular_velocity.z = angular_velocity[2] - gyro_bias_[2];
    message.linear_acceleration.x = acceleration[0];
    message.linear_acceleration.y = acceleration[1];
    message.linear_acceleration.z = acceleration[2];
    setCovarianceDiagonal(message.angular_velocity_covariance, angular_velocity_variance_);
    setCovarianceDiagonal(message.linear_acceleration_covariance, linear_acceleration_variance_);
    publisher_->publish(message);
  }

  std::string i2c_device_;
  uint8_t i2c_address_{0x68};
  std::string frame_id_;
  std::string topic_name_;
  double publish_rate_hz_{100.0};
  int64_t gyro_bias_samples_{200};
  double angular_velocity_variance_{0.0004};
  double linear_acceleration_variance_{0.04};
  std::array<int, 3> axis_map_{1, 2, 3};
  int i2c_fd_{-1};
  bmi160_dev device_{};
  std::array<double, 3> gyro_bias_sum_{};
  std::array<double, 3> gyro_bias_{};
  std::size_t bias_samples_collected_{0};
  std::size_t consecutive_read_failures_{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lekiwi_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<lekiwi_sensors::Bmi160Node>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("bmi160_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
