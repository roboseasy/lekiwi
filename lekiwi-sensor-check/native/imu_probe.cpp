#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include "probe_common.hpp"
extern "C" {
#include "bmi160.h"
}

namespace {
int bus_fd = -1;
int8_t read_register(uint8_t address, uint8_t reg, uint8_t *data, uint16_t size) {
  i2c_msg messages[2]{};
  messages[0] = {address, 0, 1, &reg};
  messages[1] = {address, I2C_M_RD, size, data};
  i2c_rdwr_ioctl_data transaction{messages, 2};
  return ioctl(bus_fd, I2C_RDWR, &transaction) == 2 ? BMI160_OK : BMI160_E_COM_FAIL;
}
int8_t write_register(uint8_t address, uint8_t reg, uint8_t *data, uint16_t size) {
  std::vector<uint8_t> bytes(size + 1);
  bytes[0] = reg;
  if (size) std::memcpy(bytes.data() + 1, data, size);
  i2c_msg message{address, 0, static_cast<uint16_t>(bytes.size()), bytes.data()};
  i2c_rdwr_ioctl_data transaction{&message, 1};
  return ioctl(bus_fd, I2C_RDWR, &transaction) == 1 ? BMI160_OK : BMI160_E_COM_FAIL;
}
void delay_ms(uint32_t milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
void require_ok(int8_t result, const char *operation) {
  if (result != BMI160_OK) throw std::runtime_error(std::string(operation) + " 실패 (SensorAPI " + std::to_string(result) + ")");
}
struct ImuSession {
  bmi160_dev device{};
  bool initialized = false;
  explicit ImuSession(const std::string &path) {
    bus_fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (bus_fd < 0) throw std::runtime_error(path + ": " + std::strerror(errno));
  }
  bool stop() {
    if (!initialized) return true;
    initialized = false;
    device.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
    device.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
    return bmi160_set_power_mode(&device) == BMI160_OK;
  }
  ~ImuSession() {
    stop();
    if (bus_fd >= 0) close(bus_fd);
    bus_fd = -1;
  }
};
}

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "imu_probe --device /dev/i2c-1 --address auto --duration 8\n";
    return 0;
  }
  install_signals();
  try {
    const auto args = arguments(argc, argv, {"--device", "--address", "--duration"});
    const double duration = duration_argument(args);
    ImuSession session(args.at("--device"));
    const auto &address_text = args.at("--address");
    std::vector<int> candidates{0x68, 0x69};
    if (address_text != "auto") {
      const int address = std::stoi(address_text, nullptr, 0);
      if (address != 0x68 && address != 0x69) throw std::invalid_argument("BMI160 주소는 0x68 또는 0x69입니다");
      candidates = {address};
    }
    std::vector<int> found;
    for (int address : candidates) {
      // 강제 접근을 사용하지 않아 커널 드라이버가 점유한 주소는 거부한다.
      if (ioctl(bus_fd, I2C_SLAVE, address) < 0) throw std::runtime_error(std::strerror(errno));
      uint8_t chip = 0;
      if (read_register(address, 0x00, &chip, 1) == BMI160_OK && chip == 0xd1) found.push_back(address);
    }
    if (found.empty()) throw std::runtime_error("BMI160 식별값 0xd1을 읽지 못했습니다. 주소·전원·배선을 확인하세요");
    if (found.size() != 1) throw std::runtime_error("BMI160 두 개가 응답합니다. --imu-address로 하나를 지정하세요");
    const int address = found.front();
    if (ioctl(bus_fd, I2C_SLAVE, address) < 0) throw std::runtime_error(std::strerror(errno));
    auto &device = session.device;
    device.id = address;
    device.intf = BMI160_I2C_INTF;
    device.read = read_register;
    device.write = write_register;
    device.delay_ms = delay_ms;
    device.read_write_len = 32;
    require_ok(bmi160_init(&device), "BMI160 초기화");
    session.initialized = true;
    device.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    device.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    device.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    device.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    device.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    device.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
    device.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    device.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
    require_ok(bmi160_set_sens_conf(&device), "100Hz 설정");
    delay_ms(500);
    uint8_t status = 0, error = 0;
    require_ok(read_register(address, 0x03, &status, 1), "전원 상태 읽기");
    require_ok(read_register(address, 0x02, &error, 1), "오류 상태 읽기");
    if ((status & 0x3c) != 0x14 || error != 0) throw std::runtime_error("BMI160 전원 모드 또는 오류 레지스터가 비정상입니다");
    std::cout << "{\"type\":\"imu_info\",\"chip_id\":209,\"address\":" << address
              << ",\"pmu_status\":" << int(status) << "}" << std::endl;
    const double start = monotonic_seconds();
    double last_sample = start;
    unsigned count = 0;
    while (!stop_requested && monotonic_seconds() - start < duration) {
      require_ok(read_register(address, 0x1b, &status, 1), "데이터 준비 상태 읽기");
      // 읽기 루프 횟수가 아니라 가속도·자이로의 실제 새 데이터만 센다.
      if ((status & 0xc0) != 0xc0) {
        if (monotonic_seconds() - last_sample > 1.0) throw std::runtime_error("IMU 새 데이터가 1초 이상 나오지 않습니다");
        delay_ms(1);
        continue;
      }
      bmi160_sensor_data accel{}, gyro{};
      require_ok(bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO | BMI160_TIME_SEL, &accel, &gyro, &device), "측정값 읽기");
      last_sample = monotonic_seconds();
      constexpr double acceleration_scale = 9.80665 / 16384.0;
      constexpr double gyro_scale = 0.017453292519943295 / 131.2;
      std::cout << std::setprecision(12) << "{\"type\":\"imu_sample\",\"t\":" << last_sample - start
                << ",\"sensor_time\":" << accel.sensortime
                << ",\"acc\":[" << accel.x * acceleration_scale << ',' << accel.y * acceleration_scale << ',' << accel.z * acceleration_scale
                << "],\"gyro\":[" << gyro.x * gyro_scale << ',' << gyro.y * gyro_scale << ',' << gyro.z * gyro_scale
                << "],\"raw\":[" << accel.x << ',' << accel.y << ',' << accel.z << ',' << gyro.x << ',' << gyro.y << ',' << gyro.z << "]}" << std::endl;
      ++count;
    }
    const bool cleanup_ok = session.stop();
    std::cout << "{\"type\":\"cleanup\",\"ok\":" << (cleanup_ok ? "true" : "false") << "}" << std::endl;
    if (!cleanup_ok) throw std::runtime_error("IMU 측정 후 suspend 전환에 실패했습니다");
    if (stop_requested) throw std::runtime_error("검사가 중단됐습니다");
    if (!count) throw std::runtime_error("IMU 표본이 없습니다");
    return 0;
  } catch (const std::exception &error) {
    emit_error(error);
    return 1;
  }
}
