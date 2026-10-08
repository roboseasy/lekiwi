#include <cmath>
#include <cstring>
#include <thread>
#include "CYdLidar.h"
#include "core/common/ydlidar_help.h"
#include "probe_common.hpp"

namespace {
struct LidarSession {
  CYdLidar laser;
  bool connected = false;
  bool stop() {
    if (!connected) return true;
    connected = false;
    const bool stopped = laser.turnOff();
    laser.disconnecting();
    return stopped;
  }
  ~LidarSession() { stop(); }
};
template<typename T> void option(CYdLidar &laser, int property, T value) {
  if (!laser.setlidaropt(property, &value, sizeof(value))) throw std::runtime_error("YDLIDAR 설정 실패: " + std::to_string(property));
}
}

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "lidar_probe --port /dev/serial/by-id/... --duration 8\n";
    return 0;
  }
  try {
    const auto args = arguments(argc, argv, {"--port", "--duration"});
    const double duration = duration_argument(args);
    const std::string port = args.at("--port");
    // SDK의 초기화 뒤 우리 종료 처리기를 설치해 cleanup 경로로 돌아온다.
    ydlidar::os_init();
    install_signals();
    LidarSession session;
    auto &laser = session.laser;
    if (!laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size())) throw std::runtime_error("라이다 포트 설정 실패");
    option(laser, LidarPropSerialBaudrate, 230400);
    option(laser, LidarPropLidarType, int(TYPE_TRIANGLE));
    option(laser, LidarPropDeviceType, int(YDLIDAR_TYPE_SERIAL));
    option(laser, LidarPropSampleRate, 4);
    option(laser, LidarPropIntenstiyBit, 8);
    option(laser, LidarPropAbnormalCheckCount, 4);
    option(laser, LidarPropFixedResolution, true);
    option(laser, LidarPropReversion, false);
    option(laser, LidarPropInverted, true);
    option(laser, LidarPropAutoReconnect, false);
    option(laser, LidarPropSingleChannel, false);
    option(laser, LidarPropIntenstiy, true);
    option(laser, LidarPropSupportMotorDtrCtrl, false);
    option(laser, LidarPropSupportHeartBeat, false);
    option(laser, LidarPropMaxAngle, 180.0f);
    option(laser, LidarPropMinAngle, -180.0f);
    option(laser, LidarPropMaxRange, 12.0f);
    option(laser, LidarPropMinRange, 0.05f);
    option(laser, LidarPropScanFrequency, 10.0f);
    laser.enableGlassNoise(false);
    laser.enableSunNoise(false);
    laser.setBottomPriority(true);
    session.connected = true; // 초기화 실패 뒤에도 연결을 닫는다.
    if (!laser.initialize()) throw std::runtime_error(std::string("라이다 초기화 실패: ") + laser.DescribeError());
    device_info info{};
    if (!laser.getDeviceInfo(info, EPT_Module | EPT_Base)) throw std::runtime_error("라이다 모델 식별 정보를 얻지 못했습니다");
    if (info.model != ydlidar::core::common::DriverInterface::YDLIDAR_TminiPlus &&
        info.model != ydlidar::core::common::DriverInterface::YDLIDAR_TminiPlusSH) {
      throw std::runtime_error("Tmini Plus가 아닌 모델입니다 (모델 코드 " + std::to_string(info.model) + ")");
    }
    std::ostringstream serial;
    const bool decimal_serial = std::all_of(std::begin(info.serialnum), std::end(info.serialnum), [](uint8_t digit) { return digit <= 9; });
    for (uint8_t digit : info.serialnum) {
      if (decimal_serial) serial << int(digit);
      else serial << std::setw(2) << std::setfill('0') << std::hex << int(digit);
    }
    std::cout << "{\"type\":\"lidar_info\",\"model_code\":" << int(info.model)
              << ",\"serial\":" << json_string(serial.str()) << "}" << std::endl;
    if (stop_requested) throw std::runtime_error("검사가 중단됐습니다");
    if (!laser.turnOn()) throw std::runtime_error(std::string("라이다 스캔 시작 실패: ") + laser.DescribeError());
    const double start = monotonic_seconds();
    unsigned consecutive_failures = 0;
    while (!stop_requested && monotonic_seconds() - start < duration) {
      LaserScan scan;
      if (!laser.doProcessSimple(scan)) {
        ++consecutive_failures;
        std::cout << "{\"type\":\"scan_error\"}" << std::endl;
        if (consecutive_failures >= 3) throw std::runtime_error(std::string("스캔 수신이 연속 3회 실패했습니다: ") + laser.DescribeError());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      consecutive_failures = 0;
      unsigned valid = 0;
      double nearest = 12.0, farthest = 0;
      for (const auto &point : scan.points) {
        if (std::isfinite(point.range) && std::isfinite(point.angle) && point.range >= 0.05 && point.range <= 12.0) {
          ++valid;
          nearest = std::min(nearest, double(point.range));
          farthest = std::max(farthest, double(point.range));
        }
      }
      std::cout << std::setprecision(12) << "{\"type\":\"lidar_scan\",\"t\":" << monotonic_seconds() - start
                << ",\"stamp_ns\":" << scan.stamp << ",\"points\":" << scan.points.size()
                << ",\"valid\":" << valid << ",\"min_range\":" << (valid ? nearest : 0)
                << ",\"max_range\":" << farthest << ",\"ranges\":[";
      bool first = true;
      for (const auto &point : scan.points) {
        if (!std::isfinite(point.range) || !std::isfinite(point.angle) || point.range < 0.05 || point.range > 12.0) continue;
        if (!first) std::cout << ',';
        first = false;
        std::cout << '[' << point.angle << ',' << point.range << ']';
      }
      std::cout << "]}" << std::endl;
    }
    const bool cleanup_ok = session.stop();
    std::cout << "{\"type\":\"cleanup\",\"ok\":" << (cleanup_ok ? "true" : "false") << "}" << std::endl;
    if (!cleanup_ok) throw std::runtime_error("라이다 종료 명령에 실패했습니다");
    if (stop_requested) throw std::runtime_error("검사가 중단됐습니다");
    return 0;
  } catch (const std::exception &error) {
    emit_error(error);
    return 1;
  }
}
