// Virtual actuator only: respond without QEMU/Python scheduling latency.
// The ARM64 ROS driver and its unchanged 5 ms deadlines still run in the Pi.
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
volatile sig_atomic_t ending = 0;
void stop(int) { ending = 1; }
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;

// Disk writes must not delay serial responses with a 5 ms driver deadline.
// Keep only the latest pending snapshot; a slow disk must not build a backlog.
class SnapshotWriter {
 public:
  explicit SnapshotWriter(std::string path) : path_(std::move(path)), thread_([this] { run(); }) {}
  ~SnapshotWriter() { if (thread_.joinable()) stop(); }

  void submit(std::string snapshot) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (error_) std::rethrow_exception(error_);
      pending_ = std::move(snapshot);
    }
    changed_.notify_one();
  }

  void finish() {
    stop();
    if (error_) std::rethrow_exception(error_);
  }

  std::atomic<int64_t> max_write_us{0};

 private:
  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    changed_.notify_one();
    thread_.join();
  }

  void run() {
    try {
      while (true) {
        std::string snapshot;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          changed_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
          if (pending_.empty()) return;
          snapshot.swap(pending_);
        }
        const auto started = Clock::now();
        std::ofstream file(path_ + ".tmp");
        file << snapshot;
        file.close();
        if (!file || rename((path_ + ".tmp").c_str(), path_.c_str()))
          throw std::runtime_error("snapshot write failed");
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
          Clock::now() - started).count();
        if (elapsed > max_write_us) max_write_us = elapsed;
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = std::current_exception();
    }
  }

  std::string path_, pending_;
  std::mutex mutex_;
  std::condition_variable changed_;
  bool stopping_ = false;
  std::exception_ptr error_;
  std::thread thread_;
};
}

int main(int argc, char ** argv) {
  if (argc != 14) return 2;
  const std::string state_file = argv[1], port = argv[2];
  std::array<int, 3> ids{std::stoi(argv[3]), std::stoi(argv[4]), std::stoi(argv[5])};
  std::array<double, 3> directions{std::stod(argv[6]), std::stod(argv[7]), std::stod(argv[8])};
  const double radius = std::stod(argv[9]), base_radius = std::stod(argv[10]);
  const double scale = std::stod(argv[11]), frame_yaw = std::stod(argv[12]);
  // Geometry x/y are supplied separately as a comma-separated final argument.
  const std::string offset = argv[13];
  const double frame_x = std::stod(offset.substr(0, offset.find(',')));
  const double frame_y = std::stod(offset.substr(offset.find(',') + 1));
  std::array<double, 3> positions{}, speeds{}, pose{};
  std::array<int, 3> torque{}, modes{};
  int reads = 0, writes = 0, syncs = 0, nonzero = 0;
  int delayed_polls = 0;
  int64_t max_poll_us = 0, max_response_us = 0;
  auto response_started = Clock::now();
  double yaw_rate = 0;
  int master = posix_openpt(O_RDWR | O_NOCTTY);
  if (master < 0 || grantpt(master) || unlockpt(master)) return 3;
  int slave = open(ptsname(master), O_RDWR | O_NOCTTY);
  termios options{};
  if (slave < 0 || tcgetattr(slave, &options)) return 3;
  cfmakeraw(&options);
  if (tcsetattr(slave, TCSANOW, &options) || symlink(ptsname(master), port.c_str())) return 3;
  signal(SIGINT, stop);
  signal(SIGTERM, stop);
  SnapshotWriter snapshots(state_file);

  auto index = [&](int id) {
    for (int i = 0; i < 3; ++i) if (ids[i] == id) return i;
    throw std::runtime_error("unknown wheel ID");
  };
  auto response = [&](int id, const std::vector<uint8_t> & data) {
    std::vector<uint8_t> bytes{255, 255, static_cast<uint8_t>(id),
      static_cast<uint8_t>(data.size() + 2), 0};
    bytes.insert(bytes.end(), data.begin(), data.end());
    int sum = 0;
    for (size_t i = 2; i < bytes.size(); ++i) sum += bytes[i];
    bytes.push_back(static_cast<uint8_t>(~sum));
    if (write(master, bytes.data(), bytes.size()) != static_cast<ssize_t>(bytes.size()))
      throw std::runtime_error("PTY response write failed");
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
      Clock::now() - response_started).count();
    if (elapsed > max_response_us) max_response_us = elapsed;
  };
  auto save = [&]() {
    std::ostringstream file;
    file << "{\"pose\":[" << pose[0] << ',' << pose[1] << ',' << pose[2]
         << "],\"yaw_rate\":" << yaw_rate << ",\"torque\":{";
    for (int i = 0; i < 3; ++i) {
      if (i) file << ',';
      file << '"' << ids[i] << "\":" << (torque[i] ? "true" : "false");
    }
    file << "},\"packets\":{\"read\":" << reads << ",\"write\":" << writes
         << ",\"sync_write\":" << syncs << ",\"nonzero_speed\":" << nonzero
         << "},\"serial_timing\":{\"max_poll_wait_us\":" << max_poll_us
         << ",\"poll_waits_over_5ms\":" << delayed_polls
         << ",\"max_response_processing_us\":" << max_response_us
         << ",\"max_snapshot_write_us\":" << snapshots.max_write_us.load()
         << "},\"error\":null}\n";
    snapshots.submit(file.str());
  };

  std::vector<uint8_t> buffer;
  auto last = Clock::now(), last_report = last;
  save();
  while (!ending) {
    pollfd descriptor{master, POLLIN, 0};
    const auto poll_started = Clock::now();
    const int available = poll(&descriptor, 1, 1);
    const auto now = Clock::now();
    response_started = now;
    const auto poll_us = std::chrono::duration_cast<std::chrono::microseconds>(
      now - poll_started).count();
    if (poll_us > max_poll_us) max_poll_us = poll_us;
    if (poll_us >= 5000) {
      ++delayed_polls;
      const auto wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      std::cerr << "virtual actuator poll delay: wall_us=" << wall_us
                << " poll_us=" << poll_us << " available=" << available << '\n';
    }
    const double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    std::array<double, 3> v{};
    for (int i = 0; i < 3; ++i) {
      const double speed = torque[i] ? speeds[i] : 0;
      positions[i] += speed * dt * 4096 / (2 * pi);
      v[i] = speed / directions[i] * radius;
    }
    yaw_rate = (v[0] + v[1] + v[2]) / (3 * base_radius);
    const double mx = (v[2] - v[0]) / std::sqrt(3), my = -(v[1] - base_radius * yaw_rate);
    const double vx = std::cos(frame_yaw)*mx - std::sin(frame_yaw)*my + yaw_rate*frame_y;
    const double vy = std::sin(frame_yaw)*mx + std::cos(frame_yaw)*my - yaw_rate*frame_x;
    const double yaw = pose[2] + yaw_rate * dt / 2;
    pose[0] += (std::cos(yaw)*vx - std::sin(yaw)*vy) * dt;
    pose[1] += (std::sin(yaw)*vx + std::cos(yaw)*vy) * dt;
    pose[2] += yaw_rate * dt;
    if (available > 0) {
      std::array<uint8_t, 4096> input{};
      const auto count = read(master, input.data(), input.size());
      if (count <= 0) throw std::runtime_error("PTY read failed");
      buffer.insert(buffer.end(), input.begin(), input.begin() + count);
      while (buffer.size() >= 4 && buffer.size() >= size_t(buffer[3] + 4)) {
        const size_t length = buffer[3] + 4;
        std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + length);
        buffer.erase(buffer.begin(), buffer.begin() + length);
        int sum = 0;
        for (size_t i = 2; i < packet.size(); ++i) sum += packet[i];
        if (packet[0] != 255 || packet[1] != 255 || (sum & 255) != 255)
          throw std::runtime_error("invalid packet");
        const int instruction = packet[4], address = packet[5];
        if (instruction == 0x83) {
          if (address != 46 || packet[6] != 2) throw std::runtime_error("invalid sync write");
          ++syncs;
          for (size_t i = 7; i + 2 < packet.size() - 1; i += 3) {
            const int raw = packet[i + 1] | (packet[i + 2] << 8);
            const int value = (raw & 0x8000) ? -(raw & 0x7fff) : raw;
            speeds[index(packet[i])] = value / scale;
            nonzero += value != 0;
          }
        } else if (instruction == 3) {
          const int i = index(packet[2]);
          ++writes;
          if (address == 40) torque[i] = packet[6];
          else if (address == 33) modes[i] = packet[6];
          else throw std::runtime_error("unknown write register");
          response(packet[2], {});
        } else if (instruction == 2) {
          const int i = index(packet[2]);
          ++reads;
          int value = 0;
          if (address == 56) value = (static_cast<int>(std::llround(positions[i])) % 4096 + 4096) % 4096;
          else if (address == 40) value = torque[i];
          else if (address == 33) value = modes[i];
          else if (address == 62) value = 120;
          else throw std::runtime_error("unknown read register");
          std::vector<uint8_t> data;
          for (int byte = 0; byte < packet[6]; ++byte) data.push_back((value >> (8*byte)) & 255);
          response(packet[2], data);
        } else throw std::runtime_error("unknown instruction");
      }
    }
    if (now - last_report >= std::chrono::milliseconds(20)) {
      last_report = now;
      save();
    }
  }
  save();
  snapshots.finish();
  close(slave);
  close(master);
  unlink(port.c_str());
}
