#pragma once
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

inline volatile std::sig_atomic_t stop_requested = 0;
inline void request_stop(int) { stop_requested = 1; }
inline void install_signals() {
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
}
inline double monotonic_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline std::string json_string(const std::string &value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c) << std::dec;
    else out << c;
  }
  out << '"';
  return out.str();
}
inline void emit_error(const std::exception &error) {
  std::cout << "{\"type\":\"error\",\"message\":" << json_string(error.what()) << "}" << std::endl;
}
inline std::map<std::string, std::string> arguments(int argc, char **argv,
                                                  std::initializer_list<std::string> allowed) {
  std::map<std::string, std::string> result;
  for (int i = 1; i < argc; i += 2) {
    if (i + 1 >= argc) throw std::invalid_argument("인자 값이 없습니다");
    bool valid = false;
    for (const auto &key : allowed) if (key == argv[i]) valid = true;
    if (!valid || result.count(argv[i])) throw std::invalid_argument("알 수 없거나 중복된 인자");
    result[argv[i]] = argv[i + 1];
  }
  return result;
}
inline double duration_argument(const std::map<std::string, std::string> &args) {
  const double value = std::stod(args.at("--duration"));
  if (!(value >= 3.0 && value <= 60.0)) throw std::invalid_argument("측정 시간은 3~60초입니다");
  return value;
}
