#ifndef LEKIWI_SENSORS__BMI160_CONVERSION_HPP_
#define LEKIWI_SENSORS__BMI160_CONVERSION_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace lekiwi_sensors
{

constexpr double kStandardGravity = 9.80665;
constexpr double kDegreesToRadians = 0.017453292519943295769;
constexpr double kBmi160AccelLsbPerG2Range = 16384.0;
constexpr double kBmi160GyroLsbPerDegreePerSecond250Range = 131.2;

inline double bmi160AccelerationToMetersPerSecondSquared(const int16_t raw)
{
  return static_cast<double>(raw) * kStandardGravity / kBmi160AccelLsbPerG2Range;
}

inline double bmi160AngularVelocityToRadiansPerSecond(const int16_t raw)
{
  return static_cast<double>(raw) /
         kBmi160GyroLsbPerDegreePerSecond250Range * kDegreesToRadians;
}

inline bool isValidAxisMap(const std::array<int, 3> & axis_map)
{
  std::array<bool, 3> used{false, false, false};
  for (const int entry : axis_map) {
    const int axis = std::abs(entry);
    if (axis < 1 || axis > 3 || used[static_cast<std::size_t>(axis - 1)]) {
      return false;
    }
    used[static_cast<std::size_t>(axis - 1)] = true;
  }
  return true;
}

inline std::array<double, 3> applyAxisMap(
  const std::array<double, 3> & sensor_vector,
  const std::array<int, 3> & axis_map)
{
  if (!isValidAxisMap(axis_map)) {
    throw std::invalid_argument("axis_map must be a signed permutation of [1, 2, 3]");
  }

  std::array<double, 3> output{};
  for (std::size_t i = 0; i < output.size(); ++i) {
    const int entry = axis_map[i];
    const std::size_t source = static_cast<std::size_t>(std::abs(entry) - 1);
    output[i] = (entry < 0 ? -1.0 : 1.0) * sensor_vector[source];
  }
  return output;
}

}  // namespace lekiwi_sensors

#endif  // LEKIWI_SENSORS__BMI160_CONVERSION_HPP_
