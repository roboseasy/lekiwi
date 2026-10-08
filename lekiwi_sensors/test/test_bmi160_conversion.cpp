#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>

#include "lekiwi_sensors/bmi160_conversion.hpp"

TEST(Bmi160ConversionTest, ConvertsConfiguredFullScaleEndpoints)
{
  EXPECT_NEAR(
    lekiwi_sensors::bmi160AccelerationToMetersPerSecondSquared(16384),
    lekiwi_sensors::kStandardGravity,
    1e-12);
  EXPECT_NEAR(
    lekiwi_sensors::bmi160AngularVelocityToRadiansPerSecond(1312),
    10.0 * lekiwi_sensors::kDegreesToRadians,
    1e-12);
}

TEST(Bmi160ConversionTest, AppliesSignedAxisPermutation)
{
  const auto output = lekiwi_sensors::applyAxisMap({1.0, 2.0, 3.0}, {2, -1, 3});
  EXPECT_DOUBLE_EQ(output[0], 2.0);
  EXPECT_DOUBLE_EQ(output[1], -1.0);
  EXPECT_DOUBLE_EQ(output[2], 3.0);
}

TEST(Bmi160ConversionTest, RejectsDuplicateOrOutOfRangeAxes)
{
  EXPECT_FALSE(lekiwi_sensors::isValidAxisMap({1, 1, 3}));
  EXPECT_FALSE(lekiwi_sensors::isValidAxisMap({0, 2, 3}));
  EXPECT_THROW(
    (void)lekiwi_sensors::applyAxisMap({1.0, 2.0, 3.0}, {1, -1, 3}),
    std::invalid_argument);
}
