#include <gtest/gtest.h>

#include <cmath>

#include "filters/SlopeScore.hpp"

using filters::SlopeScore;

namespace {

constexpr double kCriticalValue = 0.53;  // critical_value in traversability_estimation/config/robot_filter_parameter.yaml

}  // namespace

TEST(SlopeScore, FlatIsFullyTraversable)
{
  EXPECT_DOUBLE_EQ(SlopeScore(kCriticalValue).compute(1.0), 1.0);
}

TEST(SlopeScore, DecreasesLinearlyWithTheSlopeAngle)
{
  EXPECT_NEAR(SlopeScore(kCriticalValue).compute(std::cos(kCriticalValue / 2.0)), 0.5, 1e-9);
}

TEST(SlopeScore, CriticalSlopeAndSteeperAreNotTraversable)
{
  const SlopeScore score(kCriticalValue);
  EXPECT_DOUBLE_EQ(score.compute(std::cos(kCriticalValue)), 0.0);
  EXPECT_DOUBLE_EQ(score.compute(0.33), 0.0);  // steepest normal measured on real terrain
}

TEST(SlopeScore, HorizontalNormalIsUnknown)
{
  // What a plane fit returns when all neighbours lie on one line in xy: a vertical plane.
  const SlopeScore score(kCriticalValue);
  EXPECT_TRUE(std::isnan(score.compute(0.0)));
  EXPECT_TRUE(std::isnan(score.compute(1e-9)));
}
