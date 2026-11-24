#include <gtest/gtest.h>

#include <cmath>

#include <grid_map_core/grid_map_core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "filters/SlopeFilter.hpp"

namespace {

bool configureWithCriticalValue(double criticalValue)
{
  auto node = std::make_shared<rclcpp::Node>(
      "slope_filter_test",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("slope.critical_value", criticalValue)}));
  filters::SlopeFilter<grid_map::GridMap> filter;
  filters::FilterBase<grid_map::GridMap>& base = filter;  // SlopeFilter::configure() hides this overload
  return base.configure("slope", "slope", node->get_node_logging_interface(), node->get_node_parameters_interface());
}

}  // namespace

TEST(SlopeFilter, RejectsAZeroCriticalSlope)
{
  rclcpp::init(0, nullptr);
  EXPECT_TRUE(configureWithCriticalValue(M_PI_4));
  EXPECT_FALSE(configureWithCriticalValue(0.0));  // SlopeScore would divide by zero
  rclcpp::shutdown();
}

TEST(SlopeFilter, HorizontalNormalLeavesTheCellUnknown)
{
  grid_map::GridMap map({"surface_normal_z"});
  map.setGeometry(grid_map::Length(0.3, 0.1), 0.1);  // 3 x 1 cells
  map.at("surface_normal_z", grid_map::Index(0, 0)) = 1.0;  // flat
  map.at("surface_normal_z", grid_map::Index(1, 0)) = 0.0;  // vertical plane fit through collinear neighbours
  map.at("surface_normal_z", grid_map::Index(2, 0)) = 0.5;  // 60 deg: steeper than the default 45 deg critical slope

  filters::SlopeFilter<grid_map::GridMap> filter;  // defaults: critical slope PI/4, layer traversability_slope
  grid_map::GridMap out;
  ASSERT_TRUE(filter.update(map, out));

  EXPECT_FLOAT_EQ(out.at("traversability_slope", grid_map::Index(0, 0)), 1.0);
  EXPECT_TRUE(std::isnan(out.at("traversability_slope", grid_map::Index(1, 0))));
  EXPECT_FLOAT_EQ(out.at("traversability_slope", grid_map::Index(2, 0)), 0.0);
}
