/*
 * SlopeFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/SlopeFilter.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <cmath>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;

namespace filters {

namespace {

// Also the declared default of critical_value: getParam declares the parameter with the value passed in.
constexpr double kDefaultCriticalValue = M_PI_4;

}  // namespace

template<typename T>
SlopeFilter<T>::SlopeFilter()
    : score_(kDefaultCriticalValue),
      type_("traversability_slope")
{

}

template<typename T>
SlopeFilter<T>::~SlopeFilter()
{

}

template<typename T>
bool SlopeFilter<T>::configure()
{
  double criticalValue = kDefaultCriticalValue;
  if (!FilterBase<T>::getParam(std::string("critical_value"), criticalValue)) {
    RCLCPP_ERROR(rclcpp::get_logger("SlopeFilter"), "SlopeFilter did not find param critical_value");
    return false;
  }

  if (criticalValue > M_PI_2 || criticalValue < 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("SlopeFilter"), "Critical slope must be in the interval [0, PI/2]");
    return false;
  }

  score_ = SlopeScore(criticalValue);
  RCLCPP_DEBUG(rclcpp::get_logger("SlopeFilter"), "critical Slope = %f", criticalValue);

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    RCLCPP_ERROR(rclcpp::get_logger("SlopeFilter"), "SlopeFilter did not find param map_type");
    return false;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("SlopeFilter"), "Slope map type = %s", type_.c_str());

  return true;
}

template<typename T>
bool SlopeFilter<T>::update(const T& mapIn, T& mapOut)
{
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);

  // Look the layers up once: a per-cell lookup by name builds a heap std::string for "surface_normal_z".
  const Matrix& normalZ = mapOut["surface_normal_z"];
  Matrix& slope = mapOut[type_];

  for (GridMapIterator iterator(mapOut);
      !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    const float nz = normalZ(index(0), index(1));

    // Check if there is a surface normal (empty cell).
    if (!std::isfinite(nz)) continue;

    slope(index(0), index(1)) = score_.compute(nz);
  }

  return true;
}

// Exported so the filter can also be used directly, not only through pluginlib (e.g. in unit tests).
template class SlopeFilter<grid_map::GridMap>;

} /* namespace */

PLUGINLIB_EXPORT_CLASS(filters::SlopeFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
