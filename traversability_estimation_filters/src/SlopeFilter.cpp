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

template<typename T>
SlopeFilter<T>::SlopeFilter()
    : criticalValue_(M_PI_4),
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
  if (!FilterBase<T>::getParam(std::string("critical_value"), criticalValue_)) {
    RCLCPP_ERROR(rclcpp::get_logger("SlopeFilter"), "SlopeFilter did not find param critical_value");
    return false;
  }

  if (criticalValue_ > M_PI_2 || criticalValue_ < 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("SlopeFilter"), "Critical slope must be in the interval [0, PI/2]");
    return false;
  }

  cosCritical_ = std::cos(criticalValue_);
  invCritical_  = 1.0 / criticalValue_;
  RCLCPP_DEBUG(rclcpp::get_logger("SlopeFilter"), "critical Slope = %f", criticalValue_);

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

  for (GridMapIterator iterator(mapOut);
      !iterator.isPastEnd(); ++iterator) {

    // Check if there is a surface normal (empty cell).
    if (!mapOut.isValid(*iterator, "surface_normal_z")) continue;

    const double nz = mapOut.at("surface_normal_z", *iterator);

    // Fast path: if nz <= cos(criticalValue_), slope >= criticalValue_ -> score = 0.
    // This skips acos() for untraversable cells, which is the common case on rough terrain.
    if (nz <= cosCritical_) {
      mapOut.at(type_, *iterator) = 0.0;
      continue;
    }

    // Traversable cell: compute score using precomputed 1/criticalValue_.
    mapOut.at(type_, *iterator) = 1.0 - std::acos(nz) * invCritical_;
  }

  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(filters::SlopeFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
