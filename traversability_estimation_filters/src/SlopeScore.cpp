/*
 * SlopeScore.cpp
 */

#include "filters/SlopeScore.hpp"

#include <cmath>
#include <limits>

namespace filters {

namespace {

// grid_map's NormalVectorsFilter (area method) fits a vertical plane when all valid neighbours of a cell lie on one
// line in xy, as in the 1-cell-wide rows of a sparse elevation map: its degenerate-case check does not catch it
// because noise in z keeps the middle eigenvalue above its 1e-8 limit. The normal comes out exactly horizontal, which
// real terrain does not produce (|z| >= 0.33 on the 4F200 recordings), so it says nothing about the slope.
constexpr double kHorizontalNormalZ = 1e-6;

}  // namespace

SlopeScore::SlopeScore(double criticalValue)
    : cosCritical_(std::cos(criticalValue)),
      invCritical_(1.0 / criticalValue)
{

}

double SlopeScore::compute(double normalZ) const
{
  if (std::abs(normalZ) < kHorizontalNormalZ) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Fast path: slope >= critical value -> score = 0. Skips acos() for untraversable cells, the common case on rough
  // terrain.
  if (normalZ <= cosCritical_) {
    return 0.0;
  }

  return 1.0 - std::acos(normalZ) * invCritical_;
}

} /* namespace */
