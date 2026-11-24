/*
 * SlopeScore.hpp
 *
 * Slope traversability of a single cell, computed from its surface normal.
 */

#ifndef SLOPESCORE_HPP
#define SLOPESCORE_HPP

namespace filters {

/*!
 * Maps the z component of a unit surface normal (pointing up) to a slope traversability value:
 * 1.0 is flat, 0.0 is a slope at or above the critical value, NAN is unknown.
 */
class SlopeScore
{

 public:
  /*!
   * Constructor.
   * @param criticalValue maximum traversable slope [rad], in (0, PI/2].
   */
  explicit SlopeScore(double criticalValue);

  /*!
   * @param normalZ z component of the unit surface normal.
   * @return traversability in [0.0, 1.0], or NAN when the normal carries no slope information.
   */
  double compute(double normalZ) const;

 private:

  //! Precomputed cos(criticalValue): normals with a z component at or below it score 0.0 without calling acos().
  double cosCritical_;

  //! Precomputed 1.0 / criticalValue: replaces a per-cell division in the score formula.
  double invCritical_;
};

} /* namespace */

#endif
