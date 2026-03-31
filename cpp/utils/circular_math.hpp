#pragma once

#include <cmath>

/**
 * Compute the circular (angular) distance from low to high on a 2π circle.
 * Returns a value in [0, 2π).
 */
inline double circularDistance(double high, double low) {
  return low < high ? high - low : 2 * M_PI + high - low;
}
