#pragma once

#include <cmath>

namespace ADC {
namespace common {
namespace math {

/**
* @brief Linear interpolation between two points of type T.
* @param x0 The coordinate of the first point.
* @param t0 The interpolation parameter of the first point.
* @param x1 The coordinate of the second point.
* @param t1 The interpolation parameter of the second point.
* @param t The interpolation parameter for interpolation.
* @param x The coordinate of the interpolated point.
* @return Interpolated point.
*/
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
    if (std::abs(t1 - t0) <= 1.0e-6) {
        return x0;
    }
    const double r = (t - t0) / (t1 - t0);
    const T x = x0 + r * (x1 - x0);
    return x;
}

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t);


}  // namespace smooth
}  // namespace common
}  // namespace apollo
