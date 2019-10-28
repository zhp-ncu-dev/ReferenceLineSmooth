#include "common/math/linear_interpolation.h"

#include <cmath>


namespace ADC {
namespace common {
namespace math {

extern double NormalizeAngle(const double angle);

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
    double kMathEpsilon = 1.0e-10;
    if (std::abs(t1 - t0) <= kMathEpsilon) {

        return NormalizeAngle(a0);
    }
    const double a0_n = NormalizeAngle(a0);
    const double a1_n = NormalizeAngle(a1);
    double d = a1_n - a0_n;
    if (d > M_PI) {
        d = d - 2 * M_PI;
    } else if (d < -M_PI) {
        d = d + 2 * M_PI;
    }

    const double r = (t - t0) / (t1 - t0);
    const double a = a0_n + d * r;
    return NormalizeAngle(a);
}

double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

}  // namespace smooth
}  // namespace common
}  // namespace ADC
