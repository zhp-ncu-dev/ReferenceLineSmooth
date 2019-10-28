//
// Created by gaoyang on 6/26/19.
//

#ifndef PLANNING_QUINTIC_POLYNOMIAL_CURVE1D_H
#define PLANNING_QUINTIC_POLYNOMIAL_CURVE1D_H

#include <array>
#include <string>

#include "polynomial_curve1d.h"

namespace planning {

// 1D quintic polynomial curve:
// (x0, dx0, ddx0) -- [0, param] --> (x1, dx1, ddx1)
class QuinticPolynomialCurve1d : public PolynomialCurve1d {
public:
    QuinticPolynomialCurve1d() = default;

    QuinticPolynomialCurve1d(const std::array<double, 3>& start,
                             const std::array<double, 3>& end,
                             const double param);

    QuinticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                             const double x1, const double dx1, const double ddx1,
                             const double param);

    QuinticPolynomialCurve1d(const QuinticPolynomialCurve1d& other);

    virtual ~QuinticPolynomialCurve1d() = default;

    double Evaluate(const std::uint32_t order, const double p) const override;

    double ParamLength() const { return param_; }
    std::string ToString() const override;

protected:
    void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                             const double x1, const double dx1, const double ddx1,
                             const double param);

    // f = sum(coef_[i] * x^i), i from 0 to 5
    std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> start_condition_{{0.0, 0.0, 0.0}};
    std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};
};

}  // namespace planning

#endif //PLANNING_QUINTIC_POLYNOMIAL_CURVE1D_H
