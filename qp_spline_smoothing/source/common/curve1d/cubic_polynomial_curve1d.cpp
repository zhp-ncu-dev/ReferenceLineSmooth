//
// Created by gaoyang on 6/26/19.
//

#include "common/curve1d/cubic_polynomial_curve1d.h"

namespace planning {

CubicPolynomialCurve1d::CubicPolynomialCurve1d(
        const std::array<double, 3>& start, const double end, const double param)
        : CubicPolynomialCurve1d(start[0], start[1], start[2], end, param) {}

CubicPolynomialCurve1d::CubicPolynomialCurve1d(const std::array<double, 2>& start, const double end,
                                               const double param)
{
    if(param <= 0)
    {
        return;
    }
    const double p2 = param * param;
    const double p3 = param * p2;
    coef_[0] = start[0];
    coef_[1] = start[1];
    coef_[2] = 3 * (end - coef_[0]) / p2 - 2 * coef_[1] / param;
    coef_[3] = (coef_[1] * (param) - 2 * (end - coef_[0])) / p3;
}

CubicPolynomialCurve1d::CubicPolynomialCurve1d(const double x0,
                                               const double dx0,
                                               const double ddx0,
                                               const double x1,
                                               const double param) {
    ComputeCoefficients(x0, dx0, ddx0, x1, param);
    param_ = param;
    start_condition_[0] = x0;
    start_condition_[1] = dx0;
    start_condition_[2] = ddx0;
    end_condition_ = x1;
}

CubicPolynomialCurve1d::CubicPolynomialCurve1d(
        const CubicPolynomialCurve1d& other) {
    param_ = other.param_;
    start_condition_ = other.start_condition_;
    end_condition_ = other.end_condition_;
}

double CubicPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                        const double p) const {
    switch (order) {
        case 0: {
            return ((coef_[3] * p + coef_[2]) * p + coef_[1]) * p + coef_[0];
        }
        case 1: {
            return (3.0 * coef_[3] * p + 2.0 * coef_[2]) * p + coef_[1];
        }
        case 2: {
            return 6.0 * coef_[3] * p + 2.0 * coef_[2];
        }
        case 3: {
            return 6.0 * coef_[3];
        }
        default:
            return 0.0;
    }
}

std::string CubicPolynomialCurve1d::ToString() const {
    std::string s;
    for(auto &coe : coef_)
    {
        s += std::to_string(coe);
        s += " ";
    }
    return s;
}

void CubicPolynomialCurve1d::ComputeCoefficients(const double x0,
                                                 const double dx0,
                                                 const double ddx0,
                                                 const double x1,
                                                 const double param) {
    if(param <= 0)
    {
        return;
    }
    const double p2 = param * param;
    const double p3 = param * p2;
    coef_[0] = x0;
    coef_[1] = dx0;
    coef_[2] = 0.5 * ddx0;
    coef_[3] = (x1 - coef_[0] - coef_[1] * param - coef_[2] * param * param) / p3;
}

}  // namespace planning
