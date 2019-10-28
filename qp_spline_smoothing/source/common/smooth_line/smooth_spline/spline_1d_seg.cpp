//
// Created by wangzihua on 19-8-5.
//

#include "common/smooth_line/smooth_spline/spline_1d_seg.h"

namespace planning{
    Spline1dSeg::Spline1dSeg(const uint32_t order) {
        SetSplineFunc(PolynomialXd(order));
    }

    Spline1dSeg::Spline1dSeg(const std::vector<double>& params) {
        SetSplineFunc(PolynomialXd(params));
    }

    void Spline1dSeg::SetParams(const std::vector<double>& params) {
        SetSplineFunc(PolynomialXd(params));
    }

    void Spline1dSeg::SetSplineFunc(const PolynomialXd& spline_func) {
        spline_func_ = spline_func;
        derivative_ = PolynomialXd::DerivedFrom(spline_func_);
        second_order_derivative_ = PolynomialXd::DerivedFrom(derivative_);
        third_order_derivative_ = PolynomialXd::DerivedFrom(second_order_derivative_);
    }

    double Spline1dSeg::operator()(const double x) const { return spline_func_(x); }

    double Spline1dSeg::Derivative(const double x) const { return derivative_(x); }

    double Spline1dSeg::SecondOrderDerivative(const double x) const {
        return second_order_derivative_(x);
    }

    double Spline1dSeg::ThirdOrderDerivative(const double x) const {
        return third_order_derivative_(x);
    }

    const PolynomialXd& Spline1dSeg::spline_func() const { return spline_func_; }

    const PolynomialXd& Spline1dSeg::Derivative() const { return derivative_; }

    const PolynomialXd& Spline1dSeg::SecondOrderDerivative() const {
        return second_order_derivative_;
    }

    const PolynomialXd& Spline1dSeg::ThirdOrderDerivative() const {
        return third_order_derivative_;
    }
}