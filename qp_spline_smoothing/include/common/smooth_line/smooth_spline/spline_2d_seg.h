//
// Created by gaoyang on 18-12-3.
//

#ifndef PLANNING_SPLINE_2D_SEG_H
#define PLANNING_SPLINE_2D_SEG_H

#include "common/polynomial_xd/polynomial_xd.h"

namespace planning {

    class Spline2dSeg {
    public:
        // order represent the number of parameters (not the highest order);
        explicit Spline2dSeg(const uint32_t order);
        explicit Spline2dSeg(const std::vector<double>& x_param,
                             const std::vector<double>& y_param);
        ~Spline2dSeg() = default;

        bool SetParams(const std::vector<double>& x_param,
                       const std::vector<double>& y_param);

        std::pair<double, double> operator()(const double t) const;
        double x(const double t) const;
        double y(const double t) const;
        double DerivativeX(const double t) const;
        double DerivativeY(const double t) const;
        double SecondDerivativeX(const double t) const;
        double SecondDerivativeY(const double t) const;
        double ThirdDerivativeX(const double t) const;
        double ThirdDerivativeY(const double t) const;

        const PolynomialXd& spline_func_x() const;
        const PolynomialXd& spline_func_y() const;
        const PolynomialXd& DerivativeX() const;
        const PolynomialXd& DerivativeY() const;
        const PolynomialXd& SecondDerivativeX() const;
        const PolynomialXd& SecondDerivativeY() const;
        const PolynomialXd& ThirdDerivativeX() const;
        const PolynomialXd& ThirdDerivativeY() const;

    private:
        PolynomialXd spline_func_x_;
        PolynomialXd spline_func_y_;
        PolynomialXd derivative_x_;
        PolynomialXd derivative_y_;
        PolynomialXd second_derivative_x_;
        PolynomialXd second_derivative_y_;
        PolynomialXd third_derivative_x_;
        PolynomialXd third_derivative_y_;
    };

}
#endif //PLANNING_SPLINE_2D_SEG_H
