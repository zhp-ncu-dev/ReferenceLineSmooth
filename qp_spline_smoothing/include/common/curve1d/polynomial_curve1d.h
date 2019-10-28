//
// Created by gaoyang on 6/26/19.
//

#ifndef PLANNING_POLYNOMIAL_CURVE1D_H
#define PLANNING_POLYNOMIAL_CURVE1D_H

#include "curve1d.h"

namespace planning {

class PolynomialCurve1d : public Curve1d {
public:
    PolynomialCurve1d() = default;
    virtual ~PolynomialCurve1d() = default;

protected:
    double param_ = 0.0;
};

}  // namespace planning

#endif //PLANNING_POLYNOMIAL_CURVE1D_H
