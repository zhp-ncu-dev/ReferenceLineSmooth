//
// Created by gaoyang on 6/26/19.
//

#ifndef PLANNING_CURVE_H
#define PLANNING_CURVE_H

#include <string>

namespace planning
{
class Curve1d
{
public:
    Curve1d() = default;

    virtual ~Curve1d() = default;

    virtual double Evaluate(const std::uint32_t order,
                            const double param) const = 0;

    virtual double ParamLength() const = 0;

    virtual std::string ToString() const = 0;
};
}//end namespace planning

#endif //PLANNING_CURVE_H
