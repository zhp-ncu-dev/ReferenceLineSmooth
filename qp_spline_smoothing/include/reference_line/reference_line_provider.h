//
// Created by gaoyang on 18-11-23.
//

#ifndef PLANNING_REFERENCE_LINE_PROVIDER_H
#define PLANNING_REFERENCE_LINE_PROVIDER_H

#include <list>
#include "qp_spline_reference_line_smooth.h"

namespace planning
{
class ReferenceLineProvide
{
    public:
        ReferenceLineProvide() = default;
        ~ReferenceLineProvide() = default;

        bool smoothReferenceLine(const ReferenceLine &raw_reference_line, ReferenceLine *reference_line,
                                 const double &deltaS, bool dif_time_smooth);

    private:
        AnchorPoint GetAnchorPoint(
                const ReferenceLine& reference_line,
                double s,
                const std::vector<double> &rawReferenceLineKappas) const;

        void GetAnchorPoints(
                const ReferenceLine &reference_line,
                std::vector<AnchorPoint> *anchor_points,
                const std::vector<double> &rawReferenceLineKappas);

        void caculateRawReferenceLineKappas(
                const ReferenceLine &raw_reference_line,
                std::vector<double> &rawReferenceLineKappas);

        std::vector<double> const *m_rawReferenceLineKappas;
    };
}

#endif //PLANNING_REFERENCE_LINE_PROVIDER_H
