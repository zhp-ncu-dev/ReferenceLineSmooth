//
// Created by zhp on 20-7-23.
//

#pragma once

#include "reference_line/reference_line.h"
#include "reference_line/qp_spline_reference_line_smooth.h"
#include "common/config/reference_line_config/smooth_reference_line_config.h"

namespace planning {

class PiecewiseJerkSmooth
{
public:
   PiecewiseJerkSmooth();
   ~PiecewiseJerkSmooth() = default;

    void setAnchorPoints(std::vector<AnchorPoint> &anchorPoints)
    {
        m_anchorPoints = anchorPoints;
    };

    bool smooth(const ReferenceLine &rawReferenceLine,
                const double &deltaS,
                ReferenceLine *const smoothedReferenceLine);

private:
    void normalizePoints(std::vector<std::pair<double, double>>* xyPoints);

    void deNormalizePoints(std::vector<std::pair<double, double>>* xyPoints);

    bool smoothLine(const std::vector<std::pair<double, double>>& xyPoints,
                    ReferenceLine *const smoothedReferenceLine);


    bool optimizeSmooth(
            const std::array<double, 3>& init_state,
            const std::array<double, 3>& end_state, const double delta_s,
            const std::vector<std::pair<double, double>>& lat_boundaries,
            const std::vector<std::pair<double, double>>& ddl_bounds,
            const std::array<double, 5>& w, std::vector<double>* x,
            std::vector<double>* dx, std::vector<double>* ddx, const int max_iter);


    std::vector<AnchorPoint> m_anchorPoints;
    double m_zeroX = 0.0;
    double m_zeroY = 0.0;

};

}