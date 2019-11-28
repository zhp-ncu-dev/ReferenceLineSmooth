//
// Created by zhp on 19-10-29.
//

#pragma once

#include "reference_line/reference_line.h"
#include "reference_line/qp_spline_reference_line_smooth.h"
#include "common/config/reference_line_config/smooth_reference_line_config.h"

namespace planning
{
    class DiscretePointsReferenceLineSmooth
    {
    public:
        DiscretePointsReferenceLineSmooth(const FemPosDeviationSmootherConfig &config);
        ~DiscretePointsReferenceLineSmooth() = default;

        void setAnchorPoints(std::vector<AnchorPoint> &anchorPoints)
        {
            m_anchorPoints = anchorPoints;
        };

        bool smooth(const ReferenceLine &rawReferenceLine,
                    const double &deltaS,
                    ReferenceLine *const smoothedReferenceLine);

    private:
        bool femPosSmooth(
                const std::vector<std::pair<double, double>>& rawPoints2d,
                const std::vector<double>& bounds,
                std::vector<std::pair<double, double>>* ptrSmoothedPoints2d);

        void curveInterpolate(
                const std::vector<std::pair<double, double>> &discretePoint2d,
                std::vector<std::pair<double, double>> &smoothedPoint2d,
                std::vector<double> &smoothedHeadings,
                const double &deltaS);

        void normalizePoints(std::vector<std::pair<double, double>>* xyPoints);

        void deNormalizePoints(std::vector<std::pair<double, double>>* xyPoints);

        bool generateRefPointProfile(
                const std::vector<std::pair<double, double>>& xyPoints,
                const std::vector<double> &smoothedHeadings,
                ReferenceLine *const smoothedReferenceLine);

        std::vector<AnchorPoint> m_anchorPoints;

        double m_zeroX = 0.0;
        double m_zeroY = 0.0;

        FemPosDeviationSmootherConfig m_femPosDeviationSmootherConfig;
    };

}// end namespace