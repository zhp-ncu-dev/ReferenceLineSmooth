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

        bool smooth(const ReferenceLine &rawReferenceLine,
                    const double &deltaS,
                    ReferenceLine *const smoothedReferenceLine,
                    bool flag,
                    const std::vector<std::vector<AnchorPoint>> &isnotUTurnPoints);

    private:
        bool femPosSmooth(
                const std::vector<std::pair<double, double>>& rawPoints2d,
                const std::vector<double>& bounds,
                std::vector<std::pair<double, double>>* ptrSmoothedPoints2d);

        void curveInterpolate(
                const std::vector<std::pair<double, double>> &discretePoint2d,
                std::vector<std::pair<double, double>> &smoothedPoint2d,
                const double &deltaS);

        void linearInterpolate(
                const std::pair<double, double> &startPoint,
                const std::pair<double, double> &endPoint,
                const double &startHeading,
                const double &endHeading,
                const double &startKappa,
                const double &endKappa,
                const double &startDkappa,
                const double &endDkappa,
                const double &deltaS,
                std::vector<std::pair<double, double>> &points2d,
                std::vector<double> &headings,
                std::vector<double> &kappas,
                std::vector<double> &dkappas);

        void normalizePoints(std::vector<std::pair<double, double>>* xyPoints);

        void deNormalizePoints(std::vector<std::pair<double, double>>* xyPoints);

        bool generateRefPointProfile(
                const std::vector<std::pair<double, double>>& xyPoints,
                ReferenceLine *const smoothedReferenceLine);

        bool generateRefPointProfile(
                const std::vector<std::vector<std::pair<double, double>>> &smoothedPoints2dVec,
                const double &deltaS,
                ReferenceLine *const smoothedReferenceLine);

        std::vector<AnchorPoint> m_anchorPoints;

        double m_zeroX = 0.0;
        double m_zeroY = 0.0;

        FemPosDeviationSmootherConfig m_femPosDeviationSmootherConfig;
    };

}// end namespace