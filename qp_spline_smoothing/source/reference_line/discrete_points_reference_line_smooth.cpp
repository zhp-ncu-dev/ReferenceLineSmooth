//
// Created by zhp on 19-10-29.
//
#include "common/smooth_line/discreted_points_smooth/fem_pos_deviation_smoother.h"
#include "common/smooth_line/discreted_points_smooth/fem_pos_deviation_sqp_osqp_interface.h"
#include "reference_line/discrete_points_reference_line_smooth.h"
#include "common/smooth_line/discreted_points_smooth/discrete_points_math.h"
#include "common/curve1d/cubic_spline.h"
#include "common/math/math_utils.h"

#include "matplotlib_cpp/matplotlib_cpp.h"
namespace plt = matplotlibcpp;

namespace planning
{
    DiscretePointsReferenceLineSmooth::DiscretePointsReferenceLineSmooth(
            const FemPosDeviationSmootherConfig &config)
    {
        m_femPosDeviationSmootherConfig = config;
    }

    bool DiscretePointsReferenceLineSmooth::smooth(
            const planning::ReferenceLine &rawReferenceLine,
            const double &deltaS,
            planning::ReferenceLine *const smoothedReferenceLine)
    {
        std::vector<std::pair<double, double>> rawPoints2d;
        std::vector<double> anchorPointsLateralBounds;

        for(const auto &anchorPoint : m_anchorPoints)
        {
            rawPoints2d.emplace_back(anchorPoint.pointInfo.x(),
                    anchorPoint.pointInfo.y());
            anchorPointsLateralBounds.emplace_back(anchorPoint.lateral_bound);
        }
        anchorPointsLateralBounds.front() = 0.0;
        anchorPointsLateralBounds.back() = 0.0;

        normalizePoints(&rawPoints2d);

        std::vector<std::pair<double, double>> smoothedPoints2d;
        if(!femPosSmooth(rawPoints2d, anchorPointsLateralBounds, &smoothedPoints2d))
        {
            std::cout << "discrete-points algorithm failed !!!" << std::endl;
            return false;
        }
        deNormalizePoints(&smoothedPoints2d);

        std::vector<std::pair<double, double>> points;
        curveInterpolate(smoothedPoints2d, points, deltaS);
        generateRefPointProfile(points, smoothedReferenceLine);

        return true;
    }

    void DiscretePointsReferenceLineSmooth::normalizePoints(
            std::vector<std::pair<double, double>> *xyPoints)
    {
        m_zeroX = xyPoints->front().first;
        m_zeroY = xyPoints->front().second;
        std::for_each(xyPoints->begin(), xyPoints->end(),
                [this](std::pair<double, double>& point)
                {
                    auto currentX = point.first;
                    auto currentY = point.second;
                    std::pair<double, double> xy(currentX - m_zeroX,
                                                 currentY - m_zeroY);
                    point = std::move(xy);
                });
    }

    void DiscretePointsReferenceLineSmooth::deNormalizePoints(
            std::vector<std::pair<double, double>> *xyPoints)
    {
        std::for_each(xyPoints->begin(), xyPoints->end(),
                      [this](std::pair<double, double>& point)
                      {
                          auto currentX = point.first;
                          auto currentY = point.second;
                          std::pair<double, double> xy(currentX + m_zeroX,
                                                       currentY + m_zeroY);
                          point = std::move(xy);
                      });
    }

    void DiscretePointsReferenceLineSmooth::curveInterpolate(
            const std::vector<std::pair<double, double>> &discretePoint2d,
            std::vector<std::pair<double, double>> &smoothedPoint2d,
            const double &deltaS)
    {
        std::vector<double> accumulateS, x, y;
        double disSum = 0.0;
        for (size_t i = 0; i < discretePoint2d.size(); ++i)
        {
            if(i == 0)
            {
                accumulateS.emplace_back(disSum);
            }
            else
            {
                disSum += sqrt(
                        (discretePoint2d[i].first - discretePoint2d[i-1].first) *
                        (discretePoint2d[i].first - discretePoint2d[i-1].first) +
                        (discretePoint2d[i].second - discretePoint2d[i-1].second) *
                        (discretePoint2d[i].second - discretePoint2d[i-1].second));
                accumulateS.emplace_back(disSum);
            }
            x.emplace_back(discretePoint2d[i].first);
            y.emplace_back(discretePoint2d[i].second);
        }

        tk::spline cubicSX, cubicSY;
        cubicSX.set_points(accumulateS, x);
        cubicSY.set_points(accumulateS, y);

        double totalS = accumulateS.back() - accumulateS.front();
        for(double s = 0.0; s < totalS; s += deltaS)
        {
            smoothedPoint2d.emplace_back(std::make_pair(cubicSX(s), cubicSY(s)));
        }
    }

    bool DiscretePointsReferenceLineSmooth::femPosSmooth(
            const std::vector<std::pair<double, double>> &rawPoints2d,
            const std::vector<double> &bounds,
            std::vector<std::pair<double, double>> *ptrSmoothedPoints2d)
    {
        FemPosDeviationSmoother smoother(m_femPosDeviationSmootherConfig);

        // box constrains on pos are used in FemPosDeviationSmoother,
        // thus shrink the bounds by 1.0 / sqrt(2.0)
        std::vector<double> boxBounds = bounds;
        const double boxRatio = 1.0 / sqrt(2.0);
        for(auto &bound : boxBounds)
        {
            bound *= boxRatio;
        }

        std::vector<double> optX;
        std::vector<double> optY;
        if(!smoother.Solve(rawPoints2d, boxBounds, &optX, &optY))
        {
            std::cout << "FemPosDeviationSmoother failed !!!" << std::endl;
            return false;
        }

        if(optX.size() < 2 || optY.size() < 2)
        {
            std::cout << "return by FemPosDeviationSmoother is wrong, size smaller than 2." << std::endl;
            return false;
        }

        for(size_t i = 0; i < optX.size(); ++i)
        {
            ptrSmoothedPoints2d->emplace_back(optX[i], optY[i]);
        }

        return true;
    }

    // todo： 增加车道属性
    bool DiscretePointsReferenceLineSmooth::generateRefPointProfile(
            const std::vector<std::pair<double, double>> &xyPoints,
            ReferenceLine *const smoothedReferenceLine)
    {
        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<double> accumulatedS;
        if (!DiscretePointsMath::ComputePathProfile(
                xyPoints, &headings, &accumulatedS, &kappas, &dkappas))
        {
            return false;
        }

        // heading 转换:
        // todo: 平滑后的 heading 与 原始值相差了 1°, 待解决
        for(size_t i = 0; i < headings.size(); ++i)
        {
            double angle = -headings[i] - degreeToRadian(90);
            double a = std::fmod(angle + M_PI, 2.0 * M_PI);
            if(a < 0.0)
            {
                a = a + 2.0 * M_PI;
            }
            headings[i] = a;
        }

        std::vector<ReferencePoint> referencePoints;
        size_t pointsSize = xyPoints.size();
        for(size_t i = 0; i < pointsSize; ++i)
        {
            had_map::MapPoint point_info(Vec2d{xyPoints[i].first, xyPoints[i].second}, headings[i]);
            referencePoints.emplace_back(point_info, kappas[i], dkappas[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        *smoothedReferenceLine = ReferenceLine(referencePoints, accumulatedS);
        return true;
    }

}// end namespace
