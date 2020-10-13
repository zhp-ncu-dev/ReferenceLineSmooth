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

        rawPoints2d.clear();
        anchorPointsLateralBounds.clear();
        for(float index = 0.0; index < 50.0; index = index + 1.0) {
            float y = 0.0;
            if(index >= 30.0 && index < 40.0) {
                y = 10.0;
            }
            rawPoints2d.emplace_back(index, y);
            anchorPointsLateralBounds.emplace_back(40.0);
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


        std::vector<double> x, y, originX, originY;
        for (const auto &point : smoothedPoints2d)
        {
            x.emplace_back(point.first);
            y.emplace_back(point.second);
        }
        for(const auto& point : rawPoints2d)
        {
            originX.emplace_back(point.first);
            originY.emplace_back(point.second);
        }

        plt::plot(x, y, "r-");
        plt::plot(originX, originY, "b-");

        plt::grid("True");
        plt::axis("equal");
        plt::xlabel("x");
        plt::ylabel("y");


//        std::vector<std::pair<double, double>> points;
//        std::vector<double> smoothedHeadings;
//        curveInterpolate(smoothedPoints2d, points, smoothedHeadings, deltaS);
//        generateRefPointProfile(points, smoothedHeadings, smoothedReferenceLine);

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
            std::vector<double> &smoothedHeadings,
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
            smoothedHeadings.emplace_back(std::atan2(cubicSY.deriv(1, s), cubicSX.deriv(1, s)));
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
            const std::vector<double> &smoothedHeadings,
            ReferenceLine *const smoothedReferenceLine)
    {
        /*
        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<double> accumulatedS;
        if (!DiscretePointsMath::ComputePathProfile(
                xyPoints, &headings, &accumulatedS, &kappas, &dkappas))
        {
            return false;
        }
        */

///////////////////////////////////////////////////////////////////////////////////////////////
        std::vector<double> headings = smoothedHeadings;
        std::vector<double> accumulatedS;
        std::vector<double> kappas;
        std::vector<double> dkappas;

        // accumulated_s calculation
        double distance = 0.0;
        accumulatedS.emplace_back(distance);
        double fx = xyPoints[0].first;
        double fy = xyPoints[0].second;
        double nx = 0.0;
        double ny = 0.0;
        size_t pointSize = xyPoints.size();
        for (size_t i = 1; i < pointSize; ++i)
        {
            nx = xyPoints[i].first;
            ny = xyPoints[i].second;
            double end_segment_s =
                    std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
            accumulatedS.emplace_back(end_segment_s + distance);
            distance += end_segment_s;
            fx = nx;
            fy = ny;
        }

        // Kappa calculation
        double kappa = 0.0;
        double kappaLast = 0.0;
        for(size_t i = 0; i < pointSize; ++i)
        {
            double deltaAngle = 0.0;
            double deltaS = 0.0;

            if(i == 0)
            {
                deltaAngle = headings.at(i+1) - headings.at(i);
                deltaS = accumulatedS.at(i+1) - accumulatedS.at(i);
            }
            else if(i == pointSize - 1)
            {
                deltaAngle = headings.at(i) - headings.at(i-1);
                deltaS = accumulatedS.at(i) - accumulatedS.at(i-1);
            }
            else
            {
                deltaAngle = headings.at(i+1) - headings.at(i-1);
                deltaS = accumulatedS.at(i+1) - accumulatedS.at(i-1);
            }

            kappa = deltaAngle / (deltaS + 1.0e-6);
            if(i > 5)
            {
                if(std::abs(kappa - kappaLast) > 0.1 && std::abs(kappaLast) < 0.5)
                {
                    kappa = kappaLast;
                }
            }
            kappaLast = kappa;
            kappas.emplace_back(kappa);
        }

        // Dkappa calculation
        for (std::size_t i = 0; i < pointSize; ++i)
        {
            double dkappa = 0.0;
            if (i == 0)
            {
                dkappa = (kappas.at(i + 1) - kappas.at(i)) /
                         (accumulatedS.at(i + 1) - accumulatedS.at(i));
            }
            else if (i == pointSize - 1)
            {
                dkappa = (kappas.at(i) - kappas.at(i - 1)) /
                         (accumulatedS.at(i) - accumulatedS.at(i - 1));
            }
            else
            {
                dkappa = (kappas.at(i + 1) - kappas.at(i - 1)) /
                         (accumulatedS.at(i + 1) - accumulatedS.at(i - 1));
            }
            dkappas.emplace_back(dkappa);
        }

///////////////////////////////////////////////////////////////////////////////////////////////
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
