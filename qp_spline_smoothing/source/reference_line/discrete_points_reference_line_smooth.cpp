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

    bool DiscretePointsReferenceLineSmooth::smooth(
            const planning::ReferenceLine &rawReferenceLine,
            const double &deltaS,
            planning::ReferenceLine *const smoothedReferenceLine,
            bool flag,
            const std::vector<std::vector<AnchorPoint>> &isnotUTurnPoints)
    {
        if(!flag)
        {
            return smooth(rawReferenceLine, deltaS, smoothedReferenceLine);
        }
        else
        {
            if(!isnotUTurnPoints.empty())
            {
                FemPosDeviationSmootherConfig config;
                std::vector<std::vector<std::pair<double, double>>> smoothedPoints2dVec;
                for(const auto &uTurnPoint : isnotUTurnPoints)
                {
                    bool flag = false;
                    std::vector<std::pair<double, double>> rawPoints2d;
                    std::vector<double> anchorPointsLateralBounds;
                    for(const auto &anchorPoint : uTurnPoint)
                    {
                        rawPoints2d.emplace_back(anchorPoint.pointInfo.x(), anchorPoint.pointInfo.y());
                        anchorPointsLateralBounds.emplace_back(anchorPoint.lateral_bound);
                        if(anchorPoint.lateral_bound == config.uTurnLateralBoundaryBound)
                        {
                            flag = true;
                        }
                    }
                    anchorPointsLateralBounds.front() = 0.0;
                    anchorPointsLateralBounds.back() = 0.0;
                    std::vector<std::pair<double, double>> smoothedPoints2d;
                    if(!flag)
                    {
                        normalizePoints(&rawPoints2d);
                        if(!femPosSmooth(rawPoints2d, anchorPointsLateralBounds, &smoothedPoints2d))
                        {
                            smoothedPoints2d = rawPoints2d;
                        }
                        deNormalizePoints(&smoothedPoints2d);
                        std::vector<std::pair<double, double>> points;
                        curveInterpolate(smoothedPoints2d, points, deltaS);
                        smoothedPoints2dVec.emplace_back(points);
                    }
                    else
                    {
                        normalizePoints(&rawPoints2d);
                        if(!femPosSmooth(rawPoints2d, anchorPointsLateralBounds, &smoothedPoints2d))
                        {
                            smoothedPoints2d = rawPoints2d;
                        }
                        deNormalizePoints(&smoothedPoints2d);
                        std::vector<std::pair<double, double>> points;
                        curveInterpolate(smoothedPoints2d, points, deltaS);
                        smoothedPoints2dVec.emplace_back(points);
                    }
                }
                generateRefPointProfile(smoothedPoints2dVec, deltaS, smoothedReferenceLine);
                return true;
            }
        }
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

    void DiscretePointsReferenceLineSmooth::linearInterpolate(
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
            std::vector<double> &dkappas)
    {
        double s = std::sqrt((endPoint.first - startPoint.first) * (endPoint.first - startPoint.first) +
                             (endPoint.second - startPoint.second) * (endPoint.second - startPoint.second));

        int numPoints = static_cast<int>(s / deltaS + 0.01);
        if(numPoints >= 3)
        {
            double deltaX = endPoint.first - startPoint.first;
            double deltaY = endPoint.second - startPoint.second;
            for(int i = 1; i < numPoints - 1; ++i)
            {
                points2d.emplace_back(startPoint.first + i * deltaX / numPoints, startPoint.second + i * deltaY / numPoints);
                headings.emplace_back(startHeading + i * (endHeading - startHeading) / numPoints);
                kappas.emplace_back(startKappa + i * (endKappa - startKappa) / numPoints);
                dkappas.emplace_back(startDkappa + i * (endDkappa - startDkappa) / numPoints);
            }
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

    bool DiscretePointsReferenceLineSmooth::generateRefPointProfile(
            const std::vector<std::vector<std::pair<double, double>>> &smoothedPoints2dVec,
            const double &deltaS,
            planning::ReferenceLine *const smoothedReferenceLine)
    {
        /*
        std::vector<std::pair<double, double>> allXYPoints;
        for(const auto &xyPoints : smoothedPoints2dVec)
        {
            size_t num = xyPoints.size();
            for(size_t i = 0; i < num; ++i)
            {
                allXYPoints.emplace_back(xyPoints[i]);
            }
        }

        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<double> accumulatedS;
        if(!DiscretePointsMath::ComputePathProfile(
                allXYPoints, &headings, &accumulatedS, &kappas, &dkappas))
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
        size_t pointsSize = allXYPoints.size();
        for(size_t i = 0; i < pointsSize; ++i)
        {
            had_map::MapPoint point_info(Vec2d{allXYPoints[i].first, allXYPoints[i].second}, headings[i]);
            referencePoints.emplace_back(point_info, kappas[i], dkappas[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        *smoothedReferenceLine = ReferenceLine(referencePoints, accumulatedS);

        return true;
*/

        std::vector<std::vector<std::pair<double, double>>> xyPointsVec;
        std::vector<std::vector<double>> headingsVec;
        std::vector<std::vector<double>> kappasVec;
        std::vector<std::vector<double>> dkappasVec;
        for(const auto &xyPoints : smoothedPoints2dVec)
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
            xyPointsVec.emplace_back(xyPoints);
            headingsVec.emplace_back(headings);
            kappasVec.emplace_back(kappas);
            dkappasVec.emplace_back(dkappas);
        }

        std::vector<std::pair<double, double>> allXYPoints;
        std::vector<double> allHeadings;
        std::vector<double> allKappas;
        std::vector<double> allDkappas;
        std::vector<double> allAccumulatedS;
        size_t numXYPointsVec = xyPointsVec.size();
        for(size_t i = 0; i < numXYPointsVec; ++i)
        {
            const std::vector<std::pair<double, double>> xyPoints = xyPointsVec[i];
            const std::vector<double> headings = headingsVec[i];
            const std::vector<double> kappas = kappasVec[i];
            const std::vector<double> dkappas = dkappasVec[i];
            size_t num = xyPoints.size();
            for(size_t j = 0; j < num; ++j)
            {
                allXYPoints.emplace_back(xyPoints[j]);
                allHeadings.emplace_back(headings[j]);
                allKappas.emplace_back(kappas[j]);
                allDkappas.emplace_back(dkappas[j]);
            }
            if(i < numXYPointsVec - 1)
            {
                std::vector<std::pair<double, double>> points;
                std::vector<double> heading;
                std::vector<double> kappa;
                std::vector<double> dkappa;
                linearInterpolate(xyPointsVec[i].back(), xyPointsVec[i+1].front(),
                        headingsVec[i].back(), headingsVec[i+1].front(),
                        kappasVec[i].back(), kappasVec[i+1].front(),
                        dkappasVec[i].back(), dkappasVec[i+1].front(),
                        deltaS,
                        points, heading, kappa, dkappa);
                if(!points.empty())
                {
                    size_t numPoints = points.size();
                    for(size_t k = 0; k < numPoints; ++k)
                    {
                        allXYPoints.emplace_back(points[k]);
                        allHeadings.emplace_back(heading[k]);
                        allKappas.emplace_back(kappa[k]);
                        allDkappas.emplace_back(dkappa[k]);
                    }
                }
            }
        }
        size_t numAllXYPoints = allXYPoints.size();
        double distance = 0.0;
        allAccumulatedS.emplace_back(0.0);
        for(size_t i = 1; i < numAllXYPoints; ++i)
        {
            distance +=  std::sqrt((allXYPoints[i].first - allXYPoints[i-1].first)*(allXYPoints[i].first - allXYPoints[i-1].first) +
                                   (allXYPoints[i].second - allXYPoints[i-1].second)*(allXYPoints[i].second - allXYPoints[i-1].second));
            allAccumulatedS.emplace_back(distance);
        }

        // heading 转换:
        // todo: 平滑后的 heading 与 原始值相差了 1°, 待解决
        for(size_t i = 0; i < allHeadings.size(); ++i)
        {
            double angle = -allHeadings[i] - degreeToRadian(90);
            double a = std::fmod(angle + M_PI, 2.0 * M_PI);
            if(a < 0.0)
            {
                a = a + 2.0 * M_PI;
            }
            allHeadings[i] = a;
        }

        std::vector<ReferencePoint> referencePoints;
        size_t pointsSize = allXYPoints.size();
        for(size_t i = 0; i < pointsSize; ++i)
        {
            had_map::MapPoint point_info(Vec2d{allXYPoints[i].first, allXYPoints[i].second}, allHeadings[i]);
            referencePoints.emplace_back(point_info, allKappas[i], allDkappas[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        *smoothedReferenceLine = ReferenceLine(referencePoints, allAccumulatedS);
        return true;

    }

}// end namespace
