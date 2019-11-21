//
// Created by gaoyang on 18-11-26.
//
#include "common/math/math_utils.h"
#include "common/config/config.h"
#include "reference_line/reference_line_provider.h"
#include "reference_line/discrete_points_reference_line_smooth.h"
#include "common/smooth_line/discreted_points_smooth/discrete_points_math.h"
#include "common/curve1d/cubic_spline.h"

namespace planning
{
    AnchorPoint ReferenceLineProvide::GetAnchorPoint(
            const planning::ReferenceLine &reference_line, double s,
            const std::vector<double> &accumulateS,
            const std::vector<double> &rawReferenceLineKappas) const
    {
        double longitudinalBound = 0.0;
        double lateralBound = 0.0;
        if(!ReferenceLineSmoothAlgorithm)
        {
            longitudinalBound = FLAGS_longitudinal_boundary_bound;
            lateralBound = FLAGS_lateral_boundary_bound;
        }
        else
        {
            FemPosDeviationSmootherConfig config;
            longitudinalBound = config.freewayLongitudinalBoundaryBound;
            lateralBound = config.freewayLateralBoundaryBound;
        }
        AnchorPoint anchor_point;
        anchor_point.longitudinal_bound = longitudinalBound;
        anchor_point.lateral_bound = lateralBound;
        anchor_point.abs_s = s;
        ReferencePoint ref_point = reference_line.getReferencePoint(s);

        anchor_point.pointInfo = ref_point.pointInfo();
        anchor_point.xds = ref_point.xds();
        anchor_point.yds = ref_point.yds();
        anchor_point.xseconds = ref_point.xsenconds();
        anchor_point.yseconds = ref_point.ysenconds();

        return anchor_point;
    }

    void ReferenceLineProvide::GetAnchorPoints(
            const ReferenceLine &reference_line,
            std::vector<AnchorPoint> *anchor_points,
            const std::vector<double> &rawReferenceLineKappas)
    {
        FemPosDeviationSmootherConfig config;

        const std::vector<ReferencePoint> &referencePoints = reference_line.referencePoints();
        const std::vector<double> &accumulateS = reference_line.accumulateS();
        std::vector<std::pair<uint16_t, uint16_t >> uTurnStartEndIndexPair;
        std::vector<std::pair<uint16_t, uint16_t >> rightStartEndIndexPair;
        std::vector<std::pair<uint16_t, uint16_t >> leftStartEndIndexPair;
        getRoadTypeStartEndPositionPair(referencePoints, accumulateS, leftStartEndIndexPair,  2);
        getRoadTypeStartEndPositionPair(referencePoints, accumulateS, rightStartEndIndexPair, 3);
        getRoadTypeStartEndPositionPair(referencePoints, accumulateS, uTurnStartEndIndexPair, 4);
        std::vector<std::pair<double,bool>> samplePointsS;
        if(getSamplePointsSVector(referencePoints, accumulateS, leftStartEndIndexPair,
                rightStartEndIndexPair, uTurnStartEndIndexPair, samplePointsS))
        {
            for(const auto &s : samplePointsS)
            {
                AnchorPoint anchor_point;
                if(s.second)
                {
                    anchor_point.longitudinal_bound = config.turnLongitudinalBoundaryBound;
                    anchor_point.lateral_bound = config.turnLateralBoundaryBound;
                    for(const auto &indexPair : uTurnStartEndIndexPair)
                    {
                        if(s.first >= accumulateS[indexPair.first] && s.first <= accumulateS[indexPair.second])
                        {
                            anchor_point.longitudinal_bound = config.uTurnLongitudinalBoundaryBound;
                            anchor_point.lateral_bound = config.uTurnLateralBoundaryBound;
                        }
                    }
                }
                else
                {
                    anchor_point.longitudinal_bound = config.freewayLongitudinalBoundaryBound;
                    anchor_point.lateral_bound = config.freewayLateralBoundaryBound;
                }
                anchor_point.abs_s = s.first;
                ReferencePoint ref_point = reference_line.getReferencePoint(s.first);
                anchor_point.pointInfo = ref_point.pointInfo();
                anchor_point.xds = ref_point.xds();
                anchor_point.yds = ref_point.yds();
                anchor_point.xseconds = ref_point.xsenconds();
                anchor_point.yseconds = ref_point.ysenconds();

                anchor_points->emplace_back(anchor_point);
            }
        }
        else
        {
            const double interval = ReferenceLineSmoothAlgorithm ? config.freewayRoadInterval : FLAGS_max_point_interval;
            int num_of_anchor = std::max(2, static_cast<int>(reference_line.length() / interval + 0.1));
            std::vector<double> anchor_s;
            math::uniform_slice(0.0,reference_line.length(),num_of_anchor - 1,&anchor_s);
            int i = 0;
            for (const double s : anchor_s) {
                anchor_points->emplace_back(GetAnchorPoint(reference_line, s,
                                                           accumulateS, rawReferenceLineKappas));
                i++;
            }
        }
        if(std::isnan(anchor_points->back().pointInfo.x()) ||
           std::isnan(anchor_points->back().pointInfo.y()))
        {
            anchor_points->pop_back();
        }
    }

    void ReferenceLineProvide::caculateRawReferenceLineKappas(
            const planning::ReferenceLine &raw_reference_line,
            std::vector<double> &rawReferenceLineKappas)
    {
        std::vector<double> kappas;
        DiscretePointsMath::ComputeOriginalPathKappaProfile(raw_reference_line, &kappas);
        rawReferenceLineKappas = kappas;
    }

    void ReferenceLineProvide::getRoadTypeStartEndPositionPair(
            const std::vector<planning::ReferencePoint> &referencePoints,
            const std::vector<double> &accumulateS,
            std::vector<std::pair<uint16_t, uint16_t >> &startEndIndexPair,
            const int type)
    {
        uint16_t numPoints = referencePoints.size();
        for(uint16_t i = 0; i < numPoints; ++i)
        {
            uint16_t uTurnStartIndex = 0;
            uint16_t uTurnEndIndex = 0;
            uint16_t uTurnPointNum = 0;
            uint16_t uTurnPointStartIndex = 0;
            uint16_t uTurnPointOverIndex = 0;
            if(referencePoints[i].pointInfo().type() == type)
            {
                uTurnStartIndex = i;
                uTurnPointStartIndex = i;
                auto lanType1 = referencePoints[uTurnStartIndex].pointInfo().type();
                uint16_t j = uTurnStartIndex + 1;
                for(; j < numPoints - 1; ++j)
                {
                    if(referencePoints[j].pointInfo().type() == type &&
                       referencePoints[j+1].pointInfo().type() != type)
                    {
                        uTurnEndIndex = j;
                        i = j;
                        auto lanType2 = referencePoints[i].pointInfo().type();
                        break;
                    }

                    // 统计 uTurnPointNum
                    if(referencePoints[j].pointInfo().type() == type)
                    {
                        uTurnPointNum ++;
                    }
                }

                if(j == numPoints - 1)
                {
                    uTurnEndIndex = j;
                }

                FemPosDeviationSmootherConfig config;
                if(accumulateS[uTurnEndIndex] - accumulateS[uTurnStartIndex] >= config.urbanRoadTypeThreshold)
                {
                    startEndIndexPair.emplace_back(uTurnStartIndex, uTurnEndIndex);
                }
                if(uTurnPointNum >= numPoints - 5 ||
                   (uTurnPointNum == uTurnEndIndex - uTurnStartIndex - 1 && uTurnEndIndex == numPoints - 1))
                {
                    break;
                }
            }
        }

        if(!startEndIndexPair.empty())
        {
            if(type == 2)
                std::cout << "左转数量为 = " << startEndIndexPair.size() << std::endl;
            if(type == 3)
                std::cout << "右转数量为 = " << startEndIndexPair.size() << std::endl;
            if(type == 4)
                std::cout << "掉头数量为 = " << startEndIndexPair.size() << std::endl;
            for(const auto &indexPair : startEndIndexPair)
            {
                std::cout << "(start, end) = " << indexPair.first << " , " << indexPair.second << std::endl;
            }
        }
    }

    bool ReferenceLineProvide::getSamplePointsSVector(
            const std::vector<ReferencePoint> &referencePoints,
            const std::vector<double> &accumulateS,
            const std::vector<std::pair<uint16_t, uint16_t>> &leftStartEndIndexPair,
            const std::vector<std::pair<uint16_t, uint16_t>> &rightStartEndIndexPair,
            const std::vector<std::pair<uint16_t, uint16_t>> &uTurnStartEndIndexPair,
            std::vector<std::pair<double,bool>> &samplePointsS)
    {
        if(leftStartEndIndexPair.empty() &&
           rightStartEndIndexPair.empty() &&
           uTurnStartEndIndexPair.empty())
        {
            return false;
        }
        std::vector<double> startS;
        std::vector<double> endS;
        if(!leftStartEndIndexPair.empty())
        {
            for(const auto &indexPair : leftStartEndIndexPair)
            {
                startS.emplace_back(accumulateS[indexPair.first]);
                endS.emplace_back(accumulateS[indexPair.second]);
            }
        }
        if(!rightStartEndIndexPair.empty())
        {
            for(const auto &indexPair : rightStartEndIndexPair)
            {
                startS.emplace_back(accumulateS[indexPair.first]);
                endS.emplace_back(accumulateS[indexPair.second]);
            }
        }
        if(!uTurnStartEndIndexPair.empty())
        {
            for(const auto &indexPair : uTurnStartEndIndexPair)
            {
                startS.emplace_back(accumulateS[indexPair.first]);
                endS.emplace_back(accumulateS[indexPair.second]);
            }
        }
        std::sort(std::begin(startS), std::end(startS));
        std::sort(std::begin(endS), std::end(endS));
        FemPosDeviationSmootherConfig config;
        double interval = config.urbanRoadInterval;
        if(startS[0] <= 2 * interval)
        {
            startS[0] = accumulateS.front();
        }
        if(endS.back() + 2 * interval >= accumulateS.back())
        {
            endS.back() = accumulateS.back();
        }
        int num_of_anchor = 0;
        int numStartS = startS.size();
        for(int i = 0; i < numStartS; ++i)
        {
            if(i == 0)
            {
                if(startS[0] >= 2*interval)
                {
                    std::vector<double> anchor_s;
                    double startSampleS = 0.0;
                    double endSampleS = startS[0] - interval;
                    num_of_anchor = std::max(2, static_cast<int>((endSampleS - startSampleS) / interval + 0.1));
                    math::uniform_slice(startSampleS, endSampleS, num_of_anchor - 1, &anchor_s);
                    for(const auto &s : anchor_s)
                    {
                        samplePointsS.emplace_back(s, false);
                    }
                }
            }

            if(endS[i] - startS[i] <= 2 * interval)
            {
                samplePointsS.emplace_back(startS[i], true);
                samplePointsS.emplace_back(0.5 * (startS[i] + endS[i]), true);
                samplePointsS.emplace_back(endS[i], true);
            }
            else
            {
                std::vector<double> anchor_s;
                double startSampleS = startS[i];
                double endSampleS = endS[i];
                num_of_anchor = std::max(2, static_cast<int>((endSampleS - startSampleS) / interval + 0.1));
                math::uniform_slice(startSampleS, endSampleS, num_of_anchor - 1, &anchor_s);
                for(const auto &s : anchor_s)
                {
                    samplePointsS.emplace_back(s, true);
                }
            }

            if(i <= numStartS - 2)
            {
                if(startS[i+1] - endS[i] <= 2 * interval)
                {
                    samplePointsS.emplace_back(endS[i], false);
                    samplePointsS.emplace_back(0.5 * (endS[i] + startS[i+1]), false);
                    samplePointsS.emplace_back(startS[i+1], false);
                }
                else
                {
                    std::vector<double> anchor_s;
                    double startSampleS = endS[i] + interval;
                    double endSampleS = startS[i+1] - interval;
                    num_of_anchor = std::max(2, static_cast<int>((endSampleS - startSampleS) / interval + 0.1));
                    math::uniform_slice(startSampleS, endSampleS, num_of_anchor - 1, &anchor_s);
                    for(const auto &s : anchor_s)
                    {
                        samplePointsS.emplace_back(s, false);
                    }
                }
            }
            else
            {
                if(endS[i] + 2 * interval > accumulateS.back() && endS[i] + interval < accumulateS.back())
                {
                    samplePointsS.emplace_back(accumulateS.back(), false);
                }
                else
                {
                    std::vector<double> anchor_s;
                    double startSampleS = endS[i] + interval;
                    double endSampleS = accumulateS.back();
                    num_of_anchor = std::max(2, static_cast<int>((endSampleS - startSampleS) / interval + 0.1));
                    math::uniform_slice(startSampleS, endSampleS, num_of_anchor - 1, &anchor_s);
                    for(const auto &s : anchor_s)
                    {
                        samplePointsS.emplace_back(s, false);
                    }
                }
            }
        }
        return true;
    }

    bool ReferenceLineProvide::smoothReferenceLine(
            const ReferenceLine &raw_reference_line, ReferenceLine *reference_line,
            const double &deltaS,bool dif_time_smooth)
    {
        FemPosDeviationSmootherConfig config;
        std::vector<double> rawReferenceLineKappas;
        caculateRawReferenceLineKappas(raw_reference_line, rawReferenceLineKappas);
        std::vector<AnchorPoint> anchor_points;

        GetAnchorPoints(raw_reference_line, &anchor_points, rawReferenceLineKappas);
        if(!ReferenceLineSmoothAlgorithm)
        {
            QpSplineReferenceLineSmooth smoother_;
            smoother_.setAnchorPoints(anchor_points, dif_time_smooth);
            if (!smoother_.smooth(raw_reference_line, deltaS,reference_line))
            {
                return false;
            }
        }
        else
        {
            planning::DiscretePointsReferenceLineSmooth smoother(config);
            smoother.setAnchorPoints(anchor_points);
            if (!smoother.smooth(raw_reference_line, deltaS, reference_line))
            {
                return false;
            }
        }

        return true;
    }
}