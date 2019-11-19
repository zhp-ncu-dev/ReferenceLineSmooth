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
            const ReferenceLine &reference_line,
            double s, const std::vector<double> &rawReferenceLineKappas) const
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
            longitudinalBound = config.longitudinalBoundaryBound;
            lateralBound = config.lateralBoundaryBound;
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

    bool ReferenceLineProvide::GetAnchorPoints(
            const ReferenceLine &reference_line,
            std::vector<AnchorPoint> *anchor_points,
            std::vector<std::vector<AnchorPoint>> &isnotUTurnPoints,
            const std::vector<double> &rawReferenceLineKappas)
    {
        FemPosDeviationSmootherConfig config;
        const double interval = ReferenceLineSmoothAlgorithm ? config.sampleRoadInterval : FLAGS_max_point_interval;
        std::vector<ReferencePoint> referencePoints = reference_line.referencePoints();
        std::vector<double> accumulateS = reference_line.accumulateS();
        std::vector<std::pair<uint16_t, uint16_t >> uTurnStartEndIndexPair;
        getUTurnStartEndPositionPair(referencePoints, accumulateS, uTurnStartEndIndexPair);
        std::vector<sampleInformation> samplePointsS;
        if(getSamplePointsSVector(referencePoints, accumulateS, uTurnStartEndIndexPair, samplePointsS))
        {
            GetAnchorPoints(samplePointsS, reference_line, rawReferenceLineKappas, anchor_points, isnotUTurnPoints);
            // 检查最后一个点，是否未 nan
            if(std::isnan(anchor_points->back().pointInfo.x()) ||
               std::isnan(anchor_points->back().pointInfo.y()))
            {
                anchor_points->pop_back();
            }
            return true;
        }
        else
        {
            //一共的段数再加上起点坐标
            int num_of_anchor = std::max(2, static_cast<int>(reference_line.length() / interval + 0.5));
            std::vector<double> anchor_s;
            math::uniform_slice(0.0,reference_line.length(),num_of_anchor - 1,&anchor_s);
            int i = 0;
            for (const double s : anchor_s) {
                anchor_points->emplace_back(GetAnchorPoint(reference_line, s, rawReferenceLineKappas));
                i++;
            }
            // 检查最后一个点，是否未 nan
            if(std::isnan(anchor_points->back().pointInfo.x()) ||
               std::isnan(anchor_points->back().pointInfo.y()))
            {
                anchor_points->pop_back();
            }
            return false;
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

    void ReferenceLineProvide::getUTurnStartEndPositionPair(
            const std::vector<ReferencePoint> &referencePoints,
            const std::vector<double> &accumulateS,
            std::vector<std::pair<uint16_t, uint16_t >> &uTurnStartEndIndexPair)
    {
        uint16_t numPoints = referencePoints.size();
        for(uint16_t i = 0; i < numPoints; ++i)
        {
            uint16_t uTurnStartIndex = 0;
            uint16_t uTurnEndIndex = 0;
            uint16_t uTurnPointNum = 0;
            uint16_t uTurnPointStartIndex = 0;
            uint16_t uTurnPointOverIndex = 0;
            if(referencePoints[i].pointInfo().type() == 4)
            {
                uTurnStartIndex = i;
                uTurnPointStartIndex = i;
                auto lanType1 = referencePoints[uTurnStartIndex].pointInfo().type();
                uint16_t j = uTurnStartIndex + 1;
                for(; j < numPoints - 1; ++j)
                {
                    if(referencePoints[j].pointInfo().type() == 4 &&
                       referencePoints[j+1].pointInfo().type() != 4)
                    {
                        uTurnEndIndex = j;
                        i = j;
                        std::cout << "startIndex = " << i << std::endl;
                        auto lanType2 = referencePoints[i].pointInfo().type();
                        break;
                    }

                    // 统计 uTurnPointNum
                    if(referencePoints[j].pointInfo().type() == 4)
                    {
                        uTurnPointNum ++;
                    }
                }

                if(j == numPoints - 1)
                {
                    uTurnEndIndex = j;
                }

                FemPosDeviationSmootherConfig config;
                if(accumulateS[uTurnEndIndex] - accumulateS[uTurnStartIndex] >= config.threshold)
                {
                    uTurnStartEndIndexPair.emplace_back(uTurnStartIndex, uTurnEndIndex);
                }
                if(uTurnPointNum >= numPoints - 5 ||
                  (uTurnPointNum == uTurnEndIndex - uTurnStartIndex - 1 && uTurnEndIndex == numPoints - 1))
                {
                    break;
                }
            }
        }

        if(!uTurnStartEndIndexPair.empty())
        {
            for(const auto &indexPair : uTurnStartEndIndexPair)
            {
                std::cout << "(start, end) = " << indexPair.first << " , " << indexPair.second << std::endl;
            }
            std::cout << "hello world !" << std::endl;
            std::cout << "掉头数量为 = " << uTurnStartEndIndexPair.size() << std::endl;
        }
    }

    bool ReferenceLineProvide::getSamplePointsSVector(
            const std::vector<ReferencePoint> &referencePoints,
            const std::vector<double> &accumulateS,
            const std::vector<std::pair<uint16_t, uint16_t >> &uTurnStartEndIndexPair,
            std::vector<sampleInformation> &samplePointsS)
    {
        if(uTurnStartEndIndexPair.empty())
        {
            return false;
        }

        FemPosDeviationSmootherConfig config;
        sampleInformation samplePointS;
        float distanceToStartPoint;
        float distanceToEndPoint;
        size_t num = uTurnStartEndIndexPair.size();
        if(num == 1)
        {
            distanceToStartPoint = accumulateS[uTurnStartEndIndexPair[0].first] - accumulateS[0];
            distanceToEndPoint = accumulateS.back() - accumulateS[uTurnStartEndIndexPair[0].second];
            // all is uTurn
            if(distanceToStartPoint <= config.threshold &&
               distanceToEndPoint <= config.threshold)
            {
                samplePointS.startS = 0.0;
                samplePointS.endS = accumulateS.back();
                samplePointS.step = config.uTurnConstrainInterval;
                samplePointS.flag = true;
                samplePointsS.emplace_back(samplePointS);
            }

            // front is uTurn, back is not uTurn
            if(distanceToStartPoint <= config.threshold &&
               distanceToEndPoint > config.threshold)
            {
                samplePointS.startS = 0.0;
                samplePointS.endS = accumulateS[uTurnStartEndIndexPair[0].second] - config.ordinaryRoadExtendLength;
                samplePointS.step = config.uTurnConstrainInterval;
                samplePointS.flag = true;
                samplePointsS.emplace_back(samplePointS);

                samplePointS.startS = accumulateS[uTurnStartEndIndexPair[0].second] + config.deltaS;
                samplePointS.endS = accumulateS.back();
                samplePointS.step = config.maxConstraintInterval;
                samplePointS.flag = false;
                samplePointsS.emplace_back(samplePointS);
            }

            // front is not uTurn , back is uTurn
            if(distanceToStartPoint > config.threshold &&
               distanceToEndPoint <= config.threshold)
            {
                samplePointS.startS = 0.0;
                samplePointS.endS = accumulateS[uTurnStartEndIndexPair[0].first] + config.ordinaryRoadExtendLength;
                samplePointS.step = config.maxConstraintInterval;
                samplePointS.flag = false;
                samplePointsS.emplace_back(samplePointS);

                samplePointS.startS = accumulateS[uTurnStartEndIndexPair[0].first] + config.ordinaryRoadExtendLength + config.deltaS;
                samplePointS.endS = accumulateS.back();
                samplePointS.step = config.uTurnConstrainInterval;
                samplePointS.flag = true;
                samplePointsS.emplace_back(samplePointS);
            }

            // front and back are not uTurn
            if(distanceToStartPoint > config.threshold &&
               distanceToEndPoint > config.threshold)
            {
                samplePointS.startS = 0.0;
                samplePointS.endS = accumulateS[uTurnStartEndIndexPair[0].first] + config.ordinaryRoadExtendLength;
                samplePointS.step = config.maxConstraintInterval;
                samplePointS.flag = false;
                samplePointsS.emplace_back(samplePointS);

                samplePointS.startS = accumulateS[uTurnStartEndIndexPair[0].first]+ config.ordinaryRoadExtendLength + config.deltaS;
                samplePointS.endS = accumulateS[uTurnStartEndIndexPair[0].second] - config.ordinaryRoadExtendLength;
                samplePointS.step = config.uTurnConstrainInterval;
                samplePointS.flag = true;
                samplePointsS.emplace_back(samplePointS);

                samplePointS.startS = accumulateS[uTurnStartEndIndexPair[0].second] - config.ordinaryRoadExtendLength + config.deltaS;
                samplePointS.endS = accumulateS.back();
                samplePointS.step = config.maxConstraintInterval;
                samplePointS.flag = false;
                samplePointsS.emplace_back(samplePointS);
            }
        }

        if(num > 1)
        {
            distanceToStartPoint = accumulateS[uTurnStartEndIndexPair[0].first] - accumulateS[0];
            distanceToEndPoint = accumulateS.back() - accumulateS[uTurnStartEndIndexPair.back().second];

            // the first one is uTurn
            if(distanceToStartPoint <= config.threshold)
            {
                // 默认相邻两个掉头之间的累计距离大于 2 * config.maxConstraintInterval
                // todo:
                size_t i = 0;
                for(; i < num - 1; ++i)
                {
                    if(i == 0)
                    {
                        samplePointS.startS = 0.0;
                    }
                    else
                    {
                        samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].first] + config.ordinaryRoadExtendLength + config.deltaS;
                    }
                    samplePointS.endS = accumulateS[uTurnStartEndIndexPair[i].second] - config.ordinaryRoadExtendLength - config.deltaS;
                    samplePointS.step = config.uTurnConstrainInterval;
                    samplePointS.flag = true;
                    samplePointsS.emplace_back(samplePointS);

                    samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].second] - config.ordinaryRoadExtendLength;
                    samplePointS.endS = accumulateS[uTurnStartEndIndexPair[i+1].first] + config.ordinaryRoadExtendLength;
                    samplePointS.step = config.maxConstraintInterval;
                    samplePointS.flag = false;
                    samplePointsS.emplace_back(samplePointS);
                }
                if(i == num - 1)
                {
                    // the last one is uTurn
                    if(distanceToEndPoint <= config.threshold)
                    {
                        samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].first] + config.ordinaryRoadExtendLength + config.deltaS;
                        samplePointS.endS = accumulateS.back();
                        samplePointS.step = config.uTurnConstrainInterval;
                        samplePointS.flag = true;
                        samplePointsS.emplace_back(samplePointS);
                    }
                    else
                    {
                        // 将最后一个uTurn加入，然后再加入后续非uTurn段
                        samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].first] + config.ordinaryRoadExtendLength + config.deltaS;
                        samplePointS.endS = accumulateS[uTurnStartEndIndexPair.back().second] - config.ordinaryRoadExtendLength - config.deltaS;
                        samplePointS.step = config.uTurnConstrainInterval;
                        samplePointS.flag = true;
                        samplePointsS.emplace_back(samplePointS);

                        samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].second] - config.ordinaryRoadExtendLength;
                        samplePointS.endS = accumulateS.back();
                        samplePointS.step = config.maxConstraintInterval;
                        samplePointS.flag = false;
                        samplePointsS.emplace_back(samplePointS);
                    }
                }
            }

            // the first one is not uTurn
            if(distanceToStartPoint > config.threshold)
            {
                samplePointS.startS = 0.0;
                samplePointS.endS = accumulateS[uTurnStartEndIndexPair[0].first] + config.ordinaryRoadExtendLength;
                samplePointS.step = config.maxConstraintInterval;
                samplePointS.flag = false;
                samplePointsS.emplace_back(samplePointS);

                // 默认相邻两个掉头之间的累计距离大于 2 * config.maxConstraintInterval
                // todo:
                size_t i = 0;
                for(; i < num - 1; ++i)
                {
                    // this is uTurn
                    samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].first] + config.ordinaryRoadExtendLength + config.deltaS;
                    samplePointS.endS = accumulateS[uTurnStartEndIndexPair[i].second] - config.ordinaryRoadExtendLength - config.deltaS;
                    samplePointS.step = config.uTurnConstrainInterval;
                    samplePointS.flag = true;
                    samplePointsS.emplace_back(samplePointS);

                    // this is not uTurn
                    samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].second] - config.ordinaryRoadExtendLength;
                    samplePointS.endS = accumulateS[uTurnStartEndIndexPair[i+1].first] + config.ordinaryRoadExtendLength;
                    samplePointS.step = config.maxConstraintInterval;
                    samplePointS.flag = false;
                    samplePointsS.emplace_back(samplePointS);
                }
                if(i == num - 1)
                {
                    // 检查后续是否是uTurn
                    if(distanceToEndPoint <= config.threshold)
                    {
                        // is uTurn
                        samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].first] + config.ordinaryRoadExtendLength + config.deltaS;
                        samplePointS.endS = accumulateS.back();
                        samplePointS.step = config.uTurnConstrainInterval;
                        samplePointS.flag = true;
                        samplePointsS.emplace_back(samplePointS);
                    }
                    else
                    {
                        // is not uTurn
                        samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].first] + config.ordinaryRoadExtendLength + config.deltaS;
                        samplePointS.endS = accumulateS[uTurnStartEndIndexPair[i].second] - config.ordinaryRoadExtendLength - config.deltaS;
                        samplePointS.step = config.uTurnConstrainInterval;
                        samplePointS.flag = true;
                        samplePointsS.emplace_back(samplePointS);

                        samplePointS.startS = accumulateS[uTurnStartEndIndexPair[i].second] - config.ordinaryRoadExtendLength;
                        samplePointS.endS = accumulateS.back();
                        samplePointS.step = config.maxConstraintInterval;
                        samplePointS.flag = true;
                        samplePointsS.emplace_back(samplePointS);
                    }
                }
            }

        }

        return true;
    }

    void ReferenceLineProvide::GetAnchorPoints(
            const std::vector<sampleInformation> &samplePointsS,
            const ReferenceLine &reference_line,
            const std::vector<double> &rawReferenceLineKappas,
            std::vector<AnchorPoint> *anchor_points,
            std::vector<std::vector<AnchorPoint>> &isnotUTurnPoints)
    {
        FemPosDeviationSmootherConfig config;
        for(const auto samplePoints : samplePointsS)
        {
            float interval = samplePoints.step;
            std::vector<double> anchor_s;
            int num_of_anchor = 0;
            double distance = samplePoints.endS - samplePoints.startS;
            double sampleStartS = samplePoints.startS;
            double samplrEndS = samplePoints.endS;
            float deltaLongitudinalBoundaryBound = 2 * (config.longitudinalBoundaryBound - config.uTurnLongitudinalBoundaryBound);
            float deltaLateralBoundaryBound = 2 * (config.lateralBoundaryBound - config.uTurnLateralBoundaryBound);
            std::vector<AnchorPoint> isnotUTurnPoint;
            if(samplePoints.flag)
            {
                num_of_anchor = std::max(2, static_cast<int>(distance / interval + 0.1));
                math::uniform_slice(sampleStartS,samplrEndS,num_of_anchor - 1,&anchor_s);

                for(const auto s : anchor_s)
                {
                    AnchorPoint anchor_point;
                    anchor_point.longitudinal_bound = config.uTurnLongitudinalBoundaryBound;
                    anchor_point.lateral_bound = config.uTurnLateralBoundaryBound;
                    anchor_point.abs_s = s;
                    ReferencePoint ref_point = reference_line.getReferencePoint(s);
                    anchor_point.pointInfo = ref_point.pointInfo();
                    anchor_point.xds = ref_point.xds();
                    anchor_point.yds = ref_point.yds();
                    anchor_point.xseconds = ref_point.xsenconds();
                    anchor_point.yseconds = ref_point.ysenconds();
                    anchor_points->emplace_back(anchor_point);
                    isnotUTurnPoint.emplace_back(anchor_point);
                }

                isnotUTurnPoint.front().lateral_bound = 0.0;
                isnotUTurnPoint.front().longitudinal_bound = 0.0;
                isnotUTurnPoint.back().lateral_bound = 0.0;
                isnotUTurnPoint.back().longitudinal_bound = 0.0;
            }

            if(!samplePoints.flag)
            {
                num_of_anchor = std::max(2, static_cast<int>(distance / interval + 0.1));
                math::uniform_slice(sampleStartS,samplrEndS,num_of_anchor - 1,&anchor_s);

                for(const auto s : anchor_s)
                {
                    AnchorPoint anchor_point;
                    // 前后 5m 范围，放开 bound 约束
                    if(anchor_s.back() > 20.0)
                    {
                        if(anchor_s.front() + 10.0 > s)
                        {
                            anchor_point.longitudinal_bound = 0.4;
                            anchor_point.lateral_bound = 0.4;
                        }
                        if(anchor_s.front() + 10.0 <= s && s < anchor_s.back() - 10.0)
                        {
                            anchor_point.longitudinal_bound = config.maxConstraintInterval;
                            anchor_point.lateral_bound = config.maxConstraintInterval;
                        }
                        if(s >= anchor_s.back() - 10.0)
                        {
                            anchor_point.longitudinal_bound = 0.4;
                            anchor_point.lateral_bound = 0.4;
                        }
                    }
                    else if(anchor_s.back() >= 10.0 && anchor_s.back() <= 20.0)
                    {
                        if(anchor_s.front() + 5.0 > s)
                        {
                            anchor_point.longitudinal_bound = 0.4;
                            anchor_point.lateral_bound = 0.4;
                        }
                        if(anchor_s.front() + 5.0 <= s && s < anchor_s.back() - 5.0)
                        {
                            anchor_point.longitudinal_bound = config.maxConstraintInterval;
                            anchor_point.lateral_bound = config.maxConstraintInterval;
                        }
                        if(s >= anchor_s.back() - 5.0)
                        {
                            anchor_point.longitudinal_bound = 0.4;
                            anchor_point.lateral_bound = 0.4;
                        }
                    }
                    else
                    {
                        anchor_point.longitudinal_bound = config.maxConstraintInterval;
                        anchor_point.lateral_bound = config.maxConstraintInterval;
                    }
                    anchor_point.abs_s = s;
                    // 差值算法需要修改（用曲线差值）==> 适应高速公路场景
                    ReferencePoint ref_point = reference_line.getReferencePoint(s);
                    anchor_point.pointInfo = ref_point.pointInfo();
                    anchor_point.xds = ref_point.xds();
                    anchor_point.yds = ref_point.yds();
                    anchor_point.xseconds = ref_point.xsenconds();
                    anchor_point.yseconds = ref_point.ysenconds();
                    anchor_points->emplace_back(anchor_point);
                    isnotUTurnPoint.emplace_back(anchor_point);
                }

                isnotUTurnPoint.front().lateral_bound = 0.0;
                isnotUTurnPoint.front().longitudinal_bound = 0.0;
                isnotUTurnPoint.back().lateral_bound = 0.0;
                isnotUTurnPoint.back().longitudinal_bound = 0.0;
            }
            isnotUTurnPoints.emplace_back(isnotUTurnPoint);
        }
    }

    bool ReferenceLineProvide::smoothReferenceLine(
            const ReferenceLine &raw_reference_line, ReferenceLine *reference_line,
            const double &deltaS,bool dif_time_smooth)
    {
        FemPosDeviationSmootherConfig config;
        std::vector<double> rawReferenceLineKappas;
        caculateRawReferenceLineKappas(raw_reference_line, rawReferenceLineKappas);

        std::vector<AnchorPoint> anchor_points;
        std::vector<std::vector<AnchorPoint>> isnotUTurnPoints;
        bool flag = GetAnchorPoints(raw_reference_line, &anchor_points, isnotUTurnPoints, rawReferenceLineKappas);
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
            if (!smoother.smooth(raw_reference_line, deltaS, reference_line, flag, isnotUTurnPoints))
            {
                return false;
            }
        }

        return true;
    }

}