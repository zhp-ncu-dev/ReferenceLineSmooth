//
// Created by gaoyang on 18-11-26.
//
#include "common/math/math_utils.h"
#include "common/config/config.h"
#include "reference_line/reference_line_provider.h"
#include "reference_line/discrete_points_reference_line_smooth.h"
#include "common/smooth_line/discreted_points_smooth/discrete_points_math.h"

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
            // 根据原始线的曲率，进行 bound 给定： 直道放开, 弯道收紧
            FemPosDeviationSmootherConfig config;
            size_t index = reference_line.getNearstPointLocation(s);
            double currentLocationKappa = rawReferenceLineKappas[index];
            if(std::abs(currentLocationKappa) < 0.01)
            {
                longitudinalBound = 1.0;
                lateralBound = 1.0;
            }
            else
            {
                longitudinalBound = config.longitudinalBoundaryBound;
                lateralBound = config.lateralBoundaryBound;
            }
        }

        AnchorPoint anchor_point;
        anchor_point.longitudinal_bound = longitudinalBound;
        anchor_point.lateral_bound = lateralBound;
        anchor_point.abs_s = s;
        ReferencePoint ref_point = reference_line.getNearstPoint(s);
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
        const double interval = ReferenceLineSmoothAlgorithm ? config.maxConstraintInterval : FLAGS_max_point_interval;

        //一共的段数再加上起点坐标
        int num_of_anchor = std::max(2, static_cast<int>(reference_line.length() / interval + 0.5));
        std::vector<double> anchor_s;
        math::uniform_slice(0.0,reference_line.length(),num_of_anchor - 1,&anchor_s);
        int i = 0;
        for (const double s : anchor_s) {
            anchor_points->emplace_back(GetAnchorPoint(reference_line, s, rawReferenceLineKappas));
            i++;
        }
        anchor_points->front().longitudinal_bound = 1e-6;
        anchor_points->front().lateral_bound = 1e-6;
        anchor_points->front().enforced = true;
        anchor_points->back().longitudinal_bound = 1e-6;
        anchor_points->back().lateral_bound = 1e-6;
        anchor_points->back().enforced = true;
    }

    void ReferenceLineProvide::caculateRawReferenceLineKappas(
            const planning::ReferenceLine &raw_reference_line,
            std::vector<double> &rawReferenceLineKappas)
    {
        std::vector<std::pair<double, double>> xyPoints;
        std::vector<ReferencePoint> referencePoints = raw_reference_line.referencePoints();
        for(const auto point : referencePoints)
        {
            xyPoints.emplace_back(point.pointInfo().x(), point.pointInfo().y());
        }

        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<double> accumulatedS;

        if (DiscretePointsMath::ComputePathProfile(
                xyPoints, &headings, &accumulatedS, &kappas, &dkappas))
        {
            rawReferenceLineKappas = kappas;
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
