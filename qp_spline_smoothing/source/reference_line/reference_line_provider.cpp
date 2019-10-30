//
// Created by gaoyang on 18-11-26.
//

#include "common/math/math_utils.h"
#include "common/config/config.h"
#include "reference_line/reference_line_provider.h"
#include "reference_line/discrete_points_reference_line_smooth.h"

namespace planning
{
    AnchorPoint ReferenceLineProvide::GetAnchorPoint(const ReferenceLine &reference_line,
                               double s) const
    {
        FemPosDeviationSmootherConfig config;
        double longitudinalBound = ReferenceLineSmoothAlgorithm ? config.longitudinalBoundaryBound : FLAGS_longitudinal_boundary_bound;
        double lateralBound = ReferenceLineSmoothAlgorithm ? config.lateralBoundaryBound : FLAGS_lateral_boundary_bound;
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

    void ReferenceLineProvide::GetAnchorPoints(const ReferenceLine &reference_line,
                                std::vector<AnchorPoint> *anchor_points)
    {
        FemPosDeviationSmootherConfig config;
        const double interval = ReferenceLineSmoothAlgorithm ? config.maxConstraintInterval : FLAGS_max_point_interval;

        //一共的段数再加上起点坐标
        int num_of_anchor = std::max(2, static_cast<int>(reference_line.length() / interval + 0.5));
        std::vector<double> anchor_s;
        math::uniform_slice(0.0,reference_line.length(),num_of_anchor - 1,&anchor_s);
        int i = 0;
        for (const double s : anchor_s) {
            anchor_points->emplace_back(GetAnchorPoint(reference_line, s));
            i++;
        }
        anchor_points->front().longitudinal_bound = 1e-6;
        anchor_points->front().lateral_bound = 1e-6;
        anchor_points->front().enforced = true;
        anchor_points->back().longitudinal_bound = 1e-6;
        anchor_points->back().lateral_bound = 1e-6;
        anchor_points->back().enforced = true;
    }

    bool ReferenceLineProvide::smoothReferenceLine(const ReferenceLine &raw_reference_line, ReferenceLine *reference_line,
                                                   const double &deltaS,bool dif_time_smooth)
    {
        FemPosDeviationSmootherConfig config;

        std::vector<AnchorPoint> anchor_points;
        GetAnchorPoints(raw_reference_line,&anchor_points);
        if(!ReferenceLineSmoothAlgorithm)
        {
            QpSplineReferenceLineSmooth smoother_;
            smoother_.setAnchorPoints(anchor_points, dif_time_smooth);
            if (!smoother_.smooth(raw_reference_line, deltaS,reference_line)) {
                return false;
            }
        }
        else
        {
            planning::DiscretePointsReferenceLineSmooth smoother(config);
            smoother.setAnchorPoints(anchor_points);
            if (!smoother.smooth(raw_reference_line, deltaS, reference_line)) {
                return false;
            }
        }

        return true;
    }

}
