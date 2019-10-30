//
// Created by gaoyang on 18-11-26.
//

#ifndef PLANNING_QP_SPLINE_REFERENCE_LINE_SMOOTH_H
#define PLANNING_QP_SPLINE_REFERENCE_LINE_SMOOTH_H

#include "reference_line.h"
#include "common/config/reference_line_config/smooth_reference_line_config.h"
#include "common/smooth_line/smooth_spline/spline_2d_solver.h"

namespace planning
{
    struct AnchorPoint
    {
        had_map::MapPoint pointInfo;
        double abs_s;
        double xds = 0.0;
        double yds = 0.0;
        double xseconds = 0.0;
        double yseconds = 0.0;
        double lateral_bound = 0.0;
        double longitudinal_bound = 0.0;
        // enforce smoother to strictly follow this reference point
        bool enforced = false;
    };

    class QpSplineReferenceLineSmooth
    {
    public:
        QpSplineReferenceLineSmooth();

        void setAnchorPoints(const std::vector<AnchorPoint> &anchor_points,bool dif_time_smooth = false);

        bool smooth(const ReferenceLine &raw_reference_line,
                    const double &longitudinalSpeed,
                    ReferenceLine *const smoothed_reference_line);

    private:
        void clear();

        double getSpacingDis(const double &speed)const;

        bool sample();

        bool addConstraint();

        bool AddKernel();

        bool Solve();

    private:
        std::vector<AnchorPoint> anchor_points_;
        std::vector<double> t_knots_;
        std::unique_ptr<Spline2dSolver> spline_solver_;

        bool dif_time_smooth_ = false;

        double ref_x_ = 0.0;
        double ref_y_ = 0.0;
    };
}
#endif //PLANNING_QP_SPLINE_REFERENCE_LINE_SMOOTH_H
