//
// Created by zhp on 20-7-23.
//

#include "reference_line/piecewise_jerk_smooth.h"
#include "common/smooth_line/piecewise_jerk/piecewise_jerk_smooth_problem.h"

namespace planning {

namespace {

    double kInterval = 1.0;

}

bool PiecewiseJerkSmooth::smooth(const planning::ReferenceLine &rawReferenceLine, const double &deltaS,
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

    // 平滑
    std::vector<std::pair<double, double>> smoothedPoints2d;
    if(!smoothLine(rawPoints2d, smoothedReferenceLine))
    {
        return false;
    }

    // 组线
    deNormalizePoints(&smoothedPoints2d);
    return true;
}

void PiecewiseJerkSmooth::normalizePoints(std::vector<std::pair<double, double>> *xyPoints)
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

void PiecewiseJerkSmooth::deNormalizePoints(std::vector<std::pair<double, double>> *xyPoints)
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

bool PiecewiseJerkSmooth::smoothLine(const std::vector<std::pair<double, double>> &xyPoints,
                                     planning::ReferenceLine *const smoothedReferenceLine)
{




}



bool PiecewiseJerkSmooth::optimizeSmooth(
        const std::array<double, 3>& init_state,
        const std::array<double, 3>& end_state, const double delta_s,
        const std::vector<std::pair<double, double>>& bound,
        const std::vector<std::pair<double, double>>& ddl_bounds,
        const std::array<double, 5>& w, std::vector<double>* x,
        std::vector<double>* dx, std::vector<double>* ddx, const int max_iter)
{
    PiecewiseJerkSmoothProblem piecewise_jerk_problem(
            bound.size(), delta_s, init_state);
    piecewise_jerk_problem.set_weight_x(w[0]);
    piecewise_jerk_problem.set_weight_dx(w[1]);
    piecewise_jerk_problem.set_weight_ddx(w[2]);
    piecewise_jerk_problem.set_weight_dddx(w[3]);
    piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});
    piecewise_jerk_problem.set_x_bounds(bound);
    piecewise_jerk_problem.set_dx_bounds(-1.0e5,
                                         1.0e5);
    piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);
    piecewise_jerk_problem.set_dddx_bound(1.0e5);
    bool success = piecewise_jerk_problem.Optimize(max_iter);
    if (!success) {
        return false;
    }
    *x = piecewise_jerk_problem.opt_x();
    *dx = piecewise_jerk_problem.opt_dx();
    *ddx = piecewise_jerk_problem.opt_ddx();
    return true;
}



}