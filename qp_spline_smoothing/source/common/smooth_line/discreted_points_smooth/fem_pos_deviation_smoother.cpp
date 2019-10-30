#include <iostream>
#include <limits>
#include "common/smooth_line/discreted_points_smooth/fem_pos_deviation_smoother.h"
#include "common/smooth_line/discreted_points_smooth/fem_pos_deviation_osqp_interface.h"

namespace planning
{
    FemPosDeviationSmoother::FemPosDeviationSmoother(
            const FemPosDeviationSmootherConfig& config)
            : m_femPosDeviationSmootherConfig(config) {}

    bool FemPosDeviationSmoother::Solve(
            const std::vector<std::pair<double, double>>& raw_point2d,
            const std::vector<double>& bounds, std::vector<double>* opt_x,
            std::vector<double>* opt_y)
    {
        return SolveWithOsqp(raw_point2d, bounds, opt_x, opt_y);
    }

    bool FemPosDeviationSmoother::SolveWithOsqp(
            const std::vector<std::pair<double, double>>& raw_point2d,
            const std::vector<double>& bounds, std::vector<double>* opt_x,
            std::vector<double>* opt_y)
    {
        FemPosDeviationOsqpInterface solver;

        solver.set_weight_fem_pos_deviation(m_femPosDeviationSmootherConfig.weightFemPosDeviation);
        solver.set_weight_path_length(m_femPosDeviationSmootherConfig.weightPathLength);
        solver.set_weight_ref_deviation(m_femPosDeviationSmootherConfig.weightRefDeviation);

        solver.set_max_iter(m_femPosDeviationSmootherConfig.maxIter);
        solver.set_time_limit(m_femPosDeviationSmootherConfig.timeLimit);
        solver.set_verbose(m_femPosDeviationSmootherConfig.verbose);
        solver.set_scaled_termination(m_femPosDeviationSmootherConfig.scaledTermination);
        solver.set_warm_start(m_femPosDeviationSmootherConfig.warmStart);

        solver.set_ref_points(raw_point2d);

        solver.set_bounds_around_refs(bounds);

        if (!solver.Solve())
        {
            return false;
        }

        *opt_x = solver.opt_x();
        *opt_y = solver.opt_y();
        return true;
    }

}// end namespace




