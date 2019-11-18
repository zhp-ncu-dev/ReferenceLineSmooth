#include <iostream>
#include <limits>
#include "common/smooth_line/discreted_points_smooth/fem_pos_deviation_smoother.h"
#include "common/smooth_line/discreted_points_smooth/fem_pos_deviation_osqp_interface.h"
#include "common/smooth_line/discreted_points_smooth/fem_pos_deviation_sqp_osqp_interface.h"

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
        //return SqpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
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


    bool FemPosDeviationSmoother::SqpWithOsqp(
            const std::vector<std::pair<double, double>>& raw_point2d,
            const std::vector<double>& bounds, std::vector<double>* opt_x,
            std::vector<double>* opt_y) {
        if (opt_x == nullptr || opt_y == nullptr) {
            //AERROR << "opt_x or opt_y is nullptr";
            return false;
        }

        FemPosDeviationSqpOsqpInterface solver;

        solver.set_weight_fem_pos_deviation(m_femPosDeviationSmootherConfig.weightFemPosDeviation);
        solver.set_weight_path_length(m_femPosDeviationSmootherConfig.weightPathLength);
        solver.set_weight_ref_deviation(m_femPosDeviationSmootherConfig.weightRefDeviation);
        solver.set_weight_curvature_constraint_slack_var(
                m_femPosDeviationSmootherConfig.weightCurvatureConstraintSlackVar);

        solver.set_curvature_constraint(m_femPosDeviationSmootherConfig.curvatureConstraint);

        solver.set_sqp_sub_max_iter(m_femPosDeviationSmootherConfig.sqpSubMaxIter);
        solver.set_sqp_ftol(m_femPosDeviationSmootherConfig.sqpFtol);
        solver.set_sqp_pen_max_iter(m_femPosDeviationSmootherConfig.sqpPenMaxIter);
        solver.set_sqp_ctol(m_femPosDeviationSmootherConfig.sqpCtol);

        solver.set_max_iter(m_femPosDeviationSmootherConfig.maxIter);
        solver.set_time_limit(m_femPosDeviationSmootherConfig.timeLimit);
        solver.set_verbose(m_femPosDeviationSmootherConfig.verbose);
        solver.set_scaled_termination(m_femPosDeviationSmootherConfig.scaledTermination);
        solver.set_warm_start(m_femPosDeviationSmootherConfig.warmStart);

        solver.set_ref_points(raw_point2d);
        solver.set_bounds_around_refs(bounds);

//        if (!solver.Solve()) {
//            return false;
//        }
//
//        std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();
//
//        // TODO(Jinyun): unify output data container
//        opt_x->resize(opt_xy.size());
//        opt_y->resize(opt_xy.size());
//        for (size_t i = 0; i < opt_xy.size(); ++i) {
//            (*opt_x)[i] = opt_xy[i].first;
//            (*opt_y)[i] = opt_xy[i].second;
//        }
        return true;
    }

}// end namespace




