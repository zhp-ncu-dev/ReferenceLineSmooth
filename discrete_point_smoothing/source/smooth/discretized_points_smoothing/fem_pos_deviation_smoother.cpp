#include <iostream>
#include <limits>
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "smooth/discretized_points_smooth/fem_pos_deviation_ipopt_interface.h"
#include "smooth/discretized_points_smooth/fem_pos_deviation_smoother.h"
#include "smooth/discretized_points_smooth/fem_pos_deviation_osqp_interface.h"

namespace ADC {
namespace planning {

FemPosDeviationSmoother::FemPosDeviationSmoother(
        const FemPosDeviationSmootherConfig& config)
        : config_(config) {}

bool FemPosDeviationSmoother::Solve(
        const std::vector<std::pair<double, double>>& raw_point2d,
        const std::vector<double>& bounds, std::vector<double>* opt_x,
        std::vector<double>* opt_y) {
    if (config_.apply_curvature_constraint) {
        return SolveWithIpopt(raw_point2d, bounds, opt_x, opt_y);
    }else{
        return SolveWithOsqp(raw_point2d, bounds, opt_x, opt_y);
    }

    return true;
}

bool FemPosDeviationSmoother::SolveWithIpopt(
        const std::vector<std::pair<double, double>>& raw_point2d,
        const std::vector<double>& bounds, std::vector<double>* opt_x,
        std::vector<double>* opt_y) {
    FemPosDeviationIpoptInterface* smoother = new FemPosDeviationIpoptInterface(raw_point2d, bounds);

    smoother->set_weight_fem_pos_deviation(config_.weight_fem_pos_deviation);
    smoother->set_weight_path_length(config_.weight_path_length);
    smoother->set_weight_ref_deviation(config_.weight_ref_deviation);
    smoother->set_weight_curvature_constraint_slack_var(
            config_.weight_curvature_constraint_slack_var);
    smoother->set_curvature_constraint(config_.curvature_constraint);

    Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

    // Create an instance of the IpoptApplication
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetIntegerValue("print_level",
                                    static_cast<int>(config_.print_level));
    app->Options()->SetIntegerValue(
            "max_iter", static_cast<int>(config_.max_num_of_iterations));
    app->Options()->SetIntegerValue(
            "acceptable_iter",
            static_cast<int>(config_.acceptable_num_of_iterations));
    app->Options()->SetNumericValue("tol", config_.tol);
    app->Options()->SetNumericValue("acceptable_tol", config_.acceptable_tol);

    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
        std::cout << "*** Error during initialization!" << std::endl ;
        return false;
    }

    status = app->OptimizeTNLP(problem);

    if (status == Ipopt::Solve_Succeeded ||
        status == Ipopt::Solved_To_Acceptable_Level) {
        // Retrieve some statistics about the solve
        Ipopt::Index iter_count = app->Statistics()->IterationCount();
        std::cout << "*** The problem solved in " << iter_count << " iterations!" << std::endl ;
    } else {
        std::cout << "Solver fails with return code: " << static_cast<int>(status) << std::endl ;
        return false;
    }
    smoother->get_optimization_results(opt_x, opt_y);
    return true;
}


bool FemPosDeviationSmoother::SolveWithOsqp(
        const std::vector<std::pair<double, double>>& raw_point2d,
        const std::vector<double>& bounds, std::vector<double>* opt_x,
        std::vector<double>* opt_y) {
    //CHECK_NOTNULL(opt_x);
    //CHECK_NOTNULL(opt_y);

    FemPosDeviationOsqpInterface solver;

    solver.set_weight_fem_pos_deviation(config_.weight_fem_pos_deviation);
    solver.set_weight_path_length(config_.weight_path_length);
    solver.set_weight_ref_deviation(config_.weight_ref_deviation);

    solver.set_max_iter(config_.max_iter);
    solver.set_time_limit(config_.time_limit);
    solver.set_verbose(config_.verbose);
    solver.set_scaled_termination(config_.scaled_termination);
    solver.set_warm_start(config_.warm_start);

    solver.set_ref_points(raw_point2d);

    solver.set_bounds_around_refs(bounds);

    if (!solver.Solve()) {
        return false;
    }

    *opt_x = solver.opt_x();
    *opt_y = solver.opt_y();
    return true;
}


}  // namespace planning
}  // namespace ADC



