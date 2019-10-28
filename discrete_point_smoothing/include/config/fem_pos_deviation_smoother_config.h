#pragma once

namespace ADC {
namespace planning {

class FemPosDeviationSmootherConfig{
public:
    /*
    double weight_fem_pos_deviation = 1.0e10;
    double weight_ref_deviation =  1.0;
    double weight_path_length =  1.0;
    // true -- ipopt ; false -- osqp
    bool apply_curvature_constraint = false;  // 默认使用 osqp 平滑求解器
    double weight_curvature_constraint_slack_var = 1.0e2;
    */

    double weight_fem_pos_deviation = 1.0e20;
    double weight_ref_deviation =  1.0;
    double weight_path_length =  1.0;

    // true -- ipopt ; false -- osqp
    bool apply_curvature_constraint = false;  // 默认使用 osqp 平滑求解器
    double weight_curvature_constraint_slack_var = 1.0e2;

    double curvature_constraint = 0.2;

    // ipopt settings
    int32_t print_level = 0;
    int32_t max_num_of_iterations = 5000;
    int32_t acceptable_num_of_iterations = 15;
    double tol = 1e-8;
    double acceptable_tol = 1e-1;

    // osqp settings
    int32_t max_iter = 5000;
    // time_limit set to be 0.0 meaning no time limit
    double time_limit = 0.0;
    bool verbose = false;
    bool scaled_termination = true;
    bool warm_start = true;

};

}  // planing
}  // ADC
