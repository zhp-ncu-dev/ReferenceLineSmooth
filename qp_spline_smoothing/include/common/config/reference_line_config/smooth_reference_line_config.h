//
// Created by gaoyang on 18-11-26.
//

#ifndef PLANNING_SMOOTH_REFERENCE_LINE_CONFIG_H
#define PLANNING_SMOOTH_REFERENCE_LINE_CONFIG_H

#include <stdint-gcc.h>

namespace planning
{
    #define FLAGS_longitudinal_boundary_bound 1

    #define FLAGS_lateral_boundary_bound 0.1

    #define FLAGS_max_point_interval 5.0

    #define FLAGS_max_spline_length 25.0

    #define FLAGS_smooth_max_spline 8

    #define FLAGS_SPLINE_ORDER 6.0

    #define FLAGS_second_derivative_weight 1000.0
    #define FLAGS_third_derivative_weight 500.0
    #define FLAGS_regularization_weight 1e-5

    #define RESOLUTION 0.1

    #define MINDISTANCEFROMTWOPOINTS 0.05

    #define FLAGS_smooth_line_default_active_set_eps_num (-1e-6)
    #define FLAGS_smooth_line_default_active_set_eps_iter_ref 1e-6
    #define FLAGS_smooth_line_default_active_set_eps_den 1e-6
    #define FLAGS_enable_sqp_solver true
    #define FLAGS_default_qp_iteration_num 10000
    #define FLAGS_default_enable_active_set_debug_info false

    const double FLAGS_smoothLongDisTime = 6.0;
    const double FLAGS_smoothShortDis = 200.0;
    const double FLAGS_smoothShortBackDis = 30.0;
    const double FLAGS_smooth_period = 50000;

    // true:  discrete-points
    // false: qp-spline
    const bool ReferenceLineSmoothAlgorithm = true;

    struct FemPosDeviationSmootherConfig
    {
        double maxConstraintInterval;
        double longitudinalBoundaryBound;
        double lateralBoundaryBound;
        double weightFemPosDeviation;
        double weightRefDeviation;
        double weightPathLength;
        int32_t maxIter;
        double timeLimit;
        bool verbose;
        bool scaledTermination;
        bool warmStart;

        FemPosDeviationSmootherConfig() :
        maxConstraintInterval(0.5),
        longitudinalBoundaryBound(0.25),
        lateralBoundaryBound(0.25),
        weightFemPosDeviation(1.0e5),
        weightRefDeviation(1.0),
        weightPathLength(1.0),
        maxIter(500000),
        timeLimit(0.0),
        verbose(false),
        scaledTermination(true),
        warmStart(true)
        {}
    };
}
#endif //PLANNING_SMOOTH_REFERENCE_LINE_H