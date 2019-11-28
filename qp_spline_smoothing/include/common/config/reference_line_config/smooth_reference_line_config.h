//
// Created by gaoyang on 18-11-26.
//

#ifndef PLANNING_SMOOTH_REFERENCE_LINE_CONFIG_H
#define PLANNING_SMOOTH_REFERENCE_LINE_CONFIG_H

#include <stdint-gcc.h>
#include <math.h>
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
    #define FLAGS_default_qp_iteration_num 100000
    #define FLAGS_default_enable_active_set_debug_info false

    const double FLAGS_smoothLongDisTime = 6.0;
    const double FLAGS_smoothShortDis = 200.0;
    const double FLAGS_smoothShortBackDis = 30.0;
    const double FLAGS_smooth_period = 50000;

    // true:  discrete-points
    // false: qp-spline
    const bool ReferenceLineSmoothAlgorithm = false;

    struct FemPosDeviationSmootherConfig
    {
        double freewayRoadInterval;
        float urbanRoadInterval;             // 城市道路
        float urbanRoadTypeThreshold;
        double freewayLongitudinalBoundaryBound;
        double freewayLateralBoundaryBound;
        double turnLongitudinalBoundaryBound;
        double turnLateralBoundaryBound;
        double uTurnLongitudinalBoundaryBound;
        double uTurnLateralBoundaryBound;
        double weightFemPosDeviation;
        double weightRefDeviation;
        double weightPathLength;
        int32_t maxIter;
        double timeLimit;
        bool verbose;
        bool scaledTermination;
        bool warmStart;
        double weightCurvatureConstraintSlackVar;
        double curvatureConstraint;
        int32_t sqpSubMaxIter;
        double sqpFtol;
        int32_t sqpPenMaxIter;
        double sqpCtol;

        FemPosDeviationSmootherConfig() :
        freewayRoadInterval(2.0),
        urbanRoadInterval(1.0),
        urbanRoadTypeThreshold(5.0),
        freewayLongitudinalBoundaryBound(0.3),
        freewayLateralBoundaryBound(0.3),
        turnLongitudinalBoundaryBound(0.15),
        turnLateralBoundaryBound(0.15),
        uTurnLongitudinalBoundaryBound(1.0e-2),
        uTurnLateralBoundaryBound(1.0e-2),
        weightFemPosDeviation(1.0e5),
        weightRefDeviation(1.0),
        weightPathLength(1.0),
        maxIter(10000),
        timeLimit(0.0),
        verbose(false),
        scaledTermination(true),
        warmStart(true),
        weightCurvatureConstraintSlackVar(1.0e2),
        curvatureConstraint(0.2),
        sqpFtol(1.0e-4),
        sqpCtol(1.0e-3),
        sqpPenMaxIter(10),
        sqpSubMaxIter(100)
        {}
    };
}
#endif //PLANNING_SMOOTH_REFERENCE_LINE_H