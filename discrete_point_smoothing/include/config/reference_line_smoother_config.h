#pragma once

#include <sys/types.h>
#include "fem_pos_deviation_smoother_config.h"

namespace ADC {
namespace planning {

class DiscretePointsSmootherConfig {
public:
    enum SmoothingMethod {
        NOT_DEFINED,
        COS_THETA_SMOOTHING,
        FEM_POS_DEVIATION_SMOOTHING
    };
    SmoothingMethod smoothing_method = FEM_POS_DEVIATION_SMOOTHING;

    FemPosDeviationSmootherConfig fem_pos_deviation_smoothing;
};

class ReferenceLineSmootherConfig {
public:

    // The output resolution for discrete point smoother reference line is
    // directly decided by max_constraint_interval

    double max_constraint_interval = 2.2;

    double longitudinal_boundary_bound = 0.5;
    double lateral_boundary_bound = 0.5;

    double max_lateral_boundary_bound = 0.5;
    double min_lateral_boundary_bound = 0.2;

    // The output resolution for qp smoother reference line.
    int32_t num_of_total_points = 500;
    double curb_shift = 0.2;
    double lateral_buffer = 0.2;

    // The output resolution for spiral smoother reference line.
    double resolution = 0.02;

    DiscretePointsSmootherConfig discrete_points;

};

}   // planning
}   // ADC
