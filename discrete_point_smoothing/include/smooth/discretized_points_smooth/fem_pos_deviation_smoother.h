#pragma once

#include <utility>
#include <vector>

#include "config/fem_pos_deviation_smoother_config.h"

namespace ADC {
namespace planning {

class FemPosDeviationSmoother {
public:
    explicit FemPosDeviationSmoother(const FemPosDeviationSmootherConfig& config);

    bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
               const std::vector<double>& bounds, std::vector<double>* opt_x,
               std::vector<double>* opt_y);

    bool SolveWithIpopt(const std::vector<std::pair<double, double>>& raw_point2d,
                        const std::vector<double>& bounds,
                        std::vector<double>* opt_x, std::vector<double>* opt_y);

    bool SolveWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                       const std::vector<double>& bounds,
                       std::vector<double>* opt_x, std::vector<double>* opt_y);

private:
    FemPosDeviationSmootherConfig config_;
};
}  // namespace planning
}  // namespace ADC
