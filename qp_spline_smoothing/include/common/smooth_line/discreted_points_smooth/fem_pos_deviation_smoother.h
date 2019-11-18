#pragma once

#include <utility>
#include <vector>
#include "common/config/reference_line_config/smooth_reference_line_config.h"

namespace planning
{
    class FemPosDeviationSmoother
    {
    public:
        explicit FemPosDeviationSmoother(const FemPosDeviationSmootherConfig& config);

        bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
                   const std::vector<double>& bounds, std::vector<double>* opt_x,
                   std::vector<double>* opt_y);

        bool SolveWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                           const std::vector<double>& bounds,
                           std::vector<double>* opt_x, std::vector<double>* opt_y);

        bool SqpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                         const std::vector<double>& bounds,
                         std::vector<double>* opt_x, std::vector<double>* opt_y);
    private:
        FemPosDeviationSmootherConfig m_femPosDeviationSmootherConfig;
    };

} // end namespace

