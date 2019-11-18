#pragma once

#include <utility>
#include <vector>
#include "reference_line/reference_line.h"


namespace planning
{
    class DiscretePointsMath
    {
    public:
        DiscretePointsMath() = delete;

        static bool ComputePathProfile(
                const std::vector<std::pair<double, double>>& xy_points,
                std::vector<double>* headings, std::vector<double>* accumulated_s,
                std::vector<double>* kappas, std::vector<double>* dkappas);

        static void ComputeOriginalPathKappaProfile(
               const ReferenceLine &referenceLine,
               std::vector<double>* kappas);
    };

}// end namespace