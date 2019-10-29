#pragma once

#include <vector>

#include "reference_line/reference_point.h"
#include "reference_line/reference_line.h"

namespace planning
{
    struct GaussData
    {
        uint index;
        double x;
        double y;
        double heading;
        double s;
        double z;
    };

    struct InsData
    {
        double x;
        double y;
        double heading;
    };

    struct dPoint
    {
        double x;
        double y;
    };

    class TransData
    {
    public:
        TransData() = default;
        ~TransData() = default;

        bool createReferenceLine(ReferenceLine &referenceLine);

    private:
        bool ImportData(std::vector<GaussData>& raw_reference_line);

        bool ImportInsData(std::vector<InsData>& insdata);

        dPoint GaussProjCal(const dPoint bol);
        bool SpareReferenceLine(std::vector<GaussData> &raw_reference_line,
                                std::vector<GaussData> &spare_ference_line);

    };
}
