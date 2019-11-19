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
        int type;
        double s;
        double z;
    };

    struct InsData
    {
        double x;
        double y;
        double heading;
        int type;
    };

    struct TestInsData
    {
        double x;
        double y;
        double heading;
        TestInsData(double a, double b, double c) :
                x(a), y(b), heading(c)
        {}
    };

    struct TestFrame
    {
        double x;
        double y;
        double height;
        double heading;
        double speed;
        double roll;
        double pitch;
        double acc;
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

        void createPathGuass(const std::vector<ReferencePoint> &referencePoints);

        void createBaseLineData(const std::vector<GaussData> &raw_reference_line);

    private:
        bool ImportData(std::vector<GaussData>& raw_reference_line);

        bool ImportInsData(std::vector<InsData>& insdata);

        bool ImportTestFrameData(std::vector<planning::TestInsData> &insData);

        dPoint GaussProjCal(const dPoint bol);

        bool SpareReferenceLine(std::vector<GaussData> &raw_reference_line,
                                std::vector<GaussData> &spare_ference_line);

        int getLaneDefineType(int type);
    };
}
