//
// Created by gaoyang on 18-11-23.
//

#ifndef PLANNING_REFERENCE_LINE_PROVIDER_H
#define PLANNING_REFERENCE_LINE_PROVIDER_H

#include <list>
#include "qp_spline_reference_line_smooth.h"

namespace planning
{
class ReferenceLineProvide
{
    public:
        ReferenceLineProvide() = default;
        ~ReferenceLineProvide() = default;

        bool smoothReferenceLine(
                const ReferenceLine &raw_reference_line,
                ReferenceLine *reference_line,
                const double &deltaS, bool dif_time_smooth);

    public:
        struct sampleInformation
        {
            float startS;
            float endS;
            float step;
            bool flag; // true: uTurn; false: is not uTurn
        };

    private:
        AnchorPoint GetAnchorPoint(
                const ReferenceLine& reference_line,
                double s,
                const std::vector<double> &rawReferenceLineKappas) const;

        AnchorPoint GetAnchorPoint(
                const ReferenceLine& reference_line,
                double s,
                const std::vector<double> &accumulateS,
                const std::vector<std::pair<uint16_t, uint16_t>> &uTurnStartEndIndexPair,
                const std::vector<double> &rawReferenceLineKappas) const;

        bool GetAnchorPoints(
                const ReferenceLine &reference_line,
                std::vector<AnchorPoint> *anchor_points,
                std::vector<std::vector<AnchorPoint>> &isnotUTurnPoints,
                const std::vector<double> &rawReferenceLineKappas);

        void caculateRawReferenceLineKappas(
                const ReferenceLine &raw_reference_line,
                std::vector<double> &rawReferenceLineKappas);

        void getUTurnStartEndPositionPair(
                const std::vector<ReferencePoint> &referencePoints,
                const std::vector<double> &accumulateS,
                std::vector<std::pair<uint16_t , uint16_t >> &uTurnStartEndIndexPair);

        bool getSamplePointsSVector(
                const std::vector<ReferencePoint> &referencePoints,
                const std::vector<double> &accumulateS,
                const std::vector<std::pair<uint16_t , uint16_t >> &uTurnStartEndIndexPair,
                std::vector<sampleInformation> &samplePointsS);

        void GetAnchorPoints(
                const std::vector<sampleInformation> &samplePointsS,
                const ReferenceLine &reference_line,
                const std::vector<double> &rawReferenceLineKappas,
                std::vector<AnchorPoint> *anchor_points,
                std::vector<std::vector<AnchorPoint>> &isnotUTurnPoints);
    };
}

#endif //PLANNING_REFERENCE_LINE_PROVIDER_H
