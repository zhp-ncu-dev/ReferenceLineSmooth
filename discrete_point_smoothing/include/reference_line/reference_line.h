#pragma once

#include <vector>

#include "proto/pnc_point.h"
#include "config/reference_line_smoother_config.h"
#include "reference_point.h"
#include "../common/math/vec2d.h"

namespace ADC {
namespace planning {

class ReferenceLine {
public:
    ReferenceLine();

    ReferenceLine(const std::vector<planning::ReferencePoint>& reference_points,
                  const std::vector<double>& accumulate_s);


    bool XYToSL(const ADC::common::math::Vec2d &point, SLPoint &slPoint)const;




private:
//        double disPointToLine(const had_map::MapPoint point, const had_map::MapPoint orgin_point) const;

    void RemoveDuplicates();

    double distanceTwoPoints(ReferencePoint& point1, ReferencePoint& point2) const;

    void reCalculateS();

private:
    std::vector<ReferencePoint> reference_points_;

    std::vector<double> accumulate_s_;

    double spacing_dis_;

    bool smooth_flag_;

    bool m_laneId;
};

}  // namespace planning
}  // namespace ADC

