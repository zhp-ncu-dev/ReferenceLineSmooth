//
// Created by gaoyang on 7/2/19.
//

#ifndef PLANNING_MAP_POINT_H
#define PLANNING_MAP_POINT_H

#include "vec2d.h"

using namespace math;

namespace had_map
{
    class MapPoint : public Vec2d
    {
    public:
        MapPoint() = default;
        MapPoint(const Vec2d& point, const double heading)
                :Vec2d(point.x(), point.y()),m_heading(heading){}

        double heading() const {return m_heading;}
        void setHeading(const double heading) { m_heading = heading;}

    private:
        double m_heading = 0.0;    //radian
    };

}

#endif //PLANNING_MAP_POINT_H
