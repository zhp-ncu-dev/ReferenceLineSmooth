//
// Created by gaoyang on 6/27/19.
//

#ifndef PLANNING_TRAJECTORY_POINT_H
#define PLANNING_TRAJECTORY_POINT_H

#include "path_point.h"

namespace planning
{
    class TrajectoryPoint
    {
    public:
        TrajectoryPoint() = default;

        const PathPoint &pathPoint()const{
            return m_path_point;
        }

        PathPoint *mutablePathPoint()
        {
            return &m_path_point;
        }

        double v()const{
            return m_v;
        }

        double a()const{
            return m_a;
        }

        double relativeTime()const{
            return m_relative_time;
        }

        void setPathPoint(const PathPoint &pathPoint1){
            m_path_point = pathPoint1;
        }

        void setV(const double speed)
        {
            m_v = speed;
        }

        void setA(const double a)
        {
            m_a = a;
        }

        void setRelativeTime(const double relativeTime)
        {
            m_relative_time = relativeTime;
        }

    private:
        PathPoint m_path_point;

        // linear velocity
        double m_v;  // in [m/s]
        // linear acceleration
        double m_a;
        // relative time from beginning of the trajectory
        double m_relative_time;
    };
}//namespace planning

#endif //PLANNING_TRAJECTORY_POINT_H
