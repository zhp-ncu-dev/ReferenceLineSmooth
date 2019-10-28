//
// Created by gaoyang on 6/27/19.
//

#ifndef PLANNING_DISCRETIZED_PATH_H
#define PLANNING_DISCRETIZED_PATH_H

#include <vector>
#include <cstdint>

#include "common/pnc_point/path_point.h"

namespace planning {

class DiscretizedPath {
public:
    DiscretizedPath() = default;

    explicit DiscretizedPath(const std::vector<PathPoint>& path_points);

    virtual ~DiscretizedPath() = default;

    void set_path_points(const std::vector<PathPoint>& path_points);

    double Length() const;

    const PathPoint &StartPoint() const;

    const PathPoint &EndPoint() const;

    PathPoint Evaluate(const double path_s) const;

    const std::vector<PathPoint>& path_points() const;

    std::uint32_t NumOfPoints() const;

    virtual void Clear();

protected:
    std::vector<PathPoint>::const_iterator QueryLowerBound(
            const double path_s) const;

    std::vector<PathPoint> path_points_;
    PathPoint m_nothingPoint;
};

}  // namespace planning

#endif //PLANNING_DISCRETIZED_PATH_H
