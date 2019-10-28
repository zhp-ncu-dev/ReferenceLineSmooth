//
// Created by gaoyang on 6/27/19.
//

#ifndef PLANNING_PATH_DATA_H
#define PLANNING_PATH_DATA_H

#include <list>
#include <string>
#include <utility>

#include "common/path/discretized_path.h"
#include "common/path/frenet_frame_path.h"
#include "reference_line/reference_line.h"

namespace planning {

class PathData {
public:
    PathData() = default;

    bool SetDiscretizedPath(const DiscretizedPath &path);

    bool SetFrenetPath(const FrenetFramePath &frenet_path);

    void SetReferenceLine(const ReferenceLine *reference_line);

    const DiscretizedPath &discretized_path() const;

    const FrenetFramePath &frenet_frame_path() const;

    bool GetPathPointWithPathS(const double s,
                               PathPoint *const path_point) const;

    std::list<std::pair<DiscretizedPath, FrenetFramePath>> &path_data_history();

    /*
     * brief: this function will find the path_point in discretized_path whose
     * projection to reference line has s value closest to ref_s.
     */
    bool GetPathPointWithRefS(const double ref_s,
                              PathPoint *const path_point) const;

    void Clear();

    bool Empty() const;

private:
    /*
     * convert frenet path to cartesian path by reference line
     */
    bool SLToXY(const FrenetFramePath &frenet_path,
                DiscretizedPath *const discretized_path);
    bool XYToSL(const DiscretizedPath &discretized_path,
                FrenetFramePath *const frenet_path);
    const ReferenceLine *reference_line_ = nullptr;
    DiscretizedPath discretized_path_;
    FrenetFramePath frenet_path_;
    std::list<std::pair<DiscretizedPath, FrenetFramePath>> path_data_history_;
};

}  // namespace planning

#endif //PLANNING_PATH_DATA_H
