//
// Created by gaoyang on 6/27/19.
//

#include "common/path/path_data.h"

#include <algorithm>
#include <limits>
#include <vector>
#include "common/math/cartesian_frenet_conversion.h"

namespace planning {

bool PathData::SetDiscretizedPath(const DiscretizedPath &path) {
    if (reference_line_ == nullptr) {
        return false;
    }
    discretized_path_ = path;
    if (!XYToSL(discretized_path_, &frenet_path_)) {
        return false;
    }
    path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
    return true;
}

bool PathData::SetFrenetPath(const FrenetFramePath &frenet_path) {
    if (reference_line_ == nullptr) {
        return false;
    }
    frenet_path_ = frenet_path;
    if (!SLToXY(frenet_path_, &discretized_path_)) {
        return false;
    }
    path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
    return true;
}

const DiscretizedPath &PathData::discretized_path() const {
    return discretized_path_;
}

bool PathData::Empty() const {
    return discretized_path_.NumOfPoints() == 0 &&
           frenet_path_.NumOfPoints() == 0;
}

std::list<std::pair<DiscretizedPath, FrenetFramePath>>
&PathData::path_data_history() {
    return path_data_history_;
}

const FrenetFramePath &PathData::frenet_frame_path() const {
    return frenet_path_;
}

void PathData::SetReferenceLine(const ReferenceLine *reference_line) {
    Clear();
    reference_line_ = reference_line;
}

bool PathData::GetPathPointWithPathS(
        const double s, PathPoint *const path_point) const {
    *path_point = discretized_path_.Evaluate(s);
    return true;
}

bool PathData::GetPathPointWithRefS(const double ref_s,
                                    PathPoint *const path_point) const {
    if(reference_line_ == NULL)
    {
        return false;
    }
    if(path_point == NULL)
    {
        return false;
    }
    if(discretized_path_.path_points().size() != frenet_path_.points().size())
    {
        return false;
    }
    if (ref_s < 0) {
        return false;
    }
    if (ref_s > frenet_path_.points().back().s()) {
        return false;
    }

    uint32_t index = 0;
    const double kDistanceEpsilon = 1e-3;
    for (uint32_t i = 0; i + 1 < frenet_path_.points().size(); ++i) {
        if (fabs(ref_s - frenet_path_.points().at(i).s()) < kDistanceEpsilon) {
            path_point->CopyFrom(discretized_path_.path_points().at(i));
            return true;
        }
        if (frenet_path_.points().at(i).s() < ref_s &&
            ref_s <= frenet_path_.points().at(i + 1).s()) {
            index = i;
            break;
        }
    }
    double r = (ref_s - frenet_path_.points().at(index).s()) /
               (frenet_path_.points().at(index + 1).s() -
                frenet_path_.points().at(index).s());

    const double discretized_path_s =
            discretized_path_.path_points().at(index).s() +
            r * (discretized_path_.path_points().at(index + 1).s() -
                 discretized_path_.path_points().at(index).s());
    path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));

    return true;
}

void PathData::Clear() {
    discretized_path_.Clear();
    frenet_path_.Clear();
    reference_line_ = nullptr;
}

bool PathData::SLToXY(const FrenetFramePath &frenet_path,
                      DiscretizedPath *const discretized_path) {
    std::vector<PathPoint> path_points;
    for (const FrenetFramePoint &frenet_point : frenet_path.points()) {
        SLPoint sl_point;
        math::Vec2d cartesian_point;
        sl_point.setS((frenet_point.s()));
        sl_point.setL((frenet_point.l()));
        if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
            return false;
        }
        //todo:轨迹应按照线性差值给出
        ReferencePoint ref_point =
                reference_line_->getReferencePoint(frenet_point.s());
        double theta = math::CartesianFrenetConverter::CalculateTheta(
                ref_point.pointInfo().heading(), ref_point.kappa(), frenet_point.l(),
                frenet_point.dl());
        double kappa = math::CartesianFrenetConverter::CalculateKappa(
                ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
                frenet_point.dl(), frenet_point.ddl());
//        printf("s = %f, kappa = %f,rkappa = %f, rdkappa = %f, l = %f, dl = %f, ddl = %f,\r\n",
//        frenet_point.s(), kappa,ref_point.kappa(), ref_point.dkappa(), frenet_point.l(), frenet_point.dl(),frenet_point.ddl());
        PathPoint path_point(
                cartesian_point.x(), cartesian_point.y(), 0.0, theta, kappa, 0.0, 0.0);

        if (path_points.empty()) {
            path_point.setS(0.0);
            path_point.setDkappa(0.0);
        } else {
            math::Vec2d last(path_points.back().x(), path_points.back().y());
            math::Vec2d current(path_point.x(), path_point.y());
            double distance = (last - current).Length();
            path_point.setS(path_points.back().s() + distance);
            path_point.setDkappa((path_point.kappa() - path_points.back().kappa()) /
                                  distance);
        }
        path_points.push_back(std::move(path_point));
    }
    *discretized_path = DiscretizedPath(std::move(path_points));

    return true;
}

bool PathData::XYToSL(const DiscretizedPath &discretized_path,
                      FrenetFramePath *const frenet_path) {
    std::vector<FrenetFramePoint> frenet_frame_points;
    const double max_len = reference_line_->length();
    for (const auto &path_point : discretized_path.path_points()) {
        SLPoint sl_point;
        math::Vec2d point;
        point.set_x(path_point.x());
        point.set_y(path_point.y());
        if (!reference_line_->XYToSL(point, sl_point)) {
            return false;
        }
        FrenetFramePoint frenet_point;
        // NOTICE: does not set dl and ddl here. Add if needed.
        frenet_point.setS(std::max(0.0, std::min(sl_point.s(), max_len)));
        frenet_point.setL(sl_point.l());
        frenet_frame_points.push_back(std::move(frenet_point));
    }
    *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
    return true;
}

}  // namespace planning