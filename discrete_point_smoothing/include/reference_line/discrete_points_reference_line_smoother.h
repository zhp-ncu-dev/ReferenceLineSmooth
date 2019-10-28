#pragma once

#include <string>
#include <utility>
#include <vector>

#include "config/reference_line_smoother_config.h"
#include "reference_line.h"
#include "reference_line_smoother.h"
#include "reference_point.h"

namespace ADC {
namespace planning {

class DiscretePointsReferenceLineSmoother : public ReferenceLineSmoother {
public:
    // 建立默认构造函数
    DiscretePointsReferenceLineSmoother() = default;

    explicit DiscretePointsReferenceLineSmoother(
            const ReferenceLineSmootherConfig& config);

    virtual ~DiscretePointsReferenceLineSmoother() = default;

    bool Smooth(const GaussData& raw_reference_line,
                ReferenceLine* const smoothed_reference_line) override;
    bool Smooth(const std::vector<ADC::planning::GaussData>& raw_reference_line,
                std::vector<SmoothedReferencePoint> *ref_points);

    void SetAnchorPoints(const std::vector<AnchorPoint>&) override;

    void SetAnchorPoints(const std::vector<GaussData>& sparepoint,const std::vector<GaussData> &reference_points_) ;

private:
    bool FemPosSmooth(
            const std::vector<std::pair<double, double>>& raw_point2d,
            const std::vector<double>& bounds,
            std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

    void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

    void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

    bool GenerateRefPointProfile(
            const std::vector<ADC::planning::GaussData>& raw_reference_line,
            const std::vector<std::pair<double, double>>& xy_points,
            std::vector<SmoothedReferencePoint>* reference_points);

    std::vector<AnchorPoint> anchor_points_;
    std::vector<double> accumulated_s_;

    double zero_x_ = 0.0;
    double zero_y_ = 0.0;
    double kSampleDistance = 0.20;

    bool XYToSL(const common::math::Vec2d &vec2d, SLPoint &slPoint, const std::vector<GaussData>& reference_points_)const;
    bool getNearstPoint(const common::math::Vec2d &point,int& location,const double &minDis,const std::vector<GaussData>& reference_points_) const;

    bool SLToXY(const SLPoint& sl_point,
                common::math::Vec2d* const xy_point,
                const std::vector<GaussData>& reference_points_) const;
    GaussData getNearstPoint(
            const double s,
            const std::vector<GaussData>& reference_points_) const;

    ReferenceLineSmootherConfig spare_smoother;

    GaussData GetReferencePoint(
            const double s,
            const std::vector<ADC::planning::GaussData>& reference_line);
    InterpolatedIndex GetIndexFromS(
            double s ,
            const std::vector<ADC::planning::GaussData>& reference_line) const;
    GaussData InterpolateWithMatchedIndex(
            const GaussData& p0, const double s0,
            const GaussData&p1, const double s1, const InterpolatedIndex& index);

    void InitPoints();
    int num_points_ = 0;
    int num_sample_points_ = 0;
    std::vector<int> last_point_index_;
    std::vector<double> accumulated_s_path_;
    std::vector<common::math::Vec2d> unit_directions_;
    double length_ = 0.0;


};

}  // namespace planning
}  // namespace ADC
