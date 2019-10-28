
#include "reference_line/discrete_points_reference_line_smoother.h"

#include <algorithm>
#include <iostream>
#include <sys/time.h>

#include "smooth/discrete_points_math.h"
#include "smooth/discretized_points_smooth/fem_pos_deviation_smoother.h"
#include "trans/trans_data.h"
#include "common/math/linear_interpolation.h"
#include "smooth/gradient_descent_smooth/gradient_descent_smoothing.h"

using namespace HybridAStar;

namespace ADC {
namespace planning {

DiscretePointsReferenceLineSmoother::DiscretePointsReferenceLineSmoother(
        const ReferenceLineSmootherConfig& config)
        : ReferenceLineSmoother(config) {}

bool DiscretePointsReferenceLineSmoother::Smooth(
        const GaussData& raw_reference_line,
        ReferenceLine* const smoothed_reference_line) {
    std::vector<std::pair<double, double>> raw_point2d;
    std::vector<double> anchorpoints_lateralbound;

    // 存储待平滑的 anchor_point 点，以及每一个 anchor_point 的横向约束
    for (const auto& anchor_point : anchor_points_) {
        raw_point2d.emplace_back(anchor_point.path_point.x,
                                 anchor_point.path_point.y);
        anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
    }

    // fix front and back points to avoid end states deviate from the center of road
    anchorpoints_lateralbound.front() = 0.0;
    anchorpoints_lateralbound.back() = 0.0;

    // 每一个点减去第一个点，进行规范化
    NormalizePoints(&raw_point2d);

    bool status = false;

    // TODO:
    // 平滑方法选择
    const auto& smoothing_method = config_.discrete_points.smoothing_method;
    std::vector<std::pair<double, double>> smoothed_point2d;
    switch (smoothing_method) {
        case DiscretePointsSmootherConfig::FEM_POS_DEVIATION_SMOOTHING:
            status = FemPosSmooth(raw_point2d, anchorpoints_lateralbound,
                                  &smoothed_point2d);
            break;
        default:
        std::cout << "Smoother type not defined"  << std::endl ;
            return false;
    }

    if (!status) {
        std::cout << "discrete_points reference line smoother fails"  << std::endl ;
        return false;
    }

    // 平滑点还原，加上第一个点的坐标
    DeNormalizePoints(&smoothed_point2d);

    return true;
}

bool DiscretePointsReferenceLineSmoother::Smooth(
        const std::vector<ADC::planning::GaussData>& raw_reference_line,
        std::vector<SmoothedReferencePoint> *ref_points) {

    std::vector<std::pair<double, double>> raw_point2d;
    std::vector<double> anchorpoints_lateralbound;

    struct timeval timeStart,timeEnd;
    double spendTime;
    gettimeofday(&timeStart,0);

    // 存储待平滑的 anchor_point 点，以及每一个 anchor_point 的横向约束
    for (const auto& anchor_point : anchor_points_) {
        raw_point2d.emplace_back(anchor_point.path_point.x,
                                 anchor_point.path_point.y);
        anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
    }

    // fix front and back points to avoid end states deviate from the center of road
    anchorpoints_lateralbound.front() = 0.0;
    anchorpoints_lateralbound.back()  = 0.0;

    NormalizePoints(&raw_point2d);

    bool status = false;
    const auto& smoothing_method = config_.discrete_points.smoothing_method;
    std::vector<std::pair<double, double>> smoothed_point2d;
    switch (smoothing_method) {
        case DiscretePointsSmootherConfig::FEM_POS_DEVIATION_SMOOTHING:
            status = FemPosSmooth(raw_point2d, anchorpoints_lateralbound,
                                  &smoothed_point2d);
            break;
        default:
            std::cout << "Smoother type not defined"  << std::endl ;
            return false;
    }
    if (!status) {
        std::cout << "discrete_points reference line smoother fails" << std::endl ;
        return false;
    }

    DeNormalizePoints(&smoothed_point2d);

    GenerateRefPointProfile(raw_reference_line,smoothed_point2d, ref_points);

    gettimeofday(&timeEnd,0);
    spendTime=(timeEnd.tv_sec-timeStart.tv_sec)*1000+(timeEnd.tv_usec-timeStart.tv_usec)/1000;
    std::cout << "离散点平滑算法消耗时间 = " << spendTime << std::endl;

    // abcd:

/*
// 梯度下降法优化曲率超过极限值的地方
    gettimeofday(&timeStart,0);

    Smoother smoother;
    std::vector<dPoint3d> path_3d;
    dPoint3d path3d;
    for(int i = 0; i < raw_point2d.size(); ++i){
        path3d.x = raw_point2d[i].first;
        path3d.y = raw_point2d[i].second;
        path3d.heading = 0.0;
        path_3d.push_back(path3d);
    }
    smoother.tracePath(path_3d);
    smoother.smoothPath();
    std::vector<dPoint3d> gradient_descent_smoothpoint = smoother.getPath();

    // 对 经过梯度下降法 平滑后的点，进行平滑点信息求解 【无法限制曲率，平滑效果并不好】
    std::vector<std::pair<double, double>> gradient_descent_smoothed_point2d;
    for(dPoint3d point : gradient_descent_smoothpoint){
        gradient_descent_smoothed_point2d.emplace_back(point.x , point.y);
    }

    GenerateRefPointProfile(raw_reference_line,gradient_descent_smoothed_point2d, ref_points);

    // 计算梯度下降法平滑算法所消耗的时间
    gettimeofday(&timeEnd,0);
    spendTime=(timeEnd.tv_sec-timeStart.tv_sec)*1000+(timeEnd.tv_usec-timeStart.tv_usec)/1000;
    std::cout << "梯度下降法平滑算法消耗时间 = " << spendTime << std::endl;
*/
    return true;
}

bool DiscretePointsReferenceLineSmoother::FemPosSmooth(
        const std::vector<std::pair<double, double>>& raw_point2d,
        const std::vector<double>& bounds,
        std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
    const auto& fem_pos_config =
            config_.discrete_points.fem_pos_deviation_smoothing;

    FemPosDeviationSmoother smoother(fem_pos_config);

    // box contraints on pos are used in fem pos smoother, thus shrink the
    // bounds by 1.0 / sqrt(2.0)
    std::vector<double> box_bounds = bounds;
    const double box_ratio = 1.0 / std::sqrt(2.0);
    for (auto& bound : box_bounds) {
        bound *= box_ratio;
    }

    std::vector<double> opt_x;
    std::vector<double> opt_y;
    bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

    if (!status) {
        std::cout << "Fem Pos reference line smoothing failed" << std::endl ;
        return false;
    }

    if (opt_x.size() < 2 || opt_y.size() < 2) {
        std::cout << "Return by fem pos smoother is wrong. Size smaller than 2 " << std::endl ;
        return false;
    }

    size_t point_size = opt_x.size();
    for (size_t i = 0; i < point_size; ++i) {
        ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
    }

    return true;
}

// 转存 anchor_points_
void DiscretePointsReferenceLineSmoother::SetAnchorPoints(
        const std::vector<AnchorPoint>& anchor_points) {
    anchor_points_ = anchor_points;
}

// 增加设置 sparepoint 接口
void DiscretePointsReferenceLineSmoother::SetAnchorPoints(const std::vector<GaussData> &spare_ference_line,const std::vector<GaussData> &reference_points_) {

// TODO:
// 以 原始稀疏参考线，计算 heading , kappa , dkappa
// Compute path profile
    std::vector<double> s_headings;
    std::vector<double> s_kappas;
    std::vector<double> s_dkappas;
    std::vector<double> s_accumulated_s;
    std::vector<std::pair<double, double>> s_xy_points;

    int iNum = spare_ference_line.size();

    for(int i = 0; i < iNum; ++i){
        std::pair<double ,double> xy(spare_ference_line[i].x,spare_ference_line[i].y);
        s_xy_points.push_back(xy);
    }

    if (!DiscretePointsMath::ComputePathProfile(
            s_xy_points, &s_headings, &s_accumulated_s, &s_kappas, &s_dkappas)) {
        std::cout << "计算pathfile出现错误！！！" << std::endl;
    }

    // 计算最大的曲率值，根据曲率值来给定 lateral_bound
    double max_kappa = std::fabs(s_kappas[0]);
    for(int i = 1; i < s_kappas.size(); ++i){
        if(max_kappa < std::fabs(s_kappas[i])){
            max_kappa = std::fabs(s_kappas[i]);
        }
    }
//
    AnchorPoint anchorpoint_;

    ReferenceLineSmootherConfig config;

    for(int i = 0; i < iNum; ++i){
        anchorpoint_.path_point.x = spare_ference_line[i].x;
        anchorpoint_.path_point.y = spare_ference_line[i].y;
        anchorpoint_.path_point.z = spare_ference_line[i].z;
        anchorpoint_.path_point.theta = spare_ference_line[i].heading;
        anchorpoint_.path_point.s = 0.0;
        anchorpoint_.path_point.kappa = 0.0;
        anchorpoint_.path_point.dkappa = 0.0;
        anchorpoint_.path_point.ddkappa = 0.0;
        anchorpoint_.path_point.x_derivative = 0.0;
        anchorpoint_.path_point.y_derivative = 0.0;
        anchorpoint_.lateral_bound = config.longitudinal_boundary_bound;
        anchorpoint_.longitudinal_bound = config.lateral_boundary_bound;
        anchorpoint_.enforced = false;
        anchorpoint_.index = spare_ference_line[i].index;

        anchor_points_.push_back(anchorpoint_);
    }
    std::cout << "anchor_points_ is set OK !" << std::endl ;
}

// anchor_points_ 减去第一个点，进行规范化
void DiscretePointsReferenceLineSmoother::NormalizePoints(
        std::vector<std::pair<double, double>>* xy_points) {

    zero_x_ = xy_points->front().first;
    zero_y_ = xy_points->front().second;
    std::for_each(xy_points->begin(), xy_points->end(),
                  [this](std::pair<double, double>& point) {
                      auto curr_x = point.first;
                      auto curr_y = point.second;
                      std::pair<double, double> xy(curr_x - zero_x_,
                                                   curr_y - zero_y_);
                      point = std::move(xy);
                  });
}

void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
        std::vector<std::pair<double, double>>* xy_points) {
    std::for_each(xy_points->begin(), xy_points->end(),
                  [this](std::pair<double, double>& point) {
                      auto curr_x = point.first;
                      auto curr_y = point.second;
                      std::pair<double, double> xy(curr_x + zero_x_,
                                                   curr_y + zero_y_);
                      point = std::move(xy);
                  });
}

bool DiscretePointsReferenceLineSmoother::GenerateRefPointProfile(
        const std::vector<ADC::planning::GaussData>& reference_line,
        const std::vector<std::pair<double, double>> &xy_points,
        std::vector<SmoothedReferencePoint> *reference_points) {

    accumulated_s_.clear();
    for(int i = 0; i < reference_line.size(); ++i){
        accumulated_s_.push_back(reference_line[i].s);
    }

// 以 优化点 来计算 heading,kappa,dkappa
    std::vector<double> headings;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::vector<double> accumulated_s;

    if (!DiscretePointsMath::ComputePathProfile(
            xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
        return false;
    }

    size_t points_size = xy_points.size();
    SmoothedReferencePoint reference_point;
    for(size_t i = 0; i < points_size; ++i){
        reference_point.x = xy_points[i].first;
        reference_point.y = xy_points[i].second;
        reference_point.heading = headings[i];
        reference_point.kappa = kappas[i];
        reference_point.dkappa = dkappas[i];
        reference_point.s = accumulated_s[i];
        reference_points->emplace_back(reference_point);
    }

    return true;
}

// 坐标转换
bool DiscretePointsReferenceLineSmoother::XYToSL(
            const common::math::Vec2d &vec2d,
            SLPoint &slPoint,
            const std::vector<GaussData>& reference_points_)const
{
    int index;
    double maxDis = 100;
    if(!getNearstPoint(vec2d,index,maxDis,reference_points_))
    {
        return false;
    }
    double radian = reference_points_[index].heading*3.1415926/180;
    slPoint.s = accumulated_s_[index] + (vec2d.x() - reference_points_[index].x) * sin(radian)
                + (vec2d.y() - reference_points_[index].y) * cos(radian);
    slPoint.l = -(vec2d.x() - reference_points_[index].x) * std::cos(radian)
                + (vec2d.y() - reference_points_[index].y) * sin(radian);
    return true;
}

bool DiscretePointsReferenceLineSmoother::getNearstPoint(
        const common::math::Vec2d &point,
        int& location,
        const double &minDis,
        const std::vector<GaussData>& reference_points_) const
{
    double min_distance = minDis;
    bool find_point = false;
    int i= 0;
    for (auto ref_point : reference_points_)
    {
        const double temp_dis = std::hypot((ref_point.x - point.x()),(ref_point.y - point.y()));
        if(temp_dis < min_distance)
        {
            min_distance = temp_dis;
            location = i;
            find_point = true;
        }
        i++;
    }
    //std::cout << "min_distance = " << min_distance << std::endl;
    return find_point;
}

bool DiscretePointsReferenceLineSmoother::SLToXY(const SLPoint& sl_point,
            common::math::Vec2d* const point,
            const std::vector<GaussData>& reference_points_) const
{
    if(point == NULL)
    {
        return false;
    }
    if (reference_points_.size() < 2) {
        return false;
    }

    const auto matched_point = getNearstPoint(sl_point.s,reference_points_);
    point->set_x(matched_point.x - std::cos(matched_point.heading*3.1415926/180) * sl_point.l);
    point->set_y(matched_point.y + std::sin(matched_point.heading*3.1415926/180) * sl_point.l);
    return true;
}

GaussData DiscretePointsReferenceLineSmoother::getNearstPoint(
        const double s,
        const std::vector<GaussData>& reference_points_) const
{

    if (s < accumulated_s_.front() - 1e-2) {
        std::cout << "warning: The requested s " << s << " < 0";
         return reference_points_.front();
    }
    if (s > accumulated_s_.back() + 1e-2) {
        std::cout << "warning: The requested s " << s << " > reference line length " << accumulated_s_.back();
        return reference_points_.back();
    }

    auto it_lower = std::lower_bound(accumulated_s_.begin(),accumulated_s_.end(),s);
    if (it_lower == accumulated_s_.begin()) {
        return reference_points_.front();
    } else {
        auto index = std::distance(accumulated_s_.begin(), it_lower);
        if (std::fabs(accumulated_s_[index-1] - s) < std::fabs(accumulated_s_[index] - s)) {
            return reference_points_[index-1];
        } else {
            return reference_points_[index];
        }
    }
}


GaussData DiscretePointsReferenceLineSmoother::GetReferencePoint(
        const double s,
        const std::vector<ADC::planning::GaussData>& reference_line){
    const auto accumulated_s = accumulated_s_;
    // 起点、终点特殊处理
    if(s < accumulated_s.front() - 1e-2){
        return reference_line.front();
    }
    if(s > accumulated_s.back() + 1e-2){
        return reference_line.back();
    }
    // 根据 s 获取原始参考线的 Index 值
    auto interpolate_index = GetIndexFromS(s,reference_line);
    int index = interpolate_index.id;
    int next_index = index + 1;
    if (next_index >= reference_line.size()) {
        next_index = reference_line.size() - 1;
    }
    const auto& p0 = reference_line[index];
    const auto& p1 = reference_line[next_index];
    const double s0 = accumulated_s[index];
    const double s1 = accumulated_s[next_index];

    std::cout << "s = " << s << std::endl;
    std::cout << "interpolate_index.id = " << interpolate_index.id << std::endl;
    std::cout << "interpolate_index.offset = " << interpolate_index.offset << std::endl;

    // 插值
    return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}

InterpolatedIndex DiscretePointsReferenceLineSmoother::GetIndexFromS(
        double s ,
        const std::vector<ADC::planning::GaussData>& reference_line) const {
    if(s <= 0.0){
        return {0,0.0};
    }
    if(s >= length_){
        return {num_points_ - 1, 0.0};
    }
    const int sample_id = static_cast<int>(s/kSampleDistance);
    if(sample_id >= num_sample_points_){
        return {num_points_ - 1 , 0.0};
    }
    const int next_sample_id = sample_id + 1;
    int low = last_point_index_[sample_id];
    int high = (next_sample_id < num_sample_points_
                ? std::min(num_points_, last_point_index_[next_sample_id] + 1)
                : num_points_);
    // 二分查找出 low 的采样点
    while (low + 1 < high) {
        const int mid = (low + high) / 2;
        if (accumulated_s_path_[mid] <= s) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return {low, s - accumulated_s_path_[low]};
}

void DiscretePointsReferenceLineSmoother::InitPoints() {

// InitPoints():
    std::vector<common::math::Vec2d> path_points_;
    common::math::Vec2d path_point;
    for(auto anchor_point : anchor_points_){
        path_point.set_x(anchor_point.path_point.x);
        path_point.set_y(anchor_point.path_point.y);
        path_points_.push_back(path_point);
    }
    num_points_ = static_cast<int>(path_points_.size());

    accumulated_s_path_.clear();
    accumulated_s_path_.reserve(num_points_);

    unit_directions_.clear();
    unit_directions_.reserve(num_points_);
    double s = 0.0;
    for (int i = 0; i < num_points_; ++i) {
        accumulated_s_path_.push_back(s);
        common::math::Vec2d heading;
        if (i + 1 >= num_points_) {
            heading = path_points_[i] - path_points_[i - 1];
        } else {

            heading = path_points_[i + 1] - path_points_[i];
            s += heading.Length();
        }
        heading.Normalize();
        unit_directions_.push_back(heading); // 从 (i-1) 到 i 的单位向量
    }
    length_ = s;
    num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;

// InitPointIndex() :
    last_point_index_.clear();
    last_point_index_.reserve(num_sample_points_);
    double ss = 0.0;
    int last_index = 0;
    for (int i = 0; i < num_sample_points_; ++i) {
        while (last_index + 1 < num_points_ &&
                accumulated_s_path_[last_index + 1] <= ss) {
            ++last_index;
        }
        last_point_index_.push_back(last_index);
        ss += kSampleDistance;
    }
    std::cout << "调试" << std::endl;
}

GaussData DiscretePointsReferenceLineSmoother::InterpolateWithMatchedIndex(
        const GaussData &p0, const double s0, const GaussData &p1,
        const double s1, const InterpolatedIndex &index) {

    double kMathEpsilon = 1.0e-10;
    if(std::fabs(s0-s1) < kMathEpsilon){
        return p0;
    }
    double s = s0 + index.offset;

    // (x,y) 插值


    // kappa dkappa 线性插值





}

}  // namespace planning
}  // namespace ADC
