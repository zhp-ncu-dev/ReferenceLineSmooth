
#include "reference_line/reference_line_provider.h"

namespace ADC {
namespace planning {

ReferenceLineProvider::~ReferenceLineProvider() {}

bool ReferenceLineProvider::SmoothReferenceLine(const GaussData &raw_reference_line,
                                                ReferenceLine *reference_line) {
    std::vector<AnchorPoint> anchor_points;
    GetAnchorPoints(raw_reference_line,&anchor_points);
    smoother_->SetAnchorPoints(anchor_points);

    // TODO: 平滑算法接口函数
    bool smoother_success = smoother_->Smooth(raw_reference_line,reference_line);

}

void ReferenceLineProvider::GetAnchorPoints(const GaussData &reference_line,
                                            std::vector<AnchorPoint> *anchor_points) const {
    const double interval = smoother_config_.max_constraint_interval;

    // TODO :还需添加 reference_line 类的内容
/*
    int num_of_anchors = std::max(2,reference_line.Length()/interval + 0.5);
    common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1,
                                &anchor_s);
    for(const double s :anchor_s){
        AnchorPoint anchor = GetAnchorPoint(reference_line, s);
        anchor_points->emplace_back(anchor);
    }

*/
    anchor_points->front().longitudinal_bound = 1e-6;
    anchor_points->front().lateral_bound = 1e-6;
    anchor_points->front().enforced = true;
    anchor_points->back().longitudinal_bound = 1e-6;
    anchor_points->back().lateral_bound = 1e-6;
    anchor_points->back().enforced = true;


}

AnchorPoint ReferenceLineProvider::GetAnchorPoint(const ReferenceLine &reference_line, double s) const {



}

}
}