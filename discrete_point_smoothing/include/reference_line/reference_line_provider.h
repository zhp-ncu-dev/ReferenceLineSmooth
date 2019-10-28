#pragma once

#include <vector>
#include <bits/unique_ptr.h>

#include "reference_line.h"
#include "proto/pnc_point.h"
#include "reference_line_smoother.h"
#include "config/reference_line_smoother_config.h"

namespace ADC {
namespace planning {

class ReferenceLineProvider {
public:
    ReferenceLineProvider() = default;
    ~ReferenceLineProvider();

    bool SmoothReferenceLine(const GaussData& raw_reference_line,
                             ReferenceLine* reference_line);

private:
    void GetAnchorPoints(const GaussData& reference_line,
                         std::vector<AnchorPoint>* anchor_points) const;
    AnchorPoint GetAnchorPoint(const ReferenceLine& reference_line,
                               double s) const;


    std::unique_ptr<ReferenceLineSmoother> smoother_;
    ReferenceLineSmootherConfig smoother_config_;

};

}
}

