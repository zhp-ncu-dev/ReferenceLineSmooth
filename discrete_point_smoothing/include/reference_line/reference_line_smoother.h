#pragma once

#include <vector>
#include "proto/pnc_point.h"
#include "config/reference_line_smoother_config.h"
#include "reference_line.h"

namespace ADC {
namespace planning {

class ReferenceLineSmoother {
public:

    // TODO: 为什么严加呢？
    ReferenceLineSmoother() = default;

    explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig& config)
            : config_(config) {}

    /**
     * Smoothing constraints
     */
    virtual void SetAnchorPoints(const std::vector<AnchorPoint>& achor_points) = 0;

    //virtual void SetSparePoints(const std::vector<GaussData>& sparepoint) = 0;

    /**
     * Smooth a given reference line
     */
    virtual bool Smooth(const GaussData&, ReferenceLine* const) = 0;

    virtual ~ReferenceLineSmoother() = default;

protected:
    ReferenceLineSmootherConfig config_;
};

}  // namespace planning
}  // namespace ADC
