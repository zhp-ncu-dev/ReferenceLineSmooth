#pragma once

#include <vector>
#include "../proto/pnc_point.h"
#include "config/reference_line_smoother_config.h"

namespace ADC {
namespace planning {

class TransData {

public:
    TransData() = default;
    ~TransData() = default;

    bool ImportData(std::vector<GaussData>& raw_reference_line , std::vector<GaussData>& spare_ference_line);

private:
    bool ImportInsData(std::vector<InsData>& insdata);

    dPoint GaussProjCal(const dPoint bol);
    bool SpareReferenceLine(std::vector<GaussData> &raw_reference_line,
                            std::vector<GaussData> &spare_ference_line);

    ReferenceLineSmootherConfig smoother;

};

}
}