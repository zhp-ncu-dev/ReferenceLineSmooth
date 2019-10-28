#include <iostream>
#include "trans/trans_data.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_line_provider.h"
#include "matplotlib_cpp/matplotlib_cpp.h"

namespace plt = matplotlibcpp;

int main()
{
    using planning::TransData;
    using planning::ReferenceLine;
    using planning::ReferenceLineProvide;

    ReferenceLine referenceLine;
    TransData transData;
    transData.createReferenceLine(referenceLine);

    ReferenceLine *referenceLineResult;
    ReferenceLineProvide referenceLineProvider;
    referenceLineProvider.smoothReferenceLine(referenceLine, referenceLineResult, 30, true);


    return 0;
}