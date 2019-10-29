#include <iostream>
#include "trans/trans_data.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_line_provider.h"
#include "common/math/math_utils.h"

#include "matplotlib_cpp/matplotlib_cpp.h"
namespace plt = matplotlibcpp;

int main()
{
    using planning::TransData;
    using planning::ReferenceLine;
    using planning::ReferenceLineProvide;
    using planning::ReferencePoint;
    using had_map::MapPoint;

    ReferenceLine referenceLine;
    TransData transData;
    transData.createReferenceLine(referenceLine);

    ReferenceLine referenceLineResult;
    ReferenceLineProvide referenceLineProvider;
    referenceLineProvider.smoothReferenceLine(referenceLine, &referenceLineResult, 0.1, true);

    // get results
    std::vector<ReferencePoint> originPoints = referenceLine.referencePoints();
    std::vector<double> originS = referenceLine.accumulateS();
    std::vector<ReferencePoint> refPoints = referenceLineResult.referencePoints();
    std::vector<double> s = referenceLineResult.accumulateS();

    // plot
    MapPoint pointInfo;
    std::vector<double> originX, originY, originHeading;
    for(auto const & point : originPoints)
    {
        pointInfo = point.pointInfo();
        originX.emplace_back(pointInfo.x());
        originY.emplace_back(pointInfo.y());
        originHeading.emplace_back(pointInfo.heading());
    }

    std::vector<double> x, y, heading, kappa, dkappa;
    for (auto const &point : refPoints)
    {
        pointInfo = point.pointInfo();
        x.emplace_back(pointInfo.x());
        y.emplace_back(pointInfo.y());
        heading.emplace_back(radianToDegree(pointInfo.heading()));
        kappa.emplace_back(point.kappa());
        dkappa.emplace_back(point.dkappa());
    }

    plt::figure(1);
    plt::plot(x, y, "r-");
    plt::plot(originX, originY, "b.");

    plt::grid("True");
    plt::axis("equal");
    plt::xlabel("x");
    plt::ylabel("y");

    plt::figure(2);
    plt::plot(s, heading, "r-");
    plt::plot(originS, originHeading, "b.");

    plt::grid("True");
    //plt::axis("equal");
    plt::xlabel("s");
    plt::ylabel("heading");

    plt::figure(3);
    plt::plot(s, kappa, "r-");

    plt::grid("True");
    //plt::axis("equal");
    plt::xlabel("s");
    plt::ylabel("kappa");

    plt::figure(4);
    plt::plot(s, dkappa, "r-");

    plt::grid("True");
    //plt::axis("equal");
    plt::xlabel("s");
    plt::ylabel("dkappa");

    plt::show();
    return 0;
}