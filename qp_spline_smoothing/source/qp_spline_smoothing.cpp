#include <iostream>
#include <time.h>
#include "trans/trans_data.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_line_provider.h"
#include "common/math/math_utils.h"
#include "common/smooth_line/discreted_points_smooth/discrete_points_math.h"

#include "matplotlib_cpp/matplotlib_cpp.h"
namespace plt = matplotlibcpp;

int main0()
{
    clock_t startTime, endTime;
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
    startTime = clock();
    referenceLineProvider.smoothReferenceLine(referenceLine, &referenceLineResult, 0.1, true);
    endTime = clock();
    std::cout << "smooth referenceline spendTime = " <<
            static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC << std::endl;

    // get results
    std::vector<ReferencePoint> originPoints = referenceLine.referencePoints();
    std::vector<double> originS = referenceLine.accumulateS();
    std::vector<ReferencePoint> refPoints = referenceLineResult.referencePoints();
    std::vector<double> s = referenceLineResult.accumulateS();

    transData.createPathGuass(refPoints);

    std::cout << "平滑距离 s = " << s.back() << std::endl;

    // plot

    MapPoint pointInfo = originPoints[0].pointInfo();
    double zeroX = pointInfo.x();
    double zeroY = pointInfo.y();
    std::vector<double> originX, originY, originHeading;
    for(auto const & point : originPoints)
    {
        pointInfo = point.pointInfo();
        originX.emplace_back(pointInfo.x() - zeroX);
        originY.emplace_back(pointInfo.y() - zeroY);
        originHeading.emplace_back(radianToDegree(pointInfo.heading()));
    }

    std::vector<double> x, y, heading, kappa, dkappa;
    for (auto const &point : refPoints)
    {
        pointInfo = point.pointInfo();
        x.emplace_back(pointInfo.x() - zeroX);
        y.emplace_back(pointInfo.y() - zeroY);
        heading.emplace_back(radianToDegree(pointInfo.heading()));
        kappa.emplace_back(std::atan(point.kappa() * 2.95) * 180.0 / M_PI * 17.0);
//        kappa.emplace_back(point.kappa());
        dkappa.emplace_back(point.dkappa());
    }

    plt::figure(1);
    plt::plot(x, y, "r.");
    plt::plot(originX, originY, "b-");

    plt::grid("True");
    plt::axis("equal");
    plt::xlabel("x");
    plt::ylabel("y");

    plt::figure(2);
    plt::plot(s, heading, "r.");
    plt::plot(originS, originHeading, "b-");

    plt::grid("True");
    //plt::axis("equal");
    plt::xlabel("s");
    plt::ylabel("heading");

    plt::figure(3);
    plt::plot(s, kappa, "r-");

    plt::grid("True");
    //plt::axis("equal");
    plt::xlabel("s");
    plt::ylabel("steering-angle");

    plt::figure(4);
    plt::plot(s, dkappa, "r-");

    plt::grid("True");
    //plt::axis("equal");
    plt::xlabel("s");
    plt::ylabel("dkappa");

    plt::show();
    return 0;
}

int main1()
{
    using planning::TransData;
    using planning::ReferenceLine;
    using planning::ReferenceLineProvide;
    using planning::ReferencePoint;
    using had_map::MapPoint;
    using planning::DiscretePointsMath;

    ReferenceLine referenceLine;
    TransData transData;
    transData.createReferenceLine(referenceLine);

    std::vector<std::pair<double, double>> xyPoints;
    std::vector<double> originHeading;
    std::vector<ReferencePoint> referencePoints = referenceLine.referencePoints();
    for(const auto point : referencePoints)
    {
        xyPoints.emplace_back(point.pointInfo().x(), point.pointInfo().y());
        originHeading.emplace_back(radianToDegree(point.pointInfo().heading()));
    }

    std::vector<double> x, y;
    double zeroX = xyPoints[0].first;
    double zeroY = xyPoints[0].second;
    for (auto const &point : xyPoints)
    {
        x.emplace_back(point.first - zeroX);
        y.emplace_back(point.second - zeroY);
    }

    ///////////////////////////////////////////////////
    std::vector<double> headings;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::vector<double> accumulatedS;

    if (!DiscretePointsMath::ComputePathProfile(
            xyPoints, &headings, &accumulatedS, &kappas, &dkappas))
    {
        std::cout << "error" << std::endl;
    }

//    for(size_t i = 0; i < headings.size(); ++i)
//    {
//        double angle = -headings[i] - degreeToRadian(90);
//        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
//        if(a < 0.0)
//        {
//            a = a + 2.0 * M_PI;
//        }
//        headings[i] = radianToDegree(a);
//    }

    //
    std::vector<double> newKappas;
    DiscretePointsMath::ComputeOriginalPathKappaProfile(referenceLine, &newKappas);

    // plot
//    plt::plot(accumulatedS, headings, "r-");
//    plt::plot(accumulatedS, originHeading, "b.");

    plt::figure(1);
    plt::plot(accumulatedS, newKappas, "r.");

//    plt::figure(2);
//    plt::plot(x, y, "b.");

    plt::show();

    return 0;
}

int main()
{
    main0();
//    main1();

    return 0;
}

