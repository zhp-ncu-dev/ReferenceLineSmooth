#include <iostream>
#include <vector>
#include "trans/trans_data.h"
#include "reference_line/discrete_points_reference_line_smoother.h"
#include "proto/pnc_point.h"
#include "matplotlib_cpp/matplotlib_cpp.h"

namespace plt = matplotlibcpp;

int main(int argv, char* argc[])
{
    // 导入数据
    ADC::planning::TransData transdata;
    std::vector<ADC::planning::GaussData> raw_reference_line;
    std::vector<ADC::planning::GaussData> spare_ference_line;

    if(transdata.ImportData(raw_reference_line,spare_ference_line)){
        std::cout << "raw_reference_line and spare_ference_line has been imported !" << std::endl;
    } else{
        exit(1);
    }

    // 设置 anchor_points
    ADC::planning::DiscretePointsReferenceLineSmoother discretepoints_smoother;
    discretepoints_smoother.SetAnchorPoints(spare_ference_line,raw_reference_line);

    // 平滑求解
    std::vector<ADC::planning::SmoothedReferencePoint> *ref_points;
    ref_points = new std::vector<ADC::planning::SmoothedReferencePoint>();
    discretepoints_smoother.Smooth(raw_reference_line, ref_points);

    // plot
    size_t numSpareReferenceLine = spare_ference_line.size();
    std::vector<double> spareReferenceLineX(numSpareReferenceLine),
                        spareReferenceLineY(numSpareReferenceLine),
                        spareReferenceLineYaw(numSpareReferenceLine),
                        spareReferenceLineS(numSpareReferenceLine);
    for (size_t i = 0; i < numSpareReferenceLine; ++i)
    {
        spareReferenceLineX[i] = spare_ference_line[i].x - spare_ference_line[0].x;
        spareReferenceLineY[i] = spare_ference_line[i].y - spare_ference_line[0].y;
        spareReferenceLineYaw[i] = spare_ference_line[i].heading;
        spareReferenceLineS[i] = spare_ference_line[i].s;
    }

    size_t numRefPoints = ref_points->size();
    std::vector<double> refPointsX(numRefPoints),
                        refPointsY(numRefPoints),
                        refPointsYaw(numRefPoints),
                        refPointsS(numRefPoints),
                        refPointsKappa(numRefPoints),
                        refPointsDKappa(numRefPoints);
    for (size_t i = 0; i < numRefPoints; ++i)
    {
        refPointsX[i] = (*ref_points)[i].x - (*ref_points)[0].x;
        refPointsY[i] = (*ref_points)[i].y - (*ref_points)[0].y;
        refPointsYaw[i] = (*ref_points)[i].heading * 180.0 / M_PI;
        refPointsS[i] = (*ref_points)[i].s;
        refPointsKappa[i] = std::atan(((*ref_points)[i].kappa) * 2.95) * 180.0 / M_PI;
        refPointsDKappa[i] = (*ref_points)[i].dkappa;
    }

    // figure(1)
    plt::figure(1);
    plt::plot(spareReferenceLineX, spareReferenceLineY, "b.");
    plt::plot(refPointsX, refPointsY, "r-");

    plt::grid("True");
    plt::axis("equal");
    plt::xlabel("x[m]");
    plt::ylabel("y[m]");

    // figure(2)
//    plt::figure(2);
//    plt::plot(spareReferenceLineS, spareReferenceLineYaw, "b.");
//    plt::plot(refPointsS, refPointsYaw, "r.");

//    plt::grid("True");
//    //plt::axis("equal");
//    plt::xlabel("s");
//    plt::ylabel("yaw");

    // figure(3)
    plt::figure(3);
    plt::plot(refPointsS, refPointsKappa, "r.");

    plt::grid("True");
    //plt::axis("equal");
    plt::xlabel("s");
    plt::ylabel("delta");

    // figure(4)
    plt::figure(4);
    plt::plot(refPointsS, refPointsDKappa, "r.");
    plt::grid("True");
    //plt::axis("equal");
    plt::xlabel("s");
    plt::ylabel("dkappa");

    plt::show();

    return 0;
}

