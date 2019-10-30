#include <cstring>
#include <iostream>

#include "trans/trans_data.h"
#include "proj4/proj_api.h"
#include "common/pnc_point/map_point.h"
#include "common/pnc_point/vec2d.h"
#include "common/math/math_utils.h"
#include "reference_line/reference_point.h"

namespace planning
{
    bool TransData::createReferenceLine(
            planning::ReferenceLine &referenceLine)
    {
        std::vector<GaussData> raw_reference_line;

        ImportData(raw_reference_line);

        double laneWidth = 3.5;
        std::vector<ReferencePoint> referencePoints;
        std::vector<double> accumulateS;
        for (const GaussData &referencePoint : raw_reference_line)
        {
            if(referencePoint.s > 200.0)
            {
                break;
            }
            had_map::MapPoint mapPoint(
                    math::Vec2d{referencePoint.x, referencePoint.y},
                    degreeToRadian(referencePoint.heading));
            ReferencePoint point(mapPoint, laneWidth/2.0, laneWidth/2.0);
            accumulateS.emplace_back(referencePoint.s);
            referencePoints.emplace_back(point);
        }
        ReferenceLine line(referencePoints, accumulateS);
        referenceLine = line;
    }

    bool TransData::ImportData(std::vector<GaussData>& raw_reference_line )
    {
        std::vector<InsData> Ins_Data;

        if(ImportInsData(Ins_Data))
        {
            std::cout << "Import Ins_Data success" << std::endl ;
        }
        else
        {
            std::cout << "Import Ins_Data fail" << std::endl ;
            return false;
        }

        uint16_t iNum = Ins_Data.size();
        GaussData InputData;
        dPoint Bol;
        dPoint Col;
        double sumS = 0.0;
        for(uint16_t i = 0; i < iNum; ++i)
        {
            Bol.x = Ins_Data[i].x/100000000.0;
            Bol.y = Ins_Data[i].y/100000000.0;
            Col = GaussProjCal(Bol);
            InputData.index = i;
            InputData.x = Col.x;
            InputData.y = Col.y;
            InputData.heading = Ins_Data[i].heading;

            if(i == 0)
            {
                InputData.s = 0.000;
            }
            else
            {
                sumS += sqrt((InputData.x - raw_reference_line[i-1].x) *
                        (InputData.x - raw_reference_line[i-1].x) +
                        (InputData.y - raw_reference_line[i-1].y) *
                        (InputData.y - raw_reference_line[i-1].y));
                InputData.s = sumS;
            }

            InputData.z = 0.000;
            raw_reference_line.emplace_back(InputData);
        }

        return true;
    }

    bool TransData::ImportInsData(std::vector<InsData>& insdata)
    {

        InsData tempFrame;

        FILE* fp = fopen("../Ins_Data.txt","r+");
        if(fp == NULL)
        {
            std::cout << "open file fail" << std::endl ;
            exit(1);
        }

        // TODO:
        // 1.消除 txt 中的 "," 的影响 (vs_code 直接查找替换)
        // 2.补充 z 坐标值
        while(!feof(fp))
        {

            fscanf(fp,"%lf,%lf,%lf\r\n",&tempFrame.x,&tempFrame.y,&tempFrame.heading);
            insdata.push_back(tempFrame);
        }
        std::cout << "load Ins_Data.txt success" << std::endl ;

        return true;
    }

    dPoint TransData::GaussProjCal(const dPoint bol)
    {
        double x,y,z;
        x = bol.x * 0.0174533;
        y = bol.y * 0.0174533;
        z = 0;

        const char* bj54proj = " +proj=longlat +towgs84=0.0000,0.0000,0.0000 +a=6378245.0000 +rf=298.3 +lat_0=0.00000000 +lon_0=104.000000000 +lat_1=24.000000000 +lat_2=40.000000000 +x_0=0.000 +y_0=0.000 +units=m +no_defs";
        static projPJ lcc = pj_init_plus(bj54proj);
        static projPJ tmerc = pj_init_plus("+proj=tmerc +ellps=krass +lat_1=25n +lat_2=47n +lon_0=116.25e +x_0=20500000 +y_0=0 +units=m +k=1.0");
        int ret = pj_transform(lcc, tmerc, 1, 1, &x, &y, &z);

        dPoint xoy;
        xoy.x = x;
        xoy.y = y;

        return xoy;
    }

}
