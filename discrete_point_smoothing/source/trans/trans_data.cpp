#include <cstring>
#include <iostream>

#include "trans/trans_data.h"
#include "proj4/proj_api.h"

namespace ADC {
namespace planning {

    bool TransData::ImportData(std::vector<GaussData>& raw_reference_line , std::vector<GaussData>& spare_ference_line) {

        std::vector<InsData> Ins_Data;

        if(ImportInsData(Ins_Data)) {
            std::cout << "Import Ins_Data success" << std::endl ;
        }
        else{
            std::cout << "Import Ins_Data fail" << std::endl ;
            return false;
        }

        uint16_t iNum = Ins_Data.size();
        GaussData InputData;
        dPoint Bol;
        dPoint Col;
        for(uint16_t i = 0; i < iNum; ++i){
            Bol.x = Ins_Data[i].x/100000000.0;
            Bol.y = Ins_Data[i].y/100000000.0;
            Col = GaussProjCal(Bol);
            InputData.index = i;
            InputData.x = Col.x;
            InputData.y = Col.y;
            InputData.heading = Ins_Data[i].heading;
            InputData.s = 0.000;
            InputData.z = 0.000;
            raw_reference_line.emplace_back(InputData);
        }

        if(SpareReferenceLine(raw_reference_line,spare_ference_line)){
            std::cout << "raw_reference_line has been spared successfully !" << std::endl ;
        }
        else{
            std::cout << "raw_reference_line has been spared failly !" << std::endl ;
            return false;
        }

        return true;
    }

    bool TransData::SpareReferenceLine(std::vector<GaussData> &raw_reference_line,std::vector<GaussData> &spare_ference_line) {
        int iNum = raw_reference_line.size() , currentpoint = 1 ; //
        double interval = smoother.max_constraint_interval;
        double sum_dis = 0.0 , dis = 0.0 ;
        GaussData sparepoint;

        raw_reference_line[0].s = 0;

        // 忽略了最后一个点的距离
        sparepoint = raw_reference_line[0];
        spare_ference_line.push_back(sparepoint);
        for(int i = currentpoint; i < iNum-1; ++i){
            dis = sqrt(pow(raw_reference_line[i+1].x - raw_reference_line[i].x , 2)
                       + (pow(raw_reference_line[i+1].y - raw_reference_line[i].y , 2)));
            raw_reference_line[i].s = raw_reference_line[i-1].s + dis;
            sum_dis += dis;
            if(sum_dis >= interval){
                currentpoint = i;
                sparepoint = raw_reference_line[currentpoint];
                spare_ference_line.push_back(sparepoint);
                sum_dis = 0.0;
            }
        }
        raw_reference_line.back().s = raw_reference_line[iNum-2].s
                                     + sqrt(pow(raw_reference_line[iNum-1].x - raw_reference_line[iNum-2].x , 2)
                                         + (pow(raw_reference_line[iNum-1].y - raw_reference_line[iNum-2].y , 2)));
        sparepoint = raw_reference_line.back();
        spare_ference_line.push_back(sparepoint);

        return true;
    }

    bool TransData::ImportInsData(std::vector<InsData>& insdata) {

        InsData tempFrame;

        FILE* fp = fopen("Ins_Data.txt","r+");
        if(fp == NULL){
            std::cout << "open file fail" << std::endl ;
            exit(1);
        }

        // TODO:
        // 1.消除 txt 中的 "," 的影响 (vs_code 直接查找替换)
        // 2.补充 z 坐标值
        while(!feof(fp)){

            fscanf(fp,"%lf,%lf,%lf\r\n",&tempFrame.x,&tempFrame.y,&tempFrame.heading);
            insdata.push_back(tempFrame);
        }
        std::cout << "load Ins_Data.txt success" << std::endl ;

        return true;
    }

    dPoint TransData::GaussProjCal(const dPoint bol) {

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
}