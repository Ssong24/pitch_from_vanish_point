//
//  LensParameter.hpp
//  m_stitcher
//
//  Created by Song on 2022/04/12.
//

#ifndef LensParameter_hpp
#define LensParameter_hpp


#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <sstream>

#include "json.hpp"


using namespace std;

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

struct cameraParam {
    float HFov;
    int lensType;
    float phoneTilt = 0.0f;

    float fx = 0.0f;
    float fy = 0.0f;
    float Ppx = 0.0f;
    float Ppy = 0.0f;

    float k1 = 0.0f;
    float k2 = 0.0f;
    float k3 = 0.0f;

};


struct Data_Stitch_Mode {
    bool hv_horizontal = false;
    bool hv_vertical = false;
    bool horizontal_only = false;
    bool vertical_only = false;
};


namespace lens {
    //Lens Types
    enum LensType {
        R_NOLENS = 0,
        R_DAISO = 1,
        R_YOUVR = 2,
        R_SECONDARY_WIDE = 3,
        R_OLLO = 4,
        HH_NOLENS = 10,
        HH_DAISO = 11,
        HH_YOUVR = 12,
        HH_SECONDARY_WIDE = 13,
        HH_OLLO = 14
    };


//    struct cameraParam {
//        float HFov;
//        LensType lensType;
//        float phoneTilt;
//        float k1, k2, k3;
//        float Ppx, Ppy;
//    };

    const int num_of_lensType = 4;

    const int lens_hfov = 0;
    const int lens_k1 = 1;
    const int lens_k2 = 2;
    const int lens_k3 = 3;

    const int num_of_params = 4;

    std::string getString(int i);
    int getLensType(LensType Ltype);

    // read the parameter from files
    void LensLookup(std::string CSVurl, LensType Ltype, int model, float &HFoV, float &K1, float &K2, float &K3, float &PPX, float &PPY);
    void LensLookupJSON(std::string json_src, LensType Ltype, int& status_code, float &HFoV, float &K1, float &K2, float &K3, float &Ppx, float &Ppy);
    void LensLookupCSV(std::string CSVcontent, LensType Ltype, std::string manf, std::string model, float &HFoV, float &K1, float &K2, float &K3);
    
    bool readParamFromJson(string jsonFilePath, int lensType, int deviceModelNum, struct cameraParam &camP); //struct cameraParam &camP);
    bool readParamFromCSV(const char * csvURL, int model, lens::LensType activeLens, struct cameraParam &camP); // float& HFoV, float& k1, float& k2, float& k3 );
}

string set_calibData(int lens_type, float pitch, float HFov, float k1=0.0, float k2=0.0, float k3=0.0) ;

void set_yrp_angle_ucd(float upTilt, float centerTilt, float downTilt, float tiltList[]);
void save_userLens_data(int userLens, string& _srcDir, string & calibData, string& imgExt, Data_Stitch_Mode & _dsm, float tiltList[], float rollList[]=NULL);

#endif /* LensParameter_hpp */
