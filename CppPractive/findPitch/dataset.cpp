//
//  dataset.cpp
//  CppPractive
//
//  Created by Song on 2022/05/06.
//

#include "dataset.hpp"


void set_dataset_params( std::string& root_lens_Dir, std::string& imgExt ) {
    std::string rootDir = "/Users/3i-21-331/workspace/stitching/mobile_stitching/dataset/PivoX_full/galaxy_zflip3";
    std::string lensDir = "/WideLens_20deg/correct_capture_1";
    
    root_lens_Dir = rootDir + lensDir;
    
    imgExt = ".jpg";
    
//    phoneTilt = 1.6; // 0.2;
//    calibData = set_calibData(3, phoneTilt, 87.5, 0.15, -1, 0); //87.18, -0.01, 0.023, -0.015);
//    set_yrp_angle_ucd(26, phoneTilt, -19.0, tiltList);
}
