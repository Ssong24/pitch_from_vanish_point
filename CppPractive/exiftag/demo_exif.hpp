//
//  demo_exif.hpp
//  CppPractive
//
//  Created by Song on 2022/05/11.
//

#ifndef demo_exif_hpp
#define demo_exif_hpp

//#include <stdio.h>
#include <iostream>

int demo_exiftag(/*int argc, char *argv[]*/);
void find_fl_in_35mm(std::string image_path, double& focal_length);
void fl_mm_to_pix(double& f_pix, double f_mm, int image_width, int sensor_size=35 );

#endif /* demo_exif_hpp */
