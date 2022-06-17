
#include <iostream>
#include <opencv2/opencv.hpp>

#include "wls/example_RoG.hpp"
#include "vanishPt/vanishing_point.hpp"
#include "vanishPt/line_detect.hpp"
#include "findPitch/pitch_btw_imgs.hpp"
#include "exiftag/demo_exif.hpp"
#include "opticalFlow/dense_opt_flow.hpp"
using namespace cv;
using namespace std;


int main(/*int argc, char **argv*/)
{
   
//    demo_exiftag();
//    demo_VPDetection();
//    demo_vanishing_point();
//    demo_lines_and_vps();
//    demo_find_pitch_btw_up_and_center();
    
    string lucas = "lucaskanade_dense";
    string farne = "farneback";
    string rlof = "rlof";
    
    demo_dense_opt_flow(farne);
    
    return 0;
}


