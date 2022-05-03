
#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/video.hpp>

#include "wls/example_RoG.hpp"
#include "vanishPt/vanishing_point.hpp"
#include "vanishPt/line_detect.hpp"
using namespace cv;
using namespace std;




int main(/*int argc, char **argv*/)
{

//    demo_vanishing_point();
    demo_VPDetection("/Users/3i-21-331/workspace/stitching/mobile_stitching/dataset/PivoX_full/galaxy_zflip3/WideLens_20deg/roll_negative/c_4.jpg");
    
    // 11 -> c_4 pitch 0.51594  / 1.6
    // 13 -> c_4 pitch ...3.9 / 1.5
    // 15 -> c_4 pitch  ..2.85 / 8
    // 17 -> c_4 pitch   8.39 / 2.1
    
    
    return 0;
}




