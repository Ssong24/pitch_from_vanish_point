
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
    demo_VPDetection("/Users/3i-21-331/workspace/CppPractive/CppPractive/img/c_4.jpg");
    
    return 0;
}




