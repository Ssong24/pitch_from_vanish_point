//
//  line_detect.hpp
//  CppPractive
//
//  Created by Song on 2022/04/28.
//

#ifndef line_detect_hpp
#define line_detect_hpp

#include "lsd.hpp"
#include "VPDetection.hpp"

#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>

void LineDetect( cv::Mat image, double thLength, std::vector<std::vector<double> > &lines );
void drawClusters( cv::Mat &img, std::vector<std::vector<double> > &lines, std::vector<std::vector<int> > &clusters );
void demo_VPDetection(std::string inputImagePath);
#endif /* line_detect_hpp */
