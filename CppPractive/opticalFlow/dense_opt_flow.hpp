//
//  dense_opt_flow.hpp
//  CppPractive
//
//  Created by Song on 2022/05/15.
//

#ifndef dense_opt_flow_hpp
#define dense_opt_flow_hpp

#include <stdio.h>
#include <iostream>
// #include <opencv2/optflow/rlofflow.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto.hpp>
#include "Image.hpp"

template <typename Method, typename... Args>
void dense_optical_flow_images(std::string leftImgFile, std::string rightImgFile, Method method, bool to_gray, Args&&... args);
template <typename Method, typename... Args>
void dense_optical_flow_video(std::string filename, Method method, bool to_gray, Args&&... args);
void demo_dense_opt_flow(std::string method);


void draw_optical_flow(int width, int height, cv::Mat flowL2R_, cv::Mat flowR2L_, cv::Mat& prvsL_drawn, cv::Mat prvsR_drawn);
#endif /* dense_opt_flow_hpp */
