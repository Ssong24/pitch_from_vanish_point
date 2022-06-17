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
//#include <opencv2/optflow/rlofflow.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/opencv.hpp>

template <typename Method, typename... Args>
void dense_optical_flow_images(std::string leftImgFile, std::string rightImgFile, Method method, bool to_gray, Args&&... args);
template <typename Method, typename... Args>
void dense_optical_flow_video(std::string filename, Method method, bool to_gray, Args&&... args);

void demo_dense_opt_flow(std::string method);

#endif /* dense_opt_flow_hpp */
