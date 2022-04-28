//
//  wls_filter.cpp
//  CppPractive
//
//  Created by Song on 2022/04/25.
//

#include "wls_filter.hpp"


Mat wls_filter(Mat luma, float lambda_, float alpha) {
    int height, width;
    int size;
    Mat log_luma;
    Mat out;
    
    height = luma.rows;
    width = luma.cols;
    size = height * width;
    
    std::cout << "luma type: " << luma.type() << std::endl;
//    luma = luma + EPS;
    log(luma + EPS, log_luma);
    Mat y1 = Mat::zeros(log_luma.rows, log_luma.cols, log_luma.type());
    log_luma(Rect(0, 0, log_luma.cols, log_luma.rows-1)).copyTo(y1(Rect(0, 1, log_luma.cols, log_luma.rows-1)));
    Mat diff_log_luma_y, diff_log_luma_x, diff_log_luma;
    absdiff(log_luma,  y1, diff_log_luma_y);
    
    
    
    
    
    
    return out;
}
