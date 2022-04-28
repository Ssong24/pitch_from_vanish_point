//
//  wls_filter.hpp
//  CppPractive
//
//  Created by Song on 2022/04/25.
//

#ifndef wls_filter_hpp
#define wls_filter_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

#define EPS 1e-4

using namespace cv;

Mat wls_filter(Mat luma, float lambda_ = 1, float alpha=1.2);

#endif /* wls_filter_hpp */
