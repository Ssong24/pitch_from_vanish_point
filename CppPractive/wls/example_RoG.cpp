//
//  example_RoG.cpp
//  CppPractive
//
//  Created by Song on 2022/04/22.
//

#include "example_RoG.hpp"


double F(double input) // function f(...), which is used for defining L, a and b
                       // changes within [4/29,1]
{
  if (input > 0.008856)
    return std::cbrt(input); // maximum 1 --- prefer cbrt to pow for cubic root
  else
    return ((double(841) / 108) * input +
            double(4) / 29); // 841/108 = 29*29/36*16
}

//template <class float_t> struct Convert<XYZ<float_t>> {
//  template <class real_t> static XYZ<float_t> from(const RGB<real_t> &rhs) {
//    // Assume RGB has the type invariance satisfied, i.e., channels \in [0,255]
//    float_t var_R = float_t(rhs.comp1()) / 255;
//    float_t var_G = float_t(rhs.comp2()) / 255;
//    float_t var_B = float_t(rhs.comp3()) / 255;
//
//    var_R = (var_R > 0.04045) ? std::pow((var_R + 0.055) / 1.055, 2.4)
//                              : var_R / 12.92;
//    var_G = (var_G > 0.04045) ? std::pow((var_G + 0.055) / 1.055, 2.4)
//                              : var_G / 12.92;
//    var_B = (var_B > 0.04045) ? std::pow((var_B + 0.055) / 1.055, 2.4)
//                              : var_B / 12.92;
//
//    var_R *= 100;
//    var_G *= 100;
//    var_B *= 100;
//
//    return XYZ<float_t>{var_R * float_t(0.4124) + var_G * float_t(0.3576) +
//                            var_B * float_t(0.1805),
//                        var_R * float_t(0.2126) + var_G * float_t(0.7152) +
//                            var_B * float_t(0.0722),
//                        var_R * float_t(0.0193) + var_G * float_t(0.1192) +
//                            var_B * float_t(0.9505)};
//  }
//};


// RGB to XYZ
void RGBtoXYZ(uchar R, uchar G, uchar B, double &X, double &Y, double &Z)
{
    // RGB Working Space: sRGB
    // Reference White: D65
    double var_R = R / 255.;
    double var_G = G / 255.;
    double var_B = B / 255.;
    
    var_R = (var_R > 0.04045) ? std::pow((var_R + 0.055) / 1.055, 2.4)
                                  : var_R / 12.92;
    var_G = (var_G > 0.04045) ? std::pow((var_G + 0.055) / 1.055, 2.4)
                              : var_G / 12.92;
    var_B = (var_B > 0.04045) ? std::pow((var_B + 0.055) / 1.055, 2.4)
                              : var_B / 12.92;

    var_R *= 100;
    var_G *= 100;
    var_B *= 100;
    
    
//    X = 0.412453*R + 0.357580*G + 0.189423*B; // maximum value = 0.959456 * 255 = 244.66128
//    Y = 0.212671*R + 0.715160*G + 0.072169*B; // maximum value = 1 * 255 = 255
//    Z = 0.019334*R + 0.119193*G + 0.950227*B; //  maximum value = 1.088754 * 255 = 277.63227
    
    X = 0.412453*var_R + 0.357580*var_G + 0.189423*var_B; // maximum value = 0.959456 * 255 = 244.66128
    Y = 0.212671*var_R + 0.715160*var_G + 0.072169*var_B; // maximum value = 1 * 255 = 255
    Z = 0.019334*var_R + 0.119193*var_G + 0.950227*var_B; //  maximum value = 1.088754 * 255 = 277.63227
}

// XYZ to CIELab
void XYZtoLab(double X, double Y, double Z, double &L, double &a, double &b)
{
    double Xo = 95.047;
    double Yo = 100;
    double Zo = 108.883;
    L = 116 * F(Y / Yo) - 16; // maximum L = 100
    a = 500 * (F(X / Xo) - F(Y / Yo)); // maximum
    b = 200 * (F(Y / Yo) - F(Z / Zo));
}

// RGB to CIELab
void RGBtoLab(double R, double G, double B, double &L, double &a, double &b)
{
    double X, Y, Z;
    RGBtoXYZ(R, G, B, X, Y, Z);
    XYZtoLab(X, Y, Z, L, a, b);
}


void BGRtoLab_Mat(cv::Mat roi, cv::Mat& output) {
    output = Mat::zeros(roi.rows, roi.cols, CV_64F);
    for(int i = 0; i < roi.rows; i++)
    {
        for(int j = 0; j < roi.cols; j++)
        {
            double B = (double)roi.at<cv::Vec3b>(i,j)[0];
            double G = (double)roi.at<cv::Vec3b>(i,j)[1];
            double R = (double)roi.at<cv::Vec3b>(i,j)[2];
            
            double L, a, b;
            
            RGBtoLab(R, G, B, L, a, b);
            output.at<double>(i,j) = L;
//            printf("R,G,B: %3lf, %3lf, %3lf\n", R, G, B);
//            printf("i,j,L: %2d, %2d, %lf\n", i, j, L);
//            std::cout << output.at<double>(i, j) << std::endl;
            
        }
    }
}



void demo_RoG() {
    std::string filepath = "/Users/3i-21-331/workspace/CppPractive/CppPractive/img/flower.png";
    cv::Mat image = cv::imread(filepath);
    cv::Mat3b lab;
    cv::Mat lab_split[3];
    cv::cvtColor(image, lab, cv::COLOR_BGR2Lab);
    
    
    std::cout << image.type() << ", " << lab.type() << ", " << CV_64F << ", " << CV_32F << std::endl;
    int thresh = 40;
    Mat brightLAB;
    cv::cvtColor(image, brightLAB, cv::COLOR_BGR2Lab);
    
    Vec3b labPixel(lab.at<Vec3b>(0,0));
    cv::Scalar minLAB = cv::Scalar(labPixel.val[0] - thresh, labPixel.val[1] - thresh, labPixel.val[2] - thresh);
    cv::Scalar maxLAB = cv::Scalar(labPixel.val[0] + thresh, labPixel.val[1] + thresh, labPixel.val[2] + thresh);
    cv::Mat maskLAB, resultLAB;
    cv::inRange(brightLAB, minLAB, maxLAB, maskLAB);
    cv::bitwise_and(brightLAB, brightLAB, resultLAB, maskLAB);
    cv::imshow("Output LAB", resultLAB);
    cv::waitKey(2000);
    
    
    Mat luma_test;
    BGRtoLab_Mat(image, luma_test);
    double a1, a2, a3;
    RGBtoLab(56, 79, 132, a1, a2, a3);
    std::cout << a1 <<", " << a2 << ", "  << a3 << std::endl;
    
    

    cv::imshow("test luma", luma_test);
    Mat luma_test_8u ;
    luma_test.convertTo(luma_test_8u, CV_8U, 255, 0);
    cv::threshold(luma_test_8u, luma_test_8u, 255*0.9, 1., cv::THRESH_TOZERO);
    cv::imshow("8u luma", luma_test_8u);
    
    cv::waitKey(2500);
    
    
    cv::split(lab, lab_split);
    cv::Mat luma = lab_split[0];
    std::cout << "luma type: " << luma.type() << std::endl;
    
    cv::imshow("luma", luma);
    cv::waitKey(1500);
    
    // wls filter
    wls_filter(luma);
    
}



float sigmoid(float x, float a) {
    // DESCR:
    // Applies a sigmoid function on the data x in [0-1] range. Then rescales
    // the result so 0.5 will be mapped to itself.
    float y = 0.0f;
    
    // Apply sigmoid
    y = 1. / (1 + exp(-a * x)) - 0.5;
    
    float y_05 = 1./ (1 + exp(-a * 0.5)) - 0.5;
    y = y * (0.5 / y_05);
    
    
    return y;
}


float detail_enhance(float L, float L0, float L1, float weight0, float weight1) {
    float detail_0, detail_1, base, res;
    
    detail_0 = L-L0;
    detail_0 = sigmoid(detail_0, weight0);
    
    detail_1 = L0 - L1;
    detail_1 = sigmoid(detail_1, weight1);
    base = L1;
    res = base + detail_1 + detail_0;
    
    res = sigmoid(res, 1);
    
    return res;
}



float test_detail_enhance(std::string filename) {
    cv::Mat src = cv::imread(filename);
    

//    %% RoG Smooth
//    I0 = rog_smooth(I, 0.001, 0.5, 1.0, 1);
//    I1 = rog_smooth(I, 0.001, 1.0, 1.5, 1);
//
//    %% Detail Enhancement
//    coarse=detailenhance( I, I0, I1, 1, 25 );
//    figure;imshow(coarse);title('Coarse-scale boost');
//
//    fine=detailenhance( I, I0, I1, 12, 1 );
//    figure;imshow(fine);title('Fine-scale boost');
    
    return 0.0;
}


cv::Mat rog_smooth(cv::Mat image, float a, float b, float c , float d ) {
    cv::Mat src;
    
    return src;
}

