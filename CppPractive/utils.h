//
//  utils.h
//  m_stitcher
//
//  Created by Kevin Santoso on 2021/09/13.
//

#ifndef utils_h
#define utils_h

#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

using namespace std;
using namespace cv;

//Lens Types
enum LensType {
    R_NOLENS = 0,
    R_DAISO = 1,
    R_YOUVR = 2,
    R_LG_WIDE = 3
};



static inline int getPixelU(Mat src, int x, int y, int ch)
{
    int num_ch = src.channels();
    if(ch > num_ch) ch = num_ch-1;
    
    unsigned char thisPx = src.data[y * src.step + x * num_ch + ch];
    return int(thisPx);
}

static inline int getPixelS(Mat src, int x, int y, int ch)
{
    int num_ch = src.channels();
    if(ch > num_ch) ch = num_ch-1;
    
    char thisPx = src.data[y * src.step + x * num_ch + ch];
    return int(thisPx);
}

static inline void setPixel(Mat &src, int x, int y, int ch, int value)
{
    src.data[ y * src.step + x * src.channels() + ch] = value;
}

inline float to_rad(float deg) {
    return (deg/180.0 * M_PI);
}

float median(vector<float> v);

int sphere_to_equi(int distanceParam, double x_dest, double  y_dest, float* x_src, float* y_src);

void Lens_Correction(float f, float ppx, float ppy, float k1, float k2, float k3, int input_img_rows, int input_img_cols, UMat& x_d, UMat& y_d);

void movingAverage(vector<double> data, int windowSz, vector<double>& output);

void IntensitySmoothing(Mat leftGray, Mat rightGray, int yaw, Mat& leftMask, Mat& rightMask, int smoothing_wd);

int rectilinear2sphere( double x_dest,double  y_dest, double* x_src, double* y_src, double R, float pitch);

void printMat33(Mat A);

int rectilinear2cyl( double x_dest,double  y_dest, double* x_src, double* y_src, double distanceparam);

void CreateTiltMap(Size inputSz, Size outputSz, float hfov, float pitch, UMat &MapX, UMat &MapY);

inline int pers2equi(float x, float y, Mat R12, float* lon, float* lat);

void CreateTiltMap15(Size inputSz, Size outputSz, float hfov, float pitch, UMat &MapX, UMat &MapY);

double FindRotation(vector<int> Yaws, vector<int> Pitches, int num_outlier);

void Find_NSPoleCut(Mat img, int NcolIdx, int &Npole, int ScolIdx, int &Spole, float threshold, Scalar bgColor);

void BeautifyPoles(Mat cropped, Mat& result, int top, int bottom);

void LinearSmoothing(Mat grayLeft, Mat grayRight, int smoothingWd, Mat& output);


#endif /* utils_h */
