//
//  utils.cpp
//  m_stitcher
//
//  Created by Kevin Santoso on 2021/09/13.
//

#include <stdio.h>
#include "utils.h"

float median(vector<float> v)
{
    if(v.size() > 0){
        size_t n = floor( v.size() / 2 );
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }
    else{
        return 0;
    }
}

void printMat33(Mat A)
{
  cout<<A.at<float>(0,0)<<" "<<A.at<float>(0,1)<<" "<<A.at<float>(0,2)<<endl;
  cout<<A.at<float>(1,0)<<" "<<A.at<float>(1,1)<<" "<<A.at<float>(1,2)<<endl;
  cout<<A.at<float>(2,0)<<" "<<A.at<float>(2,1)<<" "<<A.at<float>(2,2)<<endl;
}

int sphere_to_equi(int distanceParam, double x_dest, double  y_dest, float* x_src, float* y_src)
{
    double phi, theta, r, s;
    double v[3];
    
    phi = x_dest / distanceParam;
    theta = -y_dest / distanceParam + CV_PI / 2;
    if (theta < 0)
    {
        theta = -theta;
        phi += CV_PI;
    }
    if (theta > CV_PI)
    {
        theta = CV_PI - (theta - CV_PI);
        phi += CV_PI;
    }
    
    s = sin(theta);
    v[0] = s * sin(phi); //  y' -> x
    v[1] = cos(theta);  //  z' -> y
    r = sqrt(v[1] * v[1] + v[0] * v[0]);
    theta = distanceParam * atan2(r, s * cos(phi));
    *x_src = theta * v[0] / r;
    *y_src = theta * v[1] / r;
    return 1;
}



void Lens_Correction(float f, float ppx, float ppy, float k1, float k2, float k3, int input_img_rows, int input_img_cols, UMat& x_d, UMat& y_d)
{
#if 1
    Mat xu, yu;
    xu.create(input_img_rows, input_img_cols, CV_32F);
    yu.create(input_img_rows, input_img_cols, CV_32F);
    
    // Creating meshgrid.
    for(int i = 0; i < input_img_cols; i++)
    {
        xu.at<float>(0,i) = float(i);
    }
    xu = repeat(xu.row(0), input_img_rows, 1);
    
    for(int i = 0; i < input_img_rows; i++)
    {
        yu.at<float>(i,0) = float(i);
    }
    yu = repeat(yu.col(0), 1, input_img_cols);
    
    Mat x, y;
    x = (xu - ppx)/f;
    y = (yu - ppy)/f;
    
    Mat x_sqr, y_sqr;
    pow(x, 2, x_sqr);
    pow(y, 2, y_sqr);
    
    Mat r2 = x_sqr + y_sqr;
    
    Mat k1r2, k2r2_sqr, k3r2_cube, r2_sqr, r2_cube;
    pow(r2,2,r2_sqr);
    pow(r2,3,r2_cube);
    k1r2 = k1*r2;
    k2r2_sqr = k2*r2_sqr;
    k3r2_cube = k3*r2_cube;
    
    // Radial Distortion.
    Mat dr = k1r2 + k2r2_sqr + k3r2_cube;
    Mat drx, dry;
    multiply(x,dr,drx);
    multiply(y,dr,dry);
    Mat x1, y1;
    
    // Tangential distortion to be added later.
    
    x1 = x + drx;
    y1 = y + dry;
    
    Mat xd = (x1*f) + ppx;
    Mat yd = (y1*f) + ppy;
    x_d = xd.getUMat(ACCESS_READ);
    y_d = yd.getUMat(ACCESS_READ);
    
    //free memory
    xu.release();
    yu.release();
    x.release();
    y.release();
    x_sqr.release();
    y_sqr.release();
    r2.release();
    k1r2.release();
    k2r2_sqr.release();
    k3r2_cube.release();
    r2_sqr.release();
    r2_cube.release();
    dr.release();
    drx.release();
    dry.release();
#endif
#if 0
    Mat Output_x = Mat::zeros(input_img_rows, input_img_cols, CV_32F);
    Mat Output_y = Mat::zeros(input_img_rows, input_img_cols, CV_32F);
    
    for (int i = 0; i < input_img_rows; i++)
    {
        for (int j = 0; j < input_img_cols; j++)
        {
            float x = (i - ppx) / f;
            float y = (j - ppy) / f;
            
            float r = x*x + y*y;
            float dr = k1*r + k2*(r*r);
            
            float xd = x + (dr*x);
            float yd = y + (dr*y);
            
            xd = (xd * f) + ppx;
            yd = (yd * f) + ppy;
            
            Output_x.at<Vec3b>(j, i) = xd;
            Output_y.at<Vec3b>(j, i) = yd;
        }
    }
    
    x_d = Output_x.getUMat(ACCESS_READ);
    y_d = Output_y.getUMat(ACCESS_READ);
#endif
}

void movingAverage(vector<double> data, int windowSz, vector<double>& output)
{
    int prevN = ceil(0.5 * windowSz);
    int nextN = floor(0.5 * windowSz);
    int Ndata = (int) data.size();
    int scanStart, scanEnd;
    double sum;
    
    for (int i=0; i<Ndata; i++) {
        scanStart = i - prevN;
        scanEnd = i + nextN;
        
        if (scanStart < 0 || scanEnd > Ndata) {
            output[i] = data[i];
        }
        else {
            sum = std::accumulate(data.begin()+scanStart, data.begin()+scanEnd, 0);
            output[i] = sum/windowSz;
        }
    }
}

void IntensitySmoothing(Mat leftGray, Mat rightGray, int yaw, Mat& leftMask, Mat& rightMask, int smoothing_wd)
{
    vector<double> LR_diffs(leftGray.rows, 0);
    vector<double> trendline(leftGray.rows, 0);
    int L_pixel, R_pixel, LR_diff;
    int L_col_idx = floor((leftGray.cols + yaw) / 2);
    int R_col_idx = ceil((rightGray.cols - yaw) / 2);
    
    for (int r=0; r<leftGray.rows; r++) {
        L_pixel = leftGray.at<uchar>(r, L_col_idx);
        R_pixel = rightGray.at<uchar>(r, R_col_idx);
        
        LR_diff = R_pixel - L_pixel;
        if(abs(LR_diff) < 70) LR_diffs[r] = LR_diff;
    }
    
    // Get the moving average estimation
    int averageWindow = round(0.035 * leftGray.rows);
    movingAverage(LR_diffs, averageWindow, trendline);
    
    // Find Start and End of smoothing
    int half_smoothing_wd = ceil(smoothing_wd*0.5);
    int smoothing_L_start = L_col_idx - half_smoothing_wd;
    int smoothing_L_wd = floor(0.8*smoothing_wd);
    if ((smoothing_L_start + smoothing_L_wd) > leftGray.cols) {
        smoothing_L_wd = leftGray.cols - smoothing_L_start;
    }
    
    int smoothing_R_end = R_col_idx + half_smoothing_wd;
    int smoothing_R_wd = floor(0.8*smoothing_wd);
    int smoothing_R_start = smoothing_R_end - smoothing_R_wd;
    if (smoothing_R_start < 0) {
        smoothing_R_start = 0;
        smoothing_R_wd = smoothing_R_end;
    }
    
    double alpha;
    for (int y=0; y<leftGray.rows; y++) {
        
        alpha = trendline[y] / 255.0;
        // Left Mask
        for (int x=0; x<=smoothing_L_wd; x++) {
            leftMask.at<float>(y, smoothing_L_start+x) += alpha / smoothing_L_wd * (x+1) ;
        }
        //Right Mask
        for (int x=smoothing_R_wd-1; x>=0; x--) {
            rightMask.at<float>(y, smoothing_R_end-x) -= alpha / smoothing_R_wd * x;
        }
    }
}


int rectilinear2sphere( double x_dest,double  y_dest, double* x_src, double* y_src, double R, float pitch)
{
  float y = pitch;
  float x = 0;
  
  float x0 = R * cos(y) * sin(x);
  float y0 = R * cos(y) * cos(x);
  float z0 = R * sin(y);
  
  float alpha = cos(y_dest) * sin(x_dest);
  float beta  = cos(y_dest) * cos(x_dest);
  float gamma = sin(y_dest);
  
  float division = x0*alpha + y0*beta + z0*gamma;
  
  float x1 = R*R * alpha / division;
  float y1 = R*R * beta / division;
  float z1 = R*R * gamma / division;
  
  Mat vec(3,1,CV_32FC1);
  vec = (Mat_<float>(3,1)<< (x1-x0), (y1-y0), (z1-z0));
                            

  Mat vecposX(3,1,CV_32FC1);
  vecposX = (Mat_<float>(3,1)<< cos(x), -sin(x), 0);
  
  Mat vpx2 = vecposX * vecposX.t();
  cv::sqrt(vpx2, vpx2);
  
  Mat deltaX(3,1,CV_32FC1);
  deltaX = vecposX * (vec.t()) / vpx2;
  
  Mat p0(3,1,CV_32FC1);
  p0 = (Mat_<float>(3,1)<< x0, y0, z0);
  
  Mat vecposY(3,1,CV_32FC1);
  vecposY = p0.cross(vecposX);
  
  Mat vpy2 = vecposY * vecposY.t();
  cv::sqrt(vpy2, vpy2);
  
  Mat deltaY(3,1,CV_32FC1);
  deltaY = vecposY * (vec.t()) / vpx2;
  
//    double  phi, theta;
//    phi     = x_dest / distanceparam;
//    theta   =  -y_dest / distanceparam  + M_PI / 2.0;
//    if(theta < 0)
//    {
//        theta = - theta;
//        phi += M_PI;
//    }
//    if(theta > M_PI)
//    {
//        theta = M_PI - (theta - M_PI);
//        phi += M_PI;
//    }
//
//    // Y direction + push top
//    *x_src = distanceparam * tan(phi);
//    *y_src = distanceparam / (tan( theta ) * cos(phi));
    
    return 1;
}

int rectilinear2cyl( double x_dest,double  y_dest, double* x_src, double* y_src, double distanceparam)
{
    double  phi, theta;
    
    phi     = x_dest / distanceparam;
    theta   =  -y_dest / distanceparam  + M_PI / 2.0;
    if(theta < 0)
    {
        theta = - theta;
        phi += M_PI;
    }
    if(theta > M_PI)
    {
        theta = M_PI - (theta - M_PI);
        phi += M_PI;
    }
    
    // Y direction + push top
    *x_src = distanceparam * tan(phi);
    *y_src = distanceparam / (tan( theta ) * cos(phi));
    
    return 1;
}

void CreateTiltMap(Size inputSz, Size outputSz, float hfov, float pitch, UMat &MapX, UMat &MapY)
{
    Mat TiltMap_x(outputSz.height, outputSz.width, CV_32FC1);
    Mat TiltMap_y(outputSz.height, outputSz.width, CV_32FC1);
    
    double a = hfov * M_PI / 180;
    double distanceparam = (double)inputSz.width / a;
    double scanX, scanY;
    double srcX, srcY;
    double tempX, tempY;
    
    int outWidth = outputSz.width;
    int outHeight = outputSz.height;
    
    double w2 = outputSz.width / 2.0 - 0.5;
    double h2 = outputSz.height / 2.0 - 0.5;
    double sw2 = inputSz.width / 2.0 - 0.5;
    double sh2 = inputSz.height / 2.0 - 0.5;
    
    double f = sw2 / tan( hfov/2 * M_PI / 180);
    vector<double> z(outHeight);
    double h, d, theta;
    
    double thetaPitch = pitch/180.0 * M_PI;
    for (int y=0; y<outHeight; y++) {
        h = h2 - y;
        d = sqrt(h*h + f*f);
        theta = atan2(h, f);
        
        z[y] = f / (d * cos(theta+thetaPitch));
    }
    
    for (int j = 0; j<outHeight; j++) { //scan y-wise
        scanY = (double)j - h2;
        
        for (int i=0; i<outWidth; i++) { //scan x-wise
            scanX = (double)i - w2;
            
            rectilinear2cyl(scanX, scanY, &srcX, &srcY, distanceparam);
            
            tempX = srcX / z[j];
            tempY = srcY / z[j];
            tempX += sw2;
            tempY += sh2;
            
            TiltMap_x.at<float>(j, i) = tempX;
            TiltMap_y.at<float>(j, i) = tempY;
        }
    }
    
    MapX = TiltMap_x.getUMat(ACCESS_READ);
    MapY = TiltMap_y.getUMat(ACCESS_READ);
    
    TiltMap_x.release();
    TiltMap_y.release();
}

inline int pers2equi(float x, float y, Mat R12, float* lon, float* lat)
{
  float x_map = cos(x) * cos(y);
  float y_map = sin(x) * cos(y);
  float z_map = sin(y);
  
//  Vec3f xyz(x_map, y_map, z_map);
//
//  Mat xyz2 = (R12 * xyz);
//
//  *lon = xyz2.at<float>(0,1) / xyz2.at<float>(0,0);
//  *lat = -xyz2.at<float>(0,2) / xyz2.at<float>(0,0);

  
  float a = R12.at<float>(0,0);
  float b = R12.at<float>(0,1);
  float c = R12.at<float>(0,2);
  float d = R12.at<float>(1,0);
  float e = R12.at<float>(1,1);
  float f = R12.at<float>(1,2);
  float g = R12.at<float>(2,0);
  float h = R12.at<float>(2,1);
  float i = R12.at<float>(2,2);

  float row1 = a*x_map + b*y_map + c*z_map;
  float row2 = d*x_map + e*y_map + f*z_map;
  float row3 = g*x_map + h*y_map + i*z_map;

  *lon = row2 / row1;
  *lat = -row3 / row1;
  
  return 1;
}

void CreateTiltMap15(Size inputSz, Size outputSz, float hfov, float pitch, UMat &MapX, UMat &MapY)
{
  Mat TiltMap_x(outputSz.height, outputSz.width, CV_32FC1);
  Mat TiltMap_y(outputSz.height, outputSz.width, CV_32FC1);
  
  float phi = to_rad(pitch);
  float hfovr = to_rad(hfov);
  float vfovr = 2 * atan( float(inputSz.height) / inputSz.width * tan(0.5*hfovr) );
  
  float scanX, scanY;
  float density = hfovr / inputSz.width;
  
  const float w2 = outputSz.width / 2.0 - 0.5;
  const float h2 = outputSz.height / 2.0 + (phi / density);
  
  int outWidth = outputSz.width;
  int outHeight = outputSz.height;
  
  float w_len = tan(hfovr / 2.0);
  float h_len = tan(vfovr / 2.0);
  
  float scale_w = float(inputSz.width) / 2 / w_len;
  float scale_h = float(inputSz.height) / 2 / h_len;
  
  Mat yaxis = (Mat_<float>(3,1) << 0, 1, 0);
  Mat zaxis = (Mat_<float>(3,1) << 0, 0, 1);
  
  Mat R1, R2, R12;
  
  Rodrigues(zaxis * 0, R1);
  Rodrigues(R1*yaxis*(-phi), R2);
  
  R1 = R1.inv();
  R2 = R2.inv();
  R12 = R1 * R2;
  
  long startTime = getTickCount();
  
  for (int j = 0; j<outHeight; j++) { //scan y-wise
    scanY = h2 - (float)j;
    float y = scanY * density;
    
    for (int i=0; i<outWidth; i++) { //scan x-wise
      scanX = (float)i - w2;
      float x = scanX * density;
      
      float lon, lat;
      pers2equi(x,y, R12, &lon, &lat);
      
      float Px = (lon + w_len) * scale_w;
      float Py = (lat + h_len) * scale_h;
      
      TiltMap_x.at<float>(j, i) = Px;
      TiltMap_y.at<float>(j, i) = Py;
    }
  }
  
  printf("MakeTemplates in %.3lf sec \n",  (getTickCount() - startTime)/getTickFrequency() );
  
  MapX = TiltMap_x.getUMat(ACCESS_READ);
  MapY = TiltMap_y.getUMat(ACCESS_READ);

  TiltMap_x.release();
  TiltMap_y.release();
}


double FindRotation(vector<int> Yaws, vector<int> Pitches, int num_outlier)
{
    int YawTotal = std::accumulate(Yaws.begin(), Yaws.end(), 0);
    int TailYshear = std::accumulate(Pitches.begin(), Pitches.end(), 0);
    
    vector<int> yawCopy = Yaws;
    vector<int> pitchCopy = Pitches;
    
    for (int i=0; i<num_outlier; i++) {
        int minIdx=0, maxIdx=0;
        int minEl = 99999999;
        int maxEl = -99999999;
        
        for (int j=0; j<pitchCopy.size(); j++) {
            if (pitchCopy[j] > maxEl) {
                maxEl = pitchCopy[j];
                maxIdx = j;
            }
        }
        
        TailYshear -= maxEl;
        YawTotal -= yawCopy[maxIdx];
        
        yawCopy.erase(yawCopy.begin() + maxIdx);
        pitchCopy.erase(pitchCopy.begin() + maxIdx);
        
        for (int j=0; j<pitchCopy.size(); j++) {
            if (pitchCopy[j] < minEl) {
                minEl = pitchCopy[j];
                minIdx = j;
            }
        }
        
        TailYshear -= minEl;
        YawTotal -= yawCopy[minIdx];
        
        yawCopy.erase(yawCopy.begin() + minIdx);
        pitchCopy.erase(pitchCopy.begin() + minIdx);
    }
    
    return ( atan2(TailYshear , YawTotal) * 180 / M_PI );
}

void Find_NSPoleCut(Mat img, int NcolIdx, int &Npole, int ScolIdx, int &Spole, float threshold, Scalar bgColor)
{
    Vec3b thisPixel;
    Vec3b scanPx;
    int B,G,R;
    int northCut = 0;
    int southCut = img.rows;
    
    scanPx = img.at<Vec3b>(0, NcolIdx);
    B = (int)scanPx[0];
    G = (int)scanPx[1];
    R = (int)scanPx[2];
    for (int y=0; y<0.4*img.rows; y++) {
        thisPixel = img.at<Vec3b>(y, NcolIdx);
        
        if ( (int(thisPixel.val[0]) != B) || (int(thisPixel.val[1]) != G) || (int(thisPixel.val[2]) != R)) {
            break;
        }
        else northCut = y;
    }
    
    scanPx = img.at<Vec3b>(img.rows-1, ScolIdx);
    B = (int)scanPx[0];
    G = (int)scanPx[1];
    R = (int)scanPx[2];
    for (int y=img.rows-1; y>0.6*img.rows; y--) {
        thisPixel = img.at<Vec3b>(y, ScolIdx);
        
        if ( (int(thisPixel.val[0]) != B) || (int(thisPixel.val[1]) != G) || (int(thisPixel.val[2]) != R)) {
            break;
        }
        else southCut = y;
    }
    
    Npole = northCut + threshold*img.rows;
    Spole = southCut - threshold*img.rows;
}

void BeautifyPoles(Mat cropped, Mat& result, int top, int bottom)
{
    Mat colorLine, small_pole;
    Mat southPole, northPole;
    int smallWd = 300;
    float gama;
    Vec3b pixel, pixel_avg;
    long sum0, sum1, sum2;
    int n, p;
    int start, end;
    
    int top_small = (top * smallWd / cropped.cols);
    int bot_small = (bottom * smallWd / cropped.cols);
    
    Rect northROI(0,0, cropped.cols,1);
    Rect southROI(0,(cropped.rows-1), cropped.cols,1);
    
    //---------- South
    resize(cropped(southROI), colorLine, Size(smallWd,1));
    blur(colorLine, colorLine, Size(15, 1), Point(-1,-1), BORDER_REPLICATE);
    
    small_pole = Mat(bot_small, smallWd, CV_8UC3);
    colorLine.copyTo(small_pole.colRange(0, smallWd).rowRange(0, 1));
    
    for (int j=1; j<bot_small; j++) {
        gama = float(j) / bot_small;
        n = ceil(gama*gama * smallWd * 0.3);
        
        for (int i=0; i<smallWd; i++) { //scan x-wise
            
            start = i - floor(0.5*n);
            end = i + ceil(0.5*n);
            
            sum0 = 0;
            sum1 = 0;
            sum2 = 0;
            
            for (int k=start; k<end; k++) {
                p = (k+smallWd) % smallWd;
                
                pixel = small_pole.at<Vec3b>(j-1,p);
                sum0 += pixel[0];
                sum1 += pixel[1];
                sum2 += pixel[2];
            }
            
            pixel_avg[0] = round((double)sum0 / n);
            pixel_avg[1] = round((double)sum1 / n);
            pixel_avg[2] = round((double)sum2 / n);
            
            small_pole.at<Vec3b>(j, i) = pixel_avg;
        }
    }
    
    resize(small_pole, southPole, Size(cropped.cols, bottom));
    copyMakeBorder(southPole, southPole, 0, 0, 25, 25, BORDER_WRAP);
    blur(southPole, southPole, Size(51, 51), Point(-1,-1), BORDER_REPLICATE);
    southPole(Rect(25,0, cropped.cols,bottom)).copyTo(result.colRange(0, result.cols).rowRange(cropped.rows+top, cropped.rows+top+bottom));
    
    //---------- North
    resize(cropped(northROI), colorLine, Size(smallWd,1));
    blur(colorLine, colorLine, Size(15, 1), Point(-1,-1), BORDER_REPLICATE);
    
    small_pole = Mat(top_small, smallWd, CV_8UC3);
    small_pole = Scalar(0,0,0);
    colorLine.copyTo(small_pole.colRange(0, smallWd).rowRange(top_small-1, top_small));
    
    for (int j=1; j<top_small; j++) {
        gama = float(j) / top_small;
        n = ceil(gama*gama * smallWd * 0.3);
        
        for (int i=0; i<smallWd; i++) { //scan x-wise
            
            start = i - floor(0.5*n);
            end = i + ceil(0.5*n);
            
            sum0 = 0;
            sum1 = 0;
            sum2 = 0;
            
            for (int k=start; k<end; k++) {
                p = (k+smallWd) % smallWd;
                
                pixel = small_pole.at<Vec3b>(top_small-j,p);
                sum0 += pixel[0];
                sum1 += pixel[1];
                sum2 += pixel[2];
            }
            
            pixel_avg[0] = round((double)sum0 / n);
            pixel_avg[1] = round((double)sum1 / n);
            pixel_avg[2] = round((double)sum2 / n);
            
            small_pole.at<Vec3b>(top_small-j-1, i) = pixel_avg;
        }
    }
    
    resize(small_pole, northPole, Size(cropped.cols, top));
    copyMakeBorder(northPole, northPole, 0, 0, 25, 25, BORDER_WRAP);
    blur(northPole, northPole, Size(51, 51), Point(-1,-1), BORDER_REPLICATE);
    
    northPole(Rect(25,0, cropped.cols,top)).copyTo(result.colRange(0, result.cols).rowRange(0, top));
    
    //---------- Copy Middle
    cropped.copyTo(result.colRange(0, result.cols).rowRange(top, top+cropped.rows));
    
    //clear memory
    colorLine.release();
    small_pole.release();
    southPole.release();
    northPole.release();
}

void LinearSmoothing(Mat grayLeft, Mat grayRight, int smoothingWd, Mat& output)
{
    int startCol = 0.5 * (grayLeft.cols - smoothingWd);
    int endCol = startCol + smoothingWd;
    
    if (startCol < 1) {
        startCol = 1;
        smoothingWd = endCol-startCol;
    }
    if (endCol > grayLeft.cols-1) {
        endCol = grayLeft.cols-1;
        smoothingWd = endCol-startCol;
    }
    
    output.create(grayLeft.size(), grayLeft.type());
    
    if (grayLeft.type() == 5) { //32F
        
        output = 0.0f;
        
        for (int x=0; x<smoothingWd; x++) {
            float alpha = (float)x/smoothingWd;
            for (int y=0; y<output.rows; y++) {
                output.at<float>(y,x+startCol) = (1.0-alpha) * grayLeft.at<float>(y,x+startCol) + alpha * grayRight.at<float>(y,x+startCol);
            }
        }
    }
    else if (grayLeft.type() == 0) { //8U
        
        output = 0;
        
        for (int x=0; x<smoothingWd; x++) {
            float alpha = (float)x/smoothingWd;
            for (int y=0; y<output.rows; y++) {
                int valLeft = getPixelU(grayLeft, x+startCol, y, 0);
                int valRight = getPixelU(grayRight, x+startCol, y, 0);
                setPixel(output, x+startCol, y, 0, ((1.0-alpha) * valLeft + alpha * valRight) );
            }
        }
    }
    else if (grayLeft.type() == 1) { //8S
        
        output = 0;
        
        for (int x=0; x<smoothingWd; x++) {
            float alpha = (float)x/smoothingWd;
            for (int y=0; y<output.rows; y++) {
                int valLeft = getPixelS(grayLeft, x+startCol, y, 0);
                int valRight = getPixelS(grayRight, x+startCol, y, 0);
                setPixel(output, x+startCol, y, 0, ((1.0-alpha) * valLeft + alpha * valRight) );
            }
        }
    }
    
    // Copy Left
    grayLeft(cv::Rect(0,0,startCol,grayLeft.rows)).copyTo(output.colRange(0, startCol));
    
    // Copy right
    grayRight(cv::Rect(endCol,0,(grayRight.cols-endCol),grayLeft.rows)).copyTo(output.colRange(endCol, grayRight.cols));
    
}
