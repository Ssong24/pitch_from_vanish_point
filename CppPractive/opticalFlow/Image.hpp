//
//  Image.hpp
//  m_stitcher
//
//  Created by Song on 2022/02/03.
//

#ifndef Image_hpp
#define Image_hpp

#include <stdio.h>

// Multi-threading
#include <thread>
#include <future>

#include <opencv2/opencv.hpp>

#include "LensParameter.hpp"




const float ratio16by9 = 16.0 / 9.0;
const float ratio9by16 = 9.0 / 16;

const cv::Scalar bgrColor = cv::Scalar(168, 168, 168);



inline int getPixelUs(cv::Mat src, int x, int y, int ch)
{
    int num_ch = src.channels();
    if(ch > num_ch) ch = num_ch-1;
    
    unsigned char thisPx = src.data[y * src.step + x * num_ch + ch];
    return int(thisPx);
}

inline int getPixelSs(cv::Mat src, int x, int y, int ch)
{
    int num_ch = src.channels();
    if(ch > num_ch) ch = num_ch-1;

    
    char thisPx = src.data[y * src.step + x * num_ch + ch];
    int thisPxInt = int(thisPx);

    int resultPx = thisPxInt;
    if (thisPxInt > 127) {
        resultPx = thisPxInt - 256;
    }
    return resultPx;
}

inline void setPixel(cv::Mat &src, int x, int y, int ch, int value)
{
    src.data[ y * src.step + x * src.channels() + ch] = value;
}



inline int pers2equi(float x, float y, cv::Mat R12, float* lon, float* lat)
{
  float x_map = cos(x) * cos(y);
  float y_map = sin(x) * cos(y);
  float z_map = sin(y);
  
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

inline void rectilinear2cyl( double x_dest,double  y_dest, double* x_src, double* y_src, double distanceparam)
{
    double  phi, theta;
    
    phi     =  x_dest / distanceparam;
    theta   = - y_dest / distanceparam  +  M_PI / 2.0;
    
    if(theta < 0)
    {
        theta = - theta;
        phi += M_PI;
    }
    else if(theta > M_PI)
    {
        theta = M_PI - (theta - M_PI);
        phi += M_PI;
    }
    
    // Y direction + push top
    *x_src = distanceparam * tan(phi);
    *y_src = distanceparam / (tan( theta ) * cos(phi));
    
    //return 1;
}


int sphere_to_equi(float distanceParam, double x_dest, double  y_dest, float* x_src, float* y_src);

int circularFisheye_to_equi(float fov, float D, double x_dest, double  y_dest, float* x_src, float* y_src);

void check_blended_part_is_matched(cv::Mat src1, cv::Mat src2, int cutRight, int bumper, int bumper_copy=0, int offset=0) ;

void setEquiMapSize(cv::Size resized, float tilt_wd_ratio, float tilt_ht_ratio, cv::Size &tiltSize);
void setImageRatio16to9(int width, int height, bool vertical, int& _src_cols, int& _src_rows);
void setCanvasSize(int _src_cols, int _src_rows, int& _finalCanvasH, cv::Size& _resizedInputSize) ;
void resizeCanvasSize(int _src_cols, int _src_rows, int outPanoHeight, float _hfov, float _vfov, float _wid_ratio, bool vertical, float tiltAngleSum, int& _finalCanvasH, cv::Size& _resizedInputSize);
void resizeAndCopyImgByRatio(int src_rows, int src_cols, cv::Size resizedImgSize, cv::UMat srcImg, cv::UMat& dstImg);

void changePoleCutThresh(float rotAngle, float& poleCutThresh_0, float& poleCutThresh_1);
bool convertImgChannel(cv::Mat srcImage, float inputAR, int s_cols, int s_rows,  cv::Mat& outImg) ;

void save_512_1024_img(cv::Mat* srcImg, std::string* srcDir, std::stringstream* fstream);
void save_5000_10000_img(cv::Mat* srcImg, std::string* srcDir, std::string finalPano_name);
void save_img_in_pano_viewer(cv::Mat* srcImg);
void save_horizontal_pano(cv::UMat *srcImg, std::string* pano_name) ;
string set_horPano_name(string srcDir, int i, string ucd_mode, float hfov, int outPanoH, float tilt_angle, string imgExt);
std::string set_finalPano_name (float hfov, struct cameraParam camP, int n_features, int outPanoH, std::string imgExt, float tiltList[3], float rollList[3]=NULL);

template<typename T>
void saveImgSize(T inputSize, T& outputSize) {
    outputSize = inputSize;
}
void get_input_image_width_and_height(string initImgPath, int& width, int& height);
void read_cam_parameters_from_txt(cv::Size inputSize, cv::Size mapSize, string mtx_file, string dist_file, struct cameraParam &camP) ;

namespace rectifymap {
    void update_Equimap(float hfov, float pitch, int src_col, int src_rows, int dest_col, int dest_rows);

    void createDistortMap( float k1, float k2, float k3, cv::Size inputSize,    cv::Size mapSize, cv::UMat& x_d, cv::UMat& y_d, string mtx_file_path="");
    void createDistortMap_size(cv::Size inputSize,  cv::Size mapSize, struct cameraParam camP, cv::UMat& mapX, cv::UMat& mapY);
    void createEquiMap15(cv::Size inputSz, cv::Size outputSz, float hfov, float pitch, cv::UMat &MapX, cv::UMat &MapY);
    void createEquiMap(cv::Size inputSz, cv::Size outputSz, float hfov, float pitch, cv::UMat &MapX, cv::UMat &MapY);
    void createPerspMap_YRP(cv::Size imgSize, cv::Mat& mapX, cv::Mat& mapY, float pitch=90.0, float yaw=90.0, float roll=90.0) ;
    void warpPerspective_YRP(cv::UMat src, cv::UMat &dst, cv::Size imgSize, float roll=0.0f, float pitch=0.0f, float yaw=0.0f);
}

namespace ftmask {
    void createFeatureMask(cv::Size imgSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left);
    void createFeatureMask_halfOnes(cv::Size imgSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left);

    void createFeatureLeftRightMasks(int features_pad, cv::Size ftMapSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left);
    void createFeatureLeftRightMasks(int features_pad, cv::Size rightMapSize, cv::Size leftMapSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left);
    void createFeatureMasks3Pair(int i, cv::Size ftMapSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left) ;

}

namespace smoothing {
    void LinearSmoothing(cv::Mat grayLeft, cv::Mat grayRight, int smoothingWd, cv::Mat& output);
    bool NewSmoothing_vecImg (vector<cv::Mat> srcImages, int cutRight, int bumper, int i, float blendStrength_0, float blendStrength_1,  cv::Mat& buffer, int bumper_copy=0, int offset=0);
    bool NewSmoothing_matImg (cv::Mat imgLeft, cv::Mat imgRight, int cutRight, int bumper,  float blendStrength_0, float blendStrength_1,  cv::Mat& buffer, int diff_cutR=0);
    bool NewSmoothing_blendedRoI (cv::Mat imgLeft, cv::Mat imgRight, float blendStrength_0, float blendStrength_1, cv::Mat & buffer );

}

void blend_cropped_point(int cropStart1, int cropWidth1, int cropStart2, int cropWidth2, int south, int north,  cv::Size _finalImgSize, cv::Mat canvas, cv::Mat& croppedImg );
void stitch_cropped_only(int cropStart1, int cropWidth1, int cropStart2, int cropWidth2, int south, int north, cv::Mat canvas, cv::Mat& croppedImg);
void BeautifyPoles(cv::Mat cropped, cv::Mat& result, int top, int bottom);


void Find_NSPoleCut(cv::Mat img, int NcolIdx, int &Npole, int ScolIdx, int &Spole, float threshold, cv::Scalar bgColor);
void find_north_and_south_cutoff(int Ltype,float poleCutThresh_0, float poleCutThresh_1, cv::Size finalSize, std::vector<int> YawOnCanvas, cv::Mat canvas, int& south, int& north);

void canvasCopyToCroppedImg(int top, int left1, int left2,  int wid1, int wid2, int height, cv::Mat canvas, cv::Mat& croppedImage );
void copyBufferToCanvas(uint8_t* pixelPtr, int cn, int startX, int endX, int offsetX, int widY, int bufferH, cv::Mat& canvas, int x_shift=0);


void setCropPoint(int num_of_images, int Ltype, float poleCutThresh_0, float poleCutThresh_1, int RotatedYawTotal, int finalCanvas_height, float phoneTilt,  cv::Size finalSize, std::vector<int> YawOnCanvas, cv::Mat canvas, cv::Mat &croppedImage, cv::Mat &outputPano);
void setCropPoint( float poleCutThresh_0, float poleCutThresh_1, float pitch, int numImages, int RotatedXTotal, cv::Size _finalImgSize, vector<int> _hLenOnCanvas, cv::Mat canvas, cv::Mat &croppedImg, int& padding_top, int& padding_bottom);



// Rotate the image with angle (degree)
void createRotatedImage(cv::Mat src, cv::Mat &dst, int theta_degree);
bool transposeAndFlipImg(cv::Mat srcImage, float inputAR, int s_cols, int s_rows,  cv::Mat& outImg);
bool checkBlendRoISize(int canvasLeft, int cutWidth, int canvas_H);




void parallel_estimate_and_warp(std::vector<cv::UMat> _vecImgs, std::vector<int> _YawOnCanvas, std::vector<std::vector<cv::Point2f>> _vecGoodKp1, std::vector<std::vector<cv::Point2f>> _vecGoodKp2, int _upDownW, int _canH, std::vector<cv::Mat> & _affined_Imgs);
void estimate_and_warp(int i, std::vector<cv::UMat> _vecImgs, std::vector<int> _YawOnCanvas,  std::vector<std::vector<cv::Point2f>> _vecGoodKp1, std::vector<std::vector<cv::Point2f>> _vecGoodKp2,  int _upDownW, int _canH, std::vector<cv::Mat> & _affined_Imgs);
void copy_center(std::vector<cv::UMat> _vecImgs, std::vector<cv::Mat> & _affined_Imgs) ;




#endif /* Image_hpp */
