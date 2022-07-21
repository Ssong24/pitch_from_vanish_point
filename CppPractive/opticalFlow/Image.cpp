//
//  Image.cpp
//  m_stitcher
//
//  Created by Song on 2022/02/03.
//

#include "Image.hpp"



extern inline int getPixelUs(cv::Mat src, int x, int y, int ch);
extern inline int getPixelSs(cv::Mat src, int x, int y, int ch);
extern inline void setPixel(cv::Mat &src, int x, int y, int ch, int value);

extern inline int pers2equi(float x, float y, cv::Mat R12, float* lon, float* lat);
extern inline void rectilinear2cyl( double x_dest,double  y_dest, double* x_src, double* y_src, double distanceparam);

int sphere_to_equi(float distanceParam, double x_dest, double  y_dest, float* x_src, float* y_src)
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


int circularFisheye_to_equi(float fov, float D, double x_dest, double  y_dest, float* x_src, float* y_src){
    float theta,phi, ry;
    cv::Point3f psph;
    
    theta = M_PI * (x_dest / D ); // -pi to pi
    phi   = M_PI * (y_dest / D );  // -pi/2 to pi/2
    
    psph.x = cos(phi) * sin(theta);
    psph.y = cos(phi) * cos(theta);
    psph.z = sin(phi);
    
    theta = atan2(psph.z,psph.x);
    phi = atan2(sqrt(psph.x*psph.x+psph.z*psph.z),psph.y);
    
    ry = D * phi / fov;
    
    *x_src = ry * cos(theta);
    *y_src = ry * sin(theta);
    
    return 1;
}


bool checkBlendRoISize(int canvasLeft, int cutWidth, int canvas_H) {
    if (canvasLeft + cutWidth > canvas_H) {
        cout << "\nRoI is bigger than the size of Canvas. Stitching failed! Check if Lens is attached." << endl;
        return false;
    }
    return true;
}


void save_img_in_pano_viewer(cv::Mat* srcImg) {
    cv::imwrite("/Users/3i-21-331/workspace/pano_viewer/docroot/pano.jpg", *srcImg);
}

void save_5000_10000_img(cv::Mat* srcImg, string* srcDir, string finalpano_name) {
    cv::Mat resized;
    
    resize(*srcImg, resized, cv::Size(10000,5000), cv::INTER_CUBIC);
    cv::imwrite(*srcDir + "/" + finalpano_name + "_10Kx5K_cubic.jpg", resized);
}

void save_512_1024_img(cv::Mat* srcImg, string* srcDir, stringstream* fstream) {
    cv::Mat resized;
    resize(*srcImg, resized, cv::Size(1024,512), cv::INTER_LINEAR);
    imwrite(*srcDir + "/" + fstream->str(), resized);
}

void save_horizontal_pano(cv::UMat *srcImg, std::string* pano_name) {
    cv::Mat hor_pano;
    createRotatedImage(srcImg->getMat(cv::ACCESS_READ), hor_pano, 90);
    imwrite(*pano_name, hor_pano);
}


string set_horPano_name(string srcDir, int i, string ucd_mode, float hfov, int outPanoH, float tilt_angle, string imgExt) {
    
    string pano_name = srcDir + "/pano_" + to_string(i) + "_" + ucd_mode + to_string( hfov ) + "_" + to_string( outPanoH ) + "_" + to_string( tilt_angle ) + imgExt;
    return pano_name;
}

string set_finalPano_name(float hfov, struct cameraParam camP, int n_features, int outPanoH, string imgExt, float tiltList[3], float rollList[3]) {
    
    stringstream fstream;
    fstream << "HFOV_";
    fstream << std::fixed << setprecision(3) << hfov /*camera_parameter.HFov */;
    fstream <<"_k1_"<< camP.k1 <<"_k2_"<< camP.k2 <<"_k3_"<< camP.k3 << "_";
    fstream << std::fixed << setprecision(1)  << tiltList[0] << "_";
    fstream << std::fixed << setprecision(1) << tiltList[1] << "_-";
    fstream << std::fixed << setprecision(1) << abs(tiltList[2]) << "_";
    if (rollList != NULL) {
        for (int i = 0; i < 3 ; i++) {
            if (rollList[i] > 0)
                fstream << std::fixed << setprecision(1)  << rollList[i] << "_";
            else
                fstream << "-" << std::fixed << setprecision(1)  << abs(rollList[i]) << "_";
        }
    }
    
    fstream << to_string(n_features) << "_H-" << outPanoH << imgExt;
    return fstream.str();
}

void setEquiMapSize(cv::Size resized, float tilt_wd_ratio, float tilt_ht_ratio, cv::Size &tiltSize) {
    int tilt_wd = tilt_wd_ratio * resized.width;
    int tilt_ht = tilt_ht_ratio * resized.height;

    tiltSize = cv::Size(tilt_wd, tilt_ht);
}



void setCanvasSize(int _src_cols, int _src_rows, int& _finalCanvasH, cv::Size& _resizedInputSize) {

    float _widthByLensHFov = _src_cols;
    _finalCanvasH = _src_rows;

    if (_src_rows < _finalCanvasH) { // source height < finH
        _finalCanvasH = _src_rows;
        _widthByLensHFov = _finalCanvasH * ratio9by16;
    }

    _resizedInputSize = cv::Size((int)_widthByLensHFov, _finalCanvasH);
}

void resizeCanvasSize(int _src_cols, int _src_rows, int outPanoHeight, float _hfov, float _vfov, float _wid_ratio, bool vertical, float tiltAngleSum, int& _finalCanvasH, cv::Size& _resizedInputSize) {
    // 1. set finalCanvasH by outPanoWidth and horizontal fov
    // 2. set resized InputSize with widthByLensHFov and finalCanvasH

    float outPanoWidth = 2.0 * outPanoHeight;
    float _widthByLensHFov;

    if (!vertical) {  // horizontal
        _widthByLensHFov = _hfov / 360.0 * outPanoWidth;  // = 4032; --> full panorama

        _widthByLensHFov *= _wid_ratio;
        _finalCanvasH = round(ratio16by9 * _widthByLensHFov);

        if (_src_rows < _finalCanvasH) { // source height < finH
            _finalCanvasH = _src_rows;
            _widthByLensHFov = _finalCanvasH * ratio9by16;
        }

    }

    else {
        float w_h_ratio = (float)_src_cols / _src_rows;
        _widthByLensHFov = _vfov / (_vfov + tiltAngleSum) * outPanoHeight;  // 50(constant) should mean different of tilt angle [0] and [2]
        _finalCanvasH = _widthByLensHFov / w_h_ratio;

        if (_src_rows < _finalCanvasH) {
            _finalCanvasH = _src_rows;
            _widthByLensHFov = _finalCanvasH * w_h_ratio;
        }
    }

    _resizedInputSize = cv::Size((int)_widthByLensHFov, _finalCanvasH);
    

}

void setImageRatio16to9(int width, int height, bool vertical, int& _src_cols, int& _src_rows)
{

    // set src_cols and rows to source image size
    _src_cols = width;
    _src_rows = height;

    // when width is smaller than src_cols (rows * 9/16), there's problem...
    if (!vertical) {
        if (height > width) {
            _src_cols = round(_src_rows * ratio9by16);
            if (_src_cols > width) {
                _src_cols = width;
                _src_rows = round(_src_cols * ratio16by9);
            }
        }
        else {
            _src_rows = round(_src_cols * ratio9by16);
        }
    }
}


void read_cam_parameters_from_txt(cv::Size inputSize, cv::Size mapSize, string mtx_file_txt, string dist_file_txt,  struct cameraParam &camP) {

    ifstream inFile_mtx, inFile_dist;
    float f, fx, fy, ppx, ppy;
    int img_rows, img_cols;
    
    img_rows = (int)mapSize.height;
    img_cols = (int)mapSize.width;

    inFile_mtx.open(mtx_file_txt);

    if (inFile_mtx.is_open()) {

        cout << "mtx_file_txt: " << mtx_file_txt << endl;

        string line;
        string delimiter = " ";
        vector<float> nums{};
        getline(inFile_mtx, line);
        size_t pos = 0;
        while((pos = line.find(delimiter)) != string::npos) {
            nums.push_back(stof(line.substr(0, pos)));
            line.erase(0, pos + delimiter.length());
        }
        

        cout << "==========================================================" << endl;
        cout << "Before f, ppx, ppy: " << (mapSize.width  < mapSize.height ? mapSize.height : mapSize.width) << ", " << mapSize.width / 2 << ", " << mapSize.height / 2 << ", " ;
       
        float widRatio = mapSize.width / (float) inputSize.width;
        float heiRatio = mapSize.height / (float)inputSize.height;
        // add * (3. / 4.) for considering different image ratio
        fx = nums[0] * widRatio;   fy = nums[4] * heiRatio;
        ppx = nums[2] * (3./4.) * widRatio;  ppy = nums[5] * heiRatio;

        cout << "ori image size: " << inputSize << endl; // already 9:16
        cout << "Ori (12:16) -> (9:16) img size: " << inputSize.width * (3. / 4.) <<", " << inputSize.height  << endl;
        cout << " --- Original fx, fy, ppx, ppy from txt file --- " << endl;
        cout << "(" << nums[0] << ", " << nums[4]  << "),   (" << nums[2] << ", " << nums[5] << ") " << endl;
        cout << " --- After resizing 12:16 -> 9:16 ---" << endl;
        cout << "(" << nums[0] * (3./4.)  << ", " << nums[4]  << "),   (" << nums[2] * (3./4.)  << ", " << nums[5] << ") " << endl;
        cout << "map img size: " << mapSize.width << ", " << mapSize.height << endl;
        cout << " --- After resizing only size ! ---" << endl;
        cout << "(" << fx * widRatio  << ", " << fy * heiRatio << "),   (" << ppx * widRatio << ", " << ppy * heiRatio << ") " << endl;


        float rad2deg = 180.0 / M_PI;
        float hfov_ori = 2 * atan(inputSize.width / (2. * nums[0])) ;
        float hfov_916 = 2 * atan(mapSize.width / (2. * fx)) ;
        float vfov_ori = 2 * atan(inputSize.height / (2. * nums[4]));


        // printf("calculated ori-hFOV: %3.3f\n", hfov_ori * rad2deg);
        // printf("calculated ori-vFOV: %3.3f\n", vfov_ori * rad2deg);
        // printf("vfov calculator: %3.3f\n", 2 * atan( tan(hfov_ori/2.0) * (float)inputSize.height/(float)inputSize.width ) * rad2deg );
        // printf("9:16 hFOV: %3.3f\n", 2 * atan( tan(hfov_ori/2.0) *  0.75) * rad2deg );
        

        camP.fx = fx;
        camP.fy = fy;
        camP.Ppx = ppx;
        camP.Ppy = ppy;
    }
    
    else {
        f = (mapSize.width  < mapSize.height ? mapSize.height : mapSize.width);
        
        fx = fy = f;
        ppx = img_cols / 2.0;
        ppy = img_rows / 2.0;

        // printf("%d, %d\n", img_cols, img_rows);
        printf("fx, fy, ppx, ppy: %.2f, %.2f, %.2f, %.2f\n", fx, fy, ppx, ppy);

        camP.fx = f;
        camP.fy = f;
        camP.Ppx = ppx;
        camP.Ppy = ppy;
    }

    inFile_dist.open(dist_file_txt);

    if (inFile_dist.is_open()) {

         cout << "dist_file_txt: " << dist_file_txt << endl;

        string line;
        string delimiter = " ";
        vector<float> nums{};
        getline(inFile_dist, line);
        size_t pos = 0;
        while((pos = line.find(delimiter)) != string::npos) {
            nums.push_back(stof(line.substr(0, pos)));
            line.erase(0, pos + delimiter.length());
        }
        // float k1, k2, k3;
        camP.k1 = nums[0];
        camP.k2 = nums[1];
        camP.k3 = nums[3];
        cout << "k1, k2, k3: " << camP.k1 << ", " << camP.k2 << ", "<< camP.k3 << endl;
    }
    cout << "===============================================" << endl;


}





namespace rectifymap {
void update_Equimap(float hfov, float pitch, int src_col, int src_rows, int dest_col, int dest_rows, cv::UMat& Equimap_xu, cv::UMat& Equimap_yu)
{
    cv::Mat Equimap_x(dest_rows, dest_col, CV_32FC1);
    cv::Mat Equimap_y(dest_rows, dest_col, CV_32FC1);
    
    double a = hfov * 2 * M_PI / 360;
    float distanceParam = src_col / a;
    float pixelPitch = pitch/hfov * src_col;
    float scale = (float)src_rows/dest_rows;
    
    double w2 = dest_col / 2.0 - 0.5;
    double h2 = dest_rows / 2.0 - 0.5 + (pixelPitch/2.0/scale);
    double sw2 = src_col / 2.0 - 0.5;
    double sh2 = src_rows / 2.0 - 0.5 + pixelPitch;
    double y_d, x_d;
    float y_s, x_s;
    
    for (int y = 0; y < dest_rows; y++)//y
    {
        y_d = (double)y - h2;
        for (int x = 0; x < dest_col; x++)//x
        {
            x_d = (double)x - w2;
            sphere_to_equi(distanceParam, x_d*scale, y_d*scale, &x_s, &y_s);
            
            Equimap_x.at<float>(y, x) = x_s + sw2;
            Equimap_y.at<float>(y, x) = y_s + sh2;
        }
    }
    
    Equimap_xu = Equimap_x.getUMat(cv::ACCESS_READ);
    Equimap_yu = Equimap_y.getUMat(cv::ACCESS_READ);
    
    Equimap_x.release();
    Equimap_y.release();
}

void createDistortMap( float k1, float k2, float k3, cv::Size inputSize, cv::Size mapSize, cv::UMat& x_d, cv::UMat& y_d)
{
    float f, ppx, ppy;

    f = (inputSize.width  < inputSize.height ? inputSize.height : inputSize.width);
    ppx = mapSize.width  / 2;
    ppy = mapSize.height / 2;
    

    cv::Mat xu, yu;
    xu.create(mapSize, CV_32F);
    yu.create(mapSize, CV_32F);
    
    // Creating meshgrid.
    for(int i = 0; i < mapSize.width; i++)
        xu.at<float>(0,i) = float(i);
    xu = repeat(xu.row(0), mapSize.height, 1);
    
    for(int i = 0; i < mapSize.height; i++)
        yu.at<float>(i,0) = float(i);
    yu = repeat(yu.col(0), 1, mapSize.width);
    
    float scale = (float)mapSize.height / inputSize.height;
    printf("scale: %f\n", scale);
    
    cv::Mat x, y;
    x = (xu/scale - ppx)/f;
    y = (yu/scale - ppy)/f;
    
    cv::Mat x_sqr, y_sqr;
    pow(x, 2, x_sqr);
    pow(y, 2, y_sqr);
    
    cv::Mat r2 = x_sqr + y_sqr;
    
    cv::Mat k1r2, k2r2_sqr, k3r2_cube, r2_sqr, r2_cube;
    pow(r2,2,r2_sqr);
    pow(r2,3,r2_cube);
    k1r2 = k1*r2;
    k2r2_sqr = k2*r2_sqr;
    k3r2_cube = k3*r2_cube;
    
    // Radial Distortion.
    cv::Mat dr = k1r2 + k2r2_sqr + k3r2_cube;
    cv::Mat drx, dry;
    multiply(x,dr,drx);
    multiply(y,dr,dry);
    cv::Mat x1, y1;
    
    // Tangential distortion to be added later.
    
    x1 = x + drx;
    y1 = y + dry;
    
    cv::Mat xd = (x1*f) + ppx;
    cv::Mat yd = (y1*f) + ppy;
    x_d = xd.getUMat(cv::ACCESS_READ);
    y_d = yd.getUMat(cv::ACCESS_READ);
    
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
    x1.release();
    y1.release();
    xd.release();
    yd.release();
    
}




void createDistortMap_size(cv::Size inputSize,  cv::Size mapSize, struct cameraParam camP, cv::UMat& mapX, cv::UMat& mapY) {
    float f;
    // Use ppx, ppy, k1, k2, k3 of intrinsic parameter
    // float f, ppx, ppy;
    float fx, fy;
    float ppx, ppy;

    fx = camP.fx;       fy = camP.fy;
    ppx = camP.Ppx;     ppy = camP.Ppy;
    int img_rows, img_cols;
    
    img_rows = (int)mapSize.height;
    img_cols = (int)mapSize.width;
    
    

    // Initialize rectification map
    cv::Mat xu, yu;
    xu.create(mapSize, CV_32F);
    yu.create(mapSize, CV_32F);

    // Creating meshgrid
    for (int i = 0; i < img_cols; i++) // x = 0~width, y = 0
        xu.at<float>(0,i) = float(i);
    xu = repeat(xu.row(0), img_rows, 1);  // 0-th row (one line containing zero value)

    for (int i = 0; i < img_rows; i++)
        yu.at<float>(i,0) = float(i);
    yu = repeat(yu.col(0), 1, img_cols);


    cv::Mat x, y;
    x = (xu - ppx)/fx;
    y = (yu - ppy)/fy;

    cv::Mat x_pow, y_pow;
    pow(x, 2, x_pow);
    pow(y, 2, y_pow);

    cv::Mat r2, r4, r6;
    r2 = x_pow + y_pow;
    pow(r2, 2, r4);
    pow(r2, 3, r6);

    cv::Mat k1r2, k2r4, k3r6;
    k1r2 = camP.k1 * r2;
    k2r4 = camP.k2 * r4; //-1 * r4;
    k3r6 = camP.k3 * r6;

    // Radial Distortion.
    cv::Mat dr = 1 + k1r2 + k2r4 + k3r6;
    cv::Mat drx, dry;
    multiply(x, dr, drx);
    multiply(y, dr, dry);

    // Tangential distortion to be added later.
    cv::Mat xd = (drx * fx) + ppx;
    cv::Mat yd = (dry * fy) + ppy;
    
    mapX = xd.getUMat(cv::ACCESS_READ);  // Convert Mat to UMat
    mapY = yd.getUMat(cv::ACCESS_READ);

    // free memory
    xu.release();
    yu.release();
    x.release();
    y.release();
    x_pow.release();
    y_pow.release();
    r2.release();
    k1r2.release();
    k2r4.release();
    k3r6.release();
    r4.release();
    r6.release();
    dr.release();
    drx.release();
    dry.release();
}

void createEquiMap(cv::Size inputSz, cv::Size outputSz, float hfov, float pitch,
                   cv::UMat &MapX, cv::UMat &MapY) {
    
    cv::Mat TiltMap_x(outputSz.height, outputSz.width, CV_32FC1);
    cv::Mat TiltMap_y(outputSz.height, outputSz.width, CV_32FC1);
    
    double hfovr = hfov * M_PI / 180;
    double distanceparam = (double)inputSz.width / hfovr;
    double scanX, scanY;
    double srcX, srcY;
    double tempX, tempY;
    
    int outWidth = outputSz.width;
    int outHeight = outputSz.height;
    
    // Why substract 0.5?
    double w2 = outputSz.width  / 2.0 - 0.5;
    double h2 = outputSz.height / 2.0 - 0.5;
    double sw2 = inputSz.width / 2.0 - 0.5;
    double sh2 = inputSz.height / 2.0 - 0.5;
    
    double f = sw2 / tan( hfov / 2 * M_PI / 180);  //  focal length
    vector<double> z(outHeight);  // ?
    double h, d, theta;
    double thetaPitch = pitch * M_PI / 180;  // tilted angle [rad]
    
    for (int y = 0; y < outHeight; y++) {
        h = h2 - y;
        d = sqrt(h*h + f*f);
        theta = atan2(h, f);
        
        z[y] = f / (d * cos(theta + thetaPitch)); //  Considering pitch, change the ' ' value
    }
    
    for (int j = 0; j < outHeight; j++) { //scan y-wise
        scanY = (double)j - h2;
        
        for (int i=0; i < outWidth; i++) { //scan x-wise
            scanX = (double)i - w2;
            
            rectilinear2cyl(scanX, scanY, &srcX, &srcY, distanceparam);
            
            tempX = srcX / z[j] + sw2;
            tempY = srcY / z[j] + sh2;
            
            TiltMap_x.at<float>(j, i) = tempX;
            TiltMap_y.at<float>(j, i) = tempY;
        }
    }
    
    MapX = TiltMap_x.getUMat(cv::ACCESS_READ);
    MapY = TiltMap_y.getUMat(cv::ACCESS_READ);
    
    // Clearing memory
    TiltMap_x.release();
    TiltMap_y.release();
    
}


void createEquiMap15(cv::Size inputSz, cv::Size outputSz, float hfov, float pitch, cv::UMat &MapX, cv::UMat &MapY)
{

    // Use hfov, tiltList of intrinsic parameter
    cv::Mat TiltMap_x(outputSz.height, outputSz.width, CV_32FC1);
    cv::Mat TiltMap_y(outputSz.height, outputSz.width, CV_32FC1);

    float phi =  pitch / 180.0 * M_PI;
    float hfovr = hfov / 180.0 * M_PI;
    float vfovr =  2 * atan( float(inputSz.height) / inputSz.width * tan( 0.5 * hfovr) ); // my- float(inputSz.height) / inputSz.width  * hfovr; //

    
    float scanX, scanY;
    float density = hfovr / inputSz.width;  // horizontal angle per pixel

    
    const float centerX = outputSz.width / 2.0 - 0.5;
    const float centerY = outputSz.height / 2.0 + (phi / density) ;

    int outWidth = outputSz.width;
    int outHeight = outputSz.height;

    float w_len = tan(hfovr / 2.0);
    float h_len = tan(vfovr / 2.0);

    float scale_w = float(inputSz.width) / 2. / w_len;
    float scale_h = float(inputSz.height) / 2. / h_len;

    cv::Mat yaxis = (cv::Mat_<float>(3,1) << 0, 1, 0);
    cv::Mat zaxis = (cv::Mat_<float>(3,1) << 0, 0, 1);

    cv::Mat R1, R2, R12;
    Rodrigues(zaxis * 0, R1);
    Rodrigues(R1*yaxis*(-phi), R2);

    R1 = R1.inv();
    R2 = R2.inv();
    R12 = R1 * R2;

    for (int j = 0; j < outHeight; j++) { //scan y-wise
        scanY = centerY - (float)j;
        float y = scanY * density;

        for (int i=0; i < outWidth; i++) { //scan x-wise
            scanX = (float)i - centerX;
            float x = scanX * density;
            
            float lon, lat;
            pers2equi(x, y, R12, &lon, &lat);
            
            float Px = (lon + w_len) * scale_w;
            float Py = (lat + h_len) * scale_h;
            
            TiltMap_x.at<float>(j, i) = Px;  // output Equimap x
            TiltMap_y.at<float>(j, i) = Py;  // output Equimap y
        }
    }
    MapX = TiltMap_x.getUMat(cv::ACCESS_READ);
    MapY = TiltMap_y.getUMat(cv::ACCESS_READ);

    TiltMap_x.release();
    TiltMap_y.release();
    
}

void warpPerspective_YRP(cv::UMat src, cv::UMat &dst, cv::Size imgSize, float roll, float pitch, float yaw) {

    // Initialize rectification map
    int input_img_rows = imgSize.height;
    int input_img_cols = imgSize.width;
    
    cv::Mat xu, yu;
    xu.create(input_img_rows, input_img_cols, CV_32F);
    yu.create(input_img_rows, input_img_cols, CV_32F);

    // Creating meshgrid
    for (int i = 0; i < input_img_cols; i++) // x = 0~width, y = 0
        xu.at<float>(0,i) = float(i);
    xu = repeat(xu.row(0), input_img_rows, 1);  // 0-th row (one line containing zero value)

    for (int i = 0; i < input_img_rows; i++)
        yu.at<float>(i,0) = float(i);
    yu = repeat(yu.col(0), 1, input_img_cols);
    
    float fl = 500;
    float rotX = (pitch) * M_PI / 180;         // pitch -90
    float rotY = (yaw) * M_PI / 180;         // yaw -90
    float rotZ = (roll) * M_PI / 180;         // roll -90
    float distX = 0.0;  // (500.0 - 500);
    float distY = 0.0;  // (500.0 - 500);
    float distZ = 0.0;  // (500.0 - 500);
    
    float K_list[9] = {fl, 0, float(imgSize.width /2.),
                        0, fl, float(imgSize.height/2.),
                        0,  0,                 1};
    
    float RX_list[16] = {1, 0, 0, 0,
                        0, cos(rotX), -sin(rotX), 0,
                        0, sin(rotX),  cos(rotX), 0,
                        0,         0,          0, 1};
    float RY_list[16] = {cos(rotY), 0, sin(rotY), 0,
                                 0, 1,         0, 0,
                        -sin(rotY), 0, cos(rotY), 0,
                                 0, 0,         0, 1};
    float RZ_list[16] = {cos(rotZ), -sin(rotZ), 0, 0,
                        sin(rotZ),   cos(rotZ), 0, 0,
                                0,           0, 1, 0,
                                0,           0, 0, 1};
    float T_list[16] = {1, 0, 0, distX,
                        0, 1, 0, distY,
                        0, 0, 1, distZ,
                        0, 0, 0,     1 };
    float last_col_K[3] = {0, 0 ,0};
    float last_row_K_inv[3] = {0, 0, 1};
    
    
    cv::Mat K_3by3  =cv::Mat (3, 3, CV_32F, K_list);
    cv::Mat K =cv::Mat (3,4, CV_32F);
    cv::Mat last_col =cv::Mat(3, 1, CV_32F, last_col_K);
    
    K_3by3.copyTo(K(cv::Rect(0, 0, 3, 3)));
    last_col.copyTo(K(cv::Rect(3, 0, 1, 3)));
    
    
    cv::Mat K_inv1 = K_3by3.inv() * fl;
    cv::Mat K_inv = cv::Mat(4, 3, CV_32F);
    cv::Mat temp = cv::Mat(1, 3, CV_32F, last_row_K_inv);
    
    K_inv1.copyTo(K_inv(cv::Rect(0, 0, 3, 3)));
    temp.copyTo(K_inv(cv::Rect(0, 3, 3, 1 )));
    
    cv::Mat RX = cv::Mat (4, 4, CV_32F, RX_list);
    cv::Mat RY = cv::Mat (4, 4, CV_32F, RY_list);
    cv::Mat RZ = cv::Mat (4, 4, CV_32F, RZ_list);
    cv::Mat T  = cv::Mat (4, 4, CV_32F, T_list);
    
    cv::Mat R = RX * RY * RZ;
    cv::Mat H = K * R * T * K_inv;
    
    
//    cv::Mat H_inv = H.inv();
    
    warpPerspective(src, dst, H, imgSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
//    warpPerspective(yu, mapY, H_inv, imgSize, INTER_LINEAR, BORDER_CONSTANT, 0);
    

}


}  // namespace rectifymap end



void resizeAndCopyImgByRatio(int src_rows, int src_cols, cv::Size projectedImgSize, cv::UMat srcImg, cv::UMat& dstImg) {
    
    cv::UMat tempImg;

    if (srcImg.cols < 1 || srcImg.rows < 1)
    {   cout << "Wrong SrcImgSize! "<< endl;
        return;
    }
    
    if (srcImg.cols > srcImg.rows) {  // if source image width > height

        if (srcImg.cols * 9 != srcImg.rows * 16)  // original image
            tempImg = srcImg(cv::Rect(0, 0.5 *(srcImg.rows - src_rows), src_cols, src_rows));
        else
            tempImg = srcImg;
    }
    
    else { // if source image width <= height

        if (srcImg.cols * 16 != srcImg.rows * 9)  {  // original image
            /* when the user capture with different image ratio such as 9:22 */
            int max_cols = max(srcImg.cols, src_cols);
            int min_cols = min(srcImg.cols, src_cols);
            
            tempImg = srcImg( cv::Rect( (0.5 * (max_cols - min_cols)), 0, min_cols, src_rows));
        }
        else {
            tempImg = srcImg;
        }
    }
    
//    cout << tempImg.size() << endl; // 2250 x4000
    
    resize(tempImg, dstImg, projectedImgSize, 0, 0, cv::INTER_LINEAR);
    
    // Clear memory
    tempImg.release();

}

void check_blended_part_is_matched(cv::Mat src1, cv::Mat src2, int cutRight, int bumper, int bumper_copy, int x_shift) {
    int A_left = cutRight;   // hLenOnCanvas[i] + bumper;
    int A_wid  = src1.cols - (cutRight + bumper);
    int B_left = bumper_copy == 0 ? bumper : bumper_copy;
    int B_wid  = A_wid;
    
//    int x_shift = 15;
    cv::Rect rectA = cv::Rect(A_left,       0, A_wid, src1.rows-x_shift);
    cv::Rect rectB = cv::Rect(B_left, x_shift, B_wid, src1.rows-x_shift);
    
    cv::Mat check1, check2;
    createRotatedImage(src1(rectA), check1, 90);
    createRotatedImage(src2(rectB), check2, 90);
    
    for(int i = 0; i < 100; i++) {
        if (i % 2 == 0)
            cv::imshow("hey", check1);
        else
            cv::imshow("hey", check2);
        
        cv::waitKey(300);
    }

}


namespace smoothing {

void LinearSmoothing(cv::Mat grayLeft, cv::Mat grayRight, int smoothingWd, cv::Mat& output) {
    
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


    if (grayLeft.type() == CV_32F) {
        output = 0.0f;
        
        for (int x=0; x<smoothingWd; x++) {
            float alpha = (float)x/smoothingWd;
            for (int y=0; y<output.rows; y++) {
                output.at<float>(y,x+startCol) = (1.0-alpha) * grayLeft.at<float>(y,x + startCol) + alpha * grayRight.at<float>(y,x + startCol);
            }
        }
    }
    
    else if (grayLeft.type() == CV_8U) {
        output = 0;

        for (int x=0; x<smoothingWd; x++) {
            float alpha = (float)x/smoothingWd;
            for (int y=0; y<output.rows; y++) {
                int valLeft = getPixelUs(grayLeft, x+startCol, y, 0);
                int valRight = getPixelUs(grayRight, x+startCol, y, 0);
                setPixel(output, x + startCol, y, 0, ((1.0-alpha) * valLeft + alpha * valRight) );
            }
        }
    }

    else if (grayLeft.type() == CV_8S) {
        output = 0;


        for (int x = 0; x < smoothingWd; x++) {
            float alpha = (float) x / smoothingWd;
            
            for (int y = 0; y < output.rows; y++) {
                int valLeft = getPixelSs(grayLeft, x + startCol, y, 0);
                int valRight = getPixelSs(grayRight, x + startCol, y, 0);
               
                float pixValue = ((1.0f - alpha) * (float) valLeft + alpha * (float) valRight);
                setPixel(output, x + startCol, y, 0, (int) pixValue);

            }
            
        }

    }

    // Copy Left
    
    grayLeft(cv::Rect(0,0,startCol,grayLeft.rows)).copyTo(output.colRange(0, startCol));

    // Copy right
    grayRight(cv::Rect(endCol,0,(grayRight.cols-endCol),grayLeft.rows)).copyTo(output.colRange(endCol, grayRight.cols));
}



bool NewSmoothing_matImg (cv::Mat imgLeft, cv::Mat imgRight,  int cutRight, int bumper,  float blendStrength_0, float blendStrength_1, cv::Mat & buffer, int diff_cutR) {

    // srcImages - warpAffined Images
    int pyrLevel = 2;
    cv::Rect rectA, rectB;
    cv::Mat gpA, gpB;
    vector<cv::Mat> GaussA, GaussB;
    vector<cv::Mat> LaplacianA, LaplacianB;
    vector<cv::Mat> LapPyr(pyrLevel+1);
    cv::Mat tempUp, tempSub;
    cv::Mat LapUp, LapResult;

    int A_left = cutRight;   // hLenOnCanvas[i] + bumper;
    int A_wid  = imgLeft.cols - (cutRight + bumper);
    int B_left = bumper;
    int B_wid  = A_wid;
    
    if (A_wid <= 0) {
        cout << "(NewSmoothing) Negative A's cutW! Check if Lens is attached." << endl;
        cout << "A_wid = " << imgLeft.cols << "- (" << cutRight << " + " << bumper << ")\n";
        cout << "A_l, A_w, B_l, B_w: " << A_left << ", " << A_wid << ", " << B_left << ", " << B_wid << endl;
        return false;
    }

    rectA = cv::Rect(A_left, 0,  A_wid, imgLeft.rows);
//    rectB = Rect(B_left, 0,  B_wid, imgLeft.rows);

    //================ Deconstruct ================
    GaussA.push_back( imgLeft (rectA).clone() );  // i:1 -> center
    GaussB.push_back( imgRight(cv::Rect(bumper + diff_cutR,0, B_wid, imgLeft.rows)).clone() );  // i:2 -> down
//    GaussB.push_back( imgRight(rectB).clone() );  // i:2 -> down

    imgLeft.release();
    // GaussA[i] contains original image, and down-scale images

    for (int i = 0; i < pyrLevel; i++){
        // generate Gaussian pyramid for A and B
        pyrDown(GaussA[i], gpA);  // pyrDown: scale down  1/2
        pyrDown(GaussB[i], gpB);
        GaussA.push_back(gpA.clone());
        GaussB.push_back(gpB.clone());

        // generate Laplacian Pyramid for A
        pyrUp(gpA, tempUp, GaussA[i].size()); // pyrUp: scale up 2
        subtract(GaussA[i], tempUp, tempSub, cv::noArray(), CV_8SC3);
//        
//        imshow("GaussA[i]", GaussA[i]);
//        waitKey(500);
//        imshow("tempUp", tempUp);
//        moveWindow("tempUp", 200, 0);
//        imshow("tempSub", tempSub);
//        moveWindow("tempSub", 400, 0);
//        waitKey(1000);
        LaplacianA.push_back(tempSub.clone());

        // generate Laplacian Pyramid for B
        pyrUp(gpB, tempUp, GaussB[i].size());
        subtract(GaussB[i], tempUp, tempSub, cv::noArray(), CV_8SC3);
        LaplacianB.push_back(tempSub.clone());
    }
    
    LaplacianA.push_back(gpA.clone());
    LaplacianB.push_back(gpB.clone());


    //------- Placing top to lowest-1 level
    for (int i=pyrLevel-2; i>=0; i--) {
        LapPyr[pyrLevel-i] = cv::Mat(LaplacianA[i].size(), CV_8SC3, cv::Scalar(0,0,0));
        int halfCols = LaplacianA[i].cols/2;
        LaplacianA[i](cv::Rect(0,0, halfCols, LaplacianA[i].rows)).copyTo(LapPyr[pyrLevel-i].colRange(0, halfCols).rowRange(0, LaplacianA[i].rows));
        LaplacianB[i](cv::Rect(halfCols,0, (LaplacianB[i].cols-halfCols),LaplacianB[i].rows)).copyTo(LapPyr[pyrLevel-i].colRange(halfCols, LaplacianB[i].cols).rowRange(0, LaplacianB[i].rows));
    }


    //------- Merging lowest-1 laplacian level
    vector<cv::Mat> LeftSmoothingBGR, RightSmoothingBGR;
    vector<cv::Mat> resultBGR(3);

    split(LaplacianA[pyrLevel-1], LeftSmoothingBGR);
    split(LaplacianB[pyrLevel-1], RightSmoothingBGR);

    for (int ch=0; ch<3; ch++) {  // smooth each bgr channel
        // 0, 1, 2: blue, green, red
        LinearSmoothing(LeftSmoothingBGR[ch], RightSmoothingBGR[ch], (blendStrength_1*LeftSmoothingBGR[ch].rows), resultBGR[ch]);
    }
    merge(resultBGR, LapPyr[1]);

    
    //------- Linear blending for lowest laplacian level
    split(LaplacianA[pyrLevel], LeftSmoothingBGR);
    split(LaplacianB[pyrLevel], RightSmoothingBGR);

    for (int ch=0; ch<3; ch++) {
        LinearSmoothing(LeftSmoothingBGR[ch], RightSmoothingBGR[ch], (blendStrength_0*LeftSmoothingBGR[ch].rows), resultBGR[ch]);
    }
    merge(resultBGR, LapPyr[0]);


    //================ Reconstruct ================//
    pyrUp(LapPyr[0], LapUp, LapPyr[1].size());
    for (int i=1; i<pyrLevel; i++) {
        add(LapPyr[i], LapUp, LapResult, cv::noArray(), CV_8UC3);
        pyrUp(LapResult, LapUp, LapPyr[i+1].size());
        
    }

    add(LapPyr[pyrLevel], LapUp, buffer, cv::noArray(), CV_8UC3);
    

    //Clear memory
    GaussA.clear();     GaussB.clear();
    LaplacianA.clear(); LaplacianB.clear();
    LapPyr.clear();
    
    
    gpA.release();
    gpB.release();
    tempUp.release();
    tempSub.release();
    LapUp.release();
    LapResult.release();
    
    return true;
}

bool NewSmoothing_blendedRoI (cv::Mat imgLeft, cv::Mat imgRight, float blendStrength_0, float blendStrength_1, cv::Mat & buffer ) {

    // srcImages - warpAffined Images
    int pyrLevel = 2;
    cv::Rect rectA, rectB;
    cv::Mat gpA, gpB;
    vector<cv::Mat> GaussA, GaussB;
    vector<cv::Mat> LaplacianA, LaplacianB;
    vector<cv::Mat> LapPyr(pyrLevel+1);
    cv::Mat tempUp, tempSub;
    cv::Mat LapUp, LapResult;

    rectA = cv::Rect(0, 0, imgLeft.cols, imgLeft.rows); //cv::Rect(A_left, 0,  A_wid, imgLeft.rows);
    rectB = cv::Rect(0,0, imgRight.cols, imgLeft.rows); //Rect(B_left, 0,  B_wid, imgLeft.rows);

    //================ Deconstruct ================
    GaussA.push_back( imgLeft(rectA).clone() );  // i:1 -> center
    GaussB.push_back( imgRight(rectB).clone() );  // i:2 -> down
//    GaussB.push_back( imgRight(rectB).clone() );  // i:2 -> down

    imgLeft.release();
    // GaussA[i] contains original image, and down-scale images

    for (int i = 0; i < pyrLevel; i++){
        // generate Gaussian pyramid for A and B
        pyrDown(GaussA[i], gpA);  // pyrDown: scale down  1/2
        pyrDown(GaussB[i], gpB);
        GaussA.push_back(gpA.clone());
        GaussB.push_back(gpB.clone());

        // generate Laplacian Pyramid for A
        pyrUp(gpA, tempUp, GaussA[i].size()); // pyrUp: scale up 2
        subtract(GaussA[i], tempUp, tempSub, cv::noArray(), CV_8SC3);

        LaplacianA.push_back(tempSub.clone());

        // generate Laplacian Pyramid for B
        pyrUp(gpB, tempUp, GaussB[i].size());
        subtract(GaussB[i], tempUp, tempSub, cv::noArray(), CV_8SC3);
        LaplacianB.push_back(tempSub.clone());
    }
    
    LaplacianA.push_back(gpA.clone());
    LaplacianB.push_back(gpB.clone());


    //------- Placing top to lowest-1 level
    for (int i=pyrLevel-2; i>=0; i--) {
        LapPyr[pyrLevel-i] = cv::Mat( LaplacianA[i].size(), CV_8SC3, cv::Scalar(0,0,0) );
        int halfCols = LaplacianA[i].cols/2;
        LaplacianA[i](cv::Rect(0,0, halfCols, LaplacianA[i].rows)).copyTo(LapPyr[pyrLevel-i].colRange(0, halfCols).rowRange(0, LaplacianA[i].rows));
        LaplacianB[i](cv::Rect(halfCols,0, (LaplacianB[i].cols-halfCols),LaplacianB[i].rows)).copyTo(LapPyr[pyrLevel-i].colRange(halfCols, LaplacianB[i].cols).rowRange(0, LaplacianB[i].rows));
    }


    //------- Merging lowest-1 laplacian level
    vector<cv::Mat> LeftSmoothingBGR, RightSmoothingBGR;
    vector<cv::Mat> resultBGR(3);

    split(LaplacianA[pyrLevel-1], LeftSmoothingBGR);
    split(LaplacianB[pyrLevel-1], RightSmoothingBGR);

    for (int ch=0; ch<3; ch++) {  // smooth each bgr channel
        // 0, 1, 2: blue, green, red
        LinearSmoothing(LeftSmoothingBGR[ch], RightSmoothingBGR[ch], (blendStrength_1*LeftSmoothingBGR[ch].rows), resultBGR[ch]);
    }
    merge(resultBGR, LapPyr[1]);

    
    //------- Linear blending for lowest laplacian level
    split(LaplacianA[pyrLevel], LeftSmoothingBGR);
    split(LaplacianB[pyrLevel], RightSmoothingBGR);

    for (int ch=0; ch<3; ch++) {
        LinearSmoothing(LeftSmoothingBGR[ch], RightSmoothingBGR[ch], (blendStrength_0*LeftSmoothingBGR[ch].rows), resultBGR[ch]);
    }
    merge(resultBGR, LapPyr[0]);


    //================ Reconstruct ================//
    pyrUp(LapPyr[0], LapUp, LapPyr[1].size());
    for (int i=1; i<pyrLevel; i++) {
        add(LapPyr[i], LapUp, LapResult, cv::noArray(), CV_8UC3);
        pyrUp(LapResult, LapUp, LapPyr[i+1].size());
        
    }

    add(LapPyr[pyrLevel], LapUp, buffer, cv::noArray(), CV_8UC3);
    

    //Clear memory
    GaussA.clear();     GaussB.clear();
    LaplacianA.clear(); LaplacianB.clear();
    LapPyr.clear();
    
    gpA.release();
    gpB.release();
    tempUp.release();
    tempSub.release();
    LapUp.release();
    LapResult.release();
    
    return true;
}



bool NewSmoothing_vecImg (vector<cv::Mat> srcImages, int cutRight, int bumper, int i, float blendStrength_0, float blendStrength_1, cv::Mat& buffer, int bumper_copy, int offset) {
//    std::string srcDir_tmp = "/Users/3i-21-331/workspace/stitching/";
//    int init_i = i;
    // srcImages - warpAffined Images
    int pyrLevel = 2;
    cv::Rect rectA, rectB;
    cv::Mat gpA, gpB;
    vector<cv::Mat> GaussA, GaussB;
    vector<cv::Mat> LaplacianA, LaplacianB;
    vector<cv::Mat> LapPyr(pyrLevel+1);
    cv::Mat tempUp, tempSub;
    cv::Mat LapUp, LapResult;

    int A_left = cutRight;   // after copy, next part of src[i]
    int A_wid  = srcImages[i].cols - (cutRight + bumper);  // bumper_blend
    int B_left = bumper_copy == 0 ? bumper : bumper_copy ; // bumper_copy
    int B_wid  = A_wid;
    
//    printf("[%2d] cutRight: %d\n", i, cutRight);
    if (A_wid <= 0) {
        printf("(NSmoothing) [%2d] Negative A's cutW! Check if Lens is attached.\n", i);
//        cout << "A_wid = " << srcImages[i].cols << "- (" << cutRight << " + " << bumper << ")\n";
        printf("A_wid = %2d - (%2d + %2d) = %d\n", srcImages[i].cols, cutRight, bumper, A_wid);
        
        return false;
    }

    rectA = cv::Rect(A_left, 0,  A_wid, srcImages[i].rows-offset);
    rectB = cv::Rect(B_left, offset,  B_wid, srcImages[i].rows-offset);


    //================ Deconstruct ================//
    GaussA.push_back( srcImages[ i ](rectA).clone() );  // i:1 -> center
    GaussB.push_back( srcImages[i+1](rectB).clone() );  // i:2 -> down
//    cout << "GaussA[0].channels(): " << GaussA[0].channels() << endl;  // 3
    srcImages[i].release();

    for (int i = 0; i < pyrLevel; i++){
        // generate Gaussian pyramid for A and B
        pyrDown( GaussA[i], gpA );
        pyrDown( GaussB[i], gpB );
        GaussA.push_back( gpA.clone() );
        GaussB.push_back( gpB.clone() );

        // generate Laplacian Pyramid for A
        pyrUp(gpA, tempUp, GaussA[i].size());
        subtract(GaussA[i], tempUp, tempSub, cv::noArray(), CV_8SC3);
        LaplacianA.push_back(tempSub.clone());
        


        // generate Laplacian Pyramid for B
        pyrUp(gpB, tempUp, GaussB[i].size());
        subtract(GaussB[i], tempUp, tempSub, cv::noArray(), CV_8SC3);
        LaplacianB.push_back(tempSub.clone());


    }

    LaplacianA.push_back(gpA.clone());
    LaplacianB.push_back(gpB.clone());

//    printf("LaplacianA[0].cols/2(%d) > 0! ", LaplacianA[0].cols / 2);
    //------- Placing top to lowest-1 level
    for (int i=pyrLevel-2; i>=0; i--) {
        LapPyr[pyrLevel-i] = cv::Mat(LaplacianA[i].size(), CV_8SC3, cv::Scalar(0,0,0));

        int halfCols = LaplacianA[i].cols/2;
        assert(halfCols > 0);
        
        
        LaplacianA[i](cv::Rect(0,0, halfCols, LaplacianA[i].rows)).copyTo(LapPyr[pyrLevel-i].colRange(0, halfCols).rowRange(0, LaplacianA[i].rows));
        LaplacianB[i](cv::Rect(halfCols,0, (LaplacianB[i].cols-halfCols),LaplacianB[i].rows)).copyTo(LapPyr[pyrLevel-i].colRange(halfCols, LaplacianB[i].cols).rowRange(0, LaplacianB[i].rows));
    }

    //------- Merging lowest-1 laplacian level
    std::vector<cv::Mat> LeftSmoothingBGR, RightSmoothingBGR;
//    vector<Mat> LsmoothMask_BGR(3), RsmoothMask_BGR(3);
    std::vector<cv::Mat> resultBGR(3);

    split(LaplacianA[pyrLevel-1], LeftSmoothingBGR);
    split(LaplacianB[pyrLevel-1], RightSmoothingBGR);

    for (int ch=0; ch<3; ch++) {  // smooth each bgr channel
        LinearSmoothing(LeftSmoothingBGR[ch], RightSmoothingBGR[ch], (blendStrength_1*LeftSmoothingBGR[ch].rows), resultBGR[ch]);
    }
    merge(resultBGR, LapPyr[1]);

    
    //------- Linear blending for lowest laplacian level
    split(LaplacianA[pyrLevel], LeftSmoothingBGR);
    split(LaplacianB[pyrLevel], RightSmoothingBGR);

    for (int ch=0; ch<3; ch++) {
        LinearSmoothing(LeftSmoothingBGR[ch], RightSmoothingBGR[ch], (blendStrength_0*LeftSmoothingBGR[ch].rows), resultBGR[ch]);
    }
    merge(resultBGR, LapPyr[0]);


    //================ Reconstruct ================//
    pyrUp(LapPyr[0], LapUp, LapPyr[1].size());
    for (int i=1; i<pyrLevel; i++) {
        add(LapPyr[i], LapUp, LapResult, cv::noArray(), CV_8UC3);
        pyrUp(LapResult, LapUp, LapPyr[i+1].size());
    }
    add(LapPyr[pyrLevel], LapUp, buffer, cv::noArray(), CV_8UC3);
    

    
    // Clear memory
    GaussA.clear();         GaussB.clear();
    LaplacianA.clear();     LaplacianB.clear();
    LapPyr.clear();
    
    
    gpA.release();          gpB.release();
    tempUp.release();       tempSub.release();
    LapUp.release();        LapResult.release();
    
    return true;
}

}  // namespace smoothing



void blend_cropped_point(int cropStart1, int cropWidth1, int cropStart2, int cropWidth2, int south, int north,  cv::Size _finalImgSize, cv::Mat canvas, cv::Mat& croppedImg ) {

    //--- Copy - 1
    int bumper = canvas.cols * 0.0033;  // 30;
    int tempW = canvas.cols * 0.01; // 100;// for now, constant.
    cv::Mat imgLeft  = canvas(cv::Rect(cropStart1, north, cropWidth1 + tempW, south - north));
    cv::Mat imgRight = canvas(cv::Rect(cropStart2 - tempW-bumper, north, cropWidth2 + tempW +bumper, south - north));
    
    int cutRight = imgLeft.cols - 2 * tempW;
    canvas(cv::Rect(cropStart1, north, cropWidth1-tempW, south - north)) // cropWidth1 - bumper
        .copyTo(croppedImg.colRange(0,  cropWidth1-tempW)); // cropWidth1 - bumper

    //--- Blending - 2
    cv::Mat blended_buf;
    
    smoothing::NewSmoothing_matImg(imgLeft, imgRight, cutRight, bumper,  0.1, 0.015, blended_buf);
    int blended_start = cropWidth1 -tempW;
    blended_buf.copyTo(croppedImg.colRange(blended_start, blended_start + blended_buf.cols));


    //--- Copy - 3
    canvas(cv::Rect(cropStart2-tempW + blended_buf.cols, north, cropWidth2 - tempW + bumper, south - north))
        .copyTo(croppedImg.colRange(blended_start + blended_buf.cols, croppedImg.cols));
}


void stitch_cropped_only(int cropStart1, int cropWidth1, int cropStart2, int cropWidth2, int south, int north, cv::Mat canvas, cv::Mat& croppedImg) {

    canvas(cv::Rect(cropStart1, north, cropWidth1, south - north)).copyTo(croppedImg.colRange(0, cropWidth1));
    canvas(cv::Rect(cropStart2, north, cropWidth2, south - north)).copyTo(croppedImg.colRange(cropWidth1, croppedImg.cols));
    
}

namespace ftmask {

void createFeatureMasks3Pair(int i, cv::Size ftMapSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left) {
    int topY = round((float)i / 3 * ftMapSize.height);
    int btmY = round((float)(i+1) / 3 * ftMapSize.height);

    float sect_num = 3.;
    
    feature_mask_right = cv::UMat::zeros(ftMapSize.height, ftMapSize.width, CV_8U);
    feature_mask_right( cv::Rect (round(ftMapSize.width/sect_num) ,topY , round((sect_num-1)* ftMapSize.width/ sect_num ), btmY- topY) ) = 1;

    feature_mask_left = cv::UMat::zeros(ftMapSize.height, ftMapSize.width, CV_8U);
    feature_mask_left( cv::Rect(0, topY, round( (sect_num-1) * ftMapSize.width/sect_num), btmY - topY)) = 1;
}


void createFeatureLeftRightMasks(int features_pad, cv::Size ftMapSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left )
{
    int featurePointY_min = features_pad * ftMapSize.height;
    int featurePointY_max = (1.0 - features_pad) * ftMapSize.height;
    float sect_num = 2.;
    
    feature_mask_right = cv::UMat::zeros(ftMapSize.height, ftMapSize.width, CV_8U);
    feature_mask_right( cv::Rect (round(ftMapSize.width/sect_num) ,featurePointY_min , ((int)sect_num-1)* ftMapSize.width/ (int)sect_num , featurePointY_max-featurePointY_min) ) = 1;

    feature_mask_left = cv::UMat::zeros(ftMapSize.height, ftMapSize.width, CV_8U);
    feature_mask_left( cv::Rect(0, featurePointY_min, round( ((int)sect_num-1) * ftMapSize.width/(int)sect_num), featurePointY_max-featurePointY_min)) = 1;

}


void createFeatureLeftRightMasks(int features_pad, cv::Size rightMapSize, cv::Size leftMapSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left) {

    //---Feature Finding and cv::Matching Mask for left or right
    int fpY_min_right = features_pad * rightMapSize.height;
    int fpY_max_right = (1.0 - features_pad) * rightMapSize.height;
    int fpY_min_left = features_pad * leftMapSize.height;
    int fpY_max_left = (1.0 - features_pad) * leftMapSize.height;

    // Set 1 to right part of half-sized image
    feature_mask_right = cv::UMat::zeros(rightMapSize.height, rightMapSize.width, CV_8U);
    feature_mask_right(cv::Rect ( round(1 * rightMapSize.width /3.), fpY_min_right, round(2 * rightMapSize.width /3.), fpY_max_right- fpY_min_right )) = 1;

    // Set 1 to left part of half-sized image
    feature_mask_left = cv::UMat::zeros(leftMapSize.height, leftMapSize.width, CV_8U);
    feature_mask_left(cv::Rect( 0, fpY_min_left, round(2 * leftMapSize.width /3.), fpY_max_left-fpY_min_left )) = 1;

}

void createFeatureMask(cv::Size imgSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left) {
    //---Feature Finding and Matching Mask
    feature_mask_right = cv::UMat::ones(imgSize.height, imgSize.width, CV_8U);
    
    feature_mask_left = cv::UMat::ones(imgSize.height, imgSize.width, CV_8U);
}


void createFeatureMask_halfOnes(cv::Size imgSize, cv::UMat &feature_mask_right, cv::UMat &feature_mask_left) {
    
    feature_mask_right = cv::UMat::zeros(imgSize.height, imgSize.width, CV_8U);
    feature_mask_right(cv::Rect(round(imgSize.width/2),0,round(imgSize.width/2), imgSize.height)) = 1;
    
    feature_mask_left = cv::UMat::zeros(imgSize.height, imgSize.width, CV_8U);
    feature_mask_left(cv::Rect(0, 0, round(imgSize.width/2), imgSize.height)) = 1;

}


} // namespace ftmask end


template <typename Arg, typename... Args>
void print_vector(const Arg& arg, const Args&... args) {
  std::cout << arg;
  ((std::cout << ", " << args), ...);
  std::cout << std::endl;
}

/* Independent parallel function */
void copy_center(std::vector<cv::UMat> _vecImgs, std::vector<cv::Mat> & _affined_Imgs) {
    _vecImgs[1].copyTo(_affined_Imgs[1]);
}

void estimate_and_warp(int i, vector<cv::UMat> _vecImgs, std::vector<int> _YawOnCanvas,  std::vector<std::vector<cv::Point2f>> _vecGoodKp1, std::vector<std::vector<cv::Point2f>> _vecGoodKp2,  int _upDownW, int _canH, std::vector<cv::Mat> & _affined_Imgs) {
    
    int x_shift = _YawOnCanvas[i];
    cv::Mat affine_mat;
    affine_mat = (i == 0 ? cv::estimateAffine2D( _vecGoodKp1[i], _vecGoodKp2[i]) : cv::estimateAffine2D(_vecGoodKp2[i], _vecGoodKp1[i]) );
    
    affine_mat.at<double>(0, 2) += (i == 0 ? x_shift: -x_shift);

    warpAffine(_vecImgs[i*2], _affined_Imgs[i*2], affine_mat, cv::Size(_upDownW, _canH), cv::INTER_LINEAR, cv::BORDER_CONSTANT, bgrColor);
    
    affine_mat.release();
    
}

void parallel_estimate_and_warp(std::vector<cv::UMat> _vecImgs, std::vector<int> _YawOnCanvas, std::vector<std::vector<cv::Point2f>> _vecGoodKp1, std::vector<std::vector<cv::Point2f>> _vecGoodKp2, int _upDownW, int _canH, std::vector<cv::Mat> & _affined_Imgs) {
    
    const int n_thread = 3;  // fixed!!!
    vector<future<void>> futures;
    futures.resize(n_thread);
    
    for(int i = 0; i < n_thread-1; i++)
        futures[i] = async(launch::async, estimate_and_warp, i, _vecImgs, _YawOnCanvas, _vecGoodKp1, _vecGoodKp2, _upDownW, _canH,  ref(_affined_Imgs));
    
    futures[n_thread-1] = async(launch::async, copy_center, _vecImgs, ref(_affined_Imgs));
    
    for (int i = 0; i < n_thread; i++)
        futures[i].get();
    
    futures.clear();
    
}

// Color the black empty spot on top and bottom of the panorama image
void BeautifyPoles(cv::Mat cropped, cv::Mat& result, int top, int bottom)
{
//    printf("BeautifyPoles start\n");
    cv::Mat colorLine, small_pole;
    cv::Mat southPole, northPole;
    int smallWd = 300;
    float gama;
    cv::Vec3b pixel, pixel_avg;
    long sum0, sum1, sum2;
    int n, p;
    int start, end;

    int top_small = top * smallWd / cropped.cols;  // padding_top * (smallWd / cropped Width)
    int bot_small = bottom * smallWd / cropped.cols; // minus

    cv::Rect northROI(0, 0, cropped.cols, 1);
    cv::Rect southROI(0, cropped.rows - 1, cropped.cols, 1);

    //---------- South
    resize(cropped(southROI), colorLine, cv::Size(smallWd, 1)); // cropped(Size) resize to colorLine
    blur(colorLine, colorLine, cv::Size(15, 1), cv::Point(-1, -1), cv::BORDER_REPLICATE); // blur the border of south

    small_pole = cv::Mat(bot_small, smallWd, CV_8UC3);
    colorLine.copyTo(small_pole.colRange(0, smallWd).rowRange(0, 1));

    for (int j = 1; j < bot_small; j++) {
        gama = float(j) / bot_small;
        n = ceil(gama*gama * smallWd * 0.3);

        for (int i = 0; i < smallWd; i++) { //scan x-wise

            start = i - floor(0.5*n);
            end = i + ceil(0.5*n);

            sum0 = 0;
            sum1 = 0;
            sum2 = 0;

            for (int k = start; k < end; k++) {
                p = (k + smallWd) % smallWd;

                pixel = small_pole.at<cv::Vec3b>(j - 1, p);
                sum0 += pixel[0];
                sum1 += pixel[1];
                sum2 += pixel[2];
            }

            pixel_avg[0] = round((double)sum0 / n);
            pixel_avg[1] = round((double)sum1 / n);
            pixel_avg[2] = round((double)sum2 / n);

            small_pole.at<cv::Vec3b>(j, i) = pixel_avg;

        }
    }

    resize(small_pole, southPole, cv::Size(cropped.cols, bottom));

    copyMakeBorder(southPole, southPole, 0, 0, 25, 25, cv::BORDER_WRAP);
    blur(southPole, southPole, cv::Size(51, 51), cv::Point(-1, -1), cv::BORDER_REPLICATE);
    // southPole : 6057 x 514
    // Result : 6007 x 3004
    // cropped: 6007 x 1978

    southPole(cv::Rect(25, 0, cropped.cols, bottom))  // 25,0,6007, 514
        .copyTo(result.colRange(0, result.cols).rowRange(cropped.rows + top, cropped.rows + top + bottom)); // W: 0~6707, H: 1978+512, 1978+512+514
    //---------- North
    resize(cropped(northROI), colorLine, cv::Size(smallWd, 1));
    blur(colorLine, colorLine, cv::Size(15, 1), cv::Point(-1, -1), cv::BORDER_REPLICATE);

    small_pole = cv::Mat(top_small, smallWd, CV_8UC3);
    small_pole = cv::Scalar(0, 0, 0);
    colorLine.copyTo(small_pole.colRange(0, smallWd).rowRange(top_small - 1, top_small));

    for (int j = 1; j < top_small; j++) {
        gama = float(j) / top_small;
        n = ceil(gama*gama * smallWd * 0.3);

        for (int i = 0; i < smallWd; i++) { //scan x-wise

            start = i - floor(0.5*n);
            end = i + ceil(0.5*n);

            sum0 = 0;
            sum1 = 0;
            sum2 = 0;

            for (int k = start; k < end; k++) {
                p = (k + smallWd) % smallWd;

                pixel = small_pole.at<cv::Vec3b>(top_small - j, p);
                sum0 += pixel[0];
                sum1 += pixel[1];
                sum2 += pixel[2];
            }

            pixel_avg[0] = round((double)sum0 / n);
            pixel_avg[1] = round((double)sum1 / n);
            pixel_avg[2] = round((double)sum2 / n);

            small_pole.at<cv::Vec3b>(top_small - j - 1, i) = pixel_avg;
        }
    }

    resize(small_pole, northPole, cv::Size(cropped.cols, top));
    copyMakeBorder(northPole, northPole, 0, 0, 25, 25, cv::BORDER_WRAP);
    blur(northPole, northPole, cv::Size(51, 51), cv::Point(-1, -1), cv::BORDER_REPLICATE);

    northPole(cv::Rect(25, 0, cropped.cols, top)).copyTo(result.colRange(0, result.cols).rowRange(0, top));

    //---------- Copy Middle
    cropped.copyTo(result.colRange(0, result.cols).rowRange(top, top + cropped.rows));
    
    assert(result.cols > 1);

    //clear memory
    colorLine.release();
    small_pole.release();
    southPole.release();
    northPole.release();
}

void createRotatedImage(cv::Mat src, cv::Mat &dst, int theta)
{
    theta = -theta;
    cv::Mat frame, frameRotated;
    
    int diagonal = (int)sqrt(src.cols*src.cols+src.rows*src.rows);
    int newWidth =  diagonal;
    int newHeight = diagonal;

    int offsetX = (newWidth - src.cols) / 2;
    int offsetY = (newHeight - src.rows) / 2;
    cv::Mat targetMat(newWidth, newHeight, src.type());
    cv::Point2f src_center(targetMat.cols/2.0F, targetMat.rows/2.0F);

    src.copyTo(frame);

    frame.copyTo(targetMat.rowRange(offsetY, offsetY + frame.rows).colRange(offsetX, offsetX + frame.cols));
    cv::Mat rot_mat = getRotationMatrix2D(src_center, theta, 1.0);
    warpAffine(targetMat, frameRotated, rot_mat, targetMat.size());
    
     //Calculate bounding rect and for exact image
     //Reference:- https://stackoverflow.com/questions/19830477/find-the-bounding-rectangle-of-rotated-rectangle/19830964?noredirect=1#19830964
    
    cv::Rect bound_Rect(frame.cols,frame.rows,0,0);

    int x1 = offsetX;
    int x2 = offsetX+frame.cols;
    int x3 = offsetX;
    int x4 = offsetX+frame.cols;

    int y1 = offsetY;
    int y2 = offsetY;
    int y3 = offsetY+frame.rows;
    int y4 = offsetY+frame.rows;

    cv::Mat co_Ordinate = (cv::Mat_<double>(3,4) << x1, x2, x3, x4,
                                            y1, y2, y3, y4,
                                            1,  1,  1,  1 );
    cv::Mat RotCo_Ordinate = rot_mat * co_Ordinate;

    for (int i=0;i<4;i++){
       if (RotCo_Ordinate.at<double>(0,i)<bound_Rect.x)
         bound_Rect.x=(int)RotCo_Ordinate.at<double>(0,i); //access smallest
       if (RotCo_Ordinate.at<double>(1,i)<bound_Rect.y)
        bound_Rect.y=RotCo_Ordinate.at<double>(1,i); //access smallest y
     }

     for (int i=0;i<4;i++){
       if (RotCo_Ordinate.at<double>(0,i)>bound_Rect.width)
         bound_Rect.width=(int)RotCo_Ordinate.at<double>(0,i); //access largest x
       if (RotCo_Ordinate.at<double>(1,i)>bound_Rect.height)
        bound_Rect.height=RotCo_Ordinate.at<double>(1,i); //access largest y
     }

    bound_Rect.width=bound_Rect.width-bound_Rect.x;
    bound_Rect.height=bound_Rect.height-bound_Rect.y;

    cv::Mat cropedResult;
    cv::Mat ROI = frameRotated(bound_Rect);
    ROI.copyTo(dst);
    
    frame.release();
    frameRotated.release();
    targetMat.release();
    rot_mat.release();
    co_Ordinate.release();
    RotCo_Ordinate.release();
    cropedResult.release();
    ROI.release();
}


void changePoleCutThresh(float rotAngle, float& poleCutThresh_0, float& poleCutThresh_1) {
    if (abs(rotAngle) > 2.5) {  // 2.5 is constant.
        printf("Since roll angle is bigger than normal, we\'ll change poleCutThresh _0, _1 value from %.3f, %.3f", poleCutThresh_0, poleCutThresh_1 );
        
        poleCutThresh_0 = poleCutThresh_1 = 0.02 + 0.0005 * rotAngle;
        printf(" to %.5f, %.5f\n", poleCutThresh_0, poleCutThresh_1 );
        
    }
}

bool transposeAndFlipImg(cv::Mat srcImage, float inputAR, int s_cols, int s_rows,  cv::Mat& outImg) {
    cv::Mat fliped;
    if (srcImage.empty())
        return false;
    
    if (srcImage.cols > srcImage.rows) {
        cv::transpose(srcImage, fliped);
        cv::flip(fliped, fliped, 1);
        
    } else {
        fliped = srcImage;
    }
    
    outImg = fliped;
    
    return true;
}


bool convertImgChannel(cv::Mat srcImage, float inputAR, int s_cols, int s_rows,  cv::Mat& outImg) {
    cv::Mat flipRGB;
    int decoded_w, decoded_h;
    
    if (srcImage.empty())
        return false;
    
    if (srcImage.cols > srcImage.rows) {
        cv::transpose(srcImage, flipRGB);
        cv::flip(flipRGB, flipRGB, 1);
        
    } else {
        // this code runs
        flipRGB = srcImage;
    }
    cvtColor(flipRGB, flipRGB, cv::COLOR_RGBA2BGR/*RGB*/);  // CV_RGBA2RGB
    
    decoded_w = flipRGB.cols;
    decoded_h = flipRGB.rows;

    if (decoded_h != decoded_w * inputAR) {
        // this code runs
        outImg = flipRGB(cv::Rect((0.5*(decoded_w-s_cols)),0,s_cols,s_rows));
    } else {
        outImg = flipRGB;
    }

//    outImg = flipRGB;
    flipRGB.release();
    
    
    return true;
}

void copyBufferToCanvas(uint8_t* pixelPtr, int cn, int startX, int endX, int offsetX, int widY, int bufferH,  cv::Mat& canvas, int x_shift ) {
    for (int y = 0; y < bufferH-x_shift; y++) {
        for (int x = startX; x < endX; x++) {  // cutLeft ~ cutRight
            canvas.at<cv::Vec3b>(x+offsetX, canvas.cols-y-1)[0] = pixelPtr[(y+x_shift) * widY * cn + x * cn + 0];  // -cutLeft+canvasLeft, cutWidth
            canvas.at<cv::Vec3b>(x+offsetX, canvas.cols-y-1)[1] = pixelPtr[(y+x_shift) * widY * cn + x * cn + 1];
            canvas.at<cv::Vec3b>(x+offsetX, canvas.cols-y-1)[2] = pixelPtr[(y+x_shift) * widY * cn + x * cn + 2];
        }
    }

}

void setCropPoint(int num_of_images, int Ltype, float poleCutThresh_0, float poleCutThresh_1, int RotatedYawTotal, int finalCanvas_height, float phoneTilt,  cv::Size finalSize, std::vector<int> YawOnCanvas, cv::Mat canvas, cv::Mat &croppedImage, cv::Mat &outputPano) {
    
    int lastPos = RotatedYawTotal - YawOnCanvas[num_of_images-1] + finalSize.width/2;
    int cropStart1 = lastPos - RotatedYawTotal/2;
    int cropWidth1 = canvas.cols - finalSize.width/2 - cropStart1;
    int cropWidth2 = RotatedYawTotal - cropWidth1;
    int cropStart2 = finalSize.width/2;

    printf("RotatedYawTotal, finalH: %d, %d\n", RotatedYawTotal, finalSize.height);
    
    if(RotatedYawTotal < 2*finalSize.height){
        croppedImage.create(finalSize.height, RotatedYawTotal, CV_8UC3);
        canvasCopyToCroppedImg(0, cropStart1, cropStart2, cropWidth1, cropWidth2, finalSize.height, canvas, croppedImage);
        resize(croppedImage, outputPano, cv::Size(2 * finalCanvas_height, finalCanvas_height), 0, 0, cv::INTER_AREA);
    }
    
    else if(RotatedYawTotal > 2*finalSize.height){ //stitched image is too wide --> add fill to the hole
        int south, north;
        find_north_and_south_cutoff(Ltype, poleCutThresh_0, poleCutThresh_1, finalSize, YawOnCanvas, canvas, south, north);
        printf("south, north: %d, %d\n", south, north);
        
        croppedImage.create(south-north, RotatedYawTotal, CV_8UC3);
        
        if (0)
            canvasCopyToCroppedImg(north, cropStart1, cropStart2, cropWidth1, cropWidth2, south-north, canvas, croppedImage);
        else
            blend_cropped_point(cropStart1, cropWidth1, cropStart2, cropWidth2, south, north, finalSize, canvas, croppedImage);
        
        printf("canvas copied to croppedImg\n");
        
        int canvas_shiftY = phoneTilt / 360.0 * croppedImage.cols;
        int padding_top = round((0.5 * croppedImage.cols - canvas.rows) / 2) - canvas_shiftY + north;
        int padding_bottom = round( 0.5 * croppedImage.cols - croppedImage.rows - padding_top);

        //Assert
        assert(padding_top > 0);    assert(padding_bottom > 0);
        outputPano = cv::Mat(padding_top+padding_bottom+croppedImage.rows, croppedImage.cols, CV_8UC3);
        printf("padding top, btm: %d, %d\n", padding_top, padding_bottom);
        BeautifyPoles(croppedImage, outputPano, padding_top, padding_bottom);
        
    }
    printf("setCropPoint done\n");
}


void setCropPoint( float poleCutThresh_0, float poleCutThresh_1, float pitch,
    int numImages, int RotatedXTotal, cv::Size _finalImgSize, std::vector<int> _YawOnCanvas, cv::Mat canvas,  cv::Mat &croppedImg, int &padding_top, int& padding_bottom) {

    const cv::Scalar bgrColor = cv::Scalar(168, 168, 168);
//    printf("RotatedYawTotal, finalH: %d, %d\n", RotatedXTotal, _finalImgSize.height);
    
    // When stitched image (width)is too wide, add fill to the hole
    if (RotatedXTotal > 2 * _finalImgSize.height) {
        // Find North South cutoff
        int north, south;
        int this_north, this_south;
        int colSearch = (_finalImgSize.width + _YawOnCanvas[0]) / 2; // The first seam position
        
        Find_NSPoleCut(canvas, colSearch-1, north, colSearch+1, south, poleCutThresh_0, bgrColor);  // 0.01 -> 0.003
        // normal lens -> 0.003,  wide lens-> 0.006 , (25) ->  0.003  0.006
        

        colSearch += _YawOnCanvas[1] + _YawOnCanvas[2] + _YawOnCanvas[3];

        Find_NSPoleCut(canvas, colSearch + 1, this_north, colSearch - 1, this_south, poleCutThresh_1, bgrColor);
        // normal lens -> 0.001 (close...),  wide lens-> 0.006  (25) -> 0.003   0.003
 

        // set bigger value of north
        if (this_north > north)     north = this_north;
        // set smaller value of south
        if (this_south < south)     south = this_south;

        // Getting position of last shot then Cropping
        croppedImg = cv::Mat::zeros(south - north, RotatedXTotal, CV_8UC3);  // height , width, channel

        //---- Crop the middle point of the stitched image
        int lastPos = RotatedXTotal - _YawOnCanvas[numImages - 1] + round(_finalImgSize.width / 2.);
        int cropStart1 = lastPos - RotatedXTotal / 2;
        int cropWidth1 = canvas.cols - round(_finalImgSize.width / 2.) - cropStart1;
        int cropStart2 = round(_finalImgSize.width / 2.);
        int cropWidth2 = RotatedXTotal - cropWidth1;

        bool do_blend = true;
        if (do_blend)
            blend_cropped_point(cropStart1, cropWidth1, cropStart2, cropWidth2, south, north, _finalImgSize, canvas, croppedImg);
        else
            stitch_cropped_only(cropStart1, cropWidth1, cropStart2, cropWidth2, south, north, canvas, croppedImg);

        
        int canvas_shiftY =  pitch/360.0 * croppedImg.cols;
        padding_top = round((0.5 * croppedImg.cols - canvas.rows) / 2) - canvas_shiftY + north;
        padding_bottom = round(0.5 * croppedImg.cols - croppedImg.rows - padding_top);

        // Assert
        if (padding_top < 0 || padding_bottom < 0) {
            cout << "!!!Negative padding exists!!!" << endl;
            return;
        }
//        printf("padding top, btm: %d, %d\n", padding_top, padding_bottom);

    }

    else {
        cout << "RotatedXTotal is smaller than 2 times of finalImgSize's Height.\n Please debug." << endl;
        return;
    }

}


void canvasCopyToCroppedImg(int top,int left1, int left2,  int wid1, int wid2, int height, cv::Mat canvas, cv::Mat& croppedImage ) {
    printf("t, s1,s2, w1, w2: %d, %d, %d, %d, %d\n", top, left1, left2, wid1, wid2);
    
    canvas(cv::Rect(left1, top, wid1, height)).copyTo(croppedImage.colRange(0, wid1));
    canvas(cv::Rect(left2, top, wid2, height)).copyTo(croppedImage.colRange(wid1, croppedImage.cols));
}

void find_north_and_south_cutoff(int Ltype, float poleCutThresh_0, float poleCutThresh_1, cv::Size finalSize, std::vector<int> YawOnCanvas, cv::Mat canvas, int& south, int& north) {

//    if (Ltype == R_NOLENS) {
//        int colSearch = (finalSize.width + YawOnCanvas[0]) / 2; // first seam position
//
//        Find_NSPoleCut(canvas, colSearch, north, colSearch, south, 0.006, bgrColor);
//    }
//    else {
    
        int this_north, this_south;
        int colSearch = (finalSize.width + YawOnCanvas[0]) / 2; // The first seam position
        Find_NSPoleCut(canvas, colSearch-1, this_north, colSearch+1, this_south, 0.006, bgrColor);
        north = this_north;
        south = this_south;

        colSearch += YawOnCanvas[1] + YawOnCanvas[2] + YawOnCanvas[3];
        Find_NSPoleCut(canvas, colSearch+1, this_north, colSearch-1, this_south, 0.003, bgrColor);
    
    
    
        if (this_north > north)     north = this_north;  // < to >
        
        if (this_south < south)     south = this_south; //  > to <
        
//    }
    
}

void Find_NSPoleCut(cv::Mat img, int NcolIdx, int &Npole, int ScolIdx, int &Spole, float threshold, cv::Scalar bgColor)
{
    // Image size: 6707 x 2062
    cv::Vec3b thisPixel;
    cv::Vec3b scanPx;
    int B, G, R;
    int northCut = 0;
    int southCut = img.rows;  // image width

    int start_point = 1;
    int count = 0;
    int maxErrorNum = 0; // When warping the up,

    scanPx = img.at<cv::Vec3b>(start_point, NcolIdx);
    B = (int)scanPx[0];
    G = (int)scanPx[1];
    R = (int)scanPx[2];
    
    
    int color_thresh = 2;
//#pragma omp parallel for num_threads(2)
    
    for (int y = start_point; y < 0.4*img.rows; y++) { // scan image vertically -- y: 0~ 0.4*H
        thisPixel = img.at<cv::Vec3b>(y, NcolIdx);
        int blue = (int)thisPixel.val[0];
        int green = (int)thisPixel.val[1];
        int red = (int)thisPixel.val[2];


        if (abs(blue - B) >= color_thresh|| abs(green - G) >= color_thresh || abs(red - R) >= color_thresh) {
            count += 1;

            northCut = y;
            if (count > maxErrorNum)
                break;
        }
        else northCut = y;

    }

    scanPx = img.at<cv::Vec3b>(img.rows - (1 + start_point), ScolIdx);
    B = (int)scanPx[0];
    G = (int)scanPx[1];
    R = (int)scanPx[2];

    count = 0;
    
//#pragma omp parallel for num_threads(2)
    for (int y = img.rows - (1 + start_point); y > 0.6 * img.rows; y--) {
        thisPixel = img.at<cv::Vec3b>(y, ScolIdx);
        int blue = (int)thisPixel.val[0];
        int green = (int)thisPixel.val[1];
        int red = (int)thisPixel.val[2];

        if (abs(blue - B) >= color_thresh|| abs(green - G) >= color_thresh || abs(red - R) >= color_thresh) {
            count += 1;

            southCut = y;
            if (count > maxErrorNum)
                break;
        }
        else southCut = y;
    }
//    cout << endl;

    Npole = northCut + threshold * img.rows;
    Spole = southCut - threshold * img.rows;

}



void get_input_image_width_and_height(string initImgPath, int& width, int& height) {
    cv::Mat initImg = cv::imread(initImgPath);
    assert(initImg.cols > 1);
    width = initImg.cols;
    height = initImg.rows;
    initImg.release();
}

