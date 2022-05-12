//
//  line_detect.cpp
//  CppPractive
//
//  Created by Song on 2022/04/28.
//

#include "line_detect.hpp"



using namespace std;
using namespace cv;

void getLine(double x1, double y1, double x2, double y2, double &a, double &b, double &c)
{
       // (x- p1X) / (p2X - p1X) = (y - p1Y) / (p2Y - p1Y)
       a = y1 - y2; // Note: this was incorrectly "y2 - y1" in the original answer
       b = x2 - x1;
       c = x1 * y2 - x2 * y1;
}

double dist(double pct1X, double pct1Y, double pct2X, double pct2Y, double pct3X, double pct3Y)
{
     double a, b, c;
     getLine(pct2X, pct2Y, pct3X, pct3Y, a, b, c);
     return abs(a * pct1X + b * pct1Y + c) / sqrt(a * a + b * b);
}




// LSD line segment detection
void LineDetect( cv::Mat image, double thLength, std::vector<std::vector<double> > &lines )
{
    cv::Mat grayImage;
    if ( image.channels() == 1 )
        grayImage = image;
    else
        cv::cvtColor(image, grayImage, COLOR_BGR2GRAY);

    image_double imageLSD = new_image_double( grayImage.cols, grayImage.rows );
    unsigned char* im_src = (unsigned char*) grayImage.data;

    int xsize = grayImage.cols;
    int ysize = grayImage.rows;
    for ( int y = 0; y < ysize; ++y )
    {
        for ( int x = 0; x < xsize; ++x )
        {
            imageLSD->data[y * xsize + x] = im_src[y * xsize + x];
        }
    }

    ntuple_list linesLSD = lsd( imageLSD );
    free_image_double( imageLSD );

    int nLines = linesLSD->size;
    int dim = linesLSD->dim;
    std::vector<double> lineTemp( 4 );
    for ( int i = 0; i < nLines; ++i )
    {
        double x1 = linesLSD->values[i * dim + 0];
        double y1 = linesLSD->values[i * dim + 1];
        double x2 = linesLSD->values[i * dim + 2];
        double y2 = linesLSD->values[i * dim + 3];

        double l = sqrt( ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 ) );
        if ( l > thLength )
        {
            lineTemp[0] = x1;
            lineTemp[1] = y1;
            lineTemp[2] = x2;
            lineTemp[3] = y2;

            lines.push_back( lineTemp );
        }
    }

    free_ntuple_list(linesLSD);
}

void drawClusters( cv::Mat &img, std::vector<std::vector<double> > &lines, std::vector<std::vector<int> > &clusters )
{
//    int cols = img.cols;
//    int rows = img.rows;

    //draw lines
    std::vector<cv::Scalar> lineColors( 3 );
    lineColors[0] = cv::Scalar( 0, 0, 255 );    // X is Red.
    lineColors[1] = cv::Scalar( 0, 255, 0 );    // Y is Green.
    lineColors[2] = cv::Scalar( 255, 0, 0 );    // Z is Blue.

    for ( int i=0; i<lines.size(); ++i )
    {
        int idx = i;
        cv::Point pt_s = cv::Point( lines[idx][0], lines[idx][1]);
        cv::Point pt_e = cv::Point( lines[idx][2], lines[idx][3]);
//        cv::Point pt_m = ( pt_s + pt_e ) * 0.5;

        cv::line( img, pt_s, pt_e, cv::Scalar(0,0,0), 2, LINE_AA );
    }

    for ( int i = 0; i < clusters.size(); ++i )
    {
        for ( int j = 0; j < clusters[i].size(); ++j )
        {
            int idx = clusters[i][j];

            cv::Point pt_s = cv::Point( lines[idx][0], lines[idx][1] );
            cv::Point pt_e = cv::Point( lines[idx][2], lines[idx][3] );
//            cv::Point pt_m = ( pt_s + pt_e ) * 0.5;

            cv::line( img, pt_s, pt_e, lineColors[i], 2, LINE_AA );
        }
    }
}


void demo_VPDetection( void ) {
    Mat image, resized, output;
    
    string dataset_name = "correct_capture_1";  // roll_negative, pitch_positive, correct_capture_1
    string srcDir = "/Users/3i-21-331/workspace/stitching/mobile_stitching/dataset/PivoX_full/galaxy_zflip3/WideLens_20deg/" + dataset_name ;
    string dstDir = "/Users/3i-21-331/workspace/CppPractive/CppPractive/img";
    printf("dataset: %s\n", dataset_name.c_str());
    string ucd_mode = "/u_";
    
    string image_path = srcDir + ucd_mode + "1.jpg";
    double f_pix, fl_in_35mm;
    image = imread(image_path);
    
    find_fl_in_35mm(image_path, fl_in_35mm);
    fl_mm_to_pix(f_pix, fl_in_35mm, image.cols);
    
    double fy_mm, fy_pix;
    find_fl_in_35mm(image_path, fy_mm);
    fl_mm_to_pix(fy_pix, fy_mm, image.cols, 24.0);
    printf("fy = %.3lf[pix]\n", fy_pix);
    
    
    printf("focal length = %.3lf [mm] = %.2lf [pix] \n", fl_in_35mm, f_pix);
    
    int start = 1;
    int end = 10;
    
    double pitch_sum = 0;
    double cnt = 0;
    double pitch_abs_max = -1;

    
    
    for (int i = start; i < end; i++) {
//        v1 = rand() % 9 + 1;
        long start_t = getTickCount();
        printf("\n[%d] ", i);
        image_path = srcDir + ucd_mode +  to_string(i) + ".jpg";
        image = imread(image_path);
        
        Mat resized;
        resize(image, resized, Size(image.cols/2, image.rows/2));
        assert ( !image.empty() );
        // estimated / real pitch
        // 11 ->  -2.55  / 1.6
        // 13 ->   3.9 / 1.5
        // 15 -> ..2.85 / 8
        // 17 ->  8.39 / 2.1


        // LSD line segment detection
        double thLength = 40.0;
        std::vector<std::vector<double> > lines;
        LineDetect( image, thLength, lines );
//        printf("lines size: %d\n", lines.size());
        
        // Camera internal parameters
        cv::Point2d pp( image.cols / 2, image.rows / 2 );  // Principle point (in pixel)
//        double f =  0.75 * max(image.cols, image.rows);  //6.053 / 0.009; // Focal length (in pixel)
        
        
        // Vanishing point(vp) detection
        std::vector< cv::Point3d > vps;            // Detected vps(in pixel)
        std::vector< std::vector<int> > clusters;  // Line segment clustering results of each vp
        std::vector< cv::Point2d > vp2D;

        VPDetection detector;
        detector.run( lines, pp, f_pix, vps, clusters );
        detector.vp3Dto2D(vps, vp2D);
        
//        printf("vp2D in main\n");
//        for(int i = 0; i < 3; i++) {
//            printf("(x,y) = (%lf, %lf)\n", vp2D[i].x, vp2D[i].y);
//        }
        
        int idx = 0;
//        printf("atan(y,x) = atan(%.2f, %.2f)\n", image.rows/2.0 - vp2D[idx].y, f);
        vector<double> pitches(3);
        double rad2deg = 180. / M_PI;

        double pitch_0 = atan2(image.rows/2.0 - vp2D[0].y, f_pix) * rad2deg;
        double pitch_1 = atan2(image.rows/2.0 - vp2D[1].y, f_pix) * rad2deg;
        double pitch_2 = atan2(image.rows/2.0 - vp2D[2].y, f_pix) * rad2deg;
        pitches[0] = pitch_0;
        pitches[1] = pitch_1;
        pitches[2] = pitch_2;
        
        
//        printf("%3.2f, %3.2f %3.2f [deg]\n", pitch_0 , pitch_1 , pitch_2 );
//        printf("pitch: %.2f[rad] = %.2f[deg] \n", pitch_angle, pitch_angle * 180. / M_PI );
        
        vector<int> idx_of_ground;
        idx_of_ground.reserve(2);
        
        double max_abs = -11111111;
        int max_idx = 0;
        for (int i = 0; i < 3; i++) {
            
            if (max_abs < abs(pitches[i])) {
                max_abs = abs(pitches[i]);
                max_idx = i;
            }
        }
        
        for (int i = 0; i < 3; i++) {
            if (i != max_idx) {
                idx_of_ground.push_back(i);
//                printf("pushed pitch: %3.3f\n", pitches[i]);
            }
        }
        
        double x1, y1, x2, y2;
//        double a, b, c;
        x1 = vp2D[idx_of_ground[0]].x;  y1 = vp2D[idx_of_ground[0]].y;
        x2 = vp2D[idx_of_ground[1]].x;  y2 = vp2D[idx_of_ground[1]].y;
        
       
        double distance_line_to_pp = dist(pp.x, pp.y, x1, y1, x2, y2);
        double pitch_abs = abs(atan(distance_line_to_pp / f_pix)) * rad2deg;
       
        
        if (pitch_abs < 40) {
            cnt++;
            pitch_abs_max = max(pitch_abs_max, pitch_abs);
            pitch_sum += pitch_abs;
            
            printf("(x1, y1), (x2, y2), (px, py) = (%.2lf, %.2lf), (%.2lf, %.2lf), (%.0lf, %.0lf)\n", x1, y2, x2, y2, pp.x, pp.y);
            printf("distance btw line and pp : %3.3lf\n", distance_line_to_pp);
            printf("pitch_abs: %.3lf\n", pitch_abs);
        }
       
        
        
        
//        circle(image, cv::Point(vp2D[idx].x, vp2D[idx].y), 20, cv::Scalar(0, 0, 255), -1);
//        drawClusters( image, lines, clusters );
        long end_t = getTickCount();
        cout << "time: " << (end_t - start_t) / getTickFrequency() << "sec\n" << endl;
        
//        imwrite( dstDir + "/result" + to_string(i) + ".jpg", image);
        
    }
    printf("pitch average: %.2lf\n", (pitch_sum-pitch_abs_max) / (cnt-1)) ;

}



void demo_lines_and_vps(cv::Mat image) {
    cv::Mat gray;
    cv::Mat edges;
    
    cvtColor(image, gray, COLOR_BGR2GRAY);
    Canny(gray, edges, 500, 460);
    
    vector<Vec2f> lines;
    HoughLines(edges, lines, 1, CV_PI/180, 200);
    
    Mat img_hough;
    image.copyTo(img_hough);
    
    Mat img_lane;
    threshold(edges, img_lane, 500, 460, THRESH_MASK);
    
    int thickness = 1;
    for (size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        
        pt1.x = cvRound(x0 + 10000 * (-b));
        pt1.y = cvRound(y0 + 10000 * (a));
        pt2.x = cvRound(x0 - 10000 * (-b));
        pt2.y = cvRound(y0 - 10000 * (a));
        
        line(img_hough, pt1, pt2, Scalar(0,0,255), thickness + 1, LINE_8);
        line(img_lane, pt1, pt2, Scalar::all(255), thickness , LINE_8);
    }
    
    imshow("lane", img_lane);
    imshow("hough", img_hough);
    waitKey(10000);
}
