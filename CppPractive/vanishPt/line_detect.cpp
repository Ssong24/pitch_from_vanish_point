//
//  line_detect.cpp
//  CppPractive
//
//  Created by Song on 2022/04/28.
//

#include "line_detect.hpp"



using namespace std;
using namespace cv;


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
    string srcDir = "/Users/3i-21-331/workspace/stitching/mobile_stitching/dataset/PivoX_full/galaxy_zflip3/WideLens_20deg/correct_capture_1" ;
    string dstDir = "/Users/3i-21-331/workspace/CppPractive/CppPractive/img";
    
    printf("source dir: %s\n", srcDir.c_str());
    
    for (int i = 8; i < 9 /*10*/; i++) {
//        v1 = rand() % 9 + 1;
        printf("\ni: %d\n", i);
        
        image = imread(srcDir + "/c_" + to_string(i) +".jpg");
        Mat resized;
        resize(image, resized, Size(image.cols/2, image.rows/2));
    
        // estimated / real
        // 11 -> c_4 pitch -0.51594  / 1.6
        // 13 -> c_4 pitch   3.9 / 1.5
        // 15 -> c_4 pitch  ..2.85 / 8
        // 17 -> c_4 pitch   8.39 / 2.1

        if ( image.empty() )
        {
            printf( "Load image error \n" );
            return ;
        }

        // LSD line segment detection
        double thLength = 150.0;
        std::vector<std::vector<double> > lines;
        LineDetect( image, thLength, lines );
//        printf("lines size: %d\n", lines.size());
        
        // Camera internal parameters
        cv::Point2d pp( image.cols / 2, image.rows / 2 );  // Principle point (in pixel)
        double f = 0.75 * max(image.cols, image.rows);  //6.053 / 0.009; // Focal length (in pixel)

        // Vanishing point(vp) detection
        std::vector< cv::Point3d > vps;            // Detected vps(in pixel)
        std::vector< std::vector<int> > clusters;  // Line segment clustering results of each vp
        std::vector< cv::Point2d > vp2D;

        VPDetection detector;
        detector.run( lines, pp, f, vps, clusters );
        detector.vp3Dto2D(vps, vp2D);
        
        printf("vp2D in main\n");
        for(int i = 0; i < 3; i++) {
            printf("(x,y) = (%lf, %lf)\n", vp2D[i].x, vp2D[i].y);
        }
        
        printf("atan(y,x) = atan(%.2f, %.2f)\n", image.rows/2.0 - vp2D[0].y, f);
        double pitch_angle = atan2(image.rows/2.0 - vp2D[0].y, f);
        
        printf("pitch: %.2f[rad] = %.2f[deg] \n", pitch_angle, pitch_angle * 180. / M_PI );
         
        circle(image, cv::Point(vp2D[0].x, vp2D[0].y), 20, cv::Scalar(0, 0, 255), -1);
        drawClusters( image, lines, clusters );
        
        imwrite( dstDir + "/result" + to_string(i) + ".jpg", image);
        
    }

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
