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
        cv::Point pt_m = ( pt_s + pt_e ) * 0.5;

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


void demo_VPDetection(string inputImagePath) {
//    string inPutImage = "D:\\DevelopCenter\\VanishingPoints\\datasets\\YorkUrbanDB\\P1020171\\P1020171.jpg";

    cv::Mat image= cv::imread( inputImagePath );
    if ( image.empty() )
    {
        printf( "Load image error : %s\n", inputImagePath.c_str() );
    }

    // LSD line segment detection
    double thLength = 100.0;  // 30.0
    std::vector<std::vector<double> > lines;
    LineDetect( image, thLength, lines );

    // Camera internal parameters
    cv::Point2d pp( image.cols / 2, image.rows / 2 );  // Principle point (in pixel)
    double f = 1.2 * max(image.cols, image.rows);  //6.053 / 0.009; // Focal length (in pixel)

    // Vanishing point detection
    std::vector<cv::Point3d> vps;              // Detected vanishing points (in pixel)
    std::vector<std::vector<int> > clusters;   // Line segment clustering results of each vanishing point
    std::vector<cv::Point2d> vp2D;
    

    VPDetection detector;
    detector.run( lines, pp, f, vps, clusters );
    detector.vp3Dto2D(vps, vp2D);
    printf("vp2D in main\n");
    for(int i = 0; i < 3; i++) {
        printf("(x,y) = (%lf, %lf)\n", vp2D[i].x, vp2D[i].y);
    }
    
    printf("\npy = %d\n", image.rows / 2);
    printf("f = %lf\n", f);
    double pitch_angle = atan2(image.rows /2 - vp2D[0].y , f);
    cout << "pitch: " << pitch_angle << " = " << pitch_angle * 180. / M_PI << endl;
    
    circle(image, cv::Point(vp2D[0].x, vp2D[0].y), 20, cv::Scalar(0, 0, 255), -1);

    drawClusters( image, lines, clusters );
    imshow("",image);
    cv::waitKey(3000);
}

