//
//  dense_opt_flow.cpp
//  CppPractive
//
//  Created by Song on 2022/05/15.
//

#include "dense_opt_flow.hpp"


void dense_opt_flow_to_left_right() {
    std::string leftImg = "";
    std::string rightImg = "";
    
    
    
}

void demo_dense_opt_flow(std::string method) {
    std::string filename = "/Users/3i-21-331/workspace/CppPractive/CppPractive/cars_moving.mp4";
    
    std::string srcDir = "/Users/3i-21-331/workspace/stitching/mobile_stitching/dataset/blending";

    std::string leftFile, rightFile;
    bool is_center = true;
    int dataset_num = 11; // 23, 13
    
    if (is_center) {
        
        leftFile  = srcDir + "/" + std::to_string(dataset_num) + "/Center/src_l_0.jpg";
        rightFile = srcDir + "/" + std::to_string(dataset_num) + "/Center/src_r_0.jpg";
    }
    else {
        leftFile  = srcDir + "/" + std::to_string(dataset_num) + "/Up-Center-Down/src_l_0.jpg";
        rightFile = srcDir + "/" + std::to_string(dataset_num) + "/Up-Center-Down/src_r_0.jpg";
    }
    
    bool to_gray = false;
    
//    dense_optical_flow_video(filename, cv::optflow::calcOpticalFlowSparseToDense, to_gray, 8, 128, 0.05f, true, 500.0f, 1.5f); // default OpenCV params
    
    if (method == "lucaskanade_dense") {
        dense_optical_flow_images(leftFile, rightFile, cv::optflow::calcOpticalFlowSparseToDense, to_gray, 8, 128, 0.1f, true, 500.0f, 1.5f);
    }
    else if (method == "farneback") {
        to_gray = true;
        int opticalWindowSize = 30; //20; // 15; --> Larger window size, more robust to noise
        int levels = 3;
        int iterations = 3;
        int poly_n =  7; // 5;
        double poly_sigma = 1.1; //  1.5; // 1.2;
        int flags = 0;
        
        dense_optical_flow_images(leftFile, rightFile, cv::calcOpticalFlowFarneback, to_gray, 0.5, levels, opticalWindowSize, iterations, poly_n, poly_sigma, flags);
    }
    
    else if (method == "rlof"){
        to_gray = false;
        dense_optical_flow_images(
           leftFile, rightFile, cv::optflow::calcOpticalFlowDenseRLOF, to_gray,
           cv::Ptr<cv::optflow::RLOFOpticalFlowParameter>(), 1.f, cv::Size(6,6),
           cv::optflow::InterpolationType::INTERP_EPIC, 128, 0.05f, 999.0f,
           15, 100, true, 500.0f, 1.5f, false); // default OpenCV params
    }
    else {
        std::cout << "Please put the right algorithm name" << std::endl;
    }
    
    return ;
}


template <typename Method, typename... Args>
void dense_optical_flow_images(std::string leftImgFile, std::string rightImgFile, Method method, bool to_gray, Args&&... args)
{
    
    std::string srcDir = "/Users/3i-21-331/workspace/CppPractive/CppPractive/opticalFlow/result/";
    bool save = true;
    int counter = 0;
    
    cv::Mat original_leftImg, original_rightImg;
    original_leftImg = cv::imread(leftImgFile);
    original_rightImg = cv::imread(rightImgFile);
    cv::Mat leftImg     = cv::imread(leftImgFile);
    cv::Mat rightImg    = cv::imread(rightImgFile);
    
    assert(!leftImg.empty() && !rightImg.empty());
//    std::cout << "left  image size: " << leftImg.size() << std::endl;
//    std::cout << "right image size: " << rightImg.size() << std::endl;
    
    
    cv::Mat resultImg = cv::Mat(leftImg.rows, leftImg.cols, leftImg.type());
    leftImg(cv::Rect(0, 0, leftImg.cols /2, leftImg.rows )).copyTo(resultImg(cv::Rect(0, 0, leftImg.cols / 2, leftImg.rows)));
    rightImg(cv::Rect(leftImg.cols /2, 0, leftImg.cols /2, leftImg.rows )).copyTo(resultImg(cv::Rect(leftImg.cols/2, 0, leftImg.cols / 2, leftImg.rows)));
    
    cv::Mat prvsL;//, prvsR;
    
    // Preprocessing for exact method
    if (to_gray)
        cvtColor(leftImg, prvsL, cv::COLOR_BGR2GRAY);
    else
        prvsL = leftImg;
    
    // Read the next frame
    cv::Mat frame2, prvsR; // next;
    
    frame2 = rightImg.clone();
    if (to_gray)
        cvtColor(frame2, prvsR, cv::COLOR_BGR2GRAY);
    else
        prvsR = frame2;
    
    // Calculate Optical Flow
//    std::cout << "prevL, next size: " << prvsL.size() << ", " << prvsR.size() << std::endl;
//    std::cout << "prevL, next channel: " << prvsL.channels() << ", " << prvsR.channels() << std::endl;
    cv::Mat flowL2R(prvsR.size(), CV_32FC2);
    cv::Mat flowR2L(prvsL.size(), CV_32FC2);
    
    // flow 1 prvs -> next
    method(prvsL, prvsR, flowL2R, std::forward<Args>(args)...);
    // flow 2 next -> prvs
    method(prvsR, prvsL, flowR2L, std::forward<Args>(args)...);
    
    // calculate updated pixel location
    // if not matched
    // use average value

//    imwrite(srcDir + "beforeflowR2L.jpg", prvsR);
    cv::Mat prvsR_drawn = prvsR.clone(); // cv::Mat(prvsR.rows, prvsR.cols, prvsR.type());
    cv::Mat prvsL_drawn = prvsL.clone(); cv::Mat(prvsL.rows, prvsL.cols, prvsL.type());
    
    int win_interval = 15;
    int radius = 0;
    for (int y = 0; y < prvsR.rows; y += win_interval) {
        for (int x = 0; x < prvsR.cols; x += win_interval) {
            
            const cv::Point2f flowatxy = flowL2R.at<cv::Point2f>(y,x);
//            printf("(%d,%d) = %.2f, %.2f\n", x, y, flowatxy.x, flowatxy.y);
            line(prvsR_drawn, cv::Point(x,y), cv::Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), cv::Scalar(255,0,0));
            circle(prvsR_drawn, cv::Point(x,y), radius, cv::Scalar (0,0,0), -1);
        }
    }
    imwrite(srcDir + "flowR2L.jpg", prvsR_drawn);
    
    
    for (int y = 0; y < prvsL.rows; y += win_interval) {
        for (int x = 0; x < prvsL.cols; x += win_interval) {
            const cv::Point2f flowatxy = flowR2L.at<cv::Point2f>(y,x);
//            printf("(%d,%d) = %.2f, %.2f\n", x, y, flowatxy.x, flowatxy.y);
            line(prvsL_drawn, cv::Point(x,y), cv::Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), cv::Scalar(255,0,0));
            circle(prvsL_drawn, cv::Point(x,y), radius, cv::Scalar (0,0,0), -1);
        }
    }
    imwrite(srcDir + "flowL2R.jpg", prvsL_drawn);
    
    
    std::string drawing = "twoOptFlow";//"halfL_halfR"; //"twoOptFlow"; // halfL_halfR, twoOptFlow
    
    if (drawing =="halfL_halfR") {
        for (int y = 230; y < prvsR.rows-230; y += 1 /*opticalWindowSize*/) {
            for (int x = 1; x < prvsR.cols / 2; x += 1 /*opticalWindowSize*/) {
                int dest_x, dest_y;
                const cv::Point2f flowPt_L2R = flowL2R.at<cv::Point2f>(y,x);
                const cv::Point2f flowPt_R2L = flowR2L.at<cv::Point2f>(y,x);
                
                // half movement of next to prvs
                double half_x_r2l = flowPt_R2L.x / 2.0;
                double half_y_r2l = flowPt_R2L.y / 2.0;  // n2p -> r2l
                // half movement of prvs to next
                double half_x_l2r = flowPt_L2R.x / 2.0;
                double half_y_l2r = flowPt_L2R.y / 2.0; // p2n -> l2r
                
//                // bilinear interpolation //
//                double xf = x + half_x_r2l + x - half_x_l2r;
//                double yf = y + half_y_r2l + y - half_y_l2r;
//
//                double xi_1 = (int)xf;
//                double yi_1 = (int)yf;
//                double xi_2 = xi_1 + 1;
//                double yi_2 = yi_1 + 1;
//
//                double a, b, p, q;
//                a = (double)yi_2 - yf;
//                b = yf - (double)yi_1;
//                p = xf - (double)xi_1;
//                q = (double)xi_2 - xf;
                
//                double M, N;
//                int P;
//                M = a * half_x_r2l + b * half_y_l2r;
                
                
                ////////////////////////////
                
                
                
                // x
                if ( abs(trunc(half_x_r2l * 10.0)) != abs(trunc(half_x_r2l * 10.0)) ) { // compare until floating point 1
                    dest_x = cvRound((x + half_x_r2l + x - half_x_r2l) / 2.0 );
                }
                else {
                    dest_x = cvRound(x - half_x_l2r) ; //flowatxy2.x); // half_x_p2n);
                }
                
                // y
                if ( abs(trunc(half_y_r2l*10.0)) != abs(trunc(half_y_l2r * 10.0)) ) { // compare until floating point 1
                    dest_y = cvRound((y + half_y_r2l + y - half_y_l2r) / 2.0 );
                }
                else {
                    dest_y = cvRound(y - half_y_l2r);  // flowatxy2.y); // half_y_p2n);
                }
                
                resultImg.at<cv::Vec3b>(dest_y,dest_x) = leftImg.at<cv::Vec3b>(y,x);
            }
        }
        
        for (int y = 230; y < prvsR.rows-230; y += 1 /*opticalWindowSize*/) {
            for (int x = prvsR.cols /2; x < prvsR.cols ; x += 1 /*opticalWindowSize*/) {
                int dest_x, dest_y;
                const cv::Point2f flowPt_R2L = flowR2L.at<cv::Point2f>(y,x);
                const cv::Point2f flowPt_L2R = flowL2R.at<cv::Point2f>(y,x);
                
                // half movement of next to prvs
                double half_x_r2l = flowPt_R2L.x / 2.0;
                double half_y_r2l = flowPt_R2L.y / 2.0;  // n2p -> r2l
                // half movement of prvs to next
                double half_x_l2r = flowPt_L2R.x / 2.0;
                double half_y_l2r = flowPt_L2R.y / 2.0;  // p2n -> l2r
                
                // x
                if ( abs(trunc(half_x_r2l*10.0)) != abs(trunc(half_x_l2r * 10.0)) ) { // compare until floating point 1
                    dest_x = cvRound(x + ( half_x_l2r - half_x_r2l ) / 2.0 );
                }
                else {
                    dest_x = cvRound(x - half_x_r2l) ; //flowatxy2.x); // bhalf_x_p2n);
                }
                
                // y
                if ( abs(trunc(half_y_r2l*10.0)) != abs(trunc(half_y_l2r * 10.0)) ) { // compare until floating point 1
                    dest_y = cvRound(y + (half_y_r2l + half_y_l2r) / 2.0 );
                }
                else {
                    dest_y = cvRound(y - half_y_r2l); // flowatxy2.y); // half_y_p2n);
                }
                
                resultImg.at<cv::Vec3b>(dest_y,dest_x) = rightImg.at<cv::Vec3b>(y,x);
            }
        }
        imwrite(srcDir + "resultImg.jpg", resultImg);

        
        
    } else if (drawing == "twoOptFlow") {

        cv::Mat resultImgL = leftImg.clone(); // cv::Mat(leftImg.rows, leftImg.cols, leftImg.type());
        cv::Mat resultImgR = rightImg.clone(); // cv::Mat(rightImg.rows, rightImg.cols, rightImg.type());
        
        // left
//        std::cout << "check: " << flowR2L.at<cv::Point2f>(prvsR.rows-1, prvsR.cols-1) << std::endl;
        for (int y = 230; y < prvsR.rows-230; y += 1 /*opticalWindowSize*/) {
            for (int x = 0; x < prvsR.cols; x += 1 /*opticalWindowSize*/) {
                int dest_x, dest_y;
                const cv::Point2f flowPt_R2L = flowR2L.at<cv::Point2f>(y,x);
                const cv::Point2f flowPt_L2R = flowL2R.at<cv::Point2f>(y,x);
                
                // half movement of next to prvs
//                printf("(%d, %d) = %.2lf, %.2lf\n", x, y, flowPt_R2L.x, flowPt_R2L.y );
                double half_x_r2l = flowPt_R2L.x / 2.0;
                double half_y_r2l = flowPt_R2L.y / 2.0;
                // half movement of prvs to next
                double half_x_l2r = flowPt_L2R.x / 2.0;
                double half_y_l2r = flowPt_L2R.y / 2.0;
                
                
                // bilinear interpolation //
                double xf = x + (-half_x_r2l + half_x_l2r) / 2.0;
                double yf = y + (-half_y_r2l + half_y_l2r) / 2.0;
                
                int xi_1 = (int)xf;
                int yi_1 = (int)yf;
                int xi_2 = (int)(xf + 0.5); // xi_1 + 1;
                int yi_2 = (int)(yf + 0.5); // yi_1 + 1;
                
                double a, b, p, q;
                a = (double)yi_2 - yf;  // yf
                b = yf - (double)yi_1;  // yf
                p = xf - (double)xi_1;  // xf
                q = (double)xi_2 - xf;  // xf
                
                double M, N;
                int P;
                
//                std::cout << "flow value " << std::endl;
//                std::cout << flowL2R.at<cv::Point2f>(yi_1, xi_1).x  << ", " << flowL2R.at<cv::Point2f>(yi_2, xi_1).x << std::endl;
//                std::cout << flowL2R.at<cv::Point2f>(yi_1, xi_2).x << ", " << flowL2R.at<cv::Point2f>(yi_2, xi_2).x << std::endl;
                
                M = a * flowL2R.at<cv::Point2f>(yi_1, xi_1).x + b * flowL2R.at<cv::Point2f>(yi_2, xi_1).x;
                N = a * flowL2R.at<cv::Point2f>(yi_1, xi_2).x + b * flowL2R.at<cv::Point2f>(yi_2, xi_2).x;
                P = (int) ( (p * N + q * M) / (p + q) );
                
//                std::cout << "(x) M,N: " << M << ", " << N << std::endl;
//                std::cout << "(x) P: " << P << std::endl;
                
                // x
//                if ( abs(trunc(half_x_r2l*10.0)) != abs(trunc(half_x_l2r * 10.0)) ) { // compare until floating point 1
                dest_x = cvRound(x + (-half_x_r2l + half_x_l2r) / 2.0 );  // P
//                }
//                else {
//                    dest_x = cvRound(x + half_x_l2r) ; //flowatxy2.x); // half_x_p2n);
//                }
                
                M = a * flowL2R.at<cv::Point2f>(yi_1, xi_1).y + b * flowL2R.at<cv::Point2f>(yi_2, xi_1).y;
                N = a * flowL2R.at<cv::Point2f>(yi_1, xi_2).y + b * flowL2R.at<cv::Point2f>(yi_2, xi_2).y;
                P = (int)(p * N + q * M) / (p + q);
//                std::cout << "(y) M,N: " << M << ", " << N << std::endl;
//                std::cout << "(y) P: " << P << std::endl;
                
                
                
                // y
//                if ( abs(trunc(half_y_r2l*10.0)) != abs(trunc(half_y_l2r * 10.0)) ) { // compare until floating point 1
                dest_y = cvRound(y + (-half_y_r2l + half_y_l2r) / 2.0 );  // P
//                }
//                else {
//                    dest_y = cvRound(y + half_y_l2r); // flowatxy2.y); // half_y_p2n);
//                }
                
                resultImgL.at<cv::Vec3b>(dest_y,dest_x) = leftImg.at<cv::Vec3b>(y,x);
            }
        }
        
        // right
        for (int y = 230; y < prvsR.rows-230; y += 1) {
            for (int x = 1; x < prvsR.cols ; x += 1) {
                int dest_x, dest_y;
                const cv::Point2f flowPt_R2L = flowR2L.at<cv::Point2f>(y,x);
                const cv::Point2f flowPt_L2R = flowL2R.at<cv::Point2f>(y,x);
                
                // half movement of next to prvs
                double half_x_r2l = flowPt_R2L.x / 2.0;
                double half_y_r2l = flowPt_R2L.y / 2.0;  // n2p -> r2l
                // half movement of prvs to next
                double half_x_l2r = flowPt_L2R.x / 2.0;
                double half_y_l2r = flowPt_L2R.y / 2.0;  // p2n -> l2r
                
                // x
                if ( abs(trunc(half_x_r2l*10.0)) != abs(trunc(half_x_l2r * 10.0)) ) { // compare until floating point 1
                    dest_x = cvRound(x + (half_x_r2l - half_x_l2r) / 2.0 );
                }
                else {
                    dest_x = cvRound(x + half_x_r2l) ; //flowatxy2.x); // half_x_p2n);
                }
                
                // y
                if ( abs(trunc(half_y_r2l*10.0)) != abs(trunc(half_y_l2r * 10.0)) ) { // compare until floating point 1
                    dest_y = cvRound(y + (half_y_r2l - half_y_l2r) / 2.0 );
                }
                else {
                    dest_y = cvRound(y + half_y_r2l); // flowatxy2.y); // half_y_p2n);
                }
                
                resultImgR.at<cv::Vec3b>(dest_y,dest_x) = rightImg.at<cv::Vec3b>(y,x);
            }
        }
        
        // save image
        imwrite(srcDir + "resultImgR.jpg", resultImgR);
        imwrite(srcDir + "resultImgL.jpg", resultImgL);
        
    }
    
    
    

    
    
    
    // Visualization part
    cv::Mat flow_parts[2];
    split(flowR2L, flow_parts);

    // Convert the algorithm's output into Polar coordinates
    cv::Mat magnitude, angle, magn_norm;
    cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
    normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));

    // Build hsv image
    cv::Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    
    // Display the results
    cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);
    if (save) {
        std::string save_path = "./optical_flow_frames/frame_" + std::to_string(counter) + ".jpg";
        imwrite(save_path, bgr);
    }
    imwrite(srcDir + "frameL.jpg", leftImg);
    imwrite(srcDir + "frameR.jpg", rightImg);
    imwrite(srcDir + "flow_hsv.jpg", bgr);
    
//    cv::waitKey(0);


    // Update the previous frame
//    prvs = next;
//    counter++;

}


template <typename Method, typename... Args>
void dense_optical_flow_video(std::string filename, Method method, bool to_gray, Args&&... args)
{
    bool save = true;
    int counter = 0;
    // Read the video and first frame
    cv::VideoCapture capture(filename);
    if (!capture.isOpened()) {
        //error in opening the video input
        std::cerr << "Unable to open file!" << std::endl;
    }
    cv::Mat frame1, prvs;
    capture >> frame1;
    
    // Preprocessing for exact method
    if (to_gray)
        cvtColor(frame1, prvs, cv::COLOR_BGR2GRAY);
    else
        prvs = frame1;

    while (true) {
        // Read the next frame
        cv::Mat frame2, next;
        capture >> frame2;
        if (frame2.empty())
            break;

        // Preprocessing for exact method
        if (to_gray)
            cvtColor(frame2, next, cv::COLOR_BGR2GRAY);
        else
            next = frame2;

        // Calculate Optical Flow
        cv::Mat flow(prvs.size(), CV_32FC2);
        method(prvs, next, flow, std::forward<Args>(args)...);

        // Visualization part
        cv::Mat flow_parts[2];
        split(flow, flow_parts);

        // Convert the algorithm's output into Polar coordinates
        cv::Mat magnitude, angle, magn_norm;
        cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
        angle *= ((1.f / 360.f) * (180.f / 255.f));

        // Build hsv image
        cv::Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        
        // Display the results
        cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);
        if (save) {
            std::string save_path = "./optical_flow_frames/frame_" + std::to_string(counter) + ".jpg";
            imwrite(save_path, bgr);
        }
        imshow("frame", frame2);
        imshow("flow", bgr);
        int keyboard = cv::waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;

        // Update the previous frame
        prvs = next;
        counter++;
    }
}
