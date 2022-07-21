//
//  dense_opt_flow.cpp
//  CppPractive
//
//  Created by Song on 2022/05/15.
//

#include "dense_opt_flow.hpp"



void demo_dense_opt_flow(std::string method) {
    std::string filename = "/Users/3i-21-331/workspace/CppPractive/CppPractive/cars_moving.mp4";
    
    std::string srcDir = "/Users/3i-21-331/workspace/stitching/mobile_stitching/dataset/blending";

    std::string leftFile, rightFile;
    bool is_center =  true; // false; // true;
    int dataset_num = 13; // 23, 13 (11, 13, 23) : (zflip, zflip, ultra)
    
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
    std::string drawing = "backwardOptFlow"; // "twoOptFlow";//"halfL_halfR";  // halfL_halfR, twoOptFlow, backwardOptFlow
    
    std::string srcDir = "/Users/3i-21-331/workspace/CppPractive/CppPractive/opticalFlow/result/";
    bool save = true;
    int counter = 0;
    
    cv::Mat original_leftImg, original_rightImg;
    original_leftImg    = cv::imread(leftImgFile);
    original_rightImg   = cv::imread(rightImgFile);
    cv::Mat leftImg     = cv::imread(leftImgFile);
    cv::Mat rightImg    = cv::imread(rightImgFile);
    
    assert(!leftImg.empty() && !rightImg.empty());
    
    
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
    cv::Mat flowL2R(prvsR.size(), CV_32FC2);
    cv::Mat flowR2L(prvsL.size(), CV_32FC2);
    
    // flow 1 prvs -> next
    method(prvsL, prvsR, flowL2R, std::forward<Args>(args)...);
    // flow 2 next -> prvs
    method(prvsR, prvsL, flowR2L, std::forward<Args>(args)...);
    std::cout << "flowL2R type: " << (int)flowL2R.type() << std::endl;
    
    
    // calculate updated pixel location
    // if not matched
    // use average value
    cv::Mat prvsR_drawn = prvsR.clone(); // cv::Mat(prvsR.rows, prvsR.cols, prvsR.type());
    cv::Mat prvsL_drawn = prvsL.clone(); cv::Mat(prvsL.rows, prvsL.cols, prvsL.type());
    // draw_optical_flow(prvsR.cols, prvsR.rows, flowL2R, flowR2L, prvsL_drawn, prvsR_drawn);
    

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
                    dest_x = cvRound(x - half_x_r2l) ; // flowatxy2.x); // half_x_p2n);
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

        
        
    }
    else if (drawing == "twoOptFlow") {
        int srcW, srcH;
        
        assert(leftImg.cols == rightImg.cols);
        assert(leftImg.rows == rightImg.rows);
        
        srcW = leftImg.cols;    srcH = leftImg.rows;
        cv::Mat resultImgL = cv::Mat::zeros(srcH, srcW, leftImg.type()); // leftImg.clone();
        cv::Mat resultImgR = cv::Mat::zeros(srcH, srcW, rightImg.type()); // rightImg.clone();
        
        // left
        int shift_ = 2;
        int dest_x, dest_y;
        double half_x_r2l, half_y_r2l, half_x_l2r, half_y_l2r;
        cv::Point2f flowPt_R2L, flowPt_L2R;
        
        for (int y = shift_; y < prvsL.rows-shift_; y += 1) {
            for (int x = shift_; x < prvsL.cols - shift_; x += 1) {
                
                // 1. Fix first to last, last to first -> first2first, last2last
                // 2. optical flow distance
                int xr, yr;
                
                flowPt_L2R = flowL2R.at<cv::Point2f>(y,x);
                
                xr = round(x + flowPt_L2R.x);   yr = round(y + flowPt_L2R.y);
                xr = max(0, xr);                yr = max(0,yr);
                xr = min(xr, prvsR.cols-1);     yr = min(yr, prvsR.rows-1);
            
                flowPt_R2L = flowR2L.at<cv::Point2f>(yr, xr);
                
                // half movement of next to prvs
                half_x_r2l = flowPt_R2L.x / 2.0;
                half_y_r2l = flowPt_R2L.y / 2.0;
                // half movement of prvs to next
                half_x_l2r = flowPt_L2R.x / 2.0;
                half_y_l2r = flowPt_L2R.y / 2.0;
                
                
                dest_x = cvRound(x + (-half_x_r2l + half_x_l2r) / 2.0 );
                dest_y = cvRound(y + (-half_y_r2l + half_y_l2r) / 2.0 );
                
                dest_x = max(0, dest_x);                dest_y = max(0, dest_y);
                dest_x = min(dest_x, prvsL.cols-1);     dest_y = min(dest_y, prvsL.rows -1);
                
                resultImgL.at<cv::Vec3b>(dest_y,dest_x) = leftImg.at<cv::Vec3b>(y,x);
                
            }
        }
        
        // right
        for (int y = shift_; y < prvsR.rows-shift_; y += 1) {
            for (int x = shift_; x < prvsR.cols - shift_; x += 1) {
                int xl, yl;
                
                flowPt_R2L = flowR2L.at<cv::Point2f>(y,x);
                
                xl = round(x + flowPt_R2L.x);   yl = round(y + flowPt_R2L.y);
                xl = max(0, xl);                yl = max(0,yl);
                xl = min(xl, prvsR.cols-1);     yl = min(yl, prvsR.rows-1);
                
                flowPt_L2R = flowL2R.at<cv::Point2f>(yl,xl);
                
                // half movement of next to prvs
                half_x_r2l = flowPt_R2L.x / 2.0;
                half_y_r2l = flowPt_R2L.y / 2.0;  // n2p -> r2l
                // half movement of prvs to next
                half_x_l2r = flowPt_L2R.x / 2.0;
                half_y_l2r = flowPt_L2R.y / 2.0;  // p2n -> l2r
                
                dest_x = cvRound(x + (half_x_r2l - half_x_l2r) / 2.0 );
                dest_y = cvRound(y + (half_y_r2l - half_y_l2r) / 2.0 );
                
                dest_x = max(0, dest_x);                dest_y = max(0, dest_y);
                dest_x = min(dest_x, prvsL.cols-1);     dest_y = min(dest_y, prvsL.rows -1);
                
                resultImgR.at<cv::Vec3b>(dest_y,dest_x) = rightImg.at<cv::Vec3b>(y,x);
            }
        }
        
        cv::Mat woOptResult, bufferResult;
        smoothing::NewSmoothing_blendedRoI(resultImgL, resultImgR, 0.1, 0.015, bufferResult);
        smoothing::NewSmoothing_blendedRoI(leftImg, rightImg, 0.1, 0.015, woOptResult);
        
        // inpainting
        cv::Mat grayL, grayR;
        cv::Mat maskL, maskR;
        cv::Mat maskXL, maskXR;
//        maskXL = cv::Mat::zeros(resultImgL.rows, resultImgL.cols, resultImgL.type());
//        maskXR = cv::Mat::zeros(resultImgR.rows, resultImgR.cols, resultImgR.type());
        
//        cv::cvtColor(resultImgL, grayL, cv::COLOR_BGR2GRAY);
//        cv::cvtColor(resultImgR, grayR, cv::COLOR_BGR2GRAY);

        cv::inRange(resultImgL, cv::Scalar(0,0,0), cv::Scalar(0,0,0), maskL);
        cv::inRange(resultImgR, cv::Scalar(0,0,0), cv::Scalar(0,0,0), maskR);
        
        
        cv::bitwise_not(maskL, maskXL);
        cv::bitwise_not(maskR, maskXR);
//        cv::inRange(resultImgL /*grayL*/, cv::Scalar(0,0,0), cv::Scalar(0,0,0), maskXL);
//        cv::inRange(resultImgR /*grayR*/, cv::Scalar(0,0,0), cv::Scalar(0,0,0), maskXR);
        
        imwrite(srcDir + "maskL.jpg", maskL);
        imwrite(srcDir + "maskR.jpg", maskR);
        imwrite(srcDir + "maskXL.jpg", maskXL);
        imwrite(srcDir + "maskXR.jpg", maskXR);
        
        
        cv::Mat dstL, dstR;
        cv::Mat xDstL, xDstR;
        xDstL = resultImgL.clone();
        xDstR = resultImgR.clone();
        
//        cv::inpaint(resultImgL, maskL, dstL, 2, cv::INPAINT_TELEA);
//        cv::inpaint(resultImgR, maskR, dstR, 2, cv::INPAINT_TELEA);
        cout << "Inpainting start! " << endl;
        cv::xphoto::inpaint(resultImgL, maskXL, xDstL, cv::xphoto::INPAINT_FSR_FAST);
        cv::xphoto::inpaint(resultImgR, maskXR, xDstR, cv::xphoto::INPAINT_FSR_FAST);
        cout << "Inpainting end! " << endl;
        
        
        cv::Mat inpaintOpt, inpaintOptX;
        
//        smoothing::NewSmoothing_blendedRoI( dstL,  dstR, 0.1, 0.015, inpaintOpt);
        smoothing::NewSmoothing_blendedRoI(xDstL, xDstR, 0.1, 0.015, inpaintOptX);
//        cv::Mat bilat;
//        cv::bilateralFilter(inpaintOpt, bilat, d, sigmaColor, sigmaSpace)
        
//        imwrite(srcDir + "grayL.jpg", grayL);
//        imwrite(srcDir + "grayR.jpg", grayR);
        
        int n_split = 5;
        for (int i =0; i<n_split; i++) {
            int x, y, w, h;
            x = 0; // round(i / (float)n_split * bufferResult.cols);
            y = round(i / (float)n_split * bufferResult.rows);
            w = bufferResult.cols; // round(bufferResult.cols / (float)n_split);
            h = round(bufferResult.rows / (float)n_split);
            
            cv::Rect crop_region(x, y, w, h);
            imwrite(srcDir + "frameL" + to_string(i) + ".jpg", leftImg(crop_region));
            imwrite(srcDir + "frameR" + to_string(i) + ".jpg", rightImg(crop_region));
            imwrite(srcDir + "xInpaint" + to_string(i) + ".jpg", inpaintOptX(crop_region));
            imwrite(srcDir + "woOpt" + to_string(i) + ".jpg",woOptResult(crop_region));

        }
        
        imwrite(srcDir + "frameL.jpg", leftImg);
        imwrite(srcDir + "frameR.jpg", rightImg);

        imwrite(srcDir + "flowR2L.jpg", prvsR_drawn);
        imwrite(srcDir + "flowL2R.jpg", prvsL_drawn);
        
//        imwrite(srcDir + "maskL.jpg", maskL);
//        imwrite(srcDir + "maskR.jpg", maskR);
//        imwrite(srcDir + "maskXL.jpg", maskXL);
//        imwrite(srcDir + "maskXR.jpg", maskXR);
//
//        imwrite(srcDir + "inpaintL.jpg", dstL);
//        imwrite(srcDir + "inpaintR.jpg", dstR);
        
        imwrite(srcDir + "xInpaintL.jpg", xDstL);
        imwrite(srcDir + "xInpaintR.jpg", xDstR);
        
        imwrite(srcDir + "resultImgR.jpg", resultImgR);
        imwrite(srcDir + "resultImgL.jpg", resultImgL);
        
        imwrite(srcDir + "bufResult.jpg", bufferResult);
        imwrite(srcDir + "woOptResult.jpg",woOptResult);
//        imwrite(srcDir + "inpaintOptResult.jpg", inpaintOpt);
        imwrite(srcDir + "xInpaintResult.jpg", inpaintOptX);
        
    }
    
    else if (drawing == "backwardOptFlow") {
        cv::Mat resultImgL = cv::Mat::zeros(leftImg.rows, leftImg.cols, leftImg.type());
        cv::Mat resultImgL_gauss = cv::Mat::zeros(leftImg.rows, leftImg.cols, leftImg.type());
        
        cv::Mat resultImgR = cv::Mat::zeros(rightImg.rows, rightImg.cols, rightImg.type());
        
        cv::Point2f flowPt_R2L, flowPt_L2R, flowPt_R2L_gauss;
        int width = resultImgL.cols, height = resultImgL.rows;
        
        int xl, yl, xr, yr;
        int src_x, src_y;
        double half_x_r2l, half_y_r2l, half_x_l2r, half_y_l2r;
        
        
        cv::Mat flowR2L_xy[2];
        split(flowR2L, flowR2L_xy);
        
        cv::Mat flowR2L_gauss, flowR2L_gaussX, flowR2L_gaussY;
        cv::GaussianBlur(flowR2L, flowR2L_gauss, cv::Size(width, 1), 1);
        cv::GaussianBlur(flowR2L_xy[0], flowR2L_gaussX,cv::Size(width,1), 1);
        cv::GaussianBlur(flowR2L_xy[1], flowR2L_gaussY,cv::Size(width,1), 1);
        
        cout << "size: " << flowR2L.size() << endl;
        for (int x = 0; x < 260; x++) {
            for (int y = 1100; y < 1101; y++) {
            std::cout << x << ", " << y <<  " : " << flowR2L.at<cv::Vec2f>(y,x) << ",   " << flowR2L_gauss.at<cv::Vec2f>(y,x) << std::endl;
            }
        }
        
        
        // result Left
        int shift_ = 235;
        for (int x = 0; x < width; x++) {
            for (int y = shift_; y < height - shift_; y++) {
                // input: (0,0) - flow (2, 3)
                flowPt_R2L = flowR2L.at<cv::Point2f>(y,x);
                // 각 자리의 optical flow
                src_x = x + flowPt_R2L.x;
                src_y = y + flowPt_R2L.y;
                
                src_x = max(0, src_x);          src_y = max(0, src_y);
                src_x = min(src_x, width-1);    src_y = min(src_y, height-1);

                resultImgL.at<cv::Vec3b>(y,x) = leftImg.at<cv::Vec3b>(src_y, src_x);
                // resultImgL(y,x) = imageLeft(src_y, src_x);
                // resultImgR.at<cv::Vec3b>(dest_y,dest_x) = rightImg.at<cv::Vec3b>(y,x);
                
//                flowPt_R2L_gauss = Point2f( flowR2L_gaussX.at(x), flowR2L_gaussY.at(y) );
//
//                src_x = x + flowPt_R2L_gauss.x;
//                src_y = y + flowPt_R2L_gauss.y;
//
//                src_x = max(0, src_x);          src_y = max(0, src_y);
//                src_x = min(src_x, width-1);    src_y = min(src_y, height-1);
//                resultImgL_gauss.at<cv::Vec3b>(y,x) = leftImg.at<cv::Vec3b>(src_y, src_x);
                
                flowPt_R2L_gauss = flowR2L_gauss.at<cv::Point2f>(y,x);
                src_x = x + flowPt_R2L_gauss.x;
                src_y = y + flowPt_R2L_gauss.y;

                src_x = max(0, src_x);          src_y = max(0, src_y);
                src_x = min(src_x, width-1);    src_y = min(src_y, height-1);
                resultImgL_gauss.at<cv::Vec3b>(y,x) = leftImg.at<cv::Vec3b>(src_y, src_x);
                
                
            }
        }
        
        imwrite(srcDir + "frameL.jpg", leftImg);
        imwrite(srcDir + "frameR.jpg", rightImg);
        imwrite(srcDir + "resultImgL_dewarp.jpg", resultImgL);
        imwrite(srcDir + "resultImgL_dewarp_Gauss.jpg", resultImgL_gauss);
        
        cout << "saving image" << endl;
        
        // result Right

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

//    imwrite(srcDir + "flow_hsv.jpg", bgr);
    
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


void draw_optical_flow(int width, int height, cv::Mat flowL2R_, cv::Mat flowR2L_, cv::Mat& prvsL_drawn, cv::Mat prvsR_drawn) {
    
    // draw optical flow
    int win_interval = 15;
    int radius = 0;
    
    for (int y = 0; y < height; y += win_interval) {
        for (int x = 0; x < width; x += win_interval) {
            
            const cv::Point2f flowatxy = flowL2R_.at<cv::Point2f>(y,x);
            line(prvsR_drawn, cv::Point(x,y), cv::Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), cv::Scalar(255,0,0));
            circle(prvsR_drawn, cv::Point(x,y), radius, cv::Scalar (0,0,0), -1);
        }
    }

    
    
    for (int y = 0; y < height; y += win_interval) {
        for (int x = 0; x < width; x += win_interval) {
            const cv::Point2f flowatxy = flowR2L_.at<cv::Point2f>(y,x);

            line(prvsL_drawn, cv::Point(x,y), cv::Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), cv::Scalar(255,0,0));
            circle(prvsL_drawn, cv::Point(x,y), radius, cv::Scalar (0,0,0), -1);
        }
    }
}
