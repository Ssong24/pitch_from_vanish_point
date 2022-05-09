//
//  pitch_btw_imgs.cpp
//  CppPractive
//
//  Created by Song on 2022/05/06.
//

#include "pitch_btw_imgs.hpp"




void demo_find_pitch_btw_up_and_center() {
    std::string root_lens_dir;
    std::string imgExt;
    
    set_dataset_params(root_lens_dir, imgExt);
    cv::Mat up_img      = cv::imread(root_lens_dir + "/u_1" + imgExt);
    cv::Mat center_img  = cv::imread(root_lens_dir + "/c_1" + imgExt);
    
    
    
    
    
}

void demo_find_pitch_btw_center_and_down() {
    
}
