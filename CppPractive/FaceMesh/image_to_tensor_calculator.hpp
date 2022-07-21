//
//  image_to_tensor_calculator.hpp
//  CppPractive
//
//  Created by Song on 2022/07/21.
//

#ifndef image_to_tensor_calculator_hpp
#define image_to_tensor_calculator_hpp

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


enum GpuOriginMode {

DEFAULT = 0,

// OpenGL: bottom-left origin
// Metal : top-left origin
CONVENTIONAL = 1,
// OpenGL: top-left origin
// Metal : top-left origin
TOP_LEFT = 2

};



enum BorderMode {
  BORDER_UNSPECIFIED = 0,
  BORDER_ZERO = 1,
  BORDER_REPLICATE = 2
};


class ImageToTensorCalculatorOptions {
private:
    float float_min ;
    float float_max ;
    int64_t int_min ;
    int64_t int_max ;

    uint64_t uInt_min ;
    uint64_t uInt_max ;
    
    int32_t output_tensor_width ;
    int32_t output_tensor_height ;
    
    bool keep_aspect_ratio ;

    
    // Output tensor element range/type image pixels are converted to.
//    oneof range {
//      FloatRange output_tensor_float_range = 4;
//      IntRange output_tensor_int_range = 7;
//      UIntRange output_tensor_uint_range = 8;
//    }
    GpuOriginMode gpu_origin;
    BorderMode border_mode;
    
    // calculator_options.proto
    bool merge_fields = 1; // [__deprecated = true];
    
    
public:
    ImageToTensorCalculatorOptions() {
        float_min = 1;
        float_max = 2;
        int_min = 1;
        int_max = 2;

        uInt_min = 1;
        uInt_max = 2;
        
        output_tensor_width = 1;
        output_tensor_height = 2;
        
        keep_aspect_ratio = 3;
        
//        gpu_origin = 5;
//        border_mode = 6;
    };
};





#endif /* image_to_tensor_calculator_hpp */
