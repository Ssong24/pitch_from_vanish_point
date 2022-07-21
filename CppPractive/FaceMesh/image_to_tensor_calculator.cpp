//
//  image_to_tensor_calculator.cpp
//  CppPractive
//
//  Created by Song on 2022/07/21.
//

#include <array>
#include <memory>
#include <vector>



#include "image_to_tensor_calculator.hpp"

//#include "mediapipe/calculators/tensor/image_to_tensor_calculator.pb.h"
#include "image_to_tensor_converter.h"
#include "image_to_tensor_utils.h"
//#include "mediapipe/framework/api2/node.h"
#include "calculator_framework.h" // "mediapipe/framework/calculator_framework.h"
#include "calculator_contract.h"


#include "image.h"          // "mediapipe/framework/formats/image.h"
#include "image_frame.h"    //mediapipe/framework/formats/image_frame.h"
//#include "mediapipe/framework/formats/rect.pb.h"
#include "tensor.h"         // "mediapipe/framework/formats/tensor.h"


#include "port.h"  // "mediapipe/framework/port.h"
#include "canonical_errors.h"
#include "ret_check.h"
#include "status.h"
#include "statusor.h"
//#include "mediapipe/gpu/gpu_origin.pb.h"

//#if !MEDIAPIPE_DISABLE_OPENCV
//#include "mediapipe/calculators/tensor/image_to_tensor_converter_opencv.h"
//#endif
//
//#if !MEDIAPIPE_DISABLE_GPU
//#include "mediapipe/gpu/gpu_buffer.h"
//
//#if MEDIAPIPE_METAL_ENABLED
//#include "mediapipe/calculators/tensor/image_to_tensor_converter_metal.h"
//#include "mediapipe/gpu/MPPMetalHelper.h"
//#elif MEDIAPIPE_OPENGL_ES_VERSION >= MEDIAPIPE_OPENGL_ES_31
//#include "mediapipe/calculators/tensor/image_to_tensor_converter_gl_buffer.h"
//#include "mediapipe/gpu/gl_calculator_helper.h"
//#else
//#include "mediapipe/calculators/tensor/image_to_tensor_converter_gl_texture.h"
//#include "mediapipe/gpu/gl_calculator_helper.h"
//#endif  // MEDIAPIPE_METAL_ENABLED
//#endif  // !MEDIAPIPE_DISABLE_GPU

namespace mediapipe {
namespace api2 {

//#if MEDIAPIPE_DISABLE_GPU
//// Just a placeholder to not have to depend on mediapipe::GpuBuffer.
//using GpuBuffer = AnyType;
//#else
//using GpuBuffer = mediapipe::GpuBuffer;
//#endif  // MEDIAPIPE_DISABLE_GPU


class ImageToTensorCalculator  { // }: public Node {
 public:
//  static constexpr Input<
//      OneOf<mediapipe::Image, mediapipe::ImageFrame>>::Optional kIn{"IMAGE"};
//  static constexpr Input<GpuBuffer>::Optional kInGpu{"IMAGE_GPU"};
//  static constexpr Input<mediapipe::NormalizedRect>::Optional kInNormRect{
//      "NORM_RECT"};
//  static constexpr Output<std::vector<Tensor>> kOutTensors{"TENSORS"};
//  static constexpr Output<std::array<float, 4>>::Optional kOutLetterboxPadding{
//      "LETTERBOX_PADDING"};
//  static constexpr Output<std::array<float, 16>>::Optional kOutMatrix{"MATRIX"};
//
//  MEDIAPIPE_NODE_CONTRACT(kIn, kInGpu, kInNormRect, kOutTensors,
//                          kOutLetterboxPadding, kOutMatrix);

  int UpdateContract(CalculatorContract* cc) {
    const auto& options =
        cc->Options<ImageToTensorCalculatorOptions>();

    RET_CHECK(options.has_output_tensor_float_range() ||
              options.has_output_tensor_int_range() ||
              options.has_output_tensor_uint_range())
        << "Output tensor range is required.";
    if (options.has_output_tensor_float_range()) {
      RET_CHECK_LT(options.output_tensor_float_range().min(),
                   options.output_tensor_float_range().max())
          << "Valid output float tensor range is required.";
    }
    if (options.has_output_tensor_uint_range()) {
      RET_CHECK_LT(options.output_tensor_uint_range().min(),
                   options.output_tensor_uint_range().max())
          << "Valid output uint tensor range is required.";
      RET_CHECK_GE(options.output_tensor_uint_range().min(), 0)
          << "The minimum of the output uint tensor range must be "
             "non-negative.";
      RET_CHECK_LE(options.output_tensor_uint_range().max(), 255)
          << "The maximum of the output uint tensor range must be less than or "
             "equal to 255.";
    }
    if (options.has_output_tensor_int_range()) {
      RET_CHECK_LT(options.output_tensor_int_range().min(),
                   options.output_tensor_int_range().max())
          << "Valid output int tensor range is required.";
      RET_CHECK_GE(options.output_tensor_int_range().min(), -128)
          << "The minimum of the output int tensor range must be greater than "
             "or equal to -128.";
      RET_CHECK_LE(options.output_tensor_int_range().max(), 127)
          << "The maximum of the output int tensor range must be less than or "
             "equal to 127.";
    }
    RET_CHECK_GT(options.output_tensor_width(), 0)
        << "Valid output tensor width is required.";
    RET_CHECK_GT(options.output_tensor_height(), 0)
        << "Valid output tensor height is required.";

    RET_CHECK(kIn(cc).IsConnected() ^ kInGpu(cc).IsConnected())
        << "One and only one of IMAGE and IMAGE_GPU input is expected.";

#if MEDIAPIPE_DISABLE_GPU
    if (kInGpu(cc).IsConnected()) {
      return absl::UnimplementedError(
          "GPU processing is disabled in build flags");
    }
#else  // !MEDIAPIPE_DISABLE_GPU
#if MEDIAPIPE_METAL_ENABLED
    MP_RETURN_IF_ERROR([MPPMetalHelper updateContract:cc]);
#else
    MP_RETURN_IF_ERROR(mediapipe::GlCalculatorHelper::UpdateContract(cc));
#endif  // MEDIAPIPE_METAL_ENABLED
#endif  // MEDIAPIPE_DISABLE_GPU

    return absl::OkStatus();
  }

  int Open(CalculatorContext* cc) {
    options_ = cc->Options<mediapipe::ImageToTensorCalculatorOptions>();
    output_width_ = options_.output_tensor_width();
    output_height_ = options_.output_tensor_height();
    is_float_output_ = options_.has_output_tensor_float_range();
    if (options_.has_output_tensor_uint_range()) {
      range_min_ =
          static_cast<float>(options_.output_tensor_uint_range().min());
      range_max_ =
          static_cast<float>(options_.output_tensor_uint_range().max());
    } else if (options_.has_output_tensor_int_range()) {
      range_min_ = static_cast<float>(options_.output_tensor_int_range().min());
      range_max_ = static_cast<float>(options_.output_tensor_int_range().max());
    } else {
      range_min_ = options_.output_tensor_float_range().min();
      range_max_ = options_.output_tensor_float_range().max();
    }
    return 0;
  }

  int Process(CalculatorContext* cc) {
    if ((kIn(cc).IsConnected() && kIn(cc).IsEmpty()) ||
        (kInGpu(cc).IsConnected() && kInGpu(cc).IsEmpty())) {
      // Timestamp bound update happens automatically.
      return absl::OkStatus();
    }

    absl::optional<mediapipe::NormalizedRect> norm_rect;
    if (kInNormRect(cc).IsConnected()) {
      if (kInNormRect(cc).IsEmpty()) {
        // Timestamp bound update happens automatically. (See Open().)
        return 0;
      }
      norm_rect = *kInNormRect(cc);
      if (norm_rect->width() == 0 && norm_rect->height() == 0) {
        // WORKAROUND: some existing graphs may use sentinel rects {width=0,
        // height=0, ...} quite often and calculator has to handle them
        // gracefully by updating timestamp bound instead of returning failure.
        // Timestamp bound update happens automatically. (See Open().)
        // NOTE: usage of sentinel rects should be avoided.
        DLOG(WARNING)
            << "Updating timestamp bound in response to a sentinel rect";
          return 0;
      }
    }

    ASSIGN_OR_RETURN(auto image, GetInputImage(cc));
    const Size size{image->width(), image->height()};
    RotatedRect roi = GetRoi(size.width, size.height, norm_rect);
    ASSIGN_OR_RETURN(auto padding, PadRoi(options_.output_tensor_width(),
                                          options_.output_tensor_height(),
                                          options_.keep_aspect_ratio(), &roi));
    if (kOutLetterboxPadding(cc).IsConnected()) {
      kOutLetterboxPadding(cc).Send(padding);
    }
    if (kOutMatrix(cc).IsConnected()) {
      std::array<float, 16> matrix;
      GetRotatedSubRectToRectTransformMatrix(roi, size.width, size.height,
                                             /*flip_horizontaly=*/false,
                                             &matrix);
      kOutMatrix(cc).Send(std::move(matrix));
    }

    // Lazy initialization of the GPU or CPU converter.
    MP_RETURN_IF_ERROR(InitConverterIfNecessary(cc, *image.get()));

    ASSIGN_OR_RETURN(Tensor tensor,
                     (image->UsesGpu() ? gpu_converter_ : cpu_converter_)
                         ->Convert(*image, roi, {output_width_, output_height_},
                                   range_min_, range_max_));

    auto result = std::make_unique<std::vector<Tensor>>();
    result->push_back(std::move(tensor));
    kOutTensors(cc).Send(std::move(result));

      return 0;
  }

 private:
  bool DoesGpuInputStartAtBottom() {
    return options_.gpu_origin() != mediapipe::GpuOrigin_Mode_TOP_LEFT;
  }

  BorderMode GetBorderMode() {
    switch (options_.border_mode()) {
      case mediapipe::
          ImageToTensorCalculatorOptions_BorderMode_BORDER_UNSPECIFIED:
        return BorderMode::kReplicate;
      case mediapipe::ImageToTensorCalculatorOptions_BorderMode_BORDER_ZERO:
        return BorderMode::kZero;
      case mediapipe::
          ImageToTensorCalculatorOptions_BorderMode_BORDER_REPLICATE:
        return BorderMode::kReplicate;
    }
  }

  Tensor::ElementType GetOutputTensorType() {
    if (is_float_output_) {
      return Tensor::ElementType::kFloat32;
    }
    if (range_min_ < 0) {
      return Tensor::ElementType::kInt8;
    } else {
      return Tensor::ElementType::kUInt8;
    }
  }

  absl::StatusOr<std::shared_ptr<const mediapipe::Image>> GetInputImage(
      CalculatorContext* cc) {
    if (kIn(cc).IsConnected()) {
      const auto& packet = kIn(cc).packet();
      return kIn(cc).Visit(
          [&packet](const mediapipe::Image&) {
            return SharedPtrWithPacket<mediapipe::Image>(packet);
          },
          [&packet](const mediapipe::ImageFrame&) {
            return std::make_shared<const mediapipe::Image>(
                std::const_pointer_cast<mediapipe::ImageFrame>(
                    SharedPtrWithPacket<mediapipe::ImageFrame>(packet)));
          });
    } else {  // if (kInGpu(cc).IsConnected())
#if !MEDIAPIPE_DISABLE_GPU
      const GpuBuffer& input = *kInGpu(cc);
      // A shallow copy is okay since the resulting 'image' object is local in
      // Process(), and thus never outlives 'input'.
      return std::make_shared<const mediapipe::Image>(input);
#else
      return absl::UnimplementedError(
          "GPU processing is disabled in build flags");
#endif  // !MEDIAPIPE_DISABLE_GPU
    }
  }

  absl::Status InitConverterIfNecessary(CalculatorContext* cc,
                                        const Image& image) {
    // Lazy initialization of the GPU or CPU converter.
    if (image.UsesGpu()) {
      if (!is_float_output_) {
        return absl::UnimplementedError(
            "ImageToTensorConverter for the input GPU image currently doesn't "
            "support quantization.");
      }
      if (!gpu_converter_) {
#if !MEDIAPIPE_DISABLE_GPU
#if MEDIAPIPE_METAL_ENABLED
        ASSIGN_OR_RETURN(gpu_converter_,
                         CreateMetalConverter(cc, GetBorderMode()));
#elif MEDIAPIPE_OPENGL_ES_VERSION >= MEDIAPIPE_OPENGL_ES_31
        ASSIGN_OR_RETURN(gpu_converter_,
                         CreateImageToGlBufferTensorConverter(
                             cc, DoesGpuInputStartAtBottom(), GetBorderMode()));
#else
        // Check whether the underlying storage object is a GL texture.
        if (image.GetGpuBuffer()
                .internal_storage<mediapipe::GlTextureBuffer>()) {
          ASSIGN_OR_RETURN(
              gpu_converter_,
              CreateImageToGlTextureTensorConverter(
                  cc, DoesGpuInputStartAtBottom(), GetBorderMode()));
        } else {
          return absl::UnimplementedError(
              "ImageToTensorConverter for the input GPU image is unavailable.");
        }
#endif  // MEDIAPIPE_METAL_ENABLED
#endif  // !MEDIAPIPE_DISABLE_GPU
      }
    } else {
      if (!cpu_converter_) {
#if !MEDIAPIPE_DISABLE_OPENCV
        ASSIGN_OR_RETURN(
            cpu_converter_,
            CreateOpenCvConverter(cc, GetBorderMode(), GetOutputTensorType()));
#else
        LOG(FATAL) << "Cannot create image to tensor opencv converter since "
                      "MEDIAPIPE_DISABLE_OPENCV is defined.";
#endif  // !MEDIAPIPE_DISABLE_OPENCV
      }
    }
    return absl::OkStatus();
  }

  std::unique_ptr<ImageToTensorConverter> gpu_converter_;
  std::unique_ptr<ImageToTensorConverter> cpu_converter_;
  ImageToTensorCalculatorOptions options_;
  int output_width_ = 0;
  int output_height_ = 0;
  bool is_float_output_ = false;
  float range_min_ = 0.0f;
  float range_max_ = 1.0f;
};

MEDIAPIPE_REGISTER_NODE(ImageToTensorCalculator);

}  // namespace api2
}  // namespace mediapipe
