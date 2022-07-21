//
//  image_format.pb.h
//  CppPractive
//
//  Created by Song on 2022/07/21.
//

#ifndef image_format_pb_h
#define image_format_pb_h

namespace ImageFormat {
  enum Format {
    // The format is unknown.  It is not valid for an ImageFrame to be
    // initialized with this value.
    UNKNOWN = 0,

    // sRGB, interleaved: one byte for R, then one byte for G, then one
    // byte for B for each pixel.
    SRGB = 1,

    // sRGBA, interleaved: one byte for R, one byte for G, one byte for B,
    // one byte for alpha or unused.
    SRGBA = 2,

    // Grayscale, one byte per pixel.
    GRAY8 = 3,

    // Grayscale, one uint16 per pixel.
    GRAY16 = 4,

    // YCbCr420P (1 bpp for Y, 0.25 bpp for U and V).
    // NOTE: NOT a valid ImageFrame format, but intended for
    // ScaleImageCalculatorOptions, VideoHeader, etc. to indicate that
    // YUVImage is used in place of ImageFrame.
    YCBCR420P = 5,

    // Similar to YCbCr420P, but the data is represented as the lower 10bits of
    // a uint16. Like YCbCr420P, this is NOT a valid ImageFrame, and the data is
    // carried within a YUVImage.
    YCBCR420P10 = 6,

    // sRGB, interleaved, each component is a uint16.
    SRGB48 = 7,

    // sRGBA, interleaved, each component is a uint16.
    SRGBA64 = 8,

    // One float per pixel.
    VEC32F1 = 9,

    // Two floats per pixel.
    VEC32F2 = 12,

    // LAB, interleaved: one byte for L, then one byte for a, then one
    // byte for b for each pixel.
    LAB8 = 10,

    // sBGRA, interleaved: one byte for B, one byte for G, one byte for R,
    // one byte for alpha or unused. This is the N32 format for Skia.
    SBGRA = 11
  };
}
#endif /* image_format_pb_h */
