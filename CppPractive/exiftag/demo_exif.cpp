//
//  demo_exif.cpp
//  CppPractive
//
//  Created by Song on 2022/05/11.
//

#include "demo_exif.hpp"

#include "exif.hpp"


void find_file_size(FILE *fp, unsigned long& fsize) {
    fseek(fp, 0, SEEK_END);
    fsize = ftell(fp);
    rewind(fp);
}

bool jpeg_file_info_to_buffer(unsigned long fsize, FILE* fp, unsigned char& buf) {
    if (fread(&buf, 1, fsize, fp) != fsize) {
        printf("Can't read file.\n");
        delete[] &buf;
        return false;
    }
    return true;
}

void fl_mm_to_pix(double& f_pix, double f_mm, int image_width,  int sensor_size) {

    f_pix = f_mm * image_width / (double)sensor_size;
}

void find_fl_in_35mm(std::string image_path, double& focal_length) {
    const char * path_ = image_path.c_str();
    FILE *fp;
    unsigned long fsize;
    unsigned char* buf;
    
    if (image_path.length() == 0) {
        printf("Please put the image path\n");
        return ;
    }
    
    fp = fopen(path_ /*argv[1]*/ , "rb");
    if (!fp) {
        printf("Can't open file.\n");
        return ;
    }
    
    find_file_size(fp, fsize);
    
    buf = new unsigned char[fsize];
    if (!jpeg_file_info_to_buffer(fsize, fp, *buf)) {  //*buf address
        return ;
    }
    
    fclose(fp);
    
    easyexif::EXIFInfo result;
    int code = result.parseFrom(buf, fsize);
    delete[] buf;
    if (code) {
        printf("Error parsing EXIF: code %d\n", code);
        return ;
    }
    
//    float fl_in_35mm = result.FocalLengthIn35mm;
//    float sensor_width = 35.0;
//    float sensor_height = 24.0;
//    int image_width = result.ImageWidth;
    
    focal_length = (double)result.FocalLengthIn35mm; // * image_width / 35.0 ;
}


int demo_exiftag(/*int argc, char **argv*/)
{
    std::string src_dir =  "/Users/3i-21-331/workspace/stitching/mobile_stitching/dataset/PivoX_full";
    std::string image_path = src_dir + "/galaxy_zflip3/WideLens_20deg/correct_capture_1/c_4.jpg";
    // "/iphone_pro_12/Widelens_25deg/c_2.JPG";
    // "//galaxy_zflip3/WideLens_20deg/correct_capture_1/c_4.jpg";

    const char * path_ = image_path.c_str();
    
    if (image_path.length() == 0) {
        printf("Please put the image path\n");
        return -1;
    }
//    if (argc < 2) {
//        printf("Usage: demo <JPEG file>\n");
//        return -1;
//    }

    // Read the JPEG file into a buffer
    FILE *fp = fopen(path_ /*argv[1]*/ , "rb");
    if (!fp) {
        printf("Can't open file.\n");
        return -1;
    }

    unsigned long fsize;
    unsigned char* buf;
    
    find_file_size(fp, fsize);
    
    buf = new unsigned char[fsize];
    if (!jpeg_file_info_to_buffer(fsize, fp, *buf)) {  //*buf address
        return -2;
    }
    
    fclose(fp);

    // Parse EXIF
    easyexif::EXIFInfo result;
    int code = result.parseFrom(buf, fsize);
    delete[] buf;
    if (code) {
        printf("Error parsing EXIF: code %d\n", code);
        return -3;
    }

    // Dump EXIF information
    printf("Camera make          : %s\n", result.Make.c_str());
    printf("Camera model         : %s\n", result.Model.c_str());
    printf("Software             : %s\n", result.Software.c_str());
    printf("Bits per sample      : %d\n", result.BitsPerSample);
    printf("Image width          : %d\n", result.ImageWidth);
    printf("Image height         : %d\n", result.ImageHeight);
    printf("Image description    : %s\n", result.ImageDescription.c_str());
    printf("Image orientation    : %d\n", result.Orientation);
    printf("Image copyright      : %s\n", result.Copyright.c_str());
    printf("Image date/time      : %s\n", result.DateTime.c_str());
    printf("Original date/time   : %s\n", result.DateTimeOriginal.c_str());
    printf("Digitize date/time   : %s\n", result.DateTimeDigitized.c_str());
    printf("Subsecond time       : %s\n", result.SubSecTimeOriginal.c_str());
    printf("Exposure time        : 1/%d s\n",
         (unsigned)(1.0 / result.ExposureTime));
    printf("F-stop               : f/%.1f\n", result.FNumber);
    printf("Exposure program     : %d\n", result.ExposureProgram);
    printf("ISO speed            : %d\n", result.ISOSpeedRatings);
    printf("Subject distance     : %f m\n", result.SubjectDistance);
    printf("Exposure bias        : %f EV\n", result.ExposureBiasValue);
    printf("Flash used?          : %d\n", result.Flash);
    printf("Flash returned light : %d\n", result.FlashReturnedLight);
    printf("Flash mode           : %d\n", result.FlashMode);
    printf("Metering mode        : %d\n", result.MeteringMode);
    printf("Lens focal length    : %f mm\n", result.FocalLength);
    printf("35mm focal length    : %u mm\n", result.FocalLengthIn35mm);
    printf("GPS Latitude         : %f deg (%f deg, %f min, %f sec %c)\n",
         result.GeoLocation.Latitude, result.GeoLocation.LatComponents.degrees,
         result.GeoLocation.LatComponents.minutes,
         result.GeoLocation.LatComponents.seconds,
         result.GeoLocation.LatComponents.direction);
    printf("GPS Longitude        : %f deg (%f deg, %f min, %f sec %c)\n",
         result.GeoLocation.Longitude, result.GeoLocation.LonComponents.degrees,
         result.GeoLocation.LonComponents.minutes,
         result.GeoLocation.LonComponents.seconds,
         result.GeoLocation.LonComponents.direction);
    printf("GPS Altitude         : %f m\n", result.GeoLocation.Altitude);
    printf("GPS Precision (DOP)  : %f\n", result.GeoLocation.DOP);
    printf("Lens min focal length: %f mm\n", result.LensInfo.FocalLengthMin);
    printf("Lens max focal length: %f mm\n", result.LensInfo.FocalLengthMax);
    printf("Lens f-stop min      : f/%.1f\n", result.LensInfo.FStopMin);
    printf("Lens f-stop max      : f/%.1f\n", result.LensInfo.FStopMax);
    printf("Lens make            : %s\n", result.LensInfo.Make.c_str());
    printf("Lens model           : %s\n", result.LensInfo.Model.c_str());
    printf("Focal plane XRes     : %f\n", result.LensInfo.FocalPlaneXResolution);
    printf("Focal plane YRes     : %f\n", result.LensInfo.FocalPlaneYResolution);

    // fl [pix] = fl [mm] * ImageWidth [pix] / sensorWidth[mm]
    printf("Focal length [pix] = %.2f\n", result.FocalLengthIn35mm * result.ImageWidth / 35.0 );
    return 0;
}
