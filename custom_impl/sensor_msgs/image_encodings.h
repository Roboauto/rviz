#pragma once

#include <string>

namespace sensor_msgs::image_encodings {
    const char RGB8[] = "rgb8";
    const char RGBA8[] = "rgba8";
    const char RGB16[] = "rgb16";
    const char RGBA16[] = "rgba16";
    const char BGR8[] = "bgr8";
    const char BGRA8[] = "bgra8";
    const char BGR16[] = "bgr16";
    const char BGRA16[] = "bgra16";
    const char MONO8[] = "mono8";
    const char MONO16[] = "mono16";

// OpenCV CvMat types
    const char TYPE_8UC1[] = "8UC1";
    const char TYPE_8UC2[] = "8UC2";
    const char TYPE_8UC3[] = "8UC3";
    const char TYPE_8UC4[] = "8UC4";
    const char TYPE_8SC1[] = "8SC1";
    const char TYPE_8SC2[] = "8SC2";
    const char TYPE_8SC3[] = "8SC3";
    const char TYPE_8SC4[] = "8SC4";
    const char TYPE_16UC1[] = "16UC1";
    const char TYPE_16UC2[] = "16UC2";
    const char TYPE_16UC3[] = "16UC3";
    const char TYPE_16UC4[] = "16UC4";
    const char TYPE_16SC1[] = "16SC1";
    const char TYPE_16SC2[] = "16SC2";
    const char TYPE_16SC3[] = "16SC3";
    const char TYPE_16SC4[] = "16SC4";
    const char TYPE_32SC1[] = "32SC1";
    const char TYPE_32SC2[] = "32SC2";
    const char TYPE_32SC3[] = "32SC3";
    const char TYPE_32SC4[] = "32SC4";
    const char TYPE_32FC1[] = "32FC1";
    const char TYPE_32FC2[] = "32FC2";
    const char TYPE_32FC3[] = "32FC3";
    const char TYPE_32FC4[] = "32FC4";
    const char TYPE_64FC1[] = "64FC1";
    const char TYPE_64FC2[] = "64FC2";
    const char TYPE_64FC3[] = "64FC3";
    const char TYPE_64FC4[] = "64FC4";

// Bayer encodings
    const char BAYER_RGGB8[] = "bayer_rggb8";
    const char BAYER_BGGR8[] = "bayer_bggr8";
    const char BAYER_GBRG8[] = "bayer_gbrg8";
    const char BAYER_GRBG8[] = "bayer_grbg8";
    const char BAYER_RGGB16[] = "bayer_rggb16";
    const char BAYER_BGGR16[] = "bayer_bggr16";
    const char BAYER_GBRG16[] = "bayer_gbrg16";
    const char BAYER_GRBG16[] = "bayer_grbg16";
}