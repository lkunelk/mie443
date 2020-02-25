#pragma once
#include <cv.h>

class Descriptor {
    public:
        int compareImages( cv::Mat img_object, cv::Mat img_scene );
};