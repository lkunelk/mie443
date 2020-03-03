#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <vector>

class Boxes {
	public:
		std::vector<cv::Mat> templates;
		std::vector<std::vector<float>> coords;
		std::vector<std::string> labels;
	public:
		bool load_coords();
		bool load_templates();
};
