#pragma once

#include <opencv2/opencv.hpp>

namespace vis_nav {

enum class Color {
	RED = 1,
	BLUE = 2,
	GREEN = 3,
	CYAN = 4,
	YELLOW = 5,
	MAGENTA = 6
};
  
class ColorManager {
  
public:
	ColorManager();
	~ColorManager();
	
	cv::Scalar getCVColor(const Color& color);
	
private:

};




}