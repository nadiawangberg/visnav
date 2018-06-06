#include <visnav/ColorManager.h>

namespace vis_nav {
  
ColorManager::ColorManager() {

}

ColorManager::~ColorManager()
{

}

cv::Scalar ColorManager::getCVColor(const Color& color){
	cv::Scalar col;
	
	switch (color) {   
	  case Color::RED :
		 col = cv::Scalar(255,0,0);
		break;
	  case Color::BLUE :
		 col = cv::Scalar(0,0,255);
		break;
	   
	  case Color::GREEN :
		 col = cv::Scalar(0,255,0);
		break;
	   
	  case Color::CYAN :
		 col = cv::Scalar(0,255,255);
		break;
	   
	  case Color::YELLOW :
		 col = cv::Scalar(255,255,0);
		break;
	   
	  case Color::MAGENTA :
		 col = cv::Scalar(255,0,255);
		break;
	}
	return col;
	  
}


  
  
}