#pragma once 

#include <opencv2/opencv.hpp>

namespace vis_nav {

class PixelPoint {
	
public:
	
	PixelPoint(int x, int y);
	~PixelPoint();

	int getX() const {return m_x;}
	int getY() const {return m_y;}
	
	int& getX() {return m_x;}
	int& getY() {return m_y;}
	
	cv::Point toPoint() const;
	
	double distanceTo(const PixelPoint& p);
	
private:
	
	int m_x;
	int m_y;
	
};


}