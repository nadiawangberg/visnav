
#include <visnav/PixelPoint.h>

namespace vis_nav {

PixelPoint::PixelPoint(int x, int y) 
	: m_x(x)
	, m_y(y) {
}

PixelPoint::~PixelPoint() {

}

cv::Point PixelPoint::toPoint() const {
	cv::Point p;
	p.x = m_x;
	p.y = m_y;
	return p;
}

double PixelPoint::distanceTo(const PixelPoint& p) {
	double x = p.getX() - m_x;
	double y = p.getY() - m_y;
	return std::sqrt(x*x + y*y);
}


}

