
#include <visnav/Visualizer.h>

namespace vis_nav {

Visualizer::Visualizer(int width, int height, bool useColor) 
	: m_hasBackground(false) 
	, m_width(width)
	, m_height(height) 
	, m_useColor(useColor) {
}

Visualizer::~Visualizer() {
}

void Visualizer::drawOpticalFlow(std::vector< cv::Point2f > featuresOld, std::vector< cv::Point2f > 
	featuresNew, std::vector< float > error, std::vector< uchar > status, cv::Scalar colorGood, cv::Scalar colorBad, int radius, int thickness, int lineType) {
	
	int numPointsPrev = featuresOld.size();
	int numPointsNext = featuresNew.size();
	
	for ( int idx = 0; idx < std::min(numPointsNext, numPointsPrev); idx++) {
		
		if (status.at(idx) == 1) {
			drawCircle(featuresOld.at(idx), radius, colorGood, -1, lineType);
			drawLine(featuresOld.at(idx), featuresNew.at(idx), colorGood, thickness, lineType);
		} else {
			drawCircle(featuresOld.at(idx), radius, colorBad, -1, lineType);
			drawLine(featuresOld.at(idx), featuresNew.at(idx), colorBad,  thickness, lineType);
		}
	}
}


void Visualizer::drawCirclesOnSet(std::vector< cv::Point2f > set, int radius, cv::Scalar color, int thickness, int lineType) {

	for (unsigned int idx = 0; idx < set.size(); idx++) {
	  
		drawCircle(cvPoint(set[idx].x,set[idx].y), radius, color, thickness, lineType);
	}
  
}


void Visualizer::drawLinesBetweenSets(std::vector< cv::Point2f > setPre, std::vector< cv::Point2f > setNext, cv::Scalar color, 
				      int lineThickness, int lineType) {
	int numPointsPrev = setPre.size();
	int numPointsNext = setNext.size();
	
// 	if (numPointsPrev != numPointsNext) {
// 		return;
// 	}
	
	for (int idx = 0; idx < std::min(numPointsNext, numPointsPrev); idx++) {

		drawLine(setPre.at(idx), setNext.at(idx), color, lineThickness, lineType );
	
	}
}


void Visualizer::drawArucoMarkers(std::vector< std::vector< cv::Point2f > > markerCorners, std::vector< int > markerIds) {
	cv::aruco::drawDetectedMarkers(m_drawImage, markerCorners, markerIds);
}



// void Visualizer::drawKeyPoints(std::vector< cv::KeyPoint > detections, cv::Scalar& color) {
// 	cv::drawKeypoints( m_backgroundImage->image, detections, m_drawImage, color, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
// }



void Visualizer::drawCircle(cv::Point center, int radius, cv::Scalar color, int thickness, int lineType) {
	cv::circle(m_drawImage,
		center,
		radius,
		color,
		thickness,
		lineType,
	        0
	);
}

void Visualizer::drawLine(cv::Point startPoint, cv::Point endPoint, cv::Scalar color, int thickness, int lineType) {
	cv::line(m_drawImage,
		startPoint,
		endPoint,
		color,
		thickness,
		lineType,
	        0
	);
}

void Visualizer::drawImageCross(cv::Scalar color, int lineThickness, int lineType) {
		
		drawLine(cv::Point(0      , m_height / 2),
			 cv::Point(m_width, m_height / 2),
			 color,
			 lineThickness,
			 lineType
		);
		drawLine(cv::Point(m_width / 2, 0        ),
			 cv::Point(m_width / 2, m_height ),
			 color,
			 lineThickness,
			 lineType
		);		

}


void Visualizer::update(const cv::Mat& backgroundImage) {
	m_drawImage = backgroundImage.clone();

}

cv::Mat Visualizer::getDrawImage() {
	return m_drawImage;
}




}