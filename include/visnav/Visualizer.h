#pragma once 

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <visnav/ImageLoader.h>
#include <visnav/PixelPoint.h>


namespace vis_nav {
  
  
	class Visualizer {
	
	public:
	  
		Visualizer(int width, int height, bool useColor);
		~Visualizer();
		
		void drawImageCross(cv::Scalar color, int lineThickness, int lineType);
		
		void drawArucoMarkers(std::vector< std::vector<cv::Point2f>> markerCorners, std::vector<int> markerIds);
		
		void drawOpticalFlow(std::vector<cv::Point2f> featuresOld, std::vector<cv::Point2f> featuresNew, std::vector<float> error, 
				std::vector<uchar> status, cv::Scalar colorGood, cv::Scalar colorBad, int radius, int thickness, int lineType);
		
		void drawCirclesOnSet(std::vector<cv::Point2f> set, int radius, cv::Scalar color, int thickness, int lineType);
		
		void drawLinesBetweenSets(std::vector<cv::Point2f> setPre, std::vector<cv::Point2f> setNext, cv::Scalar color, int lineThickness, int lineType);
// 		void drawKeyPoints(std::vector<cv::KeyPoint> detections, cv::Scalar& color);
		
		void drawCircle(cv::Point center, int radius, cv::Scalar color, int thickness, int lineType);
		void drawLine(cv::Point startPoint, cv::Point endPoint, cv::Scalar color, int thickness, int lineType);
		
		void update(const cv::Mat& backgroundImage);
		
		cv::Mat getDrawImage();
		
	private:
		bool m_hasBackground;
		int m_width;
		int m_height;
		bool m_useColor;
		
		cv::Mat m_drawImage;
		
		
	
	};



}