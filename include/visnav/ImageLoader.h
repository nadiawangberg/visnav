#pragma once

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ros/file_log.h>

namespace vis_nav {

class ImageLoader {

public:

	ImageLoader(bool doBlackAndWhite = false);
	~ImageLoader();
	
	void imageCallback(const sensor_msgs::Image& msg);
	void imageCallbackSave(const sensor_msgs::Image& msg);
	
	cv_bridge::CvImagePtr getImage() {return m_image;}
	cv::Mat getImageMatrix(int type);
	
	bool hasImage() const {return m_hasImage;}
	int getImageWidth() const {return m_imageWidth;}
	int getImageHeight() const {return m_imageHeight;}
	
	void setOutputPath(std::string name);
	void setHasProcessedImage() {m_hasImage = false;}
	
private:

	bool m_doBlackAndWhite;
	bool m_hasImage;
	int m_imageWidth;
	int m_imageHeight;
	int m_imageCount;
	std::string m_outputPath;
	cv_bridge::CvImagePtr m_image;

	
};

}