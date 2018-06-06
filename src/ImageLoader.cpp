
#include <visnav/ImageLoader.h>

using namespace vis_nav;


ImageLoader::ImageLoader(bool doBlackAndWhite) 
	: m_doBlackAndWhite(doBlackAndWhite)
	, m_hasImage(false)
	, m_imageWidth(0)
	, m_imageHeight(0) 
	, m_imageCount(0) 
	, m_outputPath("/home/aleksander/pics/picture") {
}

ImageLoader::~ImageLoader() {

}

void ImageLoader::setOutputPath(std::string outputPath) {
	m_outputPath = outputPath;
}


void ImageLoader::imageCallback(const sensor_msgs::Image& msg) {
	try {

		if (m_doBlackAndWhite) {
 			m_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); 
		} else {
			m_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); 
		}
		m_imageWidth  = m_image->image.cols;
		m_imageHeight = m_image->image.rows;
		
		m_hasImage = true;
		
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what() );
		return;
	}
}

cv::Mat ImageLoader::getImageMatrix(int type) {

	cv::Mat out( m_imageHeight, m_imageWidth, type);
	if (m_hasImage) {
		m_image->image.convertTo(out, type);
	}
	return out;
	
}


void ImageLoader::imageCallbackSave(const sensor_msgs::Image& msg) {
	try {
		m_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); 
		
		m_imageWidth  = m_image->image.cols;
		m_imageHeight = m_image->image.rows;
		
		m_hasImage = true;
		
		
		std::stringstream sstream;                              
		sstream << m_outputPath << std::setfill('0') << std::setw(6) << m_imageCount << ".png" ;                 
		
		cv::imwrite( sstream.str(),  m_image->image );    
		
		m_imageCount++;
		
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what() );
		return;
	}
}
