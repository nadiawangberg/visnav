
#include <visnav/OpticalFlowSparse.h>

namespace vis_nav {
 
OpticalFlowSparse::OpticalFlowSparse(int resetCount, int numFeatures, double featureQuality, double minDistance, double minTrackSuccessRate) 
	: m_resetCount(resetCount) 
	, m_numFeatures(numFeatures) 
	, m_featureQuality(featureQuality)
	, m_minDistance(minDistance) 
	, m_minTrackSuccessRate(minTrackSuccessRate)
	, m_stepCounter(0) 
	, m_hasFeatures(false) 
	, m_trackSuccessRatio(1.0)
	, m_isFirst(true) {
}

OpticalFlowSparse::~OpticalFlowSparse() {

}

void OpticalFlowSparse::updateWithNewImage(const cv::Mat& imageNext) {
 
	if (m_isFirst) {

		cv::cvtColor(imageNext, m_imageNext, CV_BGR2GRAY);
		m_imageLast = m_imageNext.clone();
		m_isFirst = false;
		updateFeatureSet();
		return;
	} 
	
	cv::cvtColor(imageNext, m_imageNext, CV_BGR2GRAY);
	
	if ( (m_stepCounter < m_resetCount) && (m_trackSuccessRatio > m_minTrackSuccessRate) ) {

		updateOpticalFlow();
		m_stepCounter++;
	
	} else {
		m_imageLast = m_imageNext.clone();
		updateFeatureSet();
		m_stepCounter = 0;
	
	}
	
}


bool OpticalFlowSparse::hasFeatures() const {
	return m_hasFeatures;
}



void OpticalFlowSparse::updateFeatureSet() {
  
	m_hasFeatures = true;
	
	cv::goodFeaturesToTrack(m_imageNext, // the image 
		m_featuresInit,   // the output detected features
		m_numFeatures,  // the maximum number of features 
		m_featureQuality,     // quality level
		m_minDistance     // min distance between two features
	);
	
	
	m_featuresLast = m_featuresInit;
	m_trackSuccessRatio = 1.0;
}

  
void OpticalFlowSparse::updateOpticalFlow() {
	
	cv::calcOpticalFlowPyrLK(
		m_imageLast, m_imageNext, // 2 consecutive images
		m_featuresLast, // input point positions in first im
		m_featuresNext, // output point positions in the 2nd
		m_status,    // tracking success
		m_error      // tracking error
	);
	
	m_featuresLast = m_featuresNext;
	m_imageLast = m_imageNext.clone();
	
	int trackCount = 0;
	for (unsigned int idx = 0; idx < m_status.size(); idx++) {
		if ( m_status.at(idx) ) {
		      trackCount++;
		}
	}
	m_trackSuccessRatio = trackCount / (double) m_status.size();
}


  
  
}