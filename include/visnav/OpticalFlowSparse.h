#pragma once 

#include <opencv2/opencv.hpp>

namespace vis_nav {

	class OpticalFlowSparse {
		
	public:
		
		OpticalFlowSparse(int resetCount, int numFeatures, double featureQuality = 0.1, double minDistance = 3.5, double minTrackSuccessRate = 0.8);
		~OpticalFlowSparse();
		
		void updateWithNewImage(const cv::Mat& imageNext);
		
		void updateFeatureSet();
		void updateOpticalFlow();
		
		std::vector<cv::Point2f> getInitialSet() const {return m_featuresInit;}
		std::vector<cv::Point2f> getCurrentSet() const {return m_featuresNext;}
		std::vector<float> getError() const {return m_error;}
		std::vector<uchar> getStatus() const {return m_status;}
		
		bool hasFeatures() const;
		
		double getTrackSuccessRatio() const {return m_trackSuccessRatio;}
		int getStepCounter() const {return m_stepCounter;}

		std::vector<cv::Point2f> getPrevFeatures() const {return m_featuresLast;}
		std::vector<cv::Point2f> getNextFeatures() const {return m_featuresNext;}
		
	private:
		
		int m_resetCount;
		int m_numFeatures;
		
		double m_featureQuality;
		double m_minDistance;
		double m_minTrackSuccessRate;
		
		int m_stepCounter;
		bool m_hasFeatures;
		double m_trackSuccessRatio;
		
		bool m_isFirst;
		
		cv::Mat m_imageLast;
		cv::Mat m_imageNext;
		
		std::vector<cv::Point2f> m_featuresInit;
		std::vector<cv::Point2f> m_featuresLast;
		std::vector<cv::Point2f> m_featuresNext;
		
		std::vector<uchar> m_status;
		std::vector<float> m_error;
	};
	
	
	

}