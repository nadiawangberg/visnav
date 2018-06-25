#pragma once
#include <opencv2/opencv.hpp> 


class Drone {
private:
	double x;
	double y;
	double roll;
	double pitch;
	double yaw;
	cv::Mat K;

public:
	Drone(double** k);
	void findPose(std::vector<cv::Point2f> prevPts, std::vector<cv::Point2f> nextPts);
};