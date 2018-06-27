#pragma once
#include <opencv2/opencv.hpp>


class CameraPose {
private:
	cv::Mat R_f; // current rotation
	cv::Mat t_f; // current relative translation from init position
	

	cv::Mat K; // intrinsic camera calibration matrix
	cv::Mat D;
	cv::Size DIM;
	double focal; // focal lenght of camera (average of f_x and f_y)
	cv::Point2d pp; // principle point of camera (x_0, y_0)

public:
	CameraPose(double k[3][3], double d[4][1], cv::Size dim); // k[3][3], d[4][1]
	void updatePose(std::vector<cv::Point2f> prevPts, std::vector<cv::Point2f> nextPts);
	double getFocal() const {return focal;}
	cv::Point2d getpp() const {return pp;};
	cv::Mat getK() const {return K; };
	cv::Mat getD() const {return D;};
	cv::Mat getR_f() const {return R_f;};
	cv::Mat gett_f() const {return t_f;};
	cv::Size getDIM() const {return DIM;};
};