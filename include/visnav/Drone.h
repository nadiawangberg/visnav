#pragma once
#include <opencv2/opencv.hpp> 


class Drone {
private:
	double x;
	double y;
	double roll;
	double pitch;
	double yaw;

public:
	Drone();
	void findPose();
}