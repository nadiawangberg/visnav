#include <visnav/Drone.h>



Drone::Drone(double** k) : x(0), y(0), roll(0), pitch(0), yaw(0) {
	K = cv::Mat(3, 3, CV_64F, k);
}

void Drone::findPose(std::vector<cv::Point2f> prevPts, std::vector<cv::Point2f> nextPts) {



 	// find R and t from homography (H)
 	
 	std::vector<cv::Mat> rotations, translations, normals; // will consist of 4 matricies, with one of them being the correct one xD
	cv::Mat H = findHomography(prevPts, nextPts, CV_RANSAC); // remove CV_RANSAC to use all feature points, se documentation for more parameter choices
	decomposeHomographyMat(H, K, rotations, translations, normals);
	


	// find R and t from essential matrix (E)
	/*
	double focal = 338.6458; // focal lenght of camera (average of f_x and f_y)
	cv::Point2d pp(278.594296413926, 207.64080164237097); // principle point of camera (x_0, y_0)
	cv::Mat E, R, t, mask; // mask : output mask for ransac
  	E = findEssentialMat(nextPts, prevPts, focal, pp, RANSAC, 0.999, 1.0, mask); // CV_RANSAC = RANSAC
    recoverPose(E, nextPts, prevPts, R, t, focal, pp, mask);
    */

}