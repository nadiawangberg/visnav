#include <visnav/Drone.h>



void Drone::findPose(cv::Mat prevPts, cv::Mat nextPts) {
	// find homography

	// pts_src and pts_dst are vectors of points in source 
    // and destination images. They are of type vector<Point2f>. 
    // We need at least 4 corresponding points. 
 
	cv::Mat H = findHomography(prevPts, nextPts, CV_RANSAC); // remove CV_RANSAC to use all feature points, se documentation for more parameter choices

		// find pose from homography
}