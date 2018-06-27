#include <visnav/CameraPose.h>



CameraPose::CameraPose(double k[3][3], double d[4][1], cv::Size dim) : K(cv::Mat(3, 3, CV_64F, k)),  D(cv::Mat(4, 1, CV_64F, d)){
	this->focal = (K.at<double>(0,0) + K.at<double>(1,1))/2;
	this->pp.x = K.at<double>(0,2);
	this->pp.y = K.at<double>(1,2);
	this->DIM = dim;
	this->t_f = cv::Mat(3,1, CV_64F);
	this->R_f = cv::Mat::eye(3,3, CV_64F);
}

void CameraPose::updatePose(std::vector<cv::Point2f> prevPts, std::vector<cv::Point2f> nextPts) {
 	// find R and t from homography (H)
 	
 	/*
 	std::vector<cv::Mat> rotations, translations, normals; // will consist of 4 matricies, with one of them being the correct one xD
	cv::Mat H = findHomography(prevPts, nextPts, CV_RANSAC); // remove CV_RANSAC to use all feature points, se documentation for more parameter choices
	decomposeHomographyMat(H, K, rotations, translations, normals);
	*/

	cv::Mat E, R, t, mask; // mask : output mask for ransac
  	E = findEssentialMat(nextPts, prevPts, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

    recoverPose(E, nextPts, prevPts, R, t, focal, pp, mask);

    if ((t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
      t_f = t_f + R_f*t;
      R_f = R*R_f;
	}

    std::cout << "ATTEMPTING TO PRINT OUT SHIT!!!" << std::endl;
    std::cout << "x: " << int(t_f.at<double>(0)) << std::endl;
    std::cout << "y???: " << int(t_f.at<double>(1)) << std::endl;
    std::cout << "y: " << int(t_f.at<double>(2)) << std::endl;

}