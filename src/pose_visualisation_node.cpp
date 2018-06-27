#include "visnav/PoseVisualisation.h"

void poseUpdateCallback(const geometry_msgs::Point::ConstPtr& point) {
	ROS_INFO("Pose_Visualizer: Updating position to x:%f, y:%f, z:%f", point->x, point->y, point->z);
	vis.drawCircle(cv::Point(point->x, point->y), i, CV_RGB(255,0,0), 5, 1);
	cv::imshow("Trajectory", vis.getDrawImage());
	i--;
	cv::imshow("Black box", cv::Mat::zeros(600, 600, CV_8UC3));

	cv_bridge::CvImage msg;
	msg.header = std_msgs::Header();
	msg.encoding = "bgr8"; //Mght be wrong
	msg.image = vis.getDrawImage();
	outputImagePub.publish(msg.toImageMsg());

}

int main(int argc, char **argv) {


	ros::init(argc, argv, "pose_visualizer");

	ros::NodeHandle n;
	ros::param::param<std::string>("output_image", outputImageTopic, "/visnav/image_output");
	outputImagePub = n.advertise<sensor_msgs::Image>(outputImageTopic, 1);	

	vis.update(cv::Mat::zeros(600, 600, CV_8UC3));
	
	ros::Subscriber sub = n.subscribe("position_output", 1000, poseUpdateCallback);

	ros::spin();

	return 0;

}