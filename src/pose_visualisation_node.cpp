#include "visnav/PoseVisualisation.h"

void poseUpdateCallback(const geometry_msgs::Point::ConstPtr& point) {
	//cv::Point movement = cv::Point(0,0);
	//if (vel->linear.x > 0)


	ROS_INFO("Pose_Visualizer: Updating position to x:%f, y:%f, z:%f", point->x, point->y, point->z);
	vis.drawCircle(cv::Point(point->x - i, point->y), 1, CV_RGB(255,0,0), 2, 1);
	//cv::imshow("Trajectory", vis.getDrawImage());
	i--;
	vis.drawCircle(cv::Point(point->x - i, point->y), 1, CV_RGB(0,255,0), 2, 1);

	cv_bridge::CvImage msg;
	msg.header = std_msgs::Header();
	msg.encoding = "bgr8"; //Might be subject to change
	msg.image = vis.getDrawImage();
	outputImagePub.publish(msg.toImageMsg());

}

int main(int argc, char **argv) {


	ros::init(argc, argv, "pose_visualizer");

	ros::NodeHandle n;
	ros::param::param<std::string>("output_image", outputImageTopic, "/visnav/image_output");
	outputImagePub = n.advertise<sensor_msgs::Image>(outputImageTopic, 1);	

	vis.update(cv::Mat::zeros(600, 600, CV_8UC3));
	
	ros::Subscriber sub = n.subscribe("position_output", 1000, poseUpdateCallback); //Change string to name of topic where the position is sent

	ros::spin();

	return 0;

}