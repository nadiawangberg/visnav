#include <cmath>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>

#include <std_msgs/Int64.h>
#include <sensor_msgs/Image.h>

#include <visnav/ImageLoader.h>
#include <visnav/Navigation.h>
#include <visnav/OpticalFlowSparse.h>

#include <visnav/Visualizer.h>
#include <visnav/ColorManager.h>

#include <opencv2/opencv.hpp>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include <sstream>

void poseUpdateCallback(const geometry_msgs::Point::ConstPtr& point) {
	ROS_INFO("Pose_Visualizer: Updating position to x:%f, y:%f, z:%f", point->x, point->y, point->z);
}

int main(int argc, char **argv) {


	ros::init(argc, argv, "pose_visualizer");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("position_output", 1000, poseUpdateCallback);

	ros::spin();

	return 0;

}