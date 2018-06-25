#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>

#include <sensor_msgs/Image.h>

#include <visnav/ImageLoader.h>

#include <opencv2/opencv.hpp>


int main(int argc, char** argv) {

	ros::init(argc, argv, "imagesave_node");
	ros::NodeHandle nH;
	
	ROS_INFO("IMAGESAVE: Starting up image saver node...");

	double loopRate;
	ros::param::param<double>("img_loop_rate", loopRate, 100.0);
	ROS_INFO("IMAGESAVE: Using loop rate of %f Hz", loopRate);
	ros::Rate rate(loopRate);
	
	//Image Loader 
	std::string imageTopic;
	ros::param::param<std::string>("imagesaver_image_topic", imageTopic, "/visnav/aruco_track");
	ROS_INFO("IMAGESAVE: Expecting images on topic: %s ", imageTopic.c_str());

	std::string outputPath;
	ros::param::param<std::string>("output_path", outputPath, "pictures/picture_");
	ROS_INFO("IMAGESAVE: Saving Images in : %s ", outputPath.c_str());
	
	vis_nav::ImageLoader imageLoader;
	imageLoader.setOutputPath(outputPath);
 	ros::Subscriber img_sub = nH.subscribe(imageTopic.c_str(),   100,  &vis_nav::ImageLoader::imageCallbackSave  , &imageLoader);
	
	
	while (imageLoader.hasImage() == false ) {
		ros::spinOnce();
		rate.sleep();
	}
	ROS_WARN("Imageloader ready!");
	
	int imageWidth  = imageLoader.getImageWidth();
	int imageHeight = imageLoader.getImageHeight();
	
	ROS_INFO("Image width:  %i ", imageWidth);
	ROS_INFO("Image height: %i ", imageHeight);

	
	ROS_INFO("IMAGESAVE: Image saver node is up and running");
 
	while (ros::ok() ) {
		
		/*
		 * Call all ROS subscribers 
		 */
		
		ros::spinOnce();
		
		rate.sleep();
		
		
	}	
	
	ROS_INFO("IMAGESAVE: Image saver node terminated");
	
	return 0;

}