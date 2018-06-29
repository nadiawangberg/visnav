
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


int main(int argc, char** argv) {

	ros::init(argc, argv, "visnav_node");
	ros::NodeHandle nH;
	
	ROS_INFO("VISNAV: Starting up visual navigation node...");
	
	double loopRate;
	ros::param::param<double>("img_loop_rate",   loopRate, 10.0);
	ROS_INFO("VISNAV: Using loop rate of %f Hz", loopRate);
	
	ros::Rate rate(loopRate);
	
	bool doPublishImage = false;
	ros::param::param<bool>("visnav_do_publish_output_image", doPublishImage, true);
	
	bool doWaitForNav = false;
	ros::param::param<bool>("visnav_do_wait_for_nav", doWaitForNav, false);
	
	bool useColor = false;
	ros::param::param<bool>("visnav_camera_color", useColor, false);

	double cameraRollOffSet  = 0.0;
	double cameraPitchOffSet = 0.0;
	ros::param::param<double>("visnav_camera_roll_offset", cameraRollOffSet, 0.0);
	ros::param::param<double>("visnav_camera_pitch_offset", cameraPitchOffSet, 0.0);
	cameraRollOffSet  = cameraRollOffSet  * ( M_PI / 180.0 );
	cameraPitchOffSet = cameraPitchOffSet * ( M_PI / 180.0 );
	ROS_INFO("CAMERA: Expected roll  off set: %f", cameraRollOffSet  );
	ROS_INFO("CAMERA: Expected pitch off set: %f", cameraPitchOffSet );
	
	
	std::string outputImageTopic;
	ros::param::param<std::string>("output_image", outputImageTopic, "/visnav/image_output");
	ros::Publisher outputImagePub = nH.advertise<sensor_msgs::Image>(outputImageTopic, 1);
	
	//Navigation object
	vis_nav::Navigator navigator;
	
	std::string localPosTopic;
	std::string globalPosTopic;
	std::string velocityTopic;
	std::string atitudeTopic;
	std::string altitudeTopic;
	
	ros::param::param<std::string>("navigation_localpos_topic",  localPosTopic,  "/mavros/local_position/pose");
	ros::param::param<std::string>("navigation_globalpos_topic", globalPosTopic, "/mavros/global_position/global");
	ros::param::param<std::string>("navigation_velocity_topic",  velocityTopic,  "/mavros/local_position/velocity");
	ros::param::param<std::string>("navigation_atitude_topic",   atitudeTopic,   "/mavros/imu/data");
	ros::param::param<std::string>("navigation_altitude_topic",  altitudeTopic,  "/mavros/vfr_hud");
	
	ros::Subscriber gloPosSub = nH.subscribe(globalPosTopic, 1, &vis_nav::Navigator::pos_callback,    &navigator);
	ros::Subscriber velSub    = nH.subscribe(velocityTopic,  1, &vis_nav::Navigator::vel_callback,    &navigator);
	ros::Subscriber atiSub    = nH.subscribe(atitudeTopic,   1, &vis_nav::Navigator::imu_callback,    &navigator);
	ros::Subscriber altSub    = nH.subscribe(altitudeTopic,  1, &vis_nav::Navigator::alt_callback,    &navigator);
	
	ROS_INFO("NAV: Listening for local position on: %s", localPosTopic.c_str() );
	ROS_INFO("NAV: Listening for global position on: %s", globalPosTopic.c_str() );
	ROS_INFO("NAV: Listening for velocity on: %s", velocityTopic.c_str() );
	ROS_INFO("NAV: Listening for imu data on: %s", atitudeTopic.c_str() );
	ROS_INFO("NAV: Listening for altitude on: %s", altitudeTopic.c_str() );
	
	//Image Loader 
	std::string imageTopic;
	ros::param::param<std::string>("image_topic", imageTopic, "/camera/image_raw");
	ROS_INFO("VISNAV: Expecting images on topic: %s ", imageTopic.c_str());
	
	vis_nav::ImageLoader imageLoader;
 	ros::Subscriber img_sub = nH.subscribe(imageTopic.c_str(),   1,  &vis_nav::ImageLoader::imageCallback  , &imageLoader);
	
	std::string visNavStatus;
	ros::param::param<std::string>("visnav_status", visNavStatus, "/visnav/status");
	ros::Publisher visNavStatusPublisher = nH.advertise<std_msgs::Int64>(visNavStatus, 5, true);
	
	while (navigator.checkNavigationStatus() == false && doWaitForNav) {
		ros::spinOnce();
		rate.sleep();
	}
	ROS_WARN("Navigation ready!");
	
	while (imageLoader.hasImage() == false ) {
		ros::spinOnce();
		rate.sleep();
	}
	ROS_WARN("Imageloader ready!");
	
	int imageWidth  = imageLoader.getImageWidth();
	int imageHeight = imageLoader.getImageHeight();
	
	ROS_INFO("Image width:  %i ", imageWidth);
	ROS_INFO("Image height: %i ", imageHeight);
	
	/*
	 * 		Setting up Optical flow module
	 */
	
	int resetCount = 100;
	ros::param::param<int>("opflow_reset_count", resetCount, 100);
	int numFeatures = 100;
	ros::param::param<int>("opflow_feature_count", numFeatures, 100);
	double featureQuality = 0.1;
	ros::param::param<double>("opflow_feature_qual", featureQuality, 0.10);
	double minDistance = 1.0;
	ros::param::param<double>("opflow_feature_dist_sep", minDistance, 1.0);
	double minTrackSuccessRate = 0.8;
	ros::param::param<double>("opflow_track_success_rate", minTrackSuccessRate, 0.80);
	
	//Optical Flow object
	vis_nav::OpticalFlowSparse opticalFlowSparse( resetCount,  numFeatures,  featureQuality ,  minDistance ,  minTrackSuccessRate );
	
	
	/*
	 * 		Setting up Visualizer
	 */
	vis_nav::Visualizer visualizer(imageWidth, imageHeight, useColor);
	vis_nav::ColorManager colorManager;
	
	
	
	ROS_INFO("VISNAV: Visual navigation node is up and running");
	
	int lineType = 8;
	int radius = 6;
	int lineThickness = 2;
	
	std_msgs::Int64 statusMsg;
	statusMsg.data = 1.0;
	visNavStatusPublisher.publish(statusMsg);
	
	while (ros::ok() ) {
		
		ros::spinOnce();
		if (imageLoader.hasImage() == false) {
			continue;
		}
		
		cv_bridge::CvImagePtr image = imageLoader.getImage();
		
		//visualizer.update(image->image);


        /*
        * Calibrate fisheye lens
        */

        cv::Mat frame = imageLoader.getImageMatrix(CV_8UC1);

        //DIM=(640, 480)
        //K=np.array([[338.1202749256424, 0.0, 278.594296413926], [0.0, 339.17124053097086, 207.64080164237097], [0.0, 0.0, 1.0]])
        //D=np.array([[0.0009507706696473223], [-0.027619038157897017], [0.04213022793571351], [-0.027907063457939283]])





        double k[3][3] = {{338.1202749256424, 0.0, 278.594296413926}, {0.0, 339.17124053097086, 207.64080164237097}, {0.0, 0.0, 1.0}}; // {{1076.543468167208, 0.0, 1167.7716020958378}, {0.0, 1081.9606511410773, 750.3661271924599}, {0.0, 0.0, 1.0}}
        cv::Mat K = cv::Mat(3, 3, CV_64F, k);

        double d[4][1] = {{0.0009507706696473223}, {-0.027619038157897017}, {0.04213022793571351}, {0.04213022793571351}}; // {{0.009836379664666049}, {-0.11230683007485048}, {0.19011424710630012}, {-0.045414305375157206}};
        cv::Mat D = cv::Mat(4, 1, CV_64F, d);

        //cv::Size DIM = cv::Size(640, 480);

        //cv::Mat map1, map2; // output matrices

        //cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat(), K, DIM, CV_16SC2, map1, map2); // cv::Mat::eye could be replaced with cv::Mat::eye(3,3, CV_8U)

        //cv::Mat undistorted_frame(map1.rows, map1.cols, CV_8UC1);

        //cv::remap(frame, undistorted_frame, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

        // undistorted frame must have same size and type as map1

        visualizer.update(frame);
        //cv::Mat diff = frame != undistorted_frame; // Equal if no elements disagree
        //bool eq = cv::countNonZero(diff) == 0;

        //ROS_INFO("map1 (row) : %i", map1.rows);
        //ROS_INFO("map1 (col) : %i", map1.cols);
        //ROS_INFO("undist (row) : %i", undistorted_frame.rows);
        //ROS_INFO("undist (col) : %i", undistorted_frame.cols);
        ROS_INFO("frame (row) : %i", frame.rows);
        ROS_INFO("frame (col) : %i", frame.cols);


       // ROS_INFO("type of undist: %i", undistorted_frame.type());
        ROS_INFO("type of frame: %i", frame.type());


        ROS_INFO("frame and undist should have same type. map1 and frame should have same size");


        //cv::imshow( "undistorted", undistorted_frame);
        //cv::waitKey(0);
        //cv::destroyAllWindows();
        //ROS_INFO("HEEEEEEEEEEEEEELLLOOOOOOOOOOO");
		
		
		/*
		 * 	Run optical flow
		 */
		
		opticalFlowSparse.updateWithNewImage(frame);
 		ROS_INFO("OPFLOW: Steps %i , track success: %f", opticalFlowSparse.getStepCounter(), opticalFlowSparse.getTrackSuccessRatio());
		
		/*
		 * 	Draw optical flow 
		 */
		
		visualizer.drawOpticalFlow(opticalFlowSparse.getInitialSet(), opticalFlowSparse.getCurrentSet(),opticalFlowSparse.getError(), opticalFlowSparse.getStatus(),
 			colorManager.getCVColor(vis_nav::Color::BLUE), colorManager.getCVColor(vis_nav::Color::RED), radius, lineThickness, lineType);
			
		
		/*
		 * 	Draw image cross 
		 */
		
		visualizer.drawImageCross(colorManager.getCVColor(vis_nav::Color::GREEN),  lineThickness,  lineType);
		
		vis_nav::PixelPoint centerPixel(imageWidth / 2,imageHeight / 2);
		
		cv_bridge::CvImage msg;
		msg.encoding = image->encoding;
		msg.header = image->header;
		msg.image = visualizer.getDrawImage();
		
		if (doPublishImage) {
			outputImagePub.publish(msg.toImageMsg());
		}
		
		rate.sleep();
		
	}
	
	return 0;
	
}
