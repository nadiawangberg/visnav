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
#include "opencv2/video/tracking.hpp"
#include <opencv2/aruco.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include <cv_bridge/cv_bridge.h>

#include <sstream>

int i = 70;
uint seq;
cv::Mat map;
int imageWidth, imageHeight = 600;
vis_nav::Visualizer vis(imageWidth, imageHeight, true);
std::string outputImageTopic;
ros::Publisher outputImagePub;
