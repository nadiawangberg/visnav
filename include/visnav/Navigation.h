#pragma once 

#include <string>
#include <cmath>

#include <ros/file_log.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/CommandTOL.h>

namespace vis_nav {

class Navigator {
	
public:
	
	Navigator(double expectedNavFrequency = 10.0);
	~Navigator();
	
	bool checkNavigationStatus();
	
	double getLatitude() const;
	double getLongitude() const;
	double getAltitude() const;
	
	double getVelE() const;
	double getVelN() const;
	double getVelU() const;
	
	double getRoll() const;
	double getPitch() const;
	double getYaw() const;
	
	//ROS callbacks which commuicated with drone
	void imu_callback(const sensor_msgs::Imu& data);
	void vel_callback(const geometry_msgs::TwistStamped& data);
	void alt_callback(const mavros_msgs::VFR_HUD& data);
	void pos_callback(const sensor_msgs::NavSatFix& data);
	
private:
	
	double m_expectedNavFrequency;
	
	double m_firstAltitude;
	double m_currentAltitude;
	
	bool m_hasPosition;
	bool m_hasVelocity;
	bool m_hasAltitude;
	bool m_hasAtitude;
	
	double m_latitude;
	double m_longitude;
	double m_altitude;
	double m_velE;
	double m_velN;
	double m_velU;
	double m_roll;
	double m_pitch;
	double m_yaw;
// 	
	
	
};

}