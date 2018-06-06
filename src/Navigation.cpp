
#include <visnav/Navigation.h>

namespace vis_nav { 

Navigator::Navigator(double expectedNavFrequency)
	: m_expectedNavFrequency(expectedNavFrequency) 
	, m_firstAltitude(0.0)
	, m_currentAltitude(0.0)
	, m_hasPosition(false)
	, m_hasVelocity(false)
	, m_hasAltitude(false)
	, m_hasAtitude(false) {
}

Navigator::~Navigator()  {
}

bool Navigator::checkNavigationStatus() {
	
	bool hasAll = true;
	if(m_hasAltitude == false) {
		ROS_WARN_THROTTLE(2.0, "NAV: Navigator does not have a altitude fix");
		hasAll = false;
	}
	if(m_hasAtitude == false) {
		ROS_WARN_THROTTLE(2.0, "NAV: Navigator does not have a orientation fix");
		hasAll = false;
	}
	if(m_hasPosition == false) {
		ROS_WARN_THROTTLE(2.0, "NAV: Navigator does not have a GPS Fix");
		hasAll = false;
	}
	return hasAll;
}


void Navigator::imu_callback(const sensor_msgs::Imu &data) {
	
	double x = data.orientation.x;
	double y = data.orientation.y;
	double z = data.orientation.z;
	double w = data.orientation.w;
	
	m_roll  = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x*x + y*y) );
	m_pitch = std::asin( 2.0 *  (w * y - x * z) );
	m_yaw   = std::atan2(2.0 * (z * w + y * x), 1.0 - 2.0 * (y*y + z*z ) );
		
	
	if(m_hasAtitude == false) {
		m_hasAtitude = true;
	}
	
}

void Navigator::vel_callback(const geometry_msgs::TwistStamped& data) {
	
	m_velE = data.twist.linear.x;
	m_velN = data.twist.linear.y;
	m_velU = data.twist.linear.z;
	
	if(m_hasVelocity == false) {
		m_hasVelocity = true;
	}
}
	
void Navigator::alt_callback(const   mavros_msgs::VFR_HUD& data) {
	
	if(m_hasAltitude == false) {
		m_altitude = data.altitude;
		m_hasAltitude = true;
	}
	m_currentAltitude = data.altitude;
}

void Navigator::pos_callback(const sensor_msgs::NavSatFix& data) {
	
	if(data.status.status == -1) {
		return;
	} else {
		m_hasPosition = true;
	}
	
	m_latitude  = data.latitude   * (M_PI / 180.0);
	m_longitude = data.longitude  * (M_PI / 180.0);
	
}

double Navigator::getLatitude() const {
	return m_latitude;
}

double Navigator::getLongitude() const {
	return m_longitude;
}

double Navigator::getAltitude() const {
	return m_altitude;
}

double Navigator::getRoll() const {
	return m_roll;
}

double Navigator::getPitch() const {
	return m_pitch;
}

double Navigator::getYaw() const {
	return m_yaw;
}

double Navigator::getVelE() const {
	return m_velE;
}

double Navigator::getVelN() const {
	return m_velN;
}

double Navigator::getVelU() const {
	return m_velU;
}



}