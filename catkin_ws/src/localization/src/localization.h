#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

class Localization
{
    private:
	// Data
	double m_orig_lon, m_orig_lat;
	double m_orig_easting, m_orig_northing;
	int m_zone;
	std_msgs::Float64 m_x, m_y;

        // Publisher
        ros::Publisher pub_NavX;
        ros::Publisher pub_NavY;
        ros::Publisher pub_NavLon;
        ros::Publisher pub_NavLat;
        ros::Publisher pub_NavSpeed;
        ros::Publisher pub_NavHdg;

        // Subscriber
        ros::Subscriber sub_gps;
        ros::Subscriber sub_imu;
        //ros::Subscriber sub_imu_raw;

    public:
	// Data
	ros::NodeHandle m_nh; // for timer in main
        //double m_loopFreq;    // for timer in main
	
	// Method
        Localization();
        ~Localization();
        void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void callback_imu(const sensor_msgs::Imu msg);
	std_msgs::Float64 toRosFloat(double value);
	std_msgs::Float64 toMoosAngle(double value);
        //void callback_imu_raw(const nav_msgs::Odometry::ConstPtr& msg);
        //void publishNavInfo();
};
