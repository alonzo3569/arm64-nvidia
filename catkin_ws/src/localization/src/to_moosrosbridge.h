#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include "ntu_msgs/RobotStatus.h"


class ToMoosRosBridge
{
    private:
	// Data
	std::string m_vehicle;

	// Publisher
        ros::Publisher pub_NavX;
        ros::Publisher pub_NavY;
        ros::Publisher pub_NavLon;
        ros::Publisher pub_NavLat;
        ros::Publisher pub_NavSpeed;
        ros::Publisher pub_NavHdg;

        // Subscriber
        ros::Subscriber sub_gps;
        ros::Subscriber sub_robot_status;

    public:
	// Data
	ros::NodeHandle m_nh; 

	// Method
        ToMoosRosBridge();
        ~ToMoosRosBridge();
        void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void callback_robot_status(const ntu_msgs::RobotStatus::ConstPtr& msg);
	std_msgs::Float64 toRosFloat(double value);
};
