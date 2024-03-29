#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include "ntu_msgs/RobotStatus.h"
#include "ntu_msgs/Tdoa.h"


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
        ros::Publisher pub_TdoaAngle;

        // Subscriber
        ros::Subscriber sub_gps;
        ros::Subscriber sub_robot_status;
        ros::Subscriber sub_tdoa;

    public:
	// Data
	ros::NodeHandle m_nh; 

	// Method
        ToMoosRosBridge();
        ~ToMoosRosBridge();
        void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void callback_robot_status(const ntu_msgs::RobotStatus::ConstPtr& msg);
        void callback_tdoa(const ntu_msgs::Tdoa::ConstPtr& msg);
	std_msgs::Float64 toRosFloat(double value);
};
