#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include "ntu_msgs/Tdoa.h"

class Localization
{
    private:
	// Data

        // Publisher
        ros::Publisher pub_NavHdg;
        ros::Publisher pub_TdoaAngle;

        // Subscriber
        ros::Subscriber sub_imu;
        ros::Subscriber sub_tdoa;

    public:
	// Data
	ros::NodeHandle m_nh; 

	// Method
        Localization();
        ~Localization();
        void callback_imu(const sensor_msgs::Imu msg);
	void callback_tdoa(const ntu_msgs::Tdoa::ConstPtr& msg);
	std_msgs::Float64 toRosFloat(double value);
	std_msgs::Float64 toMoosAngle(double value);
};
