#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "wamv_info.h"

class GazeboToMoos
{
    private:
	// Data
        WamvInfo lon,lat;
        WamvInfo x,y;
        WamvInfo speed; //bowspeed
        WamvInfo row,pitch,yaw;
        //WamvInfo zAngularV;

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

    public:
	// Data
	ros::NodeHandle m_nh; // for timer in main
        double m_loopFreq;    // for timer in main
	
	// Method
        GazeboToMoos();
        ~GazeboToMoos();
        void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void callback_imu(const nav_msgs::Odometry::ConstPtr& msg);
	void iterate(const ros::TimerEvent&);
        void setPose(geometry_msgs::Pose msg);
        void setVelocity(geometry_msgs::Twist msg);
        //void publishNavInfo();
};
