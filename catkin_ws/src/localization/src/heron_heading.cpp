#include <tf/tf.h>
#include "cmath"
#include "heron_heading.h"
using namespace std;

// Constructor
Localization::Localization(){
    // Initialize

    // OnStartUp

    // Publisher
    pub_NavHdg = m_nh.advertise<std_msgs::Float64>("NAV_HEADING_IMU", 10);

    // Subscriber
    sub_imu = m_nh.subscribe("imu", 1000, &Localization::callback_imu, this);
}

// Destructor
Localization::~Localization(){
}

// Callback
void Localization::callback_imu(const sensor_msgs::Imu msg){
    // transform quaterion into rpy
    tf::Quaternion q(msg.orientation.x,
                     msg.orientation.y,
                     msg.orientation.z,
                     msg.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("yaw in rad    : %f", yaw);
    yaw = yaw*180.0/M_PI;
    ROS_INFO("yaw in degree : %f", yaw);
    pub_NavHdg.publish(toMoosAngle(yaw));
}

std_msgs::Float64 Localization::toRosFloat(double value){
    std_msgs::Float64 msg;
    msg.data = value;
    return msg;
}

std_msgs::Float64 Localization::toMoosAngle(double value){
    std_msgs::Float64 msg;
    if(value < 0.0)
        value = -value;
    else
	value = -value + 360.0;
    msg.data = value;
    return msg;
    //ROS_INFO("yaw to moos   : %f", value);
}

/*==============================*/
// Main
int main (int argc, char **argv)
{
    ros::init(argc, argv, "Localization_node");
    Localization node;
    ros::spin();
    return 0;
}
