#include "to_moosrosbridge.h"
using namespace std;

// Constructor
ToMoosRosBridge::ToMoosRosBridge(){

    // Publisher
    pub_NavX = m_nh.advertise<std_msgs::Float64>("NAV_X", 10);
    pub_NavY = m_nh.advertise<std_msgs::Float64>("NAV_Y", 10);
    pub_NavLon = m_nh.advertise<std_msgs::Float64>("NAV_LONG", 10);
    pub_NavLat = m_nh.advertise<std_msgs::Float64>("NAV_LAT", 10);
    pub_NavSpeed = m_nh.advertise<std_msgs::Float64>("NAV_SPEED_X", 10);
    pub_NavHdg = m_nh.advertise<std_msgs::Float64>("NAV_HEADING", 10);

    // Subscriber
    sub_gps = m_nh.subscribe("/fix", 1000, &ToMoosRosBridge::callback_gps, this);
    sub_robot_status = m_nh.subscribe("/robot_status", 1000, &ToMoosRosBridge::callback_robot_status, this);
}

// Destructor
ToMoosRosBridge::~ToMoosRosBridge(){
}

// Callback
void ToMoosRosBridge::callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg){
    //Process gps lat lon
    double lat = msg->latitude;
    double lon = msg->longitude;
    
    pub_NavLat.publish(toRosFloat(lat));
    pub_NavLon.publish(toRosFloat(lon));
}

void ToMoosRosBridge::callback_robot_status(const ntu_msgs::RobotStatus::ConstPtr& msg){
    double x = msg->local_x;
    double y = msg->local_y;
    double heading = msg->heading;
    pub_NavX.publish(toRosFloat(x));
    pub_NavY.publish(toRosFloat(y));
    pub_NavHdg.publish(toRosFloat(heading));
    pub_NavSpeed.publish(toRosFloat(0.0));
}

std_msgs::Float64 ToMoosRosBridge::toRosFloat(double value){
    std_msgs::Float64 msg;
    msg.data = value;
    return msg;
}



/*==============================*/
// Main
int main (int argc, char **argv)
{
    ros::init(argc, argv, "Localization_node");
    ToMoosRosBridge node;
    ros::spin();
    return 0;
}
