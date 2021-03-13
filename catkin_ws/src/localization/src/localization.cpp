#include <tf/tf.h>
#include "localization.h"
#include "UTM.cpp"
using namespace std;

// Constructor
Localization::Localization(){
    // Initialize
    m_orig_lat = 0.0;
    m_orig_lon = 0.0;
    m_orig_northing = 0.0;
    m_orig_easting = 0.0;
    m_zone = 51;

    // OnStartUp
    string key;
    if (ros::param::search("latitude", key)){
        ros::param::get(key, m_orig_lat); // Hz
	ROS_INFO("!!!!!!!!!!orig_lat : %f", m_orig_lat);
    }

    if (ros::param::search("longitude", key)){
        ros::param::get(key, m_orig_lon); // Hz
	ROS_INFO("!!!!!!!!!!orig_lon : %f", m_orig_lon);
        double lonInRad = DegToRad(m_orig_lon);
        double latInRad = DegToRad(m_orig_lat);
        MapLatLonToXY (latInRad, lonInRad, UTMCentralMeridian(m_zone), m_orig_easting, m_orig_northing);
        ROS_INFO("orig_easting    : %f", m_orig_easting);
        ROS_INFO("orig_northing    : %f",m_orig_northing);
   }

    // Publisher
    pub_NavX = m_nh.advertise<std_msgs::Float64>("NAV_X", 10);
    pub_NavY = m_nh.advertise<std_msgs::Float64>("NAV_Y", 10);
    pub_NavLon = m_nh.advertise<std_msgs::Float64>("NAV_LONG", 10);
    pub_NavLat = m_nh.advertise<std_msgs::Float64>("NAV_LAT", 10);
    pub_NavSpeed = m_nh.advertise<std_msgs::Float64>("NAV_SPEED_X", 10);
    pub_NavHdg = m_nh.advertise<std_msgs::Float64>("NAV_HEADING", 10);

    // Subscriber
    sub_gps = m_nh.subscribe("/fix", 1000, &Localization::callback_gps, this);
    sub_imu = m_nh.subscribe("/imu", 1000, &Localization::callback_imu, this);
}

// Destructor
Localization::~Localization(){
}

// Callback
void Localization::callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg){
    //Process gps lat lon
    double lat = msg->latitude;
    double lon = msg->longitude;
    double latInRad = DegToRad(lat);
    double lonInRad = DegToRad(lon);
    ROS_INFO("float lat  : %f", lat);
    ROS_INFO("Rad lat    : %f", latInRad);
    
    double easting = 0.0;
    double northing = 0.0;
    MapLatLonToXY (latInRad, lonInRad, UTMCentralMeridian(m_zone), easting, northing);
    //MapLatLonToXY (FLOAT lat in rad, FLOAT lon in rad , FLOAT zone in rad, FLOAT &x, FLOAT &y);
    ROS_INFO("easting    : %f", easting);
    ROS_INFO("northing    : %f", northing);

    // Calculate x,yi
    double x = easting - m_orig_easting;
    double y = northing - m_orig_northing;
    ROS_INFO("x    : %f", x);
    ROS_INFO("y    : %f", y);


    // publish to bridge topic
    pub_NavX.publish(toRosFloat(x));
    pub_NavY.publish(toRosFloat(y));
    pub_NavLat.publish(toRosFloat(lat));
    pub_NavLon.publish(toRosFloat(lon));
}

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
    yaw = yaw*180.0/pi;
    ROS_INFO("yaw in degree : %f", yaw);
    pub_NavHdg.publish(toMoosAngle(yaw));
    pub_NavSpeed.publish(toRosFloat(0.0));
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
