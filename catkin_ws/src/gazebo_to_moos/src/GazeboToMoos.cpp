#include <tf/tf.h>
#include "GazeboToMoos.h"
using namespace std;

// Constructor
GazeboToMoos::GazeboToMoos(){
    // OnStartUp
    string key;
    if (ros::param::search("loopFreq", key)){
        ros::param::get(key, m_loopFreq); // Hz
	//ROS_INFO("!!!!!!!!!!loopFreq : %f", m_loopFreq);
    }

    // Publisher
    pub_NavX = m_nh.advertise<std_msgs::Float64>("NAV_X", 10);
    pub_NavY = m_nh.advertise<std_msgs::Float64>("NAV_Y", 10);
    pub_NavLon = m_nh.advertise<std_msgs::Float64>("NAV_LONG", 10);
    pub_NavLat = m_nh.advertise<std_msgs::Float64>("NAV_LAT", 10);
    pub_NavSpeed = m_nh.advertise<std_msgs::Float64>("NAV_SPEED_X", 10);
    pub_NavHdg = m_nh.advertise<std_msgs::Float64>("NAV_HEADING", 10);
    // Subscriber
    sub_gps = m_nh.subscribe("/fix", 1000, &GazeboToMoos::callback_gps, this);
    //sub_gps = m_nh.subscribe("wamv/sensors/gps/gps/fix", 1000, &GazeboToMoos::callback_gps, this); // for gazebo vrx
    sub_imu = m_nh.subscribe("wamv/robot_localization/odometry/filtered", 1000, &GazeboToMoos::callback_imu, this);
    // Iterate
    // fail to use createTimer in constructor
    // compile successfully but not calling iterate function
    // Thus, declare nh as public and declare createTimer in main
    //ros::Timer timer = m_nh.createTimer(ros::Duration(1.0/10.0), &GazeboToMoos::iterate, this);
}

// Destructor
GazeboToMoos::~GazeboToMoos(){
}

// Callback
void GazeboToMoos::callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg){
    lon.set(msg->longitude);
    lat.set(msg->latitude);
    //ROS_INFO("NAV_LON is : %f", lon.get());
    //ROS_INFO("loopFreq is : %f", m_loopFreq);
}

void GazeboToMoos::callback_imu(const nav_msgs::Odometry::ConstPtr& msg){
    setPose(msg->pose.pose);
    setVelocity(msg->twist.twist);
}

// Iterate
void GazeboToMoos::iterate(const ros::TimerEvent&)
{
    //ROS_INFO("This is iterate!");
    pub_NavX.publish(x.getRosFloat());
    pub_NavY.publish(y.getRosFloat());
    pub_NavLat.publish(lat.getRosFloat());
    pub_NavLon.publish(lon.getRosFloat());
    pub_NavSpeed.publish(speed.getRosFloat());
    pub_NavHdg.publish(yaw.getMoosAngle());
    //ROS_INFO("NAV_LON is : %f", lon.get());
    //ROS_INFO("loopFreq is : %f", m_loopFreq);
    ROS_INFO("ROS yaw      : %f", yaw.get());
    std_msgs::Float64 msg = yaw.getMoosAngle();
    double result = msg.data;
    ROS_INFO("MOOS heading : %f", result);
}

// setPose
// geometry_msgs::Pose contains position(xyz) & orientation(quaterion) info
void GazeboToMoos::setPose(geometry_msgs::Pose msg)
{
    // give local position to WamvInfo x,y
    // which later become NAV_X,NAV_Y
    x.set(msg.position.x);
    y.set(msg.position.y);   

    // transform quaterion into rpy
    tf::Quaternion q(msg.orientation.x,
		     msg.orientation.y,
		     msg.orientation.z,
		     msg.orientation.w);
    tf::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    row.set(r);
    pitch.set(p);
    yaw.set(y);
}


// setVelocity
void GazeboToMoos::setVelocity(geometry_msgs::Twist msg)
{
    // speed later become NAV_SPEED
    // does it mean liner.x is in "base_link" frame ?
    speed.set(msg.linear.x);
    //speed.set(0.0);
    //bowSpeed.set(msg.linear.y);
    //zAngularV.set(msg.angular.z);
}

/*==============================*/
// Main
int main (int argc, char **argv)
{
    ros::init(argc, argv, "GazeboToMoos_node");
    GazeboToMoos node;
    ros::Timer timer = node.m_nh.createTimer(ros::Duration(node.m_loopFreq), &GazeboToMoos::iterate, &node);
    ros::spin();
    return 0;
}
