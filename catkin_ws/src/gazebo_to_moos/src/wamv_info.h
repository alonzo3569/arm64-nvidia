#define pi 3.1415926
#include <std_msgs/Float64.h>

class WamvInfo
{
   public:
        WamvInfo(){};
	~WamvInfo(){};
	void set(double value)    {m_value = value;};
	double get()              {return m_value;};
	std_msgs::Float64 getRosFloat();
	std_msgs::Float64 getMoosAngle();
	std_msgs::Float64 getMoosAngularV();
   private:
	double m_value;
 
};

std_msgs::Float64 WamvInfo::getRosFloat(){
    std_msgs::Float64 msg;
    msg.data = m_value;
    return msg;
}

std_msgs::Float64 WamvInfo::getMoosAngle(){
    std_msgs::Float64 msg;
    double angle = -(m_value*180/pi)+90;
    if(angle < 0)
        angle += 360;
    msg.data = angle;
    return msg;
}

std_msgs::Float64 WamvInfo::getMoosAngularV(){
    std_msgs::Float64 msg;
    msg.data = -(m_value*180/pi);
    return msg;
};


//class WamvInfo
//{
//    public:
//        WamvInfo(){};
//	~WamvInfo(){};
//        WamvBase lon,lat;
//	WamvBase x,y;
//	WamvBase speed,bowspeed;
//	WamvBase row,pitch,yaw;
//	WamvBase zAngularV;
//};
