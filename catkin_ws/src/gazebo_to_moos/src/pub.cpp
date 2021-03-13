#include <ros/ros.h>
#include <std_msgs/Int32.h>
int main(int argc, char** argv){

  ros::init(argc, argv, "pub_bridge_node", ros::init_options::AnonymousName); // Can't be the same with other node
  ros::NodeHandle nh; // To start the node

  ros::Publisher pub = nh.advertise<std_msgs::Int32>("/CounterFromROS", 10);  // (topic name, qeue size)
  ros::Rate rate(3);

  while (ros::ok()){
    std_msgs::Int32 msg;
    msg.data = 699999;
    pub.publish(msg);
    rate.sleep();
    }
}
