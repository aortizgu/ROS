#include "ros/ros.h"
#include "std_msgs/Int64.h"

void messageCallback(const std_msgs::Int64::ConstPtr& msg)
{
  ROS_INFO("Message: [%ld]", msg->data);
}

int main(int argc, char **argv)
{

   ros::init(argc, argv, "num_subscriber");
   ros::NodeHandle n;

   ros::Subscriber sub = n.subscribe("message", 1, messageCallback);

   ros::spin();

   return 0;

 }
