#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

/* To compile:
 *  roscd 
 *  cd ..
 *  catkin_make
 * To start the node: 
 *  rosrun c_publish_subscribe listener
 */
void lidarCallback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Scanner[0]= [%f]", msg->ranges[0]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/twirtee/lidar", 1000, lidarCallback);
  ros::spin();
  return 0;
}
