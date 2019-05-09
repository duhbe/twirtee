
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <sstream>

/* To compile:
 *  roscd 
 *  cd ..
 *  catkin_make
 * To start the node: 
 *  rosrun c_publish_subscribe driver
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "driver");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("twirtee/cmd_vel", 1000);
  ros::Rate loop_rate(10);

  geometry_msgs::Twist vel;

  int count = 0;
  while (ros::ok())
  {
	// Set the longitudinal and angular speeds
	vel.angular.z = 1.0;
	vel.linear.x = 1.0;
	// Show that I am alive
    ROS_INFO("%i", count);
	// Publish the message...
    pub.publish(vel);
    ros::spinOnce();
    // and wait
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
