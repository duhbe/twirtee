#!/usr/bin/env python

# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

import rospy
from geometry_msgs.msg import Twist 
  
# To get the message structure, use "rosmsg show Twist"...

def talker():
    pub = rospy.Publisher('/twirtee/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = Twist()
    msg.linear.x = 1
    msg.linear.y = 1
    msg.angular.x= 0
    msg.angular.x = 0
    msg.angular.z = 1.0
    
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
