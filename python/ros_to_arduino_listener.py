#!/usr/bin/env python
import rospy
import struct
import serial
import math
from std_msgs.msg import Int32
from random import randint
from sensor_msgs.msg import JointState


ser = serial.Serial('/dev/ttyACM0', 115200)

# For the msg type, see http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html

Amin=[34,10,168,94]
Bmin=[-math.pi/4.0, 0,math.pi/2.0,0]
Amax=[106,156,14,18]
Bmax=[math.pi/4.0, -math.pi, -math.pi/2.0, -math.pi/2.0]

def random_callback(msg):
    rospy.loginfo(msg.position)
   # msg=','.join([str(x) for x in msg.position])
    for i in range(len(msg.position)):
        cmd = int(round((msg.position[i]-Bmin[i])*(Amax[i]-Amin[i])/(Bmax[i]-Bmin[i])+Amin[i]))
        out = '[{:1}{:3}]'.format(i,cmd)
        print(out)
        ser.write(out)
        ser.flush()


if __name__=='__main__':
    rospy.init_node('arm_driver')
    sub=rospy.Subscriber('joint_states', JointState, random_callback)
    rospy.spin()
