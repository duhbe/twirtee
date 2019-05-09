#!/usr/bin/env python

# (C) Eric JENN 2019
# This script simply subscribes to the joint_states topics and print the pose
# of the last joint of the arm (the one that holding the camera and the "finger").
# It also publishes a marker located at the end of the finger tip.


# The pose the the links can be obtained by subscribing to the gazebo/link_states/pose topic. 
# The structure of the message is described at http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html.

# The (initial) pose may be obtained by calling the appropriate service. 
# The service can be called at the command line using:
# $ rosservice call gazebo/get_link_properties '{link_name: link5}'
# Command rosservice also allows to get some information about the services (see rosservice --help)


# The actual pose of the link can be obtained using 
# $rosrun tf tf_echo chassis link5
# which returns the translation and rotation of "link5" according to the frame "chassis".

# The transformations can be obtained using:
# $ rosrun tf view_frames
# $ evince frames.pdf
# Another command:
# $ rosrun tf tf_monitor chassis link5


from gazebo_msgs.srv import GetLinkProperties
import rospy
import tf
 
from std_msgs.msg import Int32
from random import randint
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped



# This callback is used to get the angle of last joint of the arm.
def js_callback(msg):
    # Link5 is the fifth entry of the array 
    # Use "$rostopic echo  /twirtee/joint_states/name" to get the list of link names.
	# The structure of the jointstate message is described at http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
	# The position is given in radians.
    print("Angle of link5/link4: " +str(msg.position[4]))

# This callback is used to get the absolute pose of the robot as computed by
# odometry.
# This means that the odometry topic must be published
# This is done by the differential_drive_controller which is declared in file "twirtee.gazebo".
def pose_callback(msg):
    # Use "$rostopic echo  /twirtee/odom" to get the odometry topic 
    # The structure of the message can be found at http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
    # It can also be found using rqt 'Plugins/Topics/Topic monitor"
    print("Position:\n" +str(msg.pose.pose.position))
    print("Orientation:\n" +str(msg.pose.pose.orientation))

        
if __name__=='__main__':

    rospy.init_node('rand_subscriber')

    # Getting the pose of link5 in its reference frame using a service call
    rospy.wait_for_service('gazebo/get_link_properties')
    service = rospy.ServiceProxy('gazebo/get_link_properties', GetLinkProperties)
    print( "Pose of link5:\n" + str(service('link5')))

    fingerTipMarker = Marker()	    
    markerPub = rospy.Publisher('fingerTipMarker', Marker, queue_size=10)
    
    # Setup the transform listener.
    # Documentation about the transform API can be found at http://docs.ros.org/jade/api/tf/html/python/tf_python.html#transformlistener 
    listener = tf.TransformListener()
    
    # and test it
    listener.waitForTransform("link5", "footprint", rospy.Time(), rospy.Duration(10.0))
    print("Transformation received:\n")
    print(listener.allFramesAsString())


    # To get the message type, use "$ rostopic info /twirtee/odom"
    #jssub=rospy.Subscriber('/twirtee/joint_states', JointState, js_callback)
    posesub=rospy.Subscriber('/twirtee/odom', Odometry, pose_callback)
    
    while not rospy.is_shutdown():
		# Wait until we receive a consistent set of transforms from frame "odom" to frame "finger".
        listener.waitForTransform("finger", "odom", rospy.Time(), rospy.Duration(1.0))
        # Once we have the transformations, compute the coordinate of the finger tip in the odometry frame
        # We create a point located at the tip of the finger
        v = PointStamped()
        v.header.frame_id = "finger"
        v.header.stamp =rospy.Time(0)
        # The tip is located "fingerLength/2" from the origin of the frame (see file "mybo.xacro")
        v.point.x=-0.05
        v.point.y=0.0
        v.point.z=0.0
        # And we compte its coordinates in the "odom" frame. 
        p=listener.transformPoint("odom",v)
        print( p )
        # And we publish a marker at that location
        fingerTipMarker.header.frame_id = "odom"
        fingerTipMarker.header.stamp    = rospy.get_rostime()
        fingerTipMarker.ns = "twirtee"
        fingerTipMarker.id = 0
        fingerTipMarker.type = 2 # sphere
        fingerTipMarker.action = 0
        fingerTipMarker.pose.position = p.point
        fingerTipMarker.pose.orientation.x = 0
        fingerTipMarker.pose.orientation.y = 0
        fingerTipMarker.pose.orientation.z = 0
        fingerTipMarker.pose.orientation.w = 1.0
        fingerTipMarker.scale.x = 0.01
        fingerTipMarker.scale.y = 0.01
        fingerTipMarker.scale.z = 0.01

        fingerTipMarker.color.r = 0.0
        fingerTipMarker.color.g = 1.0
        fingerTipMarker.color.b = 0.0
        fingerTipMarker.color.a = 1.0

        fingerTipMarker.lifetime = rospy.Duration(100)

        markerPub.publish(fingerTipMarker)
        rospy.Rate(1).sleep()
    
    rospy.spin()
