#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped
from math import *

data_to_publish = PoseStamped()
pub = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size=10)

def callback(data):
	global data_to_publish
	data_to_publish.header = data.header #take main heading with time stamp
	#data_to_publish.pose.position = data.transform.translation
	data_to_publish.pose.orientation = data.transform.rotation

	# transformations to get ENU frame coordinate
	data_to_publish.pose.position.x = - data.transform.translation.y
	data_to_publish.pose.position.y = data.transform.translation.x
	data_to_publish.pose.position.z = data.transform.translation.z
	
	
	roll,pitch,yaw = tf.transformations.euler_from_quaternion((data.transform.rotation.x,
															   data.transform.rotation.y,
															   data.transform.rotation.z,
															   data.transform.rotation.w))
	
	q = tf.transformations.quaternion_from_euler(roll,pitch,yaw+radians(90))
	data_to_publish.pose.orientation.x = q[0]
	data_to_publish.pose.orientation.y = q[1]
	data_to_publish.pose.orientation.z = q[2]
	data_to_publish.pose.orientation.w = q[3]
	

	pub.publish(data_to_publish)
	print(data_to_publish)
	'''
	br_mocap = tf.TransformBroadcaster()
	br_mocap.sendTransform((data.transform.translation.x,data.transform.translation.y,data.transform.translation.z),
                     (data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z,data.transform.rotation.w),
                     rospy.Time.now(),
                     "mocap",
                     "world")

	br_vp = tf.TransformBroadcaster()
	br_vp.sendTransform((data_to_publish.pose.position.x,data_to_publish.pose.position.y,data_to_publish.pose.position.z),
                     (data_to_publish.pose.orientation.x,data_to_publish.pose.orientation.y,data_to_publish.pose.orientation.z,data_to_publish.pose.orientation.w),
                     rospy.Time.now(),
                     "vispos",
                     "world")
	'''

def mocap2pixhawk():
	rospy.init_node('convert', anonymous=True)
	rospy.Subscriber('/vicon/dronearm/dronearm', TransformStamped, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		mocap2pixhawk()
	except rospy.ROSInterruptException:
		pass

