#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from math import *
import tf
import numpy as np


data_to_publish = PoseStamped()
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

def imucallback(imudata):
	global roll
	global pitch
	Q_imu = np.array([imudata.orientation.x,
					  imudata.orientation.y,
					  imudata.orientation.z,
					  imudata.orientation.w])
	roll, pitch, _ = tf.transformations.euler_from_quaternion(Q_imu)
			

def ircallback(irdata):
	coords = irdata.data
	data_to_publish = PoseStamped()
	HEIGHT_OFFSET = 0.17 # distance between marvelmind sensors, when copter is on the ground [m]
	X_OFFSET = 0.0
	Y_OFFSET = 0.0 # distance between small ir-marker center and quadrotor center [m]
	data_to_publish.header.stamp = rospy.Time.now()
	data_to_publish.pose.position.x = - coords[1] + X_OFFSET
	data_to_publish.pose.position.y = coords[0] + Y_OFFSET
	data_to_publish.pose.position.z = coords[2] - HEIGHT_OFFSET
	yaw = ( coords[3] + 90/180*3.141592 )
	q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	data_to_publish.pose.orientation.x = q[0]
	data_to_publish.pose.orientation.y = q[1]
	data_to_publish.pose.orientation.z = q[2]
	data_to_publish.pose.orientation.w = q[3]
	print data_to_publish
	# pub.publish(data_to_publish)

def sensors2pixhawk():
	rospy.init_node('convert', anonymous=True)
	rospy.Subscriber('/uav/coordinates', Float32MultiArray, ircallback)
	rospy.Subscriber('/mavros/imu/data', Imu, imucallback)
	rospy.spin()

if __name__ == '__main__':
	try:
		sensors2pixhawk()
	except rospy.ROSInterruptException:
		pass
