#!/usr/bin/env python

import rospy
from altitude_sensor.msg import sensor_data
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from math import *

data_lidar = PoseStamped()
data_to_publish = PoseStamped()
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

def callback_lidar(data):
	global data_lidar
	if not data.altitude:
		print "no lidar"
	else:
		data_lidar.header = data.header
		data_lidar.pose.position.z = data.altitude
			

def callback_marker(data):
	global data_to_publish
	if not data.markers:
		print "no markers"
	else:
		for i in range(len(data.markers)):
			if data.markers[i].id == 0:

				data_to_publish.header = data.markers[i].header #take main heading with time stamp
				data_to_publish.pose = data.markers[i].pose.pose
				data_to_publish.pose.position.z = data_lidar.pose.position.z - 0.15
				data_to_publish.pose.position.y = - data_to_publish.pose.position.y #to get ENU frame coordinate
				data_to_publish.pose.position.x = data_to_publish.pose.position.x
				
				pub.publish(data_to_publish)
				print data_to_publish

def sensors2pixhawk():
	rospy.init_node('convert', anonymous=True)
	rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback_marker)
	rospy.Subscriber('altitude', sensor_data, callback_lidar)
	rospy.spin()

if __name__ == '__main__':
	try:
		sensors2pixhawk()
	except rospy.ROSInterruptException:
		pass
