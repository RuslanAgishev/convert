#!/usr/bin/env python


import rospy
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from math import *


import message_filters
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

#data_to_publish = PoseStamped()
#pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)


data_marker = AlvarMarkers()
data_imu = Imu()

data_to_publish = PoseStamped()
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)


def callback_imu(data):
	global data_imu	
	data_imu = data

def callback_marker(data):
	data_marker = data
	

	rospy.Subscriber('mavros/imu/data', Imu, callback_imu)
	quaternion = (
    		data_imu.orientation.x,
    		data_imu.orientation.y,
    		data_imu.orientation.z,
    		data_imu.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	
	#math.tan(euler[0])

	if not data_marker.markers:
		print "no markers"
	else:
		print "\nMarker position:\n"
		for i in range(len(data_marker.markers)):
			if data_marker.markers[i].id == 0:
				data_to_publish.header = data_marker.markers[i].header
				data_to_publish.pose = data_marker.markers[i].pose.pose
				marker_dist = abs(data_marker.markers[i].pose.pose.position.z)
				print "marker_dist:", marker_dist
				data_to_publish.pose.position.z = - data_to_publish.pose.position.z + 1.45 
				data_to_publish.pose.position.y = data_to_publish.pose.position.y  - tan(euler[1])*marker_dist
				data_to_publish.pose.position.x = data_to_publish.pose.position.x - tan(euler[0])*marker_dist
				# TODO: take yaw into account for x, y calculations
				#print "popravka: ", tan(euler[0])*marker_dist
				print data_to_publish.pose
				pub.publish(data_to_publish)
		
		
		
		print "\nImu data:\n"
		#print data_imu.orientation
		#print "roll: ", euler[0]
		print "roll: ", degrees(euler[0])
		#print "pitch: ", euler[1]
		print "pitch: ", degrees(euler[1])
		print




def convert():
	
	rospy.init_node('convert', anonymous=True)
	rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback_marker)
	rospy.spin()

if __name__ == '__main__':
	try:
		convert()
	except rospy.ROSInterruptException:
		pass
